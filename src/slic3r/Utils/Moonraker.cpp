#include "Moonraker.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <sstream>
#include <thread>

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <nlohmann/json.hpp>

#include "libslic3r/PrintConfig.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/format.hpp"
#include "Http.hpp"

namespace pt = boost::property_tree;

namespace Slic3r {

Moonraker::Moonraker(DynamicPrintConfig *config)
    : m_host(config->opt_string("print_host"))
    , m_apikey(config->opt_string("printhost_apikey"))
    , m_cafile(config->opt_string("printhost_cafile"))
    , m_ssl_revoke_best_effort(config->opt_bool("printhost_ssl_ignore_revoke"))
    , m_skip_precheck(config->opt_bool("printhost_skip_precheck"))
    , m_skip_foreign_detect(config->opt_bool("printhost_skip_foreign_detect"))
{}

const char* Moonraker::get_name() const { return "Moonraker"; }

namespace {

//ORCA: Moonraker's SSDP component answers an M-SEARCH only when it carries this marker header.
//      The firmware discards unmarked datagrams before any other check, which is why neither a
//      stock UPnP probe nor an mDNS browse of _moonraker._tcp finds these printers. MAN and an
//      ST of ssdp:all or upnp:rootdevice are required as well; MX spreads the reply over a random
//      delay of up to that many seconds.
const std::string SSDP_MSEARCH_REQUEST = "M-SEARCH * HTTP/1.1\r\n"
                                         "HOST: 239.255.255.250:1900\r\n"
                                         "MAN: \"ssdp:discover\"\r\n"
                                         "MX: 1\r\n"
                                         "ST: ssdp:all\r\n"
                                         "X-SPECIAL-MARKER: FIBRESEEK3D\r\n"
                                         "\r\n";

const char*          SSDP_MULTICAST_ADDRESS = "239.255.255.250";
const unsigned short SSDP_PORT              = 1900;
const int            MOONRAKER_DEFAULT_PORT = 7125;

// A pending reply suppresses further requests on the printer side, so the repeats only cover
// datagram loss.
const int                       SSDP_REQUEST_REPEATS  = 3;
const std::chrono::milliseconds SSDP_REQUEST_INTERVAL = std::chrono::milliseconds(350);

std::string ssdp_header(const std::string& reply, const std::string& key)
{
    std::istringstream stream(reply);
    std::string        line;
    while (std::getline(stream, line)) {
        const auto colon = line.find(':');
        if (colon != std::string::npos && boost::iequals(boost::trim_copy(line.substr(0, colon)), key))
            return boost::trim_copy(line.substr(colon + 1));
    }
    return {};
}

// LOCATION points at the printer's own API root (http://<ip>:7125/server/zeroconf/ssdp), so the
// API port is read from it rather than assumed.
int api_port_from_location(const std::string& location)
{
    const auto scheme = location.find("://");
    if (scheme == std::string::npos)
        return 0;

    const auto host  = scheme + 3;
    const auto path  = location.find('/', host);
    const auto colon = location.find(':', host);
    if (colon == std::string::npos || (path != std::string::npos && colon > path))
        return 0;

    try {
        return std::stoi(location.substr(colon + 1, path == std::string::npos ? std::string::npos : path - colon - 1));
    } catch (...) {
        return 0;
    }
}

std::vector<boost::asio::ip::address_v4> local_ipv4_addresses()
{
    std::vector<boost::asio::ip::address_v4> addresses;

    try {
        boost::asio::io_context        io_context;
        boost::asio::ip::tcp::resolver resolver(io_context);
        boost::system::error_code      ec;

        const auto host_name = boost::asio::ip::host_name(ec);
        if (ec)
            return addresses;

        for (const auto& entry : resolver.resolve(boost::asio::ip::tcp::v4(), host_name, "", ec)) {
            const auto address = entry.endpoint().address();
            if (address.is_v4() && !address.is_loopback())
                addresses.push_back(address.to_v4());
        }
    } catch (...) {
    }

    return addresses;
}

// /server/info is the endpoint test() uses, so a host that answers it is one upload() can talk to.
bool answers_moonraker_api(const MoonrakerDiscoveredPrinter& printer)
{
    bool answered = false;

    Http::get((boost::format("http://%1%:%2%/server/info") % printer.ip_address % printer.port).str())
        .timeout_connect(3)
        .timeout_max(5)
        .on_complete([&answered](std::string body, unsigned) { answered = body.find("klippy_state") != std::string::npos; })
        .perform_sync();

    return answered;
}

} // namespace

std::string Moonraker::default_webui_url(const std::string& host)
{
    if (host.empty())
        return {};

    std::string url = host;
    if (!boost::istarts_with(url, "http://") && !boost::istarts_with(url, "https://"))
        url = "http://" + url;

    const auto scheme_end      = url.find("://");
    const auto authority_start = scheme_end == std::string::npos ? 0 : scheme_end + 3;
    const auto slash           = url.find('/', authority_start);
    const auto authority_end   = slash == std::string::npos ? url.size() : slash;
    const auto at              = url.find('@', authority_start);
    const auto host_start      = (at != std::string::npos && at < authority_end) ? at + 1 : authority_start;
    const auto colon           = url.find(':', host_start);

    if (colon != std::string::npos && colon < authority_end) {
        int port = 0;
        try {
            port = std::stoi(url.substr(colon + 1, authority_end - colon - 1));
        } catch (...) {
            port = 0;
        }
        if (port == MOONRAKER_DEFAULT_PORT)
            return url.substr(0, colon);
    }

    return url.substr(0, authority_end);
}

bool Moonraker::parse_ssdp_reply(const std::string& reply, const std::string& ip_address, MoonrakerDiscoveredPrinter& printer)
{
    // Unrelated UPnP devices answer an ssdp:all search too. The marked M-SEARCH
    // is required for FibreSeeker firmware to reply; stock Moonraker that does
    // answer SSDP is accepted without echoing the probe marker.
    if (!boost::icontains(reply, "SERVER: Moonraker"))
        return false;

    const int port = api_port_from_location(ssdp_header(reply, "LOCATION"));

    printer.ip_address   = ip_address;
    printer.port         = port > 0 ? port : MOONRAKER_DEFAULT_PORT;
    printer.machine_name = ssdp_header(reply, "X-MACHINE-NAME");
    printer.machine_id   = ssdp_header(reply, "X-MACHINE-ID");
    return true;
}

bool Moonraker::discover_printers(std::vector<MoonrakerDiscoveredPrinter>& printers, wxString& msg, int timeout_ms)
{
    printers.clear();

    const auto local_addresses = local_ipv4_addresses();
    if (local_addresses.empty()) {
        msg = _(L("No local network interface with an IPv4 address was found."));
        return false;
    }

    std::map<std::string, MoonrakerDiscoveredPrinter> by_ip;

    try {
        boost::asio::io_context                                    io_context;
        std::vector<std::unique_ptr<boost::asio::ip::udp::socket>> sockets;

        // Access points routinely drop one of the two, and the printer listens on both: the SSDP
        // multicast group and the subnet broadcast.
        const std::vector<boost::asio::ip::udp::endpoint> targets {
            {boost::asio::ip::make_address_v4(SSDP_MULTICAST_ADDRESS), SSDP_PORT},
            {boost::asio::ip::address_v4::broadcast(), SSDP_PORT}};

        for (const auto& local_address : local_addresses) {
            auto                      socket = std::make_unique<boost::asio::ip::udp::socket>(io_context);
            boost::system::error_code ec;

            socket->open(boost::asio::ip::udp::v4(), ec);
            if (ec)
                continue;

            socket->set_option(boost::asio::socket_base::reuse_address(true), ec);
            socket->set_option(boost::asio::socket_base::broadcast(true), ec);
            socket->set_option(boost::asio::ip::multicast::outbound_interface(local_address), ec);
            socket->set_option(boost::asio::ip::multicast::hops(2), ec);

            // Bound per interface so that the multicast and broadcast datagrams leave on the
            // interface whose subnet the printer is on.
            socket->bind({local_address, 0}, ec);
            if (ec)
                continue;

            socket->non_blocking(true, ec);
            if (!ec)
                sockets.push_back(std::move(socket));
        }

        if (sockets.empty()) {
            msg = _(L("Could not open a socket to search the local network."));
            return false;
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        auto       next_request = std::chrono::steady_clock::now();
        int        requests_sent = 0;

        while (std::chrono::steady_clock::now() < deadline) {
            if (requests_sent < SSDP_REQUEST_REPEATS && std::chrono::steady_clock::now() >= next_request) {
                for (auto& socket : sockets)
                    for (const auto& target : targets) {
                        boost::system::error_code ec;
                        socket->send_to(boost::asio::buffer(SSDP_MSEARCH_REQUEST), target, 0, ec);
                    }
                ++requests_sent;
                next_request = std::chrono::steady_clock::now() + SSDP_REQUEST_INTERVAL;
            }

            bool received_any = false;
            for (auto& socket : sockets) {
                std::vector<char>              buffer(2048);
                boost::asio::ip::udp::endpoint remote;
                boost::system::error_code      ec;

                const auto received = socket->receive_from(boost::asio::buffer(buffer), remote, 0, ec);
                if (ec || received == 0)
                    continue;

                received_any = true;
                MoonrakerDiscoveredPrinter printer;
                if (parse_ssdp_reply(std::string(buffer.data(), received), remote.address().to_string(), printer))
                    by_ip[printer.ip_address] = printer;
            }

            if (!received_any)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } catch (const std::exception& ex) {
        msg = wxString::FromUTF8(ex.what());
        return false;
    }

    for (const auto& [ip_address, printer] : by_ip)
        if (answers_moonraker_api(printer))
            printers.push_back(printer);

    // /server/info legitimately refuses a client outside the host's `trusted_clients`, so a
    // printer that announced itself is still offered; Test then reports the real error.
    if (printers.empty() && !by_ip.empty())
        for (const auto& [ip_address, printer] : by_ip)
            printers.push_back(printer);

    std::sort(printers.begin(), printers.end(), [](const MoonrakerDiscoveredPrinter& lhs, const MoonrakerDiscoveredPrinter& rhs) {
        if (lhs.machine_name != rhs.machine_name)
            return lhs.machine_name < rhs.machine_name;
        return lhs.ip_address < rhs.ip_address;
    });

    if (printers.empty()) {
        msg = _(L("No Moonraker printers were discovered on the local network."));
        return false;
    }

    return true;
}

wxString Moonraker::get_test_ok_msg() const
{
    return _(L("Connection to Moonraker is working correctly."));
}

wxString Moonraker::get_test_failed_msg(wxString &msg) const
{
    return GUI::format_wxstr("%s: %s", _L("Could not connect to Moonraker"), msg);
}

std::string Moonraker::make_url(const std::string &path) const
{
    if (m_host.find("http://") == 0 || m_host.find("https://") == 0) {
        if (m_host.back() == '/')
            return (boost::format("%1%%2%") % m_host % path).str();
        return (boost::format("%1%/%2%") % m_host % path).str();
    }
    return (boost::format("http://%1%/%2%") % m_host % path).str();
}

void Moonraker::set_auth(Http &http) const
{
    //ORCA: Moonraker accepts unauthenticated requests by default; X-Api-Key is the only auth header
    //      defined by the Moonraker spec. HTTP Basic / Digest do NOT belong here even if the user
    //      filled the user/password fields — those are PrusaLink/OctoPrint conventions.
    if (!m_apikey.empty())
        http.header("X-Api-Key", m_apikey);
    if (!m_cafile.empty())
        http.ca_file(m_cafile);
}

bool Moonraker::test(wxString &msg) const
{
    //ORCA: Moonraker's /server/info returns
    //          { "result": { "klippy_state": "ready|startup|shutdown|error|disconnected", ... } }
    //      We treat the connection as healthy as long as the envelope is valid and `klippy_state`
    //      is present — matching the OctoPrint/PrusaLink convention of "can I reach this host?".
    //      Klipper state (idle, error, etc.) is surfaced to the log but does not gate the test:
    //      buddy-fork firmwares legitimately report non-`ready` states at idle, and any real upload
    //      problem will surface a contextual error at upload() time anyway.
    const char *name = get_name();
    bool res = true;
    auto url = make_url("server/info");

    BOOST_LOG_TRIVIAL(info) << boost::format("%1%: Get server info at: %2%") % name % url;

    auto http = Http::get(std::move(url));
    set_auth(http);
    http.on_error([&](std::string body, std::string error, unsigned status) {
        BOOST_LOG_TRIVIAL(error) << boost::format("%1%: Error getting server info: %2%, HTTP %3%, body: `%4%`")
            % name % error % status % body;
        res = false;
        msg = format_error(body, error, status);
    })
    .on_complete([&](std::string body, unsigned) {
        BOOST_LOG_TRIVIAL(debug) << boost::format("%1%: /server/info body: %2%") % name % body;
        try {
            std::stringstream ss(body);
            pt::ptree ptree;
            pt::read_json(ss, ptree);

            const auto klippy_state = ptree.get_optional<std::string>("result.klippy_state");
            if (!klippy_state) {
                //ORCA: response wasn't shaped like a Moonraker /server/info reply — likely an OctoPrint
                //      or PrusaLink host the user mis-selected as Moonraker, or a totally different
                //      service. Treat as a connection failure with a clear hint.
                res = false;
                msg = _L("The host responded but it doesn't look like Moonraker (missing result.klippy_state).");
                return;
            }
            BOOST_LOG_TRIVIAL(info) << boost::format("%1%: klippy_state = %2%") % name % (*klippy_state);
        } catch (const std::exception &ex) {
            res = false;
            msg = GUI::format_wxstr(_L("Could not parse Moonraker server response: %s"), ex.what());
        }
    })
#ifdef WIN32
    .ssl_revoke_best_effort(m_ssl_revoke_best_effort)
#endif
    .perform_sync();

    return res;
}

bool Moonraker::get_storage(wxArrayString &storage_path, wxArrayString &storage_name) const
{
    //ORCA: GET /server/files/roots enumerates Moonraker's storage roots (default "gcodes" plus any
    //      configured extras like "config", "logs", "timelapse"). Only roots with permissions
    //      including "rw" or "rwd" can receive uploads; we filter to those so the UI dropdown only
    //      offers usable destinations. The base class returns false (no per-host storage); returning
    //      true here populates the storage picker in PrintHostDialogs's send-to-print dialog.
    //      Failures (404 — older Moonraker, or a buddy-fork that doesn't implement the endpoint)
    //      gracefully degrade to false so upload() falls back to the hardcoded "gcodes" default.
    const char *name = get_name();
    bool got_any = false;
    auto url = make_url("server/files/roots");

    BOOST_LOG_TRIVIAL(info) << boost::format("%1%: Enumerating storage roots at: %2%") % name % url;

    auto http = Http::get(std::move(url));
    set_auth(http);
    http.on_error([&](std::string body, std::string error, unsigned status) {
        //ORCA: /server/files/roots is optional in the Moonraker spec and absent on older versions
        //      and slimmer shims (e.g. Prusa-Firmware-Buddy 0.8.x prusalink-shim returns 501). A
        //      missing endpoint here is benign — upload() silently falls back to the hardcoded
        //      "gcodes" root — so don't pollute the log at warning level for it. Other HTTP
        //      errors still warn.
        if (status == 404 || status == 501) {
            BOOST_LOG_TRIVIAL(debug) << boost::format("%1%: /server/files/roots not implemented (HTTP %2%); upload() will fall back to the \"gcodes\" root.")
                % name % status;
        } else {
            BOOST_LOG_TRIVIAL(warning) << boost::format("%1%: Could not enumerate roots: %2%, HTTP %3%, body: `%4%`")
                % name % error % status % body;
        }
    })
    .on_complete([&](std::string body, unsigned) {
        BOOST_LOG_TRIVIAL(debug) << boost::format("%1%: /server/files/roots body: %2%") % name % body;
        try {
            std::stringstream ss(body);
            pt::ptree ptree;
            pt::read_json(ss, ptree);
            const auto result_node = ptree.get_child_optional("result");
            if (!result_node)
                return;
            for (const auto &child : *result_node) {
                const std::string &root = child.second.get<std::string>("name", "");
                const std::string &perms = child.second.get<std::string>("permissions", "");
                if (root.empty() || perms.find('w') == std::string::npos)
                    continue;
                storage_path.Add(wxString::FromUTF8(root));
                storage_name.Add(wxString::FromUTF8(root));
                got_any = true;
            }
        } catch (const std::exception &ex) {
            BOOST_LOG_TRIVIAL(warning) << boost::format("%1%: Could not parse roots: %2%") % name % ex.what();
        }
    })
#ifdef WIN32
    .ssl_revoke_best_effort(m_ssl_revoke_best_effort)
#endif
    .perform_sync();

    return got_any;
}

std::string Moonraker::make_start_print_body(const std::string &filename, bool skip_precheck, bool skip_foreign_detect)
{
    //ORCA: `filename` is what /server/files/upload returned as result.item.path (the storage-relative
    //      path inside `root`, no leading slash, with extension). Serializing through the JSON library
    //      keeps special characters escaped (a server-side collision suffix could produce paths with
    //      quotes / backslashes on exotic file systems).
    //
    //      Klipper forks that gate a job behind a pre-print check take two more flags here: Moonraker
    //      reads them with get_boolean(), which accepts a JSON bool, and resolves each absent one from
    //      the printer's own settings. So an unset flag is not the same as a false one, and both are
    //      left out unless asked for.
    nlohmann::json body{{"filename", filename}};
    if (skip_precheck)
        body["skip_precheck"] = true;
    if (skip_foreign_detect)
        body["skip_foreign_detect"] = true;
    return body.dump();
}

wxString Moonraker::explain_start_refusal(const std::string &body, unsigned status)
{
    //ORCA: a pre-print check refuses the start by raising a ServerError whose message is a bare token
    //      ("material_mismatch", "filament"), which Moonraker renders as the HTTP reason. The detail
    //      that says which channel is at fault is only published over the websocket, so on its own the
    //      HTTP error tells the user nothing they can act on. Translate the tokens we know about.
    if (status != 403 && status != 503)
        return {};

    const auto root = nlohmann::json::parse(body, nullptr, false);
    const auto err  = root.find("error");
    if (err == root.end())
        return {};
    const auto message = err->find("message");
    if (message == err->end() || !message->is_string())
        return {};
    const auto &reason = message->get_ref<const std::string &>();

    if (reason == "material_mismatch")
        return _L("The printer reports different filament loaded than this file was sliced for. Load the matching "
                  "material, or turn on \"Skip the printer's pre-print checks\" for this printer to print anyway.");
    if (reason == "filament")
        return _L("The printer's filament sensors report an empty channel. Load filament into every channel this "
                  "print uses, or turn on \"Skip the printer's pre-print checks\" for this printer to print anyway.");
    if (boost::algorithm::starts_with(reason, "AI Detection:"))
        return _L("The printer's camera check refused the job. Clear the build plate and make sure it is seated, or "
                  "turn on \"Skip the printer's AI object detection\" for this printer to print anyway.");
    return {};
}

bool Moonraker::start_print(wxString &error_msg, const std::string &filename) const
{
    //ORCA: POST /printer/print/start with the JSON body from make_start_print_body().
    const char *name = get_name();
    bool res = true;
    auto url = make_url("printer/print/start");
    const std::string body = make_start_print_body(filename, m_skip_precheck, m_skip_foreign_detect);

    BOOST_LOG_TRIVIAL(info) << boost::format("%1%: Starting print of %2% at %3% (body=%4%)") % name % filename % url % body;

    auto http = Http::post(std::move(url));
    set_auth(http);
    http.header("Content-Type", "application/json")
        .set_post_body(body)
        .on_complete([&](std::string body, unsigned status) {
            BOOST_LOG_TRIVIAL(debug) << boost::format("%1%: print/start HTTP %2%: %3%") % name % status % body;
        })
        .on_error([&](std::string body, std::string error, unsigned status) {
            BOOST_LOG_TRIVIAL(error) << boost::format("%1%: Error starting print at %2%: %3%, HTTP %4%, body: `%5%`")
                % name % url % error % status % body;
            res = false;
            error_msg = format_error(body, error, status);
            const wxString advice = explain_start_refusal(body, status);
            if (!advice.empty())
                error_msg += "\n\n" + advice;
        })
#ifdef WIN32
        .ssl_revoke_best_effort(m_ssl_revoke_best_effort)
#endif
        .perform_sync();

    return res;
}

bool Moonraker::upload(PrintHostUpload upload_data, ProgressFn progress_fn, ErrorFn error_fn, InfoFn info_fn) const
{
    //ORCA: POST /server/files/upload as multipart/form-data with:
    //          file = <gcode file>
    //          root = <storage root>     (Moonraker default: "gcodes")
    //      Successful response shape:
    //          { "result": { "item": { "path": "<name>.gcode", "root": "<root>" }, "print_started": <bool> } }
    //      We always start the print explicitly via /printer/print/start regardless of `print_started`
    //      so the user can rely on a single call site for state.
    wxString test_msg;
    if (!test(test_msg)) {
        error_fn(std::move(test_msg));
        return false;
    }

    const char *name = get_name();
    const auto upload_filename = upload_data.upload_path.filename();
    const auto upload_parent_path = upload_data.upload_path.parent_path();
    //ORCA: upload_data.storage is plumbed from the (future) per-printer storage dropdown. When unset,
    //      fall back to the Moonraker-standard "gcodes" root. Reading it through here means a UI
    //      addition later (storage picker) needs no change to this method.
    const std::string root = upload_data.storage.empty() ? std::string("gcodes") : upload_data.storage;

    std::string url = make_url("server/files/upload");
    bool result = true;
    std::string uploaded_path;

    //ORCA: gcode inside a .gcode.3mf is index-coded (Metadata/plate_<N>.gcode), so the upload names the
    //      plate via a 1-based `plateindex` (set only in the .3mf path, see Plater::send_gcode_legacy);
    //      servers that don't use it ignore the unknown form field.
    const std::string plateindex = upload_data.extended("plateindex");

    BOOST_LOG_TRIVIAL(info) << boost::format("%1%: Uploading file %2% to %3% (root=%4%, filename=%5%, plateindex=%6%, start_print=%7%)")
        % name
        % upload_data.source_path
        % url
        % root
        % upload_filename.string()
        % (plateindex.empty() ? "-" : plateindex)
        % (upload_data.post_action == PrintHostPostUploadAction::StartPrint ? "true" : "false");

    auto http = Http::post(std::move(url));
    set_auth(http);
    http.form_add("root", root);
    if (!plateindex.empty())
        http.form_add("plateindex", plateindex);
    http.form_add_file("file", upload_data.source_path.string(), upload_filename.string())
        .on_complete([&](std::string body, unsigned status) {
            BOOST_LOG_TRIVIAL(debug) << boost::format("%1%: upload HTTP %2%: %3%") % name % status % body;
            try {
                std::stringstream ss(body);
                pt::ptree ptree;
                pt::read_json(ss, ptree);

                //ORCA: Moonraker confirms the storage-relative path in result.item.path. We pass exactly
                //      that string to /printer/print/start so any server-side renaming (collision suffix,
                //      etc.) is respected.
                const auto stored_path = ptree.get_optional<std::string>("result.item.path");
                if (stored_path) {
                    uploaded_path = *stored_path;
                } else {
                    //ORCA: fallback if the server response omits result.item.path (older Moonraker, or
                    //      a buddy-fork that returns a slimmer envelope). Use the original filename.
                    uploaded_path = upload_filename.string();
                    BOOST_LOG_TRIVIAL(warning) << boost::format(
                        "%1%: upload response missing result.item.path, falling back to original filename `%2%`")
                        % name % uploaded_path;
                }
            } catch (const std::exception &ex) {
                BOOST_LOG_TRIVIAL(warning) << boost::format(
                    "%1%: could not parse upload response (%2%); falling back to original filename")
                    % name % ex.what();
                uploaded_path = upload_filename.string();
            }
        })
        .on_error([&](std::string body, std::string error, unsigned status) {
            BOOST_LOG_TRIVIAL(error) << boost::format("%1%: Error uploading to %2%: %3%, HTTP %4%, body: `%5%`")
                % name % url % error % status % body;
            error_fn(format_error(body, error, status));
            result = false;
        })
        .on_progress([&](Http::Progress progress, bool &cancel) {
            progress_fn(std::move(progress), cancel);
            if (cancel) {
                BOOST_LOG_TRIVIAL(info) << name << ": Upload canceled";
                result = false;
            }
        })
#ifdef WIN32
        .ssl_revoke_best_effort(m_ssl_revoke_best_effort)
#endif
        .perform_sync();

    if (!result)
        return false;

    if (upload_data.post_action == PrintHostPostUploadAction::StartPrint && !uploaded_path.empty()) {
        wxString start_msg;
        if (!start_print(start_msg, uploaded_path)) {
            error_fn(std::move(start_msg));
            return false;
        }
    }
    return true;
}

}
