#include <catch2/catch_all.hpp>

#include <nlohmann/json.hpp>

#include "libslic3r/PrintConfig.hpp"
#include "slic3r/Utils/Moonraker.hpp"
#include "slic3r/Utils/PrintHost.hpp"

using namespace Slic3r;

namespace {

class TestPrintHost : public PrintHost
{
public:
    using PrintHost::format_error;

    const char* get_name() const override { return "Test"; }
    bool test(wxString&) const override { return true; }
    wxString get_test_ok_msg() const override { return {}; }
    wxString get_test_failed_msg(wxString&) const override { return {}; }
    bool upload(PrintHostUpload, ProgressFn, ErrorFn, InfoFn) const override { return true; }
    bool has_auto_discovery() const override { return false; }
    bool can_test() const override { return false; }
    PrintHostPostUploadActions get_post_upload_actions() const override { return {}; }
    std::string get_host() const override { return {}; }
};

std::string format_error(const std::string& body, const std::string& error, unsigned status)
{
    return TestPrintHost().format_error(body, error, status).ToStdString();
}

std::string envelope(int code, const std::string& message, const std::string& traceback)
{
    return nlohmann::json{{"error", {{"code", code}, {"message", message}, {"traceback", traceback}}}}.dump();
}

std::string moonraker_error(int code, const std::string& message, const std::string& detail = {})
{
    std::string line = "tornado.web.HTTPError: HTTP " + std::to_string(code) + ": " + message;
    if (!detail.empty())
        line += " (" + detail + ")";
    return envelope(code, message, "Traceback (most recent call last):\n  ...\n" + line + "\n");
}

// A real Moonraker body for uploading a file that is being printed.
constexpr const char* k_busy_file_403 =
    R"JSON({"error": {"code": 403, "message": "Forbidden", "traceback": "Traceback (most recent call last):\n\n  File \"/home/lava/moonraker/moonraker/components/file_manager/file_manager.py\", line 1017, in _finish_gcode_upload\n    can_start = self._handle_operation_check(check_path)\n                ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n\nmoonraker.utils.exceptions.ServerError: File currently in use\n\nDuring handling of the above exception, another exception occurred:\n\nTraceback (most recent call last):\n\n  File \"/home/lava/moonraker/moonraker/components/application.py\", line 1069, in post\n    raise tornado.web.HTTPError(\ntornado.web.HTTPError: HTTP 403: Forbidden (File is loaded, upload not permitted)\n"}})JSON";

// A real SSDP reply from a FibreSeeker3, which answers only a marked M-SEARCH.
constexpr const char* k_ssdp_reply =
    "HTTP/1.1 200 OK\r\n"
    "USN: uuid:00000000-0000-4000-8000-000000000001::upnp:rootdevice\r\n"
    "LOCATION: http://10.0.0.10:7125/server/zeroconf/ssdp\r\n"
    "ST: upnp:rootdevice\r\n"
    "EXT:\r\n"
    "SERVER: Moonraker SSDP/UPNP Server\r\n"
    "X-SPECIAL-MARKER: FIBRESEEK3D\r\n"
    "X-MACHINE-ID: TEST-MACHINE-00\r\n"
    "X-MACHINE-NAME: FibreSeeker 3\r\n"
    "CACHE-CONTROL: max-age=1800\r\n"
    "\r\n";

std::string without_line(const std::string& reply, const std::string& prefix)
{
    const auto start = reply.find(prefix);
    return reply.substr(0, start) + reply.substr(reply.find("\r\n", start) + 2);
}

} // namespace

TEST_CASE("A marked Moonraker SSDP reply yields the host and its API port", "[PrintHost][Moonraker]")
{
    MoonrakerDiscoveredPrinter printer;
    REQUIRE(Moonraker::parse_ssdp_reply(k_ssdp_reply, "10.0.0.10", printer));

    CHECK(printer.ip_address == "10.0.0.10");
    CHECK(printer.port == 7125);
    CHECK(printer.machine_name == "FibreSeeker 3");
    CHECK(printer.machine_id == "TEST-MACHINE-00");
}

TEST_CASE("An SSDP reply from Moonraker is accepted without the probe marker", "[PrintHost][Moonraker]")
{
    MoonrakerDiscoveredPrinter printer;
    REQUIRE(Moonraker::parse_ssdp_reply(without_line(k_ssdp_reply, "X-SPECIAL-MARKER"), "10.0.0.10", printer));
    CHECK(printer.ip_address == "10.0.0.10");
}

TEST_CASE("An SSDP reply from something other than Moonraker is ignored", "[PrintHost][Moonraker]")
{
    const std::string reply = "HTTP/1.1 200 OK\r\n"
                              "LOCATION: http://192.168.10.9:80/description.xml\r\n"
                              "SERVER: Linux/4.9 UPnP/1.0 SomeRouter/1.0\r\n"
                              "X-SPECIAL-MARKER: FIBRESEEK3D\r\n"
                              "\r\n";

    MoonrakerDiscoveredPrinter printer;
    CHECK_FALSE(Moonraker::parse_ssdp_reply(reply, "192.168.10.9", printer));
}

TEST_CASE("Moonraker Device tab opens the printer UI, not the API port", "[PrintHost][Moonraker]")
{
    CHECK(Moonraker::default_webui_url("http://192.168.10.19:7125") == "http://192.168.10.19");
    CHECK(Moonraker::default_webui_url("192.168.10.19:7125") == "http://192.168.10.19");
    CHECK(Moonraker::default_webui_url("http://192.168.10.19:7125/") == "http://192.168.10.19");
    CHECK(Moonraker::default_webui_url("http://192.168.10.19") == "http://192.168.10.19");
    CHECK(Moonraker::default_webui_url("http://192.168.10.19:80") == "http://192.168.10.19:80");
    CHECK(Moonraker::default_webui_url("") == "");

    DynamicPrintConfig cfg;
    cfg.set_key_value("host_type", new ConfigOptionEnum<PrintHostType>(htMoonraker));
    cfg.set_key_value("print_host", new ConfigOptionString("http://192.168.10.19:7125"));
    cfg.set_key_value("print_host_webui", new ConfigOptionString(""));
    CHECK(PrintHost::get_print_host_webui(&cfg) == "http://192.168.10.19");

    cfg.set_key_value("print_host_webui", new ConfigOptionString("http://192.168.10.19:4408"));
    CHECK(PrintHost::get_print_host_webui(&cfg) == "http://192.168.10.19:4408");
}

TEST_CASE("A Moonraker host that announces no port falls back to the default", "[PrintHost][Moonraker]")
{
    const std::string reply = without_line(k_ssdp_reply, "LOCATION");

    MoonrakerDiscoveredPrinter printer;
    REQUIRE(Moonraker::parse_ssdp_reply(reply, "10.0.0.10", printer));
    CHECK(printer.port == 7125);
}

// Hidden: needs a Moonraker printer powered on the same LAN, so it cannot run in CI. Run it by
// name to check the SSDP scan itself against real hardware.
TEST_CASE("Moonraker printers are discovered on the local network", "[.][PrintHost][Moonraker]")
{
    std::vector<MoonrakerDiscoveredPrinter> printers;
    wxString                                msg;
    REQUIRE(Moonraker::discover_printers(printers, msg));

    for (const auto& printer : printers)
        WARN("found " << printer.machine_name << " at " << printer.ip_address << ":" << printer.port);

    CHECK(printers.front().port > 0);
}

TEST_CASE("A Klipper upload error shows its reason instead of a Python traceback", "[PrintHost][Regression]")
{
    const std::string msg = format_error(k_busy_file_403, "", 403);
    INFO("actual: " << msg);

    CHECK(msg == "HTTP 403: Forbidden (File is loaded, upload not permitted)");
    CHECK_THAT(msg, !Catch::Matchers::ContainsSubstring("Traceback"));
    CHECK_THAT(msg, !Catch::Matchers::ContainsSubstring("file_manager.py"));
}

TEST_CASE("The specific cause is recovered from a file endpoint's traceback", "[PrintHost]")
{
    SECTION("a plain detail")
    {
        const std::string body = moonraker_error(403, "Forbidden", "File is loaded, upload not permitted");
        CHECK(format_error(body, "", 403) == "HTTP 403: Forbidden (File is loaded, upload not permitted)");
    }

    SECTION("a detail whose own parentheses nest (a filename)")
    {
        const std::string detail = "Directory does not exist (/home/pi/gcodes/plate (1).gcode)";
        const std::string body   = moonraker_error(400, "Bad Request", detail);
        CHECK(format_error(body, "", 400) == "HTTP 400: Bad Request (" + detail + ")");
    }

    SECTION("a detail that contains the reason phrase")
    {
        const std::string body = moonraker_error(403, "Forbidden", "Forbidden zone: access denied");
        CHECK(format_error(body, "", 403) == "HTTP 403: Forbidden (Forbidden zone: access denied)");
    }

    SECTION("a detail that spans lines")
    {
        const std::string detail = "Move out of range\nX=250.000 Y=10.000";
        const std::string body   = moonraker_error(400, "Bad Request", detail);
        CHECK(format_error(body, "", 400) == "HTTP 400: Bad Request (" + detail + ")");
    }

    SECTION("a detail that only repeats the reason phrase is dropped")
    {
        const std::string body = moonraker_error(401, "Unauthorized", "Unauthorized");
        CHECK(format_error(body, "", 401) == "HTTP 401: Unauthorized");
    }
}

TEST_CASE("An unhandled exception shows its type and message", "[PrintHost]")
{
    const std::string frame = "Traceback (most recent call last):\n"
                              "  File \"/home/pi/moonraker/moonraker/components/file_manager/file_manager.py\", line 1, in write\n"
                              "    self._write(data)\n";

    SECTION("a one-line message")
    {
        const std::string body = envelope(500, "Internal Server Error", frame + "OSError: [Errno 28] No space left on device\n");
        CHECK(format_error(body, "", 500) == "HTTP 500: Internal Server Error (OSError: [Errno 28] No space left on device)");
    }

    SECTION("a message that spans lines")
    {
        const std::string body = envelope(500, "Internal Server Error", frame + "ServerError: Klippy request failed\n  see klippy.log\n");
        CHECK(format_error(body, "", 500) == "HTTP 500: Internal Server Error (ServerError: Klippy request failed\n  see klippy.log)");
    }

    SECTION("raised while handling an HTTPError with the same code")
    {
        const std::string traceback = frame + "tornado.web.HTTPError: HTTP 500: Internal Server Error (Database locked)\n\n"
                                              "During handling of the above exception, another exception occurred:\n\n" +
                                      frame + "OSError: [Errno 5] Input/output error\n";
        const std::string body      = envelope(500, "Internal Server Error", traceback);
        CHECK(format_error(body, "", 500) == "HTTP 500: Internal Server Error (OSError: [Errno 5] Input/output error)");
    }

    SECTION("a traceback with no header")
    {
        const std::string body = envelope(500, "Internal Server Error", "OSError: [Errno 5] Input/output error");
        CHECK(format_error(body, "", 500) == "HTTP 500: Internal Server Error (OSError: [Errno 5] Input/output error)");
    }
}

TEST_CASE("A reason already complete in message is shown unchanged", "[PrintHost]")
{
    SECTION("message is the whole reason, no trailing detail")
    {
        const std::string body = moonraker_error(503, "Klippy is not ready");
        CHECK(format_error(body, "", 503) == "HTTP 503: Klippy is not ready");
    }

    SECTION("a message that itself contains parentheses is not duplicated")
    {
        const std::string reason = "Requested blocks (0-5) are unavailable";
        const std::string body   = moonraker_error(400, reason);
        CHECK(format_error(body, "", 400) == "HTTP 400: " + reason);
    }
}

TEST_CASE("A Moonraker error with no usable detail shows just the reason phrase", "[PrintHost]")
{
    struct Case
    {
        const char* name;
        const char* body;
        unsigned status;
        const char* expected;
    };

    const auto c = GENERATE(
        Case{"an empty traceback", R"JSON({"error": {"code": 500, "message": "Internal Server Error", "traceback": ""}})JSON", 500,
             "HTTP 500: Internal Server Error"},
        Case{"a traceback of only whitespace", R"JSON({"error": {"code": 500, "message": "Internal Server Error", "traceback": "\n \n"}})JSON",
             500, "HTTP 500: Internal Server Error"});

    DYNAMIC_SECTION(c.name) { CHECK(format_error(c.body, "", c.status) == c.expected); }
}

TEST_CASE("A percent sign in the reason is not a format specifier", "[PrintHost]")
{
    const std::string body = moonraker_error(507, "Insufficient Storage", "disk 100% full");
    CHECK(format_error(body, "", 507) == "HTTP 507: Insufficient Storage (disk 100% full)");
}

TEST_CASE("Error bodies that are not a Moonraker envelope are left unchanged", "[PrintHost]")
{
    SECTION("OctoPrint's string-valued error member")
    {
        const std::string body = R"JSON({"error": "File not found"})JSON";
        CHECK(format_error(body, "", 404) == "HTTP 404: " + body);
    }

    SECTION("PrusaLink's top-level message, not under error")
    {
        const std::string body = R"JSON({"title": "Conflict", "message": "Printer is printing"})JSON";
        CHECK(format_error(body, "", 409) == "HTTP 409: " + body);
    }

    SECTION("a body that is not JSON")
    {
        const std::string html = "<html><head><title>502 Bad Gateway</title></head></html>";
        CHECK(format_error(html, "", 502) == "HTTP 502: " + html);
    }

    SECTION("an error object with no traceback")
    {
        const std::string body = R"JSON({"error": {"code": 500, "message": "Internal Server Error"}})JSON";
        CHECK(format_error(body, "", 500) == "HTTP 500: " + body);
    }

    SECTION("an error object whose traceback is null")
    {
        const std::string body = R"JSON({"error": {"code": 500, "message": "Internal Server Error", "traceback": null}})JSON";
        CHECK(format_error(body, "", 500) == "HTTP 500: " + body);
    }

    SECTION("an envelope whose reason phrase is empty")
    {
        const std::string body = envelope(403, "", "Traceback (most recent call last):\nOSError: denied\n");
        CHECK(format_error(body, "", 403) == "HTTP 403: " + body);
    }

    SECTION("a transport error with no HTTP status")
    {
        CHECK(format_error("", "curl:Could not connect", 0) == "curl:Could not connect");
    }
}

TEST_CASE("The print start body names the file and nothing else by default", "[PrintHost][Moonraker]")
{
    const auto body = nlohmann::json::parse(Moonraker::make_start_print_body("subdir/part.gcode", false, false));

    CHECK(body.at("filename") == "subdir/part.gcode");
    // Moonraker resolves an absent flag from the printer's own settings, which is not what sending
    // false does, so a host that was not asked to skip anything must see no flags at all.
    CHECK(body.size() == 1);
}

TEST_CASE("A pre-print check is skipped only when the printer is asked to", "[PrintHost][Moonraker]")
{
    struct Case
    {
        const char* name;
        bool        skip_precheck;
        bool        skip_foreign_detect;
    };

    const auto c = GENERATE(Case{"neither", false, false}, Case{"the whole precheck", true, false},
                            Case{"only the object detection", false, true}, Case{"both", true, true});

    DYNAMIC_SECTION(c.name)
    {
        const auto body = nlohmann::json::parse(
            Moonraker::make_start_print_body("part.gcode", c.skip_precheck, c.skip_foreign_detect));

        CHECK(body.contains("skip_precheck") == c.skip_precheck);
        CHECK(body.contains("skip_foreign_detect") == c.skip_foreign_detect);
        if (c.skip_precheck)
            CHECK(body.at("skip_precheck") == true);
        if (c.skip_foreign_detect)
            CHECK(body.at("skip_foreign_detect") == true);
    }
}

TEST_CASE("A file name with JSON punctuation reaches the printer intact", "[PrintHost][Moonraker]")
{
    const std::string filename = R"(sub\dir/a "quoted" part.gcode)";

    const auto body = nlohmann::json::parse(Moonraker::make_start_print_body(filename, false, false));
    CHECK(body.at("filename") == filename);
}

TEST_CASE("A refused pre-print check says what to fix", "[PrintHost][Moonraker]")
{
    const wxString material = Moonraker::explain_start_refusal(moonraker_error(403, "material_mismatch"), 403);
    const wxString filament = Moonraker::explain_start_refusal(moonraker_error(403, "filament"), 403);
    const wxString camera =
        Moonraker::explain_start_refusal(moonraker_error(403, "AI Detection: Build plate check failed"), 403);

    CHECK_FALSE(material.IsEmpty());
    CHECK_FALSE(filament.IsEmpty());
    CHECK_FALSE(camera.IsEmpty());
    // Each of the three has its own remedy, so advice that did not tell them apart would be no advice.
    CHECK(material != filament);
    CHECK(material != camera);
    CHECK(filament != camera);
}

TEST_CASE("Only a refusal we recognize is explained", "[PrintHost][Moonraker]")
{
    SECTION("a 403 from somewhere other than the precheck")
    {
        CHECK(Moonraker::explain_start_refusal(k_busy_file_403, 403).IsEmpty());
    }

    SECTION("the printer being in the wrong state")
    {
        const std::string body = moonraker_error(409, "Printer state 'printing' does not allow start printing");
        CHECK(Moonraker::explain_start_refusal(body, 409).IsEmpty());
    }

    SECTION("a body that is not JSON") { CHECK(Moonraker::explain_start_refusal("<html>403</html>", 403).IsEmpty()); }

    SECTION("no body at all") { CHECK(Moonraker::explain_start_refusal("", 403).IsEmpty()); }
}
