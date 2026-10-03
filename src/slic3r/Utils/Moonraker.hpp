#ifndef slic3r_Moonraker_hpp_
#define slic3r_Moonraker_hpp_

#include <string>
#include <vector>
#include <wx/string.h>
#include <wx/arrstr.h>

#include "PrintHost.hpp"
#include "libslic3r/PrintConfig.hpp"


namespace Slic3r {

class DynamicPrintConfig;
class Http;

// Moonraker is the JSON / WebSocket gateway that ships in front of Klipper
// (and on Klipper-API-compatible firmwares like the Prusa-Firmware-Buddy
// Buddy-Klipper fork). REST shape differs from OctoPrint: distinct paths,
// JSON body for print/start, {"result":...}/{"error":...} envelope.
//
// Endpoints used:
//   GET  /server/info                      -- connection test, reads klippy_state
//   POST /server/files/upload (multipart)  -- upload gcode (form fields: file, root)
//   POST /printer/print/start (json)       -- {"filename":"<name>.gcode"} starts print,
//                                             plus the optional skip_precheck /
//                                             skip_foreign_detect flags described on
//                                             make_start_print_body()
//
// Auth: X-Api-Key header if `printhost_apikey` is non-empty; Moonraker accepts
// unauthenticated LAN access by default, so the key is optional. HTTP Basic /
// Digest are not part of the Moonraker spec and are not sent.

// A Moonraker host found on the LAN by Moonraker::discover_printers().
struct MoonrakerDiscoveredPrinter
{
    std::string ip_address;
    int         port {0};
    std::string machine_name;
    std::string machine_id;
};

class Moonraker : public PrintHost
{
public:
    Moonraker(DynamicPrintConfig *config);
    ~Moonraker() override = default;

    const char* get_name() const override;

    bool test(wxString &curl_msg) const override;
    wxString get_test_ok_msg() const override;
    wxString get_test_failed_msg(wxString &msg) const override;
    bool upload(PrintHostUpload upload_data, ProgressFn progress_fn, ErrorFn error_fn, InfoFn info_fn) const override;
    bool has_auto_discovery() const override { return true; }
    bool can_test() const override { return true; }

    // Scans the LAN for Moonraker hosts over SSDP. Static because discovery runs before a host
    // is configured. Returns false with `msg` set when nothing answered.
    static bool discover_printers(std::vector<MoonrakerDiscoveredPrinter>& printers, wxString& msg, int timeout_ms = 6000);
    // Parses one SSDP reply, rejecting anything that is not a marked Moonraker answer. Public so
    // that the discovery handshake can be exercised without a printer on the network.
    static bool parse_ssdp_reply(const std::string& reply, const std::string& ip_address, MoonrakerDiscoveredPrinter& printer);
    // Hostname for the Device tab and the system browser: the printer's own web UI
    // (Mainsail / Fluidd / nginx, typically port 80), not Moonraker's API root on 7125.
    // An explicit print_host_webui is left alone; this only rewrites a bare API host.
    static std::string default_webui_url(const std::string& host);
    // Builds the /printer/print/start body. Each flag is omitted when false, so a host that knows
    // nothing about them receives the plain { "filename": ... } request and the ones that do keep
    // using their own configured defaults. Public so the body shape can be checked without a printer.
    static std::string make_start_print_body(const std::string& filename, bool skip_precheck, bool skip_foreign_detect);
    // Advice for a print/start that a pre-print check refused, or empty when the refusal is not one
    // we recognize. Public for the same reason.
    static wxString explain_start_refusal(const std::string& body, unsigned status);
    PrintHostPostUploadActions get_post_upload_actions() const override { return PrintHostPostUploadAction::StartPrint; }
    std::string get_host() const override { return m_host; }
    bool get_storage(wxArrayString &storage_path, wxArrayString &storage_name) const override;
    const std::string& get_apikey() const { return m_apikey; }
    const std::string& get_cafile() const { return m_cafile; }

protected:
    std::string m_host;
    std::string m_apikey;
    std::string m_cafile;
    bool        m_ssl_revoke_best_effort;
    bool        m_skip_precheck;
    bool        m_skip_foreign_detect;

    void set_auth(Http &http) const;
    std::string make_url(const std::string &path) const;
    bool start_print(wxString &error_msg, const std::string &filename) const;
};

}

#endif
