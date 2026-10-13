// TETRA screens: the base station table (frequency, network code, location area, colour code, encryption, last heard), the details of
// the selected carrier, its neighbour cells, the carriers seen in the spectrum, a scan over a span, and the log. Only what base stations
// broadcast in the clear to every terminal is shown: nothing is decrypted, no voice, no short data, no subscriber identities.
#include "app.h"
#include "dect2/gsmr_rx.h"
#include "dect2/tetra_pdu.h"
#include "dect2/tetra_rx.h"
#include "dect2/tetra_tel.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

namespace {

struct State {
    int view = 0;                 // 0 cells, 1 neighbours, 2 carriers, 3 log
    int sel = -1;                 // selected cell (index in cells)
    int span = 0;                 // 0 390 - 395, 1 410 - 430, 2 450 - 470, 3 custom
    double lo = 380, hi = 400;    // custom span, MHz
    bool scanning = false;
    std::vector<double> plan;     // centres, MHz
    size_t step = 0;
    double stepAt = 0;
    float dwell = 6.0f;           // a capture is 2.3 s, and the search takes about as long again
};
State S;

const char* kHelp =
    "TETRA network information (ETSI EN 300 392-2), receive only.\n\n"
    "This mode reads only what a TETRA base station broadcasts in the clear to every terminal: the network code (MCC, MNC), colour code, "
    "location area, the cell's carriers and duplex spacing, its neighbour cells, the services it offers and whether the network uses air "
    "interface encryption. The same information is shown by open-source tools such as osmo-tetra.\n\n"
    "It does not decrypt anything, does not transmit, does not join or disturb any network, and does not decode or play voice, short "
    "data messages or subscriber identities. Encrypted signalling is only reported as encrypted.\n\n"
    "You are responsible for following your country's rules on receiving radio signals.";

bool live(const App& a) { return a.engine.running() && a.rx.standard == 47; }

std::string ago(int64_t t) {
    if (t <= 0) return "-";
    const long d = (long)(std::time(nullptr) - t);
    char b[32];
    if (d < 60) snprintf(b, sizeof b, "%ld s", std::max(0L, d)); else if (d < 3600) snprintf(b, sizeof b, "%ld min", d / 60); else snprintf(b, sizeof b, "%ld h", d / 3600);
    return b;
}

std::string netText(const dect2::TetraCell& c) {
    if (c.mcc < 0) return "-";
    char b[96];
    const char* country = c.mcc == 1 ? "test network" : dect2::gsmrCountry(c.mcc);
    if (*country) snprintf(b, sizeof b, "%d-%d %s", c.mcc, c.mnc, country); else snprintf(b, sizeof b, "%d-%d", c.mcc, c.mnc);
    return b;
}

void spanOf(double& lo, double& hi) {
    switch (S.span) {
    case 0: lo = 390; hi = 395; break;
    case 1: lo = 410; hi = 430; break;
    case 2: lo = 450; hi = 470; break;
    default: lo = std::min(S.lo, S.hi); hi = std::max(S.lo, S.hi); break;
    }
}

void makePlan(const App& a) {
    S.plan.clear();
    double lo, hi;
    spanOf(lo, hi);
    const double w = std::max(0.15, a.tune.sampleRate / 1e6 * 0.8);   // the part of the sample band searched whole
    for (double c = lo + w / 2; c - w / 2 < hi && S.plan.size() < 400; c += w) S.plan.push_back(std::round(c * 1000) / 1000);
}

void helpButton() {
    if (ImGui::SmallButton("?##tetrahelp")) ImGui::OpenPopup("##tetrahelppop");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("What this mode reads, and what it never does");
    if (ImGui::BeginPopup("##tetrahelppop")) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32);
        ImGui::TextUnformatted(kHelp);
        ImGui::PopTextWrapPos();
        ImGui::EndPopup();
    }
}

void scanBar(App& a) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("Span");
    flowNext(16 * gUi);
    static const char* spans[] = {"390 - 395 MHz (emergency services)", "410 - 430 MHz", "450 - 470 MHz", "Custom"};
    ImGui::SetNextItemWidth(220 * gUi);
    ImGui::Combo("##tspan", &S.span, spans, 4);
    if (S.span == 3) {
        flowNext(10 * gUi);
        ImGui::SetNextItemWidth(80 * gUi); ImGui::InputDouble("##tlo", &S.lo, 0, 0, "%.3f");
        ImGui::SameLine(); ImGui::TextDisabled("to"); ImGui::SameLine();
        ImGui::SetNextItemWidth(80 * gUi); ImGui::InputDouble("MHz##thi", &S.hi, 0, 0, "%.3f");
    }
    flowNext(10 * gUi);
    if (!S.scanning) {
        if (ImGui::SmallButton("Scan") && a.engine.running()) { makePlan(a); S.step = 0; S.stepAt = 0; S.scanning = !S.plan.empty(); }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Step the radio over the span; every TETRA carrier in each step is listed with its network information. Receive only.");
    } else {
        if (ImGui::SmallButton("Stop scan")) S.scanning = false;
        ImGui::SameLine(); ImGui::TextDisabled("%zu / %zu", S.step + 1, S.plan.size());
    }
    flowNext(10 * gUi);
    if (ImGui::SmallButton("Clear##tclr")) { a.engine.tetra().clearCells(); S.sel = -1; }
    flowNext(4 * gUi);
    helpButton();
    flowEnd();
}

void cellTable(App& a) {
    const dect2::TetraTelemetry& t = a.rx.tetra;
    const bool wide = ImGui::GetContentRegionAvail().x > 860 * gUi;
    const int nc = wide ? 9 : 6;
    const ImGuiTableFlags fl = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_BordersInnerV;
    if (!ImGui::BeginTable("##tcells", nc, fl, ImVec2(0, ImGui::GetContentRegionAvail().y))) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    if (wide) for (const char* h : {"MHz", "Network", "LA", "Colour", "Encryption", "Carrier", "Level", "SNR", "Last heard"}) ImGui::TableSetupColumn(h);
    else for (const char* h : {"MHz", "Network", "LA", "Colour", "Encryption", "Heard"}) ImGui::TableSetupColumn(h);
    ImGui::TableHeadersRow();
    for (size_t i = 0; i < t.cells.size(); i++) {
        const auto& c = t.cells[i];
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        char l[64]; snprintf(l, sizeof l, "%.4f##t%zu", c.nominalHz / 1e6, i);
        if (ImGui::Selectable(l, S.sel == (int)i, ImGuiSelectableFlags_SpanAllColumns)) S.sel = (int)i;
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s, colour code %d, offset %+.0f Hz, heard %s ago", netText(c).c_str(), c.colourCode, c.cfoHz, ago(c.lastWall).c_str());
        ImGui::TableNextColumn(); ImGui::TextUnformatted(netText(c).c_str());
        ImGui::TableNextColumn(); if (c.la >= 0) ImGui::Text("%d", c.la); else ImGui::TextDisabled("-");
        ImGui::TableNextColumn(); ImGui::Text("%d", c.colourCode);
        ImGui::TableNextColumn();
        if (c.encryption()) ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.35f, 1), "yes"); else if (c.hasSysinfo) ImGui::TextUnformatted("no"); else ImGui::TextDisabled("?");
        if (wide) { ImGui::TableNextColumn(); if (c.hasSysinfo) ImGui::TextUnformatted(c.isMain ? "main" : "other"); else ImGui::TextDisabled("-"); }
        if (wide) { ImGui::TableNextColumn(); ImGui::Text("%.0f", c.levelDb); }
        if (wide) { ImGui::TableNextColumn(); ImGui::Text("%.0f", c.snrDb); }
        ImGui::TableNextColumn(); ImGui::TextUnformatted(ago(c.lastWall).c_str());
    }
    ImGui::EndTable();
}

void detail(App& a) {
    ImGui::PushTextWrapPos(0);
    struct Pop { ~Pop() { ImGui::PopTextWrapPos(); } } pop_;
    const dect2::TetraTelemetry& t = a.rx.tetra;
    if (S.sel < 0 || S.sel >= (int)t.cells.size()) { ImGui::TextDisabled("select a carrier"); return; }
    const auto& c = t.cells[(size_t)S.sel];
    ImGui::Text("%.4f MHz, network %s, colour code %d, system code %d", c.nominalHz / 1e6, netText(c).c_str(), c.colourCode, c.systemCode);
    static const char* share[] = {"continuous transmission", "carrier sharing", "MCCH sharing", "traffic carrier sharing"};
    static const char* load[] = {"unknown", "low", "medium", "high"};
    ImGui::Text("%s, cell load %s, late entry %s, neighbour broadcast %s; last sync timeslot %d frame %d multiframe %d",
                share[c.sharingMode & 3], load[c.cellLoad & 3], c.lateEntry ? "yes" : "no", (c.nbrBroadcast & 2) ? "yes" : "no", c.tn, c.fn, c.mn);
    ImGui::Text("level %.1f dBFS, SNR %.1f dB, offset %+.0f Hz; %llu sync bursts, %llu SYSINFO, %llu blocks good, %llu failed",
                c.levelDb, c.snrDb, c.cfoHz, (unsigned long long)c.bschOk, (unsigned long long)c.bnchOk, (unsigned long long)c.blocksOk, (unsigned long long)c.blocksBad);
    if (c.hasSysinfo) {
        ImGui::Text("Location area %d; main carrier %d = %.4f MHz (band %d, offset %d)%s; uplink %s", c.la, c.mainCarrier, c.mainHz / 1e6, c.band, c.offset,
                    c.isMain ? ", this carrier" : "", c.uplinkHz > 0 ? (std::to_string(c.uplinkHz / 1e6).substr(0, 8) + " MHz").c_str() : "unknown");
        ImGui::Text("secondary control channels %d, max terminal power level %d, access level min %d, access parameter %d, radio downlink timeout %d",
                    c.numScch, c.msTxPwrMax, c.rxLevAccessMin, c.accessParam, c.dlTimeout);
        ImGui::Text("services: %s", dect2::tetra::serviceText(c.bsService).c_str());
        if (c.cckFlag) ImGui::Text("common cipher key identifier %d", c.hyperframeOrCck); else ImGui::Text("hyperframe %d", c.hyperframeOrCck);
        if (c.secValid) ImGui::Text("security: authentication %s, class 1 %s, class 2 %s, class 3 %s", c.authRequired ? "required" : "not required",
                                    c.secClass1 ? "yes" : "no", c.secClass2 ? "yes" : "no", c.secClass3 ? "yes" : "no");
    }
    ImGui::Text("encryption: %s%s", c.encryption() ? "yes" : (c.hasSysinfo ? "no" : "not known yet"),
                c.encryptedPdus ? (", " + std::to_string(c.encryptedPdus) + " encrypted signalling PDUs (not decrypted)").c_str() : "");
    std::string ts;
    for (int i = 0; i < 4; i++) { ts += "TS" + std::to_string(i + 1) + " " + dect2::tetra::dlUsageText(c.tsUsage[i]); if (i < 3) ts += ", "; }
    ImGui::Text("timeslots: %s", ts.c_str());
    if (!c.netTime.empty()) ImGui::Text("network time: %s", c.netTime.c_str());
}

void neighbourView(App& a) {
    const dect2::TetraTelemetry& t = a.rx.tetra;
    if (!ImGui::BeginChild("##tnbr")) { ImGui::EndChild(); return; }
    bool any = false;
    for (const auto& c : t.cells) {
        if (!c.hasNwrk) continue;
        any = true;
        char h[96]; snprintf(h, sizeof h, "%.4f MHz %s LA %d##n%.0f", c.nominalHz / 1e6, netText(c).c_str(), c.la, c.freqHz);
        if (!ImGui::TreeNode(h)) continue;
        for (const auto& n : c.neighbours) {
            if (n.freqHz > 0) ImGui::BulletText("cell %d: carrier %d (%.4f MHz), LA %d%s", n.cellId, n.carrier, n.freqHz / 1e6, n.la, n.synced ? ", synchronised" : "");
            else ImGui::BulletText("cell %d: carrier %d, LA %d", n.cellId, n.carrier, n.la);
        }
        if (c.neighbours.empty()) ImGui::TextDisabled("no neighbours listed");
        ImGui::TreePop();
    }
    if (!any) ImGui::TextDisabled("no neighbour cell broadcast read yet (D-NWRK-BROADCAST on the main control channel)");
    ImGui::EndChild();
}

void carrierView(App& a) {
    const dect2::TetraTelemetry& t = a.rx.tetra;
    if (t.carriers.empty()) { ImGui::TextDisabled("no carriers in the last block"); return; }
    for (const auto& c : t.carriers)
        ImGui::BulletText("%.4f MHz, %.0f dB over the floor, %s", c.freqHz / 1e6, c.levelDb, c.synced ? "TETRA synchronisation" : "no TETRA synchronisation in this block");
}

void logView(App& a) {
    for (const auto& e : a.rx.tetra.events) ImGui::TextUnformatted(e.text.c_str());
    if (a.rx.tetra.events.empty()) ImGui::TextDisabled("nothing yet");
}

void tick(App& a) {
    if (!S.scanning) return;
    if (!a.engine.running()) { S.scanning = false; return; }
    const double now = ImGui::GetTime();
    if (S.stepAt == 0 || now - S.stepAt >= S.dwell) {
        if (S.stepAt != 0) S.step++;
        if (S.step >= S.plan.size()) { S.scanning = false; return; }
        tuneFreq(a, S.plan[S.step]);
        S.stepAt = now;
    }
}

void tab(App& a) {
    scanBar(a);
    ImGui::PushTextWrapPos(0);
    ImGui::TextDisabled("Receive only: the public information every TETRA base station broadcasts to all terminals (network code, location area, services, encryption in use). Nothing is decrypted; no voice, messages or identities are decoded.");
    ImGui::PopTextWrapPos();
    // Make the distinction between RF detection, encryption and voice support explicit.
    // The current receiver processes public BCCH signalling only; it does not
    // follow traffic bearers or contain a TETRA ACELP voice decoder.
    if (live(a)) {
        int identified = 0, encrypted = 0, unencrypted = 0;
        for (const auto& cell : a.rx.tetra.cells) {
            if (!cell.hasSysinfo) continue;
            ++identified;
            if (cell.encryption()) ++encrypted;
            else ++unencrypted;
        }
        ImGui::Text("Identified cells: %d | Encryption indicated: %d | No encryption indicated: %d",
                    identified, encrypted, unencrypted);
        ImGui::TextDisabled("Voice: not implemented (control-channel scan only; no traffic or speech decoding).");
        if (encrypted > 0)
            ImGui::TextDisabled("Encrypted cells cannot be listened to as intelligible speech without authorised network access.");
    }
    subNav("tetrav", S.view, {"Cells", "Neighbours", "Carriers", "Log"});
    if (!live(a)) { ImGui::TextDisabled("%s", a.engine.running() ? "starting" : "start the receiver to find the TETRA base stations around you"); return; }
    if (S.view == 0) {
        const float h = ImGui::GetContentRegionAvail().y;
        if (ImGui::BeginChild("##tct", ImVec2(0, h * 0.55f))) cellTable(a);
        ImGui::EndChild();
        detail(a);
    } else if (S.view == 1) neighbourView(a);
    else if (S.view == 2) carrierView(a);
    else logView(a);
}

void list(App& a) {
    if (!live(a)) { ImGui::TextDisabled("%s", a.engine.running() ? "starting" : "stopped"); return; }
    for (const auto& c : a.rx.tetra.cells) ImGui::Text("%.4f %-10s %s", c.nominalHz / 1e6, netText(c).c_str(), c.encryption() ? "enc" : "");
    if (a.rx.tetra.cells.empty()) ImGui::TextDisabled("none yet");
}

void receiver(App& a) {
    if (!live(a)) { ImGui::TextDisabled("%s", a.engine.running() ? "starting" : "stopped"); return; }
    const dect2::TetraTelemetry& t = a.rx.tetra;
    ImGui::Text("%s", dect2::tetraSummary(t).c_str());
    ImGui::Text("%llu blocks searched, %llu signalling blocks decoded, %llu failed", (unsigned long long)t.blocks, (unsigned long long)t.blocksOk, (unsigned long long)t.blocksBad);
    ImGui::Text("input %.2f Msps, %.1f dBFS", t.inputRate / 1e6, t.levelDb);
    ImGui::TextDisabled("pi/4-DQPSK 18 ksym/s, 25 kHz channels; %.1f MHz of the band at a time: use Scan for a wider span", t.inputRate / 1e6 * 0.8);
}

void status(App& a) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("State"); ImGui::SameLine();
    ImGui::TextUnformatted(live(a) ? dect2::tetraSummary(a.rx.tetra).c_str() : (a.engine.running() ? "starting" : "stopped"));
}

void summary(const App& a, std::string& l1, std::string& l2) {
    l1 = "TETRA";
    if (live(a)) l2 = dect2::tetraSummary(a.rx.tetra);
}

} // namespace

extern const ModeUi kTetraUi;
const ModeUi kTetraUi = {
    .sideTitle = "CELLS",
    .tabName = "TETRA",
    .tabIcon = Ic::Radio,
    .tab = tab,
    .receiver = receiver,
    .list = list,
    .status = status,
    .summary = summary,
    .tick = tick,
};
