// General receiver screens: the listening frequency with a step selector, the demodulator (NFM, WFM, AM, USB, LSB, CW), filter width
// presets, squelch (auto or level), AGC or a fixed gain, de-emphasis, CW pitch, the shared volume, an S-meter and recording to WAV,
// over the spectrum and the waterfall. On those plots the channel is a marker: a click puts it there (inside the band the radio
// covers, without retuning; outside it the radio is retuned), its filter edges can be dragged.
#include "app.h"
#include "plot.h"
#include "dect2/general_gen.h"
#include "dect2/general_rx.h"
#include "dect2/general_tel.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

namespace {

const double kSteps[] = {10, 100, 1000, 2500, 5000, 6250, 8333.333333, 9000, 10000, 12500, 25000, 50000, 100000};
const char* const kStepNames[] = {"10 Hz", "100 Hz", "1 kHz", "2.5 kHz", "5 kHz", "6.25 kHz", "8.33 kHz", "9 kHz", "10 kHz", "12.5 kHz", "25 kHz", "50 kHz", "100 kHz"};
constexpr int kNumSteps = (int)(sizeof kSteps / sizeof *kSteps);
const double kDeemph[] = {0, 50, 75, 750};
const char* const kDeemphNames[] = {"off", "50 us", "75 us", "750 us"};

struct State {
    bool loaded = false;
    int demod = dect2::GenNfm;
    double width[dect2::GenDemodCount] = {};
    int step[dect2::GenDemodCount] = {};
    bool sqOn = true, sqAuto = true;
    float sqDb = -60;
    bool agc = true;
    float gainDb = 20;
    int deNfm = 0, deWfm = 1;
    float pitch = 700;
    double chanOff = 0;          // the channel relative to the dial frequency (a.freqMhz)
    double lastDial = -1;        // a new dial frequency typed elsewhere puts the channel back on it
    int drag = -1, dragPlot = -1;   // the filter edge held (0 low, 1 high) and on which plot
    bool dragged = false;
    std::string recErr;
};
State S;

void tip(const char* t) { if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", t); }
bool live(const App& a) { return a.engine.running() && a.rx.standard == 56; }   // the engine reports its standard code minus one
const dect2::GeneralTelemetry& T(const App& a) { return a.rx.general; }

const int kDefStep[dect2::GenDemodCount] = {9, 11, 2, 1, 1, 1};   // 12.5 kHz NFM, 50 kHz WFM, 1 kHz AM, 100 Hz SSB / CW

void load() {
    if (S.loaded) return;
    S.loaded = true;
    auto& p = plat::prefs();
    S.demod = (int)std::clamp(p.getI("genDemod", dect2::GenNfm), 0L, (long)dect2::GenDemodCount - 1);
    for (int d = 0; d < dect2::GenDemodCount; d++) {
        char k[24];
        snprintf(k, sizeof k, "genWidth%d", d); S.width[d] = dect2::generalClampWidth(d, p.getD(k, dect2::generalDefaultWidth(d)));
        snprintf(k, sizeof k, "genStep%d", d); S.step[d] = (int)std::clamp(p.getI(k, kDefStep[d]), 0L, (long)kNumSteps - 1);
    }
    S.sqOn = p.getB("genSqOn", true); S.sqAuto = p.getB("genSqAuto", true); S.sqDb = (float)p.getD("genSqDb", -60);
    S.agc = p.getB("genAgc", true); S.gainDb = (float)p.getD("genGain", 20);
    S.deNfm = (int)std::clamp(p.getI("genDeNfm", 0), 0L, 3L); S.deWfm = (int)std::clamp(p.getI("genDeWfm", 1), 0L, 3L);
    S.pitch = (float)std::clamp(p.getD("genPitch", 700), 200.0, 1500.0);
    S.chanOff = p.getD("genOffset", 0);
}
void save() {
    auto& p = plat::prefs();
    p.setI("genDemod", S.demod);
    for (int d = 0; d < dect2::GenDemodCount; d++) {
        char k[24];
        snprintf(k, sizeof k, "genWidth%d", d); p.setD(k, S.width[d]);
        snprintf(k, sizeof k, "genStep%d", d); p.setI(k, S.step[d]);
    }
    p.setB("genSqOn", S.sqOn); p.setB("genSqAuto", S.sqAuto); p.setD("genSqDb", S.sqDb);
    p.setB("genAgc", S.agc); p.setD("genGain", S.gainDb);
    p.setI("genDeNfm", S.deNfm); p.setI("genDeWfm", S.deWfm); p.setD("genPitch", S.pitch); p.setD("genOffset", S.chanOff);
}

double listenHz(const App& a) { return a.freqMhz * 1e6 + S.chanOff; }
double stepHz() { return kSteps[S.step[S.demod]]; }
void edges(double& lo, double& hi) { dect2::generalFilterEdges(S.demod, S.width[S.demod], lo, hi); }

// Put the channel on hz: inside the band the radio covers it only moves the marker; outside, the radio is retuned onto it
void setListen(App& a, double hz) {
    hz = std::max(1e3, hz);
    const double fs = a.engine.sampleRate() > 0 ? a.engine.sampleRate() : a.tune.sampleRate;
    const double rc = radioCenterMhz(a) * 1e6;
    double lo, hi;
    edges(lo, hi);
    if (hz + lo < rc - 0.45 * fs || hz + hi > rc + 0.45 * fs) {
        S.chanOff = 0;
        tuneFreq(a, hz / 1e6);
        S.lastDial = a.freqMhz;
    } else S.chanOff = hz - a.freqMhz * 1e6;
    save();
}
double snap(double hz) { const double s = stepHz(); return std::round(hz / s) * s; }

void setDemod(int d) {
    if (d == S.demod) return;
    S.demod = d;
    save();
}

void startRecording(App& a) {
    std::error_code ec;
    const std::string dir = plat::dataDir() + "/audio-recordings";
    std::filesystem::create_directories(dir, ec);
    const std::time_t now = std::time(nullptr);
    char ts[32];
    std::strftime(ts, sizeof ts, "%Y%m%d-%H%M%S", std::localtime(&now));
    char name[128];
    snprintf(name, sizeof name, "/onair_%s_%.4fMHz_%s.wav", dect2::generalDemodName(S.demod), listenHz(a) / 1e6, ts);
    if (!a.engine.general().startRecording(dir + name)) S.recErr = "cannot write " + dir + name;
    else S.recErr.clear();
}

// the S-meter: a bar from -130 to 0 dBFS with the squelch threshold marked
void sMeter(const App& a) {
    const auto& t = T(a);
    const bool on = live(a) && t.levelDb > -199;
    const float w = std::min(360 * gUi, ImGui::GetContentRegionAvail().x), h = ImGui::GetFrameHeight() * 0.7f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(26, 30, 36, 255), 3.f);
    auto xOf = [&](float db) { return p.x + w * std::clamp((db + 130.f) / 130.f, 0.f, 1.f); };
    if (on) {
        const ImU32 col = t.squelchOpen ? IM_COL32(90, 200, 120, 255) : IM_COL32(110, 130, 150, 255);
        dl->AddRectFilled(p, ImVec2(xOf(t.levelDb), p.y + h), col, 3.f);
        if (t.noiseDb > -199) dl->AddLine(ImVec2(xOf(t.noiseDb), p.y), ImVec2(xOf(t.noiseDb), p.y + h), IM_COL32(200, 200, 200, 140), 1.f);
        if (t.squelchDb > -199) dl->AddLine(ImVec2(xOf(t.squelchDb), p.y - 2), ImVec2(xOf(t.squelchDb), p.y + h + 2), IM_COL32(240, 170, 60, 255), 2.f);
    }
    for (int db = -120; db <= -10; db += 10) dl->AddLine(ImVec2(xOf((float)db), p.y + h * 0.65f), ImVec2(xOf((float)db), p.y + h), IM_COL32(0, 0, 0, 120), 1.f);
    ImGui::Dummy(ImVec2(w, h));
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Channel level, -130 to 0 dBFS (after the channel filter).\nGrey line: the noise in the channel; amber: the squelch threshold.");
    ImGui::SameLine(0, 8 * gUi);
    if (on) {
        if (t.dataValid) ImGui::Text("%.1f dBFS  SNR %.1f dB", t.levelDb, t.snrDb);
        else ImGui::Text("%.1f dBFS", t.levelDb);
        ImGui::SameLine(0, 8 * gUi);
        ImGui::TextColored(t.squelchOpen ? ImVec4(0.45f, 0.85f, 0.55f, 1) : ImVec4(0.6f, 0.6f, 0.6f, 1), "%s", t.squelchOpen ? "open" : "closed");
    } else ImGui::TextDisabled("%s", a.engine.running() ? "no level yet" : "stopped");
}

void controls(App& a) {
    load();
    // the listening frequency and the step
    ImGui::PushFont(a.mono, 0);
    ImGui::Text("%.4f MHz", listenHz(a) / 1e6);
    ImGui::PopFont();
    tip("The frequency listened to: the dial plus where the channel marker sits.\nClick the spectrum or the waterfall to move it; drag the edges of the light band to change the filter.");
    flowNext(10 * gUi);
    if (ImGui::ArrowButton("##gdn", ImGuiDir_Left)) setListen(a, snap(listenHz(a)) - stepHz());
    tip("Down one step");
    ImGui::SameLine(0, 2 * gUi);
    if (ImGui::ArrowButton("##gup", ImGuiDir_Right)) setListen(a, snap(listenHz(a)) + stepHz());
    tip("Up one step");
    ImGui::SameLine(0, 4 * gUi);
    ImGui::SetNextItemWidth(95 * gUi);
    if (ImGui::Combo("##gstep", &S.step[S.demod], kStepNames, kNumSteps)) save();
    tip("Frequency step: the arrows move by it and a click on the spectrum snaps to it");
    flowNext(6 * gUi);
    if (ImGui::SmallButton("Centre")) { S.chanOff = 0; tuneFreq(a, snap(listenHz(a)) / 1e6); S.lastDial = a.freqMhz; save(); }
    tip("Tune the radio so the channel sits on the dial (the marker back in the middle)");
    flowEnd();

    // the demodulator
    for (int d = 0; d < dect2::GenDemodCount; d++) {
        if (d) ImGui::SameLine();
        if (ImGui::RadioButton(dect2::generalDemodName(d), S.demod == d)) setDemod(d);
    }
    // filter width presets
    ImGui::SameLine(0, 16 * gUi);
    ImGui::TextDisabled("filter");
    int np = 0;
    const double* pr = dect2::generalWidthPresets(S.demod, np);
    for (int i = 0; i < np; i++) {
        ImGui::SameLine(0, 4 * gUi);
        char b[32];
        snprintf(b, sizeof b, "%.4g k##gw%d", pr[i] / 1e3, i);
        const bool sel = std::fabs(S.width[S.demod] - pr[i]) < 1;
        if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::SmallButton(b)) { S.width[S.demod] = pr[i]; save(); }
        if (sel) ImGui::PopStyleColor();
    }
    ImGui::SameLine(0, 8 * gUi);
    ImGui::Text("%.4g kHz", S.width[S.demod] / 1e3);

    // squelch, AGC, de-emphasis, pitch
    if (ImGui::Checkbox("Squelch", &S.sqOn)) save();
    tip("Mute the noise between transmissions");
    flowNext(6 * gUi);
    ImGui::BeginDisabled(!S.sqOn);
    ImGui::SetNextItemWidth(80 * gUi);
    int sm = S.sqAuto ? 0 : 1;
    if (ImGui::Combo("##gsqm", &sm, "Auto\0Level\0")) { S.sqAuto = sm == 0; save(); }
    tip("Auto: opens 6 dB over the noise measured beside the channel. Level: opens at the threshold in dBFS.");
    if (!S.sqAuto) {
        ImGui::SameLine(0, 4 * gUi);
        ImGui::SetNextItemWidth(140 * gUi);
        ImGui::SliderFloat("##gsql", &S.sqDb, -130, 0, "%.0f dBFS");
        if (ImGui::IsItemDeactivatedAfterEdit()) save();
        ImGui::SameLine(0, 4 * gUi);
        if (ImGui::SmallButton("Set") && live(a) && T(a).levelDb > -199) { S.sqDb = std::round(T(a).levelDb + 3); save(); }
        tip("Threshold 3 dB over the level now (set it while no one transmits)");
    }
    ImGui::EndDisabled();
    const bool fm = S.demod == dect2::GenNfm || S.demod == dect2::GenWfm;
    flowNext(14 * gUi);
    if (fm) {
        ImGui::TextDisabled("de-emphasis"); flowNext(4 * gUi);
        int& de = S.demod == dect2::GenNfm ? S.deNfm : S.deWfm;
        ImGui::SetNextItemWidth(85 * gUi);
        if (ImGui::Combo("##gde", &de, kDeemphNames, 4)) save();
        tip("FM de-emphasis: 50 us in Europe, 75 us in the Americas for broadcast FM; 750 us for pre-emphasised narrow FM");
    } else {
        if (ImGui::Checkbox("AGC", &S.agc)) save();
        tip("Automatic gain: AM follows the carrier, SSB and CW the envelope (fast attack, slow release)");
        if (!S.agc) {
            flowNext(4 * gUi);
            ImGui::SetNextItemWidth(130 * gUi);
            ImGui::SliderFloat("##ggain", &S.gainDb, 0, 80, "gain %.0f dB");
            if (ImGui::IsItemDeactivatedAfterEdit()) save();
        }
        if (S.demod == dect2::GenCw) {
            flowNext(10 * gUi);
            ImGui::SetNextItemWidth(130 * gUi);
            ImGui::SliderFloat("##gpitch", &S.pitch, 200, 1500, "pitch %.0f Hz");
            if (ImGui::IsItemDeactivatedAfterEdit()) save();
            tip("BFO: the tone a carrier on the channel frequency gives");
        }
    }
    flowNext(14 * gUi);
    if (iconFlat(a.muted ? Ic::Mute : Ic::Speaker, a.muted ? "Unmute" : "Mute")) a.muted = !a.muted;
    ImGui::SameLine(0, 4 * gUi);
    ImGui::SetNextItemWidth(100 * gUi);
    float vol = a.volume * 100.f;
    if (ImGui::SliderFloat("##gvol", &vol, 0, 100, "%.0f %%")) a.volume = vol / 100.f;
    tip("Volume (shared with the other modes)");
    flowEnd();

    // S-meter and recording
    sMeter(a);
    const bool rec = live(a) ? T(a).recording : a.engine.general().recording();
    if (!rec) {
        ImGui::BeginDisabled(!a.engine.running());
        if (ImGui::SmallButton("Record WAV")) startRecording(a);
        ImGui::EndDisabled();
        tip("Record what you hear (after the squelch, before the volume) to a 16-bit 48 kHz WAV file in the audio-recordings folder");
    } else if (ImGui::SmallButton("Stop recording")) a.engine.general().stopRecording();
    if (live(a) && (T(a).recording || !T(a).recordPath.empty())) {
        ImGui::SameLine(0, 8 * gUi);
        if (T(a).recording) ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "REC %.1f s", T(a).recordSec);
        else ImGui::TextDisabled("saved");
        ImGui::SameLine(0, 6 * gUi);
        ImGui::TextDisabled("%s", T(a).recordPath.c_str());
    }
    const std::string& err = S.recErr.empty() && live(a) ? T(a).recordError : S.recErr;
    if (!err.empty()) { ImGui::SameLine(0, 8 * gUi); ImGui::TextColored(pal::warnAmber(), "%s", err.c_str()); }
}

void tick(App& a) {
    load();
    if (S.lastDial >= 0 && std::fabs(a.freqMhz - S.lastDial) > 1e-9) { S.chanOff = 0; save(); }   // the dial was moved elsewhere
    S.lastDial = a.freqMhz;
    auto& r = a.engine.general();
    r.setDemod(S.demod);
    r.setWidthHz(S.width[S.demod]);
    r.setChannelOffsetHz(S.chanOff);
    r.setSquelch(S.sqOn, S.sqAuto, S.sqDb);
    r.setAgc(S.agc, S.gainDb);
    r.setDeemphUs(S.demod == dect2::GenNfm ? kDeemph[S.deNfm] : S.demod == dect2::GenWfm ? kDeemph[S.deWfm] : 0);
    r.setCwPitchHz(S.pitch);
    r.setVolume(a.volume); r.setMuted(a.muted);
}

void tab(App& a) {
    controls(a);
    const float h = ImGui::GetContentRegionAvail().y;
    spectrumPlot(a, ImVec2(-1, std::max(60.f, h * 0.45f - 2)), h < 300 * gUi);
    waterfallPlot(a, ImVec2(-1, -1));
}

void receiver(App& a) {
    const auto& t = T(a);
    if (!live(a)) { ImGui::TextDisabled("%s", a.engine.running() ? "starting" : "stopped"); return; }
    ImGui::TextWrapped("%s", dect2::generalSummary(t).c_str());
    ImGui::Text("Listening on %.6f MHz (%+.3f kHz from the dial)", listenHz(a) / 1e6, t.chanOffsetHz / 1e3);
    ImGui::Text("Input %.3f Msps, demodulated at %.0f kHz", t.rate / 1e6, t.demodRate / 1e3);
    ImGui::Text("Filter %+.2f to %+.2f kHz around the channel", t.filterLoHz / 1e3, t.filterHiHz / 1e3);
    if (t.noiseDb > -199) ImGui::Text("Noise in the channel %.1f dBFS", t.noiseDb); else ImGui::TextDisabled("Noise not measured yet");
    if (t.squelchDb > -199) ImGui::Text("Squelch threshold %.1f dBFS", t.squelchDb);
    if (t.demod <= dect2::GenWfm && t.squelchOpen) ImGui::Text("FM offset %+.0f Hz", t.cfoHz);
    ImGui::Text("Audio %.1f dBFS", t.audioDb);
}

void list(App& a) {
    load();
    ImGui::TextDisabled("%s %.4g kHz", dect2::generalDemodName(S.demod), S.width[S.demod] / 1e3);
    ImGui::Text("%.4f MHz", listenHz(a) / 1e6);
    if (live(a)) ImGui::TextDisabled("%s", T(a).squelchOpen ? "squelch open" : "squelch closed");
}

void status(App& a) {
    StatusPanel panel;
    const bool on = live(a);
    lamp("RX", on ? (T(a).state == 2 ? 1 : T(a).state == 1 ? 2 : 3) : 0, (int)Ic::Speaker); flowNext(12 * gUi);
    ImGui::TextUnformatted(on ? dect2::generalSummary(T(a)).c_str() : (a.engine.running() ? "starting" : "stopped"));
    flowEnd();
}

void summary(const App& a, std::string& l1, std::string& l2) {
    char b[96];
    snprintf(b, sizeof b, "%s %.4f MHz", dect2::generalDemodName(S.demod), listenHz(a) / 1e6);
    l1 = b;
    if (live(a)) l2 = dect2::generalSummary(T(a));
}

void meters(const App& a, std::vector<ModeMeter>& out) {
    const auto& t = T(a);
    out.push_back({"LEVEL  dBFS", "%.0f", (double)t.levelDb, -130, 0, t.squelchOpen ? 1 : 0});
    out.push_back({"SNR  dB", "%.0f", (double)t.snrDb, 0, 40, !t.dataValid ? 0 : t.snrDb >= 15 ? 1 : t.snrDb >= 6 ? 2 : 3});
}

void synth(App& a, bool& changed) {
    dect2::SynthConfig& sc = a.tune.synth;
    { ImGui::PushTextWrapPos(0); ImGui::TextDisabled("one station on the dial frequency, modulated by a 1 kHz tone (CW: a steady carrier), over noise"); ImGui::PopTextWrapPos(); }
    int m = std::clamp(sc.modeOpt[0], 0, (int)dect2::GenDemodCount - 1);
    ImGui::SetNextItemWidth(120 * gUi);
    if (ImGui::Combo("Modulation##gsyn", &m, "NFM\0WFM\0AM\0USB\0LSB\0CW\0")) { sc.modeOpt[0] = m; changed = true; }
    bool quiet = sc.modeOpt[1] == 1;
    if (ImGui::Checkbox("Noise only##gsyn", &quiet)) { sc.modeOpt[1] = quiet ? 1 : 0; changed = true; }
    tip("No station: the squelch must stay closed");
}

} // namespace

// ---- the channel marker on the spectrum and the waterfall (plots.cpp)

double generalChannelWidthMhz(const App&) { load(); double lo, hi; edges(lo, hi); return (hi - lo) / 1e6; }

void generalChannelBand(const App& a, bool fill) {
    load();
    double lo, hi;
    edges(lo, hi);
    const double f = listenHz(a) / 1e6, x0 = f + lo / 1e6, x1 = f + hi / 1e6;
    if (fill) { plt::Spec band; band.FillColor = pal::accent(pal::dev() ? 0.14f : 0.16f); plt::PlotVBand("band", x0, x1, band); }
    const double e[2] = {x0, x1};
    plt::Spec es; es.LineColor = pal::accent(fill ? 0.55f : 0.70f); es.LineWeight = S.drag >= 0 ? 2.f : 1.f;
    plt::PlotInfLines("bandedges", e, 2, es);
    const double c[1] = {f};
    plt::Spec cs; cs.LineColor = ImVec4(0.95f, 0.35f, 0.30f, 0.85f); cs.LineWeight = 1.2f;
    plt::PlotInfLines("channel", c, 1, cs);
}

bool generalDragging(int plotId) { return S.drag >= 0 && S.dragPlot == plotId; }

void generalPlotMouse(App& a, int plotId) {
    load();
    const plt::Rect lim = plt::GetPlotLimits();
    const ImVec2 pp = plt::GetPlotPos(), ps = plt::GetPlotSize();
    const ImVec2 pos = ImGui::GetIO().MousePos;
    // The plot has an invisible mouse-capture item. Check the plotted area
    // geometrically rather than requiring ImGui hover ownership.
    const bool hov = pos.x >= pp.x && pos.x <= pp.x + ps.x && pos.y >= pp.y && pos.y <= pp.y + ps.y;
    if (!hov && !(S.drag >= 0 && S.dragPlot == plotId)) return;
    const double span = std::max(1e-9, lim.X.Max - lim.X.Min);
    auto px = [&](double mhz) { return pp.x + (float)((mhz - lim.X.Min) / span * ps.x); };
    const double mhz = plt::GetPlotMousePos().x, mx = ImGui::GetIO().MousePos.x;
    double lo, hi;
    edges(lo, hi);
    const double f = listenHz(a) / 1e6;
    const double el = f + lo / 1e6, eh = f + hi / 1e6;
    // which edges can be held: both for the symmetric filters, only the outer one for USB / LSB (the inner sits 300 Hz from the dial)
    const bool canLo = S.demod != dect2::GenUsb, canHi = S.demod != dect2::GenLsb;
    int edgeNear = -1;
    const float grab = 12 * gUi;  // usable with narrow AM/SSB/CW channels even when zoomed out
    if (canLo && std::fabs(mx - px(el)) < grab) edgeNear = 0;
    if (canHi && std::fabs(mx - px(eh)) < grab && (edgeNear < 0 || std::fabs(mx - px(eh)) < std::fabs(mx - px(el)))) edgeNear = 1;
    if (S.drag >= 0 && S.dragPlot == plotId) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        if (ImGui::IsMouseDown(0)) {
            const double d = (mhz - f) * 1e6;
            double w;
            if (S.demod == dect2::GenUsb) w = d - dect2::kGeneralSsbLowCutHz;
            else if (S.demod == dect2::GenLsb) w = -d - dect2::kGeneralSsbLowCutHz;
            else w = 2 * std::fabs(d);
            S.width[S.demod] = std::round(dect2::generalClampWidth(S.demod, w) / 10) * 10;
            ImGui::SetTooltip("filter %.4g kHz", S.width[S.demod] / 1e3);
        } else { S.drag = -1; S.dragPlot = -1; S.dragged = true; save(); }
        return;
    }
    if (!hov) return;
    if (edgeNear >= 0) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        if (ImGui::IsMouseClicked(0)) { S.drag = edgeNear; S.dragPlot = plotId; S.dragged = false; return; }
        ImGui::SetTooltip("drag to change the filter (%.4g kHz)", S.width[S.demod] / 1e3);
        return;
    }
    if (ImGui::IsMouseClicked(0)) S.dragged = false;
    const double to = snap(mhz * 1e6);
    const ImVec2 dd = ImGui::GetMouseDragDelta(0, 0);
    if (ImGui::IsMouseReleased(0) && !S.dragged && std::fabs(dd.x) < 4 && std::fabs(dd.y) < 4) { setListen(a, to); return; }
    if (std::fabs(to - listenHz(a)) > 0.5) {
        const double lx[2] = {to / 1e6, to / 1e6}, ly[2] = {lim.Y.Min, lim.Y.Max};
        plt::Spec ls; ls.LineColor = ImVec4(0.95f, 0.25f, 0.25f, 0.5f); ls.LineWeight = 1.f;
        plt::PlotLine("##gtunehere", lx, ly, 2, ls);
        ImGui::SetTooltip("click to listen on %.4f MHz", to / 1e6);
    }
}

extern const ModeUi kGeneralUi;
const ModeUi kGeneralUi = {
    .sideTitle = "CHANNEL",
    .tabName = "Listen",
    .tabIcon = Ic::Speaker,
    .tab = tab,
    .receiver = receiver,
    .list = list,
    .status = status,
    .summary = summary,
    .synth = synth,
    .tick = tick,
    .meters = meters,
};
