// TrackUnlimited Phase 3.5: what settings EXIST, as data.
// Plain C++17, no engine dependency.
//
// ===================== THE MENU IS GENERATED, NOT MAINTAINED =====================
//
// `Settings.h` is a key/value store: it knows how to hold "audio.master" = "0.8"
// and how not to lose it. It has no idea that setting exists, what type it is,
// what range it takes or which page it belongs on.
//
// A settings screen needs all four, and there are only two ways to give it them.
// Hardcode the list in the UI, and the day somebody adds a setting the menu does
// not show it — which is not a bug anybody notices, because the symptom is
// something MISSING. Or declare the list once and let the menu walk it.
//
// This project has already taken the second answer three times: the control panel
// walks the same block and zone lists the geometry walks, `PlcImage::Describe()`
// enumerates its own slots so the docs cannot drift, and `PlcProgram` generates
// the permissive text it actually runs. Same reason each time — a second list is
// a second thing to keep true.
//
// ===================== WHAT IS OURS AND WHAT IS NOT =====================
//
// **Video and graphics are UE's.** `UGameUserSettings` already owns window mode,
// VSync, frame cap and the five scalability groups, it already persists them,
// and it already knows what the hardware can do. Reimplementing that would be
// worse in every respect. Those entries appear here so the MENU is
// uniform — one walk builds every page — and the engine binding says which
// property each maps to.
//
// **Audio, controls and simulation are ours.** Nothing in the engine knows what
// a scan rate is or which key dispatches a train.
//
// So an entry is a DESCRIPTION, and who stores the value is a property of the
// entry rather than a second system.

#pragma once

#include <string>
#include <vector>

enum class ESettingKind
{
    Bool,
    // A slider. Min/Max/Step are meaningful and the default must lie inside
    // them, which the suite asserts rather than trusting.
    Scalar,
    // A fixed set of choices, stored by its OPTION STRING rather than an index —
    // because an index silently changes meaning the day somebody inserts an
    // option, and a config file full of indices cannot be read by a human.
    Choice,
    // A key binding. Owned by FInputMap; this entry exists so the page can be
    // generated with everything else.
    Key,
};

// Where the value actually lives. Not a detail: it decides what "reset to
// defaults" means, and it is the difference between the menu owning a setting
// and merely displaying one.
enum class ESettingOwner
{
    // FSettings — ours, written to our config, subject to "a default is not a
    // value" and "unknown keys survive".
    Ours,
    // UGameUserSettings — UE's, persisted by UE, and it knows things we do not
    // (what resolutions the monitor has). The Engine field names the property.
    Engine,
    // FInputMap.
    Input,
};

enum class ESettingPage
{
    Video,
    Graphics,
    Audio,
    Controls,
    Simulation,
};

inline const char* SettingPageName(ESettingPage P)
{
    switch (P)
    {
    case ESettingPage::Video:    return "Video";
    case ESettingPage::Graphics: return "Graphics";
    case ESettingPage::Audio:    return "Audio";
    case ESettingPage::Controls: return "Controls";
    default:                     return "Simulation";
    }
}

struct FSettingEntry
{
    std::string Key;          // "audio.master" — the config key, and stable for ever
    std::string Label;        // "Master volume"
    std::string Help;         // one sentence, shown next to the control
    ESettingKind Kind = ESettingKind::Bool;
    ESettingOwner Owner = ESettingOwner::Ours;
    ESettingPage Page = ESettingPage::Simulation;

    std::string Default;      // as a string, same as FSettings stores it
    double Min = 0.0;
    double Max = 1.0;
    double Step = 0.1;
    std::vector<std::string> Options;   // Choice only

    // For Owner == Engine: which UGameUserSettings property this is. Named
    // rather than switched on the key, so the mapping is visible in the table
    // instead of buried in whatever code applies it.
    std::string Engine;

    // A RESTART IS A PROMISE, and one the UI has to keep. Some engine settings
    // do not take effect until the application restarts, and a slider that
    // appears to do nothing is worse than one labelled as deferred.
    bool bNeedsRestart = false;
};

// ===================== THE TABLE =====================
//
// One place. Adding a setting is one entry and every page that shows it updates,
// because there is nothing else to update.
//
// KEYS ARE STABLE FOR EVER. They are what lands in somebody's config file, so
// renaming one silently resets that setting for everybody who had touched it —
// the same class of loss "unknown keys survive" exists to prevent, arriving from
// the other direction.
inline std::vector<FSettingEntry> SettingsSchema()
{
    std::vector<FSettingEntry> S;

    auto Add = [&S](FSettingEntry E) { S.push_back(std::move(E)); };

    // ---- VIDEO. UE's, every one of them.
    {
        FSettingEntry E;
        E.Owner = ESettingOwner::Engine;
        E.Page = ESettingPage::Video;

        E.Key = "video.windowMode"; E.Label = "Window mode";
        E.Help = "Fullscreen takes exclusive control of the display. Windowed Fullscreen is a "
                 "borderless window at desktop size — same look, and it alt-tabs instantly. "
                 "Windowed is resizable.";
        E.Kind = ESettingKind::Choice;
        E.Options = {"Fullscreen", "Windowed Fullscreen", "Windowed"};
        E.Default = "Windowed Fullscreen";
        E.Engine = "FullscreenMode";
        Add(E);

        // NO RESOLUTION PICKER, and that is a decision rather than an omission.
        // Two of the three modes have no resolution to pick — they are the
        // desktop's — and the third is a window somebody drags to the size they
        // want, which is a better control than a dropdown of sizes their monitor
        // may not have. What makes this affordable is the DPI rule: the scale
        // curve clamps at 1.5 so a bigger window shows MORE ROWS rather than
        // larger ones, and no list hardcodes a row count. A layout that only
        // worked at one size would need the picker back.

        E = FSettingEntry(); E.Owner = ESettingOwner::Engine; E.Page = ESettingPage::Video;
        E.Key = "video.vsync"; E.Label = "V-Sync";
        E.Help = "Caps to the display's refresh rate and removes tearing.";
        E.Kind = ESettingKind::Bool; E.Default = "true";
        E.Engine = "bUseVSync";
        Add(E);

        E = FSettingEntry(); E.Owner = ESettingOwner::Engine; E.Page = ESettingPage::Video;
        E.Key = "video.frameCap"; E.Label = "Frame rate limit";
        E.Help = "0 is uncapped. A cap below the scan rate does not slow the ride — the "
                 "simulation runs on its own fixed period.";
        E.Kind = ESettingKind::Scalar; E.Min = 0.0; E.Max = 360.0; E.Step = 10.0;
        E.Default = "0";
        E.Engine = "FrameRateLimit";
        Add(E);
    }

    // ---- GRAPHICS. UE's scalability groups, 0-4, and named as UE names them so
    // somebody who knows the engine recognises them.
    {
        const char* Groups[][3] = {
            {"graphics.viewDistance", "View distance",   "ViewDistanceQuality"},
            {"graphics.shadows",      "Shadows",         "ShadowQuality"},
            {"graphics.textures",     "Textures",        "TextureQuality"},
            {"graphics.effects",      "Effects",         "VisualEffectQuality"},
            {"graphics.antiAliasing", "Anti-aliasing",   "AntiAliasingQuality"},
            {"graphics.postProcess",  "Post processing", "PostProcessQuality"},
        };
        for (const auto& G : Groups)
        {
            FSettingEntry E;
            E.Owner = ESettingOwner::Engine;
            E.Page = ESettingPage::Graphics;
            E.Key = G[0]; E.Label = G[1]; E.Engine = G[2];
            E.Kind = ESettingKind::Choice;
            E.Options = {"Low", "Medium", "High", "Epic", "Cinematic"};
            E.Default = "High";
            E.Help = "Scalability group, as the engine defines it.";
            Add(E);
        }
    }

    // ---- AUDIO. Ours, because nothing in the engine knows what a chain lift is.
    {
        const char* Buses[][3] = {
            {"audio.master", "Master",         "Everything."},
            {"audio.ride",   "Ride",           "Wheels, chain, brakes, launch."},
            {"audio.ui",     "Interface",      "Clicks, confirmations, alarms."},
            {"audio.ambient","Ambience",       "Wind, crowd, park background."},
        };
        for (const auto& B : Buses)
        {
            FSettingEntry E;
            E.Page = ESettingPage::Audio;
            E.Key = B[0]; E.Label = B[1];
            // SAID, BECAUSE A CONTROL THAT APPEARS TO DO NOTHING IS WORSE THAN ONE
            // LABELLED AS DEFERRED — the same rule bNeedsRestart exists for. There
            // is no audio system yet, so these persist and change nothing you can
            // hear, and the row says so rather than letting somebody conclude their
            // speakers are broken.
            E.Help = std::string(B[2]) + " Stored — there is no audio system yet.";
            E.Kind = ESettingKind::Scalar;
            E.Min = 0.0; E.Max = 1.0; E.Step = 0.05;
            E.Default = "0.8";
            Add(E);
        }

        FSettingEntry E;
        E.Page = ESettingPage::Audio;
        E.Key = "audio.muteUnfocused"; E.Label = "Mute when not focused";
        E.Help = "Silence the application while another window has focus.";
        E.Kind = ESettingKind::Bool; E.Default = "true";
        Add(E);
    }

    // ---- CONTROLS. The bindings the ride already has. Owned by FInputMap, and
    // listed here so one walk builds this page like every other.
    //
    // THE NAMES ARE UE'S OWN FKey NAMES, and that is load-bearing rather than a
    // convenience: the shell checks each one against what is actually bound on the
    // input component at startup, so a page that promises a key nothing answers to
    // is reported. This table drifted from the bindings for exactly one commit —
    // it said F2 for the overlay toggle after F2 was given up to the editor's
    // wireframe view — and nothing could see it. Now something can.
    {
        const char* Keys[][3] = {
            {"key.mode",        "Cycle mode",           "Tab"},
            {"key.camera",      "Cycle camera",         "C"},
            {"key.train",       "Next train",           "T"},
            {"key.frameAll",    "Frame whole track",    "F"},
            {"key.frameSel",    "Frame selection",      "Z"},
            {"key.segPrev",     "Previous segment",     "LeftBracket"},
            {"key.segNext",     "Next segment",         "RightBracket"},
            {"key.editor",      "Segment editor",       "B"},
            {"key.diagnostics", "Diagnostics",          "V"},
            {"key.graph",       "Cycle profile channel","G"},
            {"key.graphHide",   "Hide profile graph",   "H"},
            {"key.panel",       "Control panel",        "P"},
            {"key.overlays",    "Hide all overlays",    "U"},
            {"key.settings",    "Settings screen",      "O"},
            {"key.menu",        "Back to the menu",     "M"},
            // NOT Ctrl+S. The editor owns that chord and its bindings are live in
            // PIE, so the familiar one would save the level as well as the track.
            // Every letter that spells save is taken, [S] most of all — it moves.
            // Shift+K is save-as and is NOT a second row: the checker looks each
            // row's key up in the input component, and nothing is bound to a
            // chord — the shift is read inside the handler. A row promising a key
            // nothing answers to is exactly what that check exists to catch.
            {"key.save",        "Save the track (Shift: save as)", "K"},
            // Either side of save, for the same reason it is not Ctrl+S: the
            // editor's undo is live in PIE and would step back through the LEVEL.
            {"key.undo",        "Undo",                 "J"},
            {"key.redo",        "Redo",                 "L"},
            {"key.insert",      "Insert a segment",     "I"},
            {"key.remove",      "Remove the segment",   "R"},
            {"key.dispatch",    "Dispatch",             "SpaceBar"},
            {"key.estop",       "Emergency stop",       "BackSpace"},
            {"key.reset",       "Reset E-stop",         "End"},
            {"key.acknowledge", "Acknowledge faults",   "Home"},
            {"key.pause",       "Pause simulation",     "Pause"},
            {"key.step",        "Step one scan",        "Period"},
            {"key.slower",      "Slow the sim clock",   "Comma"},
            {"key.faster",      "Speed the sim clock",  "Slash"},
        };
        for (const auto& K : Keys)
        {
            FSettingEntry E;
            E.Owner = ESettingOwner::Input;
            E.Page = ESettingPage::Controls;
            E.Kind = ESettingKind::Key;
            E.Key = K[0]; E.Label = K[1]; E.Default = K[2];
            Add(E);
        }

        // ===================== SENSITIVITY, AND INVERSION PER CAMERA =====================
        //
        // This was ONE bool flipping both cameras' Y, on the argument that
        // "somebody who wants Y inverted wants it inverted". That argument did not
        // survive its first user: orbit felt backwards and free-fly felt right,
        // which is exactly the case one knob cannot express.
        //
        // The DEFAULTS still encode each camera's own convention — orbit drags the
        // SUBJECT, free-fly turns your HEAD, and they are meant to disagree — so
        // these flip from there rather than from a single raw axis.
        //
        // `input.invertLookY` is deliberately not carried forward. Keys are
        // permanent, so it is not reused for something else; anybody who set it
        // keeps the line in their file untouched by the unknown-key rule, and it
        // simply stops being read.
        FSettingEntry E;
        E.Page = ESettingPage::Controls;
        E.Key = "input.sensitivity"; E.Label = "Mouse sensitivity";
        E.Help = "Multiplies both look cameras. 1.0 is the shipped feel.";
        E.Kind = ESettingKind::Scalar; E.Min = 0.25; E.Max = 4.0; E.Step = 0.05;
        E.Default = "1";
        Add(E);

        const char* Inverts[][3] = {
            {"input.orbitInvertX", "Invert orbit (X)",
             "The Build-mode camera, which swings around what you are looking at."},
            {"input.orbitInvertY", "Invert orbit (Y)",
             "Orbit drags the SUBJECT, so mouse down swings the camera up over the top of it."},
            {"input.flyInvertX",   "Invert free-fly (X)",
             "The camera you steer yourself, on [C]."},
            {"input.flyInvertY",   "Invert free-fly (Y)",
             "Free-fly turns your HEAD, so mouse down looks down."},
        };
        for (const auto& I : Inverts)
        {
            E = FSettingEntry(); E.Page = ESettingPage::Controls;
            E.Key = I[0]; E.Label = I[1]; E.Help = I[2];
            E.Kind = ESettingKind::Bool; E.Default = "false";
            Add(E);
        }
    }

    // ---- SIMULATION. Entirely ours, and the page that makes this a tool.
    {
        FSettingEntry E;
        E.Page = ESettingPage::Simulation;

        E.Key = "sim.scanHz"; E.Label = "Controller scan rate";
        E.Help = "How often the PLC scans, in Hz. A real one scans on a fixed period and does "
                 "not go faster when the graphics card is idle. Changing this changes every "
                 "rate in the control system.";
        E.Kind = ESettingKind::Scalar; E.Min = 30.0; E.Max = 480.0; E.Step = 30.0;
        E.Default = "240";
        E.bNeedsRestart = true;
        Add(E);

        E = FSettingEntry(); E.Page = ESettingPage::Simulation;
        E.Key = "sim.pauseUnfocused"; E.Label = "Pause when not focused";
        E.Help = "A ride left running behind another window drops the time it could not "
                 "simulate, which reports as overruns and cannot be judged afterwards.";
        E.Kind = ESettingKind::Bool; E.Default = "true";
        Add(E);

        E = FSettingEntry(); E.Page = ESettingPage::Simulation;
        E.Key = "sim.autosaveSeconds"; E.Label = "Autosave interval";
        E.Help = "Seconds between autosaves. Writes a SIDECAR and never your document.";
        E.Kind = ESettingKind::Scalar; E.Min = 30.0; E.Max = 900.0; E.Step = 30.0;
        E.Default = "120";
        Add(E);

        E = FSettingEntry(); E.Page = ESettingPage::Simulation;
        E.Key = "sim.units"; E.Label = "Readout units";
        E.Help = "Display only. Authored fields stay in metres and degrees whatever this says — "
                 "converting on the way IN is how a save format stops round-tripping.";
        E.Kind = ESettingKind::Choice;
        E.Options = {"Metric", "Imperial"};
        E.Default = "Metric";
        Add(E);
    }

    return S;
}

// Declare every schema default into a settings store. The store keeps defaults
// separately from explicit values, so this is what makes "a default is not a
// value" work across a schema rather than one key at a time.
template <typename TSettings>
inline void DeclareSchemaDefaults(TSettings& Store)
{
    for (const FSettingEntry& E : SettingsSchema())
    {
        Store.Declare(E.Key, E.Default);
    }
}

// ponytail: no per-entry validator function. A Scalar's range and a Choice's
// option list are enough to clamp and to reject, and the suite asserts every
// default satisfies its own constraint — which is the failure worth catching,
// because a default outside its own slider is a control that cannot show the
// value it starts on.

