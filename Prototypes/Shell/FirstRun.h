// TrackUnlimited Phase 3.5: the first five minutes.
// Plain C++17, no engine dependency.
//
// The hybrid positioning lives or dies here — pro-grade core, friendly on-ramp.
// A numeric segment editor with an empty list is the single most intimidating
// first screen this project could ship, and it is exactly what happens by
// default.
//
// ===================== WHY THIS IS CODE AND NOT A DESIGN DOCUMENT =====================
//
// Because the check is the point. Every field the editor can show must have a
// tooltip, a unit and a typical range, and the assert suite walks the FIELD LIST
// to prove it — so adding a field without help fails the build rather than
// shipping a box with no label.
//
// That is the same rule as the process image describing itself and the reference
// figures being reproducible: a hand-kept list of help text drifts the first time
// somebody adds a field, and the drift is silent and lands on a beginner.
//
// ===================== THE ONE GENUINELY UNUSUAL CONCEPT =====================
//
// People arriving from Planet Coaster will try to drag the track. They need a
// straight answer for why they cannot, and "constraint 1" is not it. The answer
// below is written for somebody who has never heard of a clothoid.

#pragma once

#include "SegmentEditorModel.h"

#include <cstddef>
#include <string>

// ===================== FIELD HELP =====================
//
// A tooltip and a typical range for every field, because a numeric box with no
// context is a box somebody types 1000 into to see what happens.

struct FFieldHelp
{
    const char* Tooltip = "";
    double TypicalMin = 0.0;
    double TypicalMax = 0.0;
    bool bHasRange = false;
};

inline FFieldHelp HelpFor(EEditField F)
{
    switch (F)
    {
    case EEditField::Kind:
        return {"What SHAPE this piece is. Straight, Arc (a constant-radius turn), "
                "Clothoid (a transition that eases from one curvature into another), "
                "Helix, or Pitch (a vertical curve: a climb, a crest, a pull-out). Click "
                "to change it. Nothing you have typed is lost when you do -- a radius "
                "you set on an arc is still there if you come back.",
                0.0, 0.0, false};
    case EEditField::Length:
        return {"How far this piece of track runs, measured along the track itself "
                "rather than across the ground.",
                2.0, 200.0, true};
    case EEditField::Radius:
        return {"How tight the turn is. SMALLER IS TIGHTER — a 10 m radius is a hairpin "
                "and a 60 m one is a gentle sweep. Below about 5 m no real coaster goes.",
                8.0, 80.0, true};
    case EEditField::CurvatureStart:
        return {"Curvature is one divided by radius, so 0 is dead straight and larger "
                "numbers bend harder. A transition starts at whatever the piece before "
                "it ended on — that is what makes the join smooth.",
                0.0, 0.125, true};
    case EEditField::CurvatureEnd:
        return {"What the curve has bent to by the end of this piece. Set it equal to the "
                "next piece's start and a rider feels no kick at the join.",
                0.0, 0.125, true};
    case EEditField::ClimbAngle:
        return {"How steeply a helix climbs as it turns. Zero is a flat circle.",
                5.0, 35.0, true};
    case EEditField::Turns:
        return {"How many times round. Fractions are allowed — 1.5 turns is a helix that "
                "ends facing the opposite way.",
                0.25, 3.0, true};
    case EEditField::PitchDelta:
        return {"How far the nose comes UP through this piece, in degrees; negative "
                "points it down. A lift climb is one piece easing in to +25, a crest "
                "is one easing in to -30 and one easing out by the same, a pull-out "
                "brings it back to 0. Length follows from this and the radius.",
                -60.0, 60.0, true};
    case EEditField::PitchEase:
        return {"The SHAPE of the vertical curve. EASE IN starts straight and bends "
                "harder to the radius; EASE OUT does the opposite; CONSTANT holds the "
                "radius the whole way. A hill that eases in and out again is two "
                "pieces, and a rider feels no kick at either end of it.",
                0.0, 0.0, false};
    case EEditField::RollStart:
        return {"How far the track is banked where this piece BEGINS. Set it to the "
                "previous piece's roll end and the bank flows through the joint; leave "
                "them different and it steps, which a rider feels as a jolt and the "
                "validator reports as a roll step.",
                -90.0, 90.0, true};
    case EEditField::Roll:
        return {"How far the track is banked where this piece ENDS, measured from level. "
                "Positive rolls to the rider's right. A correctly banked turn is one "
                "where the rider feels pushed into the seat rather than sideways.",
                -90.0, 90.0, true};
    case EEditField::ZoneKind:
        return {"What kind of machinery is on this stretch: a lift chain, a launch, a "
                "brake, or a station platform. Plain track has none.",
                0.0, 0.0, false};
    case EEditField::ZoneSpeed:
        return {"What this device is trying to make the train do. A brake's number is the "
                "speed it RELEASES at, not the speed it stops at — brakes rest closed.",
                1.5, 40.0, true};
    case EEditField::ZoneAccel:
        return {"How hard this device can PUSH, in metres per second squared. It is the "
                "hardware somebody specified rather than one global figure: a chain "
                "hauling at 0.6 is nothing like a real chain, and a launch is ten times "
                "that. A device with no tractive authority — a brake, a trim — ignores "
                "it whatever you type here.",
                1.0, 12.0, true};
    case EEditField::ZoneDecel:
        return {"How hard this device can PULL BACK, in metres per second squared. The "
                "same idea as the push and specified separately, because the two are "
                "rarely the same machine. A launch has no braking authority however "
                "large you make this.",
                1.0, 12.0, true};
    case EEditField::ZoneBrakeDecel:
        return {"The FRICTION PAD, which is a second and separate machine from the drive "
                "tyres above — either can fail without the other. Zero means this device "
                "has none. A pad can only ever remove energy, so it is a ceiling and "
                "never a setpoint: below the commanded speed it does nothing at all, "
                "which is what makes it a brake rather than a motor.",
                2.0, 10.0, true};
    case EEditField::StartsNewDevice:
        return {"Tick this when a run of identical devices should count as a NEW one. "
                "Three loading positions in a row are the same kind at the same speed, so "
                "nothing else can tell them apart.",
                0.0, 0.0, false};
    case EEditField::Count:
        break;
    }
    // ===================== NO `default:`, AND THIS IS WHY =====================
    //
    // There was one, and it handed the three device rates above the TICK BOX's
    // tooltip — a beginner hovering "Accel" was told to tick it when a run of
    // identical devices should count as a new one. Silently, for as long as they
    // have existed, because a default case is a switch that never admits it is
    // missing anything.
    //
    // The header at the top of this file claims adding a field without help fails
    // the build. The default is what made that untrue: it satisfied the compiler
    // and the suite caught it only because the suite walks the enum. Exhaustive
    // now, so the NEXT field added warns where it is added rather than here.
    return {"", 0.0, 0.0, false};
}

// ===================== EMPTY STATES =====================
//
// Every panel says what goes there and how to put it there. A blank panel is a
// panel somebody assumes is broken.

enum class EPanelKind
{
    SegmentList,
    Diagnostics,
    RideProfile,
    RecentTracks,
    ControlProgram,
    Count
};

inline const char* EmptyStateFor(EPanelKind P)
{
    switch (P)
    {
    case EPanelKind::SegmentList:
        return "No track yet. Add a straight to begin — every coaster here is a list of "
               "pieces, and the first one starts at the station.";
    case EPanelKind::Diagnostics:
        return "Nothing to report. Warnings and errors about your track appear here, and "
               "clicking one takes you to it.";
    case EPanelKind::RideProfile:
        return "No ride to measure yet. Once the track carries a train from end to end, "
               "the speed and G traces appear here.";
    case EPanelKind::RecentTracks:
        return "Nothing opened yet. Start from a template, or open a track file.";
    default:
        // NOT "no program". The control layer is a LAYER OF CHOICE and an empty
        // panel here would read as something missing rather than something
        // working — which is exactly the alienation risk the card names.
        return "Your ride is running on its generated program, which is shown here for "
               "your track. You never have to change it. If you want to, an override "
               "attaches to one block and everything else keeps working.";
    }
}

// ===================== THE QUESTION EVERYBODY ARRIVING WILL ASK =====================
//
// Written for somebody who has never heard of a clothoid, and does not begin by
// saying no.
inline const char* WhyCannotIDragTheTrack()
{
    return
        "The 3D view is a picture of your track, not a place to build it.\n"
        "\n"
        "Track here is described by NUMBERS: this piece is 40 metres long, that one "
        "curves at a 35 metre radius, this one banks to 18 degrees. You type them, and "
        "the shape follows exactly.\n"
        "\n"
        "That sounds like more work and mostly it is less. Dragging gets you a shape "
        "that looks right and rides badly, and fixing it means nudging control points "
        "until the wobble goes away. Typing a radius gets you a turn that is exactly "
        "that radius, every time, and a transition that a rider cannot feel the join "
        "of — because the curve is defined rather than fitted.\n"
        "\n"
        "It is also how the people who design real coasters work, and it is why the "
        "G-force numbers this shows you mean something.";
}

// The one-line version, for a tooltip on the viewport itself.
inline const char* ViewportHint()
{
    return "This view is a preview — edit the numbers in the panel. Why?";
}

// ===================== TEMPLATES =====================
//
// "New track" must not mean an empty list. The first edit should be CHANGING A
// NUMBER ON SOMETHING THAT ALREADY RUNS, not authoring geometry from nothing —
// which is a completely different and much harder first task.
//
// AND A TEMPLATE IS A PRESET, NOT NEW GEOMETRY. `ATUCoasterRide::PresetLayout`
// already ships five worked examples, every one of them MEASURED before it went
// in — the flat rig is what rolling resistance was calibrated on, the circuit
// closes to 0.000000 m, the small-batch platform's 5.5-second figure came off it.
//
// Authoring a parallel set of "starter" layouts would be a second set of tracks
// to keep working, drifting from the ones the docs quote. So a template names a
// preset and adds the two things a preset does not carry: who it is for, and
// what to change first.
enum class ETemplatePreset
{
    FlatRig,
    OutAndBack,
    Reference,
    TwoTrainCircuit,
    SmallBatch,
    Showcase,
    Sidewinder,
    Wing,
    Blank,
};

struct FTemplate
{
    const char* Name = "";
    const char* Description = "";
    const char* WhatToTryFirst = "";   // the on-ramp: one concrete first edit
    ETemplatePreset Preset = ETemplatePreset::Blank;
};

inline std::size_t NumTemplates() { return 7; }

inline FTemplate TemplateAt(std::size_t i)
{
    switch (i)
    {
    case 0:
        return {"Starter — lift and drop",
                "Station, eased chain lift, a 34 degree drop, a loop and a banked turn "
                "into the brakes. The reference layout, and the one every number in the "
                "docs is quoted from.",
                "Make the lift taller. Change the lift hill's length and watch the drop "
                "speed and the loop's G reading change with it.",
                ETemplatePreset::Reference};
    case 1:
        return {"Launched circuit",
                "A launch instead of a lift, going all the way round and back to the "
                "station. The only preset that carries more than one train.",
                "Change the launch speed. Too slow and the train will not make the top "
                "of the hill — the profile panel will say exactly where it stalls.",
                ETemplatePreset::TwoTrainCircuit};
    case 2:
        return {"Out and back",
                "Lift, drop, an airtime hill, a banked turnaround, brakes. No inversion, "
                "and it does not pass near itself — the gentlest place to start.",
                "Change the airtime hill's crest curvature. Sharper gives more airtime, "
                "and the vertical G trace shows it going negative.",
                ETemplatePreset::OutAndBack};
    case 3:
        return {"Showcase — every device",
                "The launched circuit as a milestone: a four-position platform, a launch, "
                "a trim, a mid-course block brake with a real friction pad, kicker tyres "
                "out of it, and a full helix finale round the last turn. Every device "
                "specified as itself, and the fleet measured against the interlocking.",
                "Change the mid-course brake's friction pad rate to 0. The pad and the "
                "drive tyres are two separate machines on one stretch of track, so with "
                "the pad gone the tyres alone have to stop the train — and the [F3] "
                "panel says whether the block is long enough for that.",
                ETemplatePreset::Showcase};
    case 4:
        return {"Sidewinder — a ride built to a brief",
                "Chain lift, 40 degree drop, a banked 180, a snake banked both ways, two "
                "airtime hills, a helix, and a station three metres above the ground. "
                "Seven cars, three trains on four holding places. The first layout here "
                "designed as a ride rather than to prove a device.",
                "Change the mid-course block brake's release speed from 25 to 19 and "
                "ride it: the return leg runs out of energy on the final rise. The "
                "brake is a BLOCK more than a brake on this ride, and the profile "
                "graph shows why.",
                ETemplatePreset::Sidewinder};
    case 5:
        return {"Wing — the same ride, from a pod",
                "The Sidewinder's track exactly, with a wing vehicle on it: two seats a "
                "row, 3.6 m apart, each in its own pod beside the track with open air "
                "above and below. A type is a preset here, never a mode, so nothing "
                "about the ride changed except what you are sitting in.",
                "Change the seat spacing from 3.6 m to 0.6 and the two pods collapse "
                "into one tub over the track: the whole type is that one number. Then "
                "ride the snake and press [N] between the pods -- the telemetry reads a "
                "different vertical G through every roll.",
                ETemplatePreset::Wing};
    case 6:
    default:
        return {"Blank",
                "Nothing at all. For somebody who already knows what they are doing.",
                "Add a straight for the station, then a lift.",
                ETemplatePreset::Blank};
    }
}

// WHICH ONE OPENS ON A FIRST EVER LAUNCH. Not the blank one, and not a menu —
// boot into something that works and can be ridden in ten seconds, because that
// is the fastest possible demonstration that the thing does what it says.
inline std::size_t DefaultTemplate() { return 0; }

// ponytail: no tutorial sequence, no coach marks, no "press N to continue". Every
// one of those is a thing to dismiss, and the argument of this project is that
// the tool tells you the truth plainly — a first run that starts by talking over
// the track would contradict it. The template's own "what to try first" line is
// the whole of the guidance, and it sits next to the thing it describes.
