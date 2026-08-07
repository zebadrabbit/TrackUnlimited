// TrackUnlimited: a timeline of faults, against the scan clock.
// Plain C++17, no dependencies.
//
// Three faults have been injectable for a while — a stuck restraint group, a
// jammed gate, a sensor that has died, stuck on or started chattering — and
// every one of them was set by hand in a test. Nothing said "jam restraint
// group 2 at the load platform forty seconds in, and free it at ninety". That
// gap is the whole of this file.
//
// ===================== AGAINST THE SCAN, NEVER THE CLOCK =====================
//
// A scenario indexed by wall time reproduces differently on a different machine,
// which makes it useless as evidence. Indexed by SCAN NUMBER it is exactly as
// reproducible as the simulation is — and since the scan period became fixed,
// that is bit-identical. `FSimDigest` is the instrument that proves it.
//
// ===================== IT SCHEDULES; IT DOES NOT APPLY =====================
//
// `Due()` hands back the steps that come due this scan and the caller performs
// them. Deliberately: this file has no idea what a restraint bank or a sensor
// is, so it cannot acquire an opinion about them, and a scenario for a subsystem
// that does not exist yet is a new enumerator rather than a new dependency.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

enum class EScenarioAction
{
    // Devices that can fail. The two integer parameters mean whatever the
    // action needs — a platform and a group, a sensor and a mode.
    JamRestraintGroup,   // A = platform, B = group index (-1 frees it)
    JamGateSection,      // A = platform, B = section  (-1 frees it)
    FailSensor,          // A = sensor,   B = mode
    HealSensor,          // A = sensor

    // Hardware that no longer delivers what it is rated for: a glazed pad, low
    // line pressure, a worn tyre. NOT a wrong command and not a drive fault — the
    // command is right, the drive writes it, and less comes out.
    //
    // B is a PERCENTAGE rather than a fraction, because a step carries two ints
    // and widening it for one action would be paying for this everywhere. 100 is
    // healthy hardware, so healing is DegradeDrive with B = 100 and no second
    // action is needed.
    DegradeDrive,        // A = zone,     B = percent of rated authority delivered

    // The operator. Included because half of what a scenario is testing is what
    // a person does about a fault, not only the fault.
    PressEmergencyStop,
    ReleaseEmergencyStop,
    RequestDispatch,
    ReleaseDispatch,

    // The controller, which can now be switched off.
    PowerCyclePlc,
    DeclareCourseClear,
};

struct FScenarioStep
{
    std::uint64_t AtScan = 0;
    EScenarioAction Action = EScenarioAction::FailSensor;
    int A = 0;
    int B = 0;
};

class FScenario
{
public:
    void Add(std::uint64_t AtScan, EScenarioAction Action, int A = 0, int B = 0)
    {
        FScenarioStep S;
        S.AtScan = AtScan;
        S.Action = Action;
        S.A = A;
        S.B = B;
        Steps.push_back(S);
        bSorted = false;
    }

    std::size_t Num() const { return Steps.size(); }

    // Every step due at exactly this scan, in the order they were added.
    //
    // EXACTLY, not "at or before". A caller that skipped a scan would silently
    // lose a step, and a scenario that quietly did less than it says is worse
    // evidence than one that fails — so the caller is expected to pump every
    // scan, which the fixed scan period makes trivial.
    //
    // Insertion order is preserved among steps sharing a scan, because two
    // faults injected on the same scan are ordered by the person who wrote them
    // and reordering would make the scenario mean something else.
    std::vector<FScenarioStep> Due(std::uint64_t Scan)
    {
        Sort();
        std::vector<FScenarioStep> Out;
        while (Cursor < Steps.size() && Steps[Cursor].AtScan < Scan)
        {
            // Behind the cursor: a caller that did not pump every scan. Skipped
            // rather than fired late, and counted, because firing an injection
            // at the wrong moment produces a run that reproduces nothing.
            ++Missed;
            ++Cursor;
        }
        while (Cursor < Steps.size() && Steps[Cursor].AtScan == Scan)
        {
            Out.push_back(Steps[Cursor]);
            ++Cursor;
        }
        return Out;
    }

    // Steps that came due while nobody was asking. Non-zero means the caller is
    // not pumping every scan, and the scenario did not run as written.
    std::size_t MissedSteps() const { return Missed; }

    bool IsFinished() const { return Cursor >= Steps.size(); }

    void Rewind()
    {
        Cursor = 0;
        Missed = 0;
    }

    // The scan the last step comes due, so a caller can size a run to the
    // scenario rather than guessing at it.
    std::uint64_t LastScan()
    {
        Sort();
        return Steps.empty() ? 0 : Steps.back().AtScan;
    }

private:
    void Sort()
    {
        if (bSorted) { return; }
        // STABLE, so steps sharing a scan keep the order they were written in.
        std::stable_sort(Steps.begin(), Steps.end(),
                         [](const FScenarioStep& X, const FScenarioStep& Y)
                         { return X.AtScan < Y.AtScan; });
        bSorted = true;
    }

    std::vector<FScenarioStep> Steps;
    std::size_t Cursor = 0;
    std::size_t Missed = 0;
    bool bSorted = true;
};

// ponytail: no scenario FILE format, and no authoring surface. Both are real and
// both are answerable once something other than a test writes one — the shape
// that is painful to retrofit is scan-indexing and caller-applied steps, and
// that is what is here. A loader is a parser over this struct.
