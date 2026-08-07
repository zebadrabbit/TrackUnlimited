// TrackUnlimited Phase 3.5: the diagnostics panel, as a model.
// Plain C++17, no engine dependency.
//
// `TrackValidate.h` already produces exactly the right data — errors with named
// fields, warnings with locations and quantities, joint continuity steps, roll
// rate and head-snap G — and none of it is visible outside a log today.
//
// What this owns is not the checking. It is the PRESENTATION RULES, and there
// are four of them that matter enough to be tested rather than left to whichever
// widget gets written first.
//
// ===================== 1. REPORT, NEVER REPAIR — IN THE UI TOO =====================
//
// There is no "fix it for me" here, and there is nowhere to put one: a row has a
// place to go and no action to take. That is deliberate and it is measured rather
// than principled — `PHASE0_FINDINGS.md` records that clamping a degenerate arc to
// a straight yields a plausible 1.00 g and a clean continuity pass, which is WORSE
// than leaving it visibly broken, because the ride now looks correct and is not.
//
// ===================== 2. A FINDING WITHOUT A PLACE IS TRIVIA =====================
//
// "Curvature implying a radius under 2 m" is only useful if it takes you there.
// Every row carries a target, and the panel's job is to make clicking it move the
// camera and the selection — which is why `FCameraRig::Frame` exists.
//
// ===================== 3. HEIGHT IS CALLED OUT SEPARATELY =====================
//
// The vertical slice shipped 8.5 m low because plan view looked closed. A closure
// gap reported as one number hides exactly that, so the vertical component is its
// own row with its own severity.
//
// ===================== 4. NEVER BLOCK SAVING =====================
//
// The format tolerates work in progress and so should the UI. A validator that
// refused to let somebody save an unfinished ride would teach them to work
// somewhere else.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

enum class EDiagSeverity
{
    // Something is measurably wrong with the ride: it does not close, the train
    // does not complete, a value is not finite.
    Error,

    // Buildable, but look at it.
    Warning,

    // Neither. A measured fact worth putting on screen — the lap time, the peak
    // G and where it happened. Present because a panel that only ever appears
    // when something is broken is a panel people learn to dread.
    Info,
};

// WHERE A ROW TAKES YOU. Either is optional, and a row with neither is a
// statement about the whole ride rather than a defect in part of it.
struct FDiagTarget
{
    int Segment = -1;      // index into the authored list
    double S = -1.0;       // arc length, for anything derived rather than authored

    bool HasPlace() const { return Segment >= 0 || S >= 0.0; }
};

struct FDiagRow
{
    EDiagSeverity Severity = EDiagSeverity::Warning;
    std::string Group;      // "Geometry", "Closure", "Ride", "Structure"
    std::string Text;
    FDiagTarget Target;

    // NO ACTION FIELD, DELIBERATELY. There is nowhere to put a "fix it" because
    // repairing a finding is measured to be worse than showing it: clamping a
    // degenerate arc to a straight yields a plausible 1.00 g and a clean
    // continuity pass, and the ride then looks correct and is not.
};

class FDiagnostics
{
public:
    void Clear() { Row.clear(); }

    void Add(EDiagSeverity Sev, const std::string& Group, const std::string& Text,
             FDiagTarget Target = FDiagTarget())
    {
        Row.push_back({Sev, Group, Text, Target});
    }

    // ===================== ORDERING =====================
    //
    // Severity first, then LOCATION — not insertion order, and not alphabetically
    // by message.
    //
    // Location, because somebody working on the first drop wants the findings
    // about the first drop together, and a list ordered by whichever check ran
    // first scatters them. Rows with no place sort last within their severity,
    // because a statement about the whole ride is context rather than a next
    // thing to go and fix.
    //
    // STABLE, so two findings at the same place keep the order the checks
    // produced — which is the order they depend on each other in.
    void Sort()
    {
        std::stable_sort(Row.begin(), Row.end(), [](const FDiagRow& A, const FDiagRow& B)
        {
            if (A.Severity != B.Severity) { return A.Severity < B.Severity; }
            const bool Pa = A.Target.HasPlace();
            const bool Pb = B.Target.HasPlace();
            if (Pa != Pb) { return Pa; }
            if (!Pa) { return false; }
            return Key(A.Target) < Key(B.Target);
        });
    }

    std::size_t Num() const { return Row.size(); }
    const FDiagRow& At(std::size_t i) const { return Row[i]; }

    std::size_t Count(EDiagSeverity S) const
    {
        std::size_t N = 0;
        for (const FDiagRow& R : Row) { if (R.Severity == S) { ++N; } }
        return N;
    }

    bool HasErrors() const { return Count(EDiagSeverity::Error) > 0; }

    // ===================== SAVING IS NEVER BLOCKED =====================
    //
    // The format tolerates work in progress and so does this. A validator that
    // refused to let somebody save an unfinished ride would teach them to work
    // somewhere else — and the one thing worse than an invalid file is no file.
    //
    // Present as a function rather than simply omitted, because somebody WILL
    // eventually wire a save button to `HasErrors()` and this is the thing that
    // makes them stop and read a sentence first.
    bool BlocksSaving() const { return false; }

    // A one-line summary for the status bar, because the panel is not always open
    // and "3 errors" is what makes somebody open it.
    std::string Summary() const
    {
        const std::size_t E = Count(EDiagSeverity::Error);
        const std::size_t W = Count(EDiagSeverity::Warning);
        if (E == 0 && W == 0) { return "no findings"; }
        std::string Out;
        if (E > 0) { Out += std::to_string(E) + (E == 1 ? " error" : " errors"); }
        if (E > 0 && W > 0) { Out += ", "; }
        if (W > 0) { Out += std::to_string(W) + (W == 1 ? " warning" : " warnings"); }
        return Out;
    }

    // ===================== THE TWO THAT NEED SPECIAL HANDLING =====================

    // THE CLOSURE GAP, WITH HEIGHT ON ITS OWN ROW. The vertical slice shipped
    // 8.5 m low because plan view looked closed, and one combined number hides
    // exactly that.
    void AddClosure(double GapX, double GapY, double GapZ, double Tolerance)
    {
        const double Plan = std::sqrt(GapX * GapX + GapY * GapY);
        if (Plan > Tolerance)
        {
            Add(EDiagSeverity::Error, "Closure",
                "the track does not close in plan: " + Metres(Plan) + " apart");
        }
        if (std::fabs(GapZ) > Tolerance)
        {
            // SEPARATE, AND SAYS WHICH WAY. "8.5 m apart" and "ends 8.5 m LOW"
            // are the same number and only one of them tells you what to change.
            Add(EDiagSeverity::Error, "Closure",
                std::string("the track ends ") + Metres(std::fabs(GapZ))
                + (GapZ < 0.0 ? " LOW" : " HIGH") + " of where it started");
        }
    }

    // THE RIDE NOT COMPLETING, PROMINENTLY. "This train does not crest the second
    // hill, stalls at S = 310 m" belongs on screen and not in a log line — and it
    // is an ERROR rather than a warning, because a ride the train cannot finish is
    // not a ride with a problem, it is not a ride.
    //
    // The envelope suite already learned this the hard way: it reported "within
    // envelope, zero findings" three times over a train that stalled at 46 m, and
    // a conformance verdict on a ride that did not happen is the one failure this
    // must not have.
    void AddRideProfile(bool bCompleted, double StalledAtS, double TopSpeedMs,
                        double PeakVerticalG, double PeakAtS, double LapSeconds)
    {
        if (!bCompleted)
        {
            FDiagTarget T;
            T.S = StalledAtS;
            Add(EDiagSeverity::Error, "Ride",
                "the train does not complete the circuit: it stalls at "
                + Metres(StalledAtS), T);
            // AND NOTHING DERIVED FROM THE RUN IS SHOWN, because a top speed or a
            // peak G from a ride that did not happen is worse than no number: it
            // reads as a result.
            return;
        }

        FDiagTarget T;
        T.S = PeakAtS;
        Add(EDiagSeverity::Info, "Ride",
            "peak vertical " + Fixed(PeakVerticalG, 2) + " g at " + Metres(PeakAtS), T);
        Add(EDiagSeverity::Info, "Ride",
            "top speed " + Fixed(TopSpeedMs * 3.6, 1) + " km/h, lap "
            + Fixed(LapSeconds, 1) + " s");
    }

private:
    static double Key(const FDiagTarget& T)
    {
        // Arc length if there is one, otherwise the segment index — which sorts
        // in the same direction, since segments are stored in travel order.
        return T.S >= 0.0 ? T.S : static_cast<double>(T.Segment) * 1e-6;
    }

    static std::string Fixed(double V, int Dp)
    {
        double Scale = 1.0;
        for (int i = 0; i < Dp; ++i) { Scale *= 10.0; }
        const long long N = static_cast<long long>(V * Scale + (V >= 0 ? 0.5 : -0.5));
        std::string S = std::to_string(N / static_cast<long long>(Scale));
        if (Dp > 0)
        {
            long long Frac = N % static_cast<long long>(Scale);
            if (Frac < 0) { Frac = -Frac; }
            std::string F = std::to_string(Frac);
            while (static_cast<int>(F.size()) < Dp) { F = "0" + F; }
            S += "." + F;
        }
        return S;
    }
    static std::string Metres(double V) { return Fixed(V, 2) + " m"; }

    std::vector<FDiagRow> Row;
};

// ponytail: no filtering, no search, no severity toggles. Those are worth adding
// the first time somebody has a list long enough to need them, and a panel that
// shipped with three filter dropdowns over four rows would be the wrong first
// impression for a tool whose whole argument is that it tells you the truth
// plainly.
