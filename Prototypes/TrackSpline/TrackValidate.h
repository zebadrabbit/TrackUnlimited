// TrackUnlimited Phase 1: authored-value validation at the editor boundary.
// Plain C++17, no engine dependency, same conventions as TrackSpline.h.
//
// PHASE0_FINDINGS deferred this here deliberately: `FTrack::AddSegment` rejects
// only bad lengths, because that is the one shared choke point the geometry
// itself cannot survive. Everything else — a NaN curvature, a radius typed into
// a curvature field, degrees typed into a radians field — is an AUTHORING
// mistake, and the right place to catch an authoring mistake is where the
// authoring happens, once, rather than scattered through the math.
//
// The rule this file follows, and the reason it exists at all:
//
//   REPORT, NEVER REPAIR.
//
// The findings measured what repairing costs. Clamping `MakeArc(L, 0)` — an
// infinite radius, i.e. a divide by zero — to a straight produces a plausible
// 1.00 G reading and a clean continuity pass. That is strictly WORSE than the
// obvious NaN it replaced, because a NaN is visibly broken and a wrong 1.00 G
// is not. Every check below therefore returns a diagnostic and changes nothing.
//
// Validate the authored segment list, then build the FTrack from it. Not the
// other way round.
//
// Units: metres, radians. Same frame conventions as TrackSpline.h.

#pragma once

#include "TrackProfile.h"
#include "TrackSpline.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

enum class ETrackIssue
{
    // Errors — the geometry cannot be trusted. Do not build a track from this.
    NotFinite,     // NaN or infinity in an authored field
    BadLength,     // zero, negative, or non-finite
    // Warnings — buildable, but almost certainly not what was meant.
    TinyRadius,    // curvature so high the radius is physically implausible
    HugeRoll,      // more than a full revolution; often degrees in a radians field
    ShortSegment,  // shorter than the integrator's own step; the editor cannot show it
    CurvatureStep, // discontinuity at a joint: the jolt clothoids exist to remove
    RollStep,      // roll discontinuity at a joint
    RollRateStep,  // roll *rate* discontinuity: felt as a snap, invisible to felt-G
    RollModeMixed, // joint where the two sides measure roll from different references
};

struct FTrackDiagnostic
{
    ETrackIssue Issue = ETrackIssue::NotFinite;
    bool bIsError = false;      // false means "buildable, but look at it"
    std::size_t SegmentIndex = 0;
    std::string Message;
};

// Thresholds, not laws. Every one is a judgement about what a human plausibly
// meant, so they are knobs rather than constants baked into the checks.
struct FTrackValidationLimits
{
    // Tighter than this and it is far more likely a radius was typed into a
    // curvature field than that someone wants a 1 m radius. Real inversions
    // bottom out around 5 m.
    double MinRadius = 2.0;

    // A full revolution of bank. Legitimate for a barrel roll, so this warns
    // rather than errors — but it is also exactly what 45 looks like when the
    // field wanted 0.785.
    double MaxRoll = 2.0 * 3.14159265358979323846;

    // The integrator's own step is 0.01 m. Below a few of those a segment
    // cannot be meaningfully previewed or sampled.
    double MinLength = 0.05;

    // Joint tolerances. Curvature is 1/m, so this is the step in 1/m that
    // counts as a discontinuity rather than floating-point noise.
    double CurvatureJointTolerance = 1e-9;
    double RollJointTolerance = 1e-9;

    // Roll RATE step, in rad per metre of track. Speed-independent, so the
    // check needs no design speed — multiply by v to get rad/s.
    //
    // This is not floating-point noise territory like the two above: a roll
    // rate step is a real authored discontinuity that the standard banking
    // pattern produces on purpose (ramp bank over the clothoid, hold it through
    // the arc, ramp out). PHASE0_FINDINGS measured 53.7 deg/s of step at four
    // joints per banked turn, with felt-G reporting nothing at all, because
    // felt G has no roll-rate term. Default is deliberately loose enough not to
    // flag every banked turn ever authored.
    double RollRateJointTolerance = 0.02;
};

namespace TrackValidateDetail
{

inline bool Finite(double V)
{
    return std::isfinite(V);
}

inline FTrackDiagnostic Make(ETrackIssue Issue, bool bError, std::size_t Index, const char* Text)
{
    FTrackDiagnostic D;
    D.Issue = Issue;
    D.bIsError = bError;
    D.SegmentIndex = Index;
    D.Message = Text;
    return D;
}

} // namespace TrackValidateDetail

// Check one segment in isolation. Joint problems need two segments and are
// handled by ValidateTrack.
inline void ValidateSegment(const FTrackSegment& Seg, std::size_t Index,
                            const FTrackValidationLimits& Limits,
                            std::vector<FTrackDiagnostic>& Out)
{
    using namespace TrackValidateDetail;
    char Buf[192];

    // Non-finite first, and return immediately. Every comparison below would be
    // false against a NaN, so continuing would silently report a NaN segment as
    // clean — which is exactly how a NaN passes IsCurvatureContinuous today.
    const double Fields[8] = {Seg.Length,
                              Seg.YawCurvatureStart,
                              Seg.YawCurvatureEnd,
                              Seg.PitchCurvatureStart,
                              Seg.PitchCurvatureEnd,
                              Seg.Torsion,
                              Seg.RollStart,
                              Seg.RollEnd};
    const char* Names[8] = {"Length",           "YawCurvatureStart", "YawCurvatureEnd",
                            "PitchCurvatureStart", "PitchCurvatureEnd", "Torsion",
                            "RollStart",        "RollEnd"};
    for (int i = 0; i < 8; ++i)
    {
        if (!Finite(Fields[i]))
        {
            std::snprintf(Buf, sizeof(Buf),
                          "%s is not a finite number. A common cause is a zero radius — "
                          "curvature is 1/radius, so radius 0 is a divide by zero.",
                          Names[i]);
            Out.push_back(Make(ETrackIssue::NotFinite, true, Index, Buf));
            return;
        }
    }

    if (!(Seg.Length > 0.0))
    {
        // THE CONSEQUENCE, not just the rule: a segment with no length is NOT
        // BUILT (FTrack::AddSegment refuses it), so everything after it sits
        // that much earlier than authored. On the Sidewinder four turns whose
        // easements ate more than the turn each lost 9.7 m of arc this way,
        // the circuit still closed by symmetry, and every later device sat
        // 39 m early -- with nothing saying so.
        std::snprintf(Buf, sizeof(Buf),
                      "Length is %.6g m; it must be greater than zero. NOT BUILT: everything "
                      "after it sits %.4g m earlier than authored. Easements longer than "
                      "their turn is the usual cause.",
                      Seg.Length, -Seg.Length);
        Out.push_back(Make(ETrackIssue::BadLength, true, Index, Buf));
        return;
    }

    if (Seg.Length < Limits.MinLength)
    {
        std::snprintf(Buf, sizeof(Buf),
                      "Length %.4g m is below %.4g m. The integrator steps at 0.01 m, so a "
                      "segment this short cannot be previewed or sampled meaningfully.",
                      Seg.Length, Limits.MinLength);
        Out.push_back(Make(ETrackIssue::ShortSegment, false, Index, Buf));
    }

    // Curvature magnitude is checked on the combined yaw/pitch vector at both
    // ends, because a segment curving hard in pitch is exactly as tight as one
    // curving hard in yaw and neither component alone shows it.
    const double MaxCurvature = 1.0 / Limits.MinRadius;
    const double StartK = std::sqrt(Seg.YawCurvatureStart * Seg.YawCurvatureStart
                                    + Seg.PitchCurvatureStart * Seg.PitchCurvatureStart);
    const double EndK = std::sqrt(Seg.YawCurvatureEnd * Seg.YawCurvatureEnd
                                  + Seg.PitchCurvatureEnd * Seg.PitchCurvatureEnd);
    const double Worst = StartK > EndK ? StartK : EndK;
    if (Worst > MaxCurvature)
    {
        std::snprintf(Buf, sizeof(Buf),
                      "Curvature reaches %.4g 1/m, a radius of %.4g m. Below %.4g m this is "
                      "far more likely a RADIUS typed into a curvature field than an "
                      "intended value.",
                      Worst, 1.0 / Worst, Limits.MinRadius);
        Out.push_back(Make(ETrackIssue::TinyRadius, false, Index, Buf));
    }

    const double WorstRoll =
        std::fabs(Seg.RollStart) > std::fabs(Seg.RollEnd) ? Seg.RollStart : Seg.RollEnd;
    if (std::fabs(WorstRoll) > Limits.MaxRoll)
    {
        std::snprintf(Buf, sizeof(Buf),
                      "Roll reaches %.4g rad (%.1f degrees, %.2f full revolutions). Legitimate "
                      "for a barrel roll — but this is also what a value in DEGREES looks "
                      "like in a radians field.",
                      WorstRoll, WorstRoll * 180.0 / 3.14159265358979323846,
                      std::fabs(WorstRoll) / (2.0 * 3.14159265358979323846));
        Out.push_back(Make(ETrackIssue::HugeRoll, false, Index, Buf));
    }
}

// Check a whole authored list: every segment, plus the joints between them.
//
// Joint checks go through CurvatureAt rather than the raw End fields, for the
// same reason IsCurvatureContinuous does — a segment with torsion exits with
// its curvature vector rotated, so the raw fields describe a joint nobody
// crosses.
inline std::vector<FTrackDiagnostic> ValidateTrack(
    const std::vector<FTrackSegment>& Segments,
    const FTrackValidationLimits& Limits = FTrackValidationLimits())
{
    using namespace TrackValidateDetail;
    std::vector<FTrackDiagnostic> Out;
    char Buf[192];

    for (std::size_t i = 0; i < Segments.size(); ++i)
    {
        ValidateSegment(Segments[i], i, Limits, Out);
    }

    for (std::size_t i = 0; i + 1 < Segments.size(); ++i)
    {
        const FTrackSegment& A = Segments[i];
        const FTrackSegment& B = Segments[i + 1];
        // Skip joints touching a segment already reported as unusable; the
        // numbers would be meaningless and the noise would bury the real error.
        bool bSkip = false;
        for (const FTrackDiagnostic& D : Out)
        {
            if (D.bIsError && (D.SegmentIndex == i || D.SegmentIndex == i + 1))
            {
                bSkip = true;
            }
        }
        if (bSkip)
        {
            continue;
        }

        double AYaw = 0.0, APitch = 0.0, BYaw = 0.0, BPitch = 0.0;
        CurvatureAt(A, A.Length, AYaw, APitch);
        CurvatureAt(B, 0.0, BYaw, BPitch);

        const double Step = std::sqrt((AYaw - BYaw) * (AYaw - BYaw)
                                      + (APitch - BPitch) * (APitch - BPitch));
        if (Step > Limits.CurvatureJointTolerance)
        {
            std::snprintf(Buf, sizeof(Buf),
                          "Curvature steps by %.6g 1/m into segment %zu. This is a jolt the "
                          "rider feels; a clothoid transition is what removes it.",
                          Step, i + 1);
            Out.push_back(Make(ETrackIssue::CurvatureStep, false, i, Buf));
        }

        // Across a change of roll REFERENCE the two numbers are not in the same
        // frame, so subtracting them means nothing. Say so and skip, rather than
        // reporting a fabricated step or a fabricated clean bill. PathRelative
        // and FollowsTorsion share a reference (the path frame, the second with
        // an analytic offset), so a joint between them is compared through
        // PathRollAt; only WorldBank needs the walked frame, and
        // AnalyseResolvedRollRate is the check that has it.
        const bool bAWorld = A.RollMode == ERollMode::WorldBank;
        const bool bBWorld = B.RollMode == ERollMode::WorldBank;
        if (bAWorld != bBWorld)
        {
            auto ModeName = [](ERollMode M)
            {
                return M == ERollMode::WorldBank ? "world bank"
                     : M == ERollMode::FollowsTorsion ? "follows torsion" : "path-relative";
            };
            std::snprintf(Buf, sizeof(Buf),
                          "Roll reference changes into segment %zu (%s -> %s); roll checks here "
                          "were skipped, the two values measure from different things.",
                          i + 1, ModeName(A.RollMode), ModeName(B.RollMode));
            Out.push_back(Make(ETrackIssue::RollModeMixed, false, i, Buf));
            continue;
        }

        const double RollStep = bAWorld ? std::fabs(A.RollEnd - B.RollStart)
                                        : std::fabs(PathRollAt(A, A.Length) - PathRollAt(B, 0.0));
        if (RollStep > Limits.RollJointTolerance)
        {
            std::snprintf(Buf, sizeof(Buf),
                          "Roll steps by %.6g rad (%.2f degrees) into segment %zu — the track "
                          "twists instantaneously.",
                          RollStep, RollStep * 180.0 / 3.14159265358979323846, i + 1);
            Out.push_back(Make(ETrackIssue::RollStep, false, i, Buf));
        }

        // The one felt G cannot see. Roll rate is rad per metre here, so it is
        // speed-independent; multiply by v for rad/s.
        const double RateA = (A.RollEnd - A.RollStart) / A.Length;
        const double RateB = (B.RollEnd - B.RollStart) / B.Length;
        const double RateStep = std::fabs(RateA - RateB);
        if (RateStep > Limits.RollRateJointTolerance)
        {
            std::snprintf(Buf, sizeof(Buf),
                          "Roll RATE steps by %.4g rad/m into segment %zu — %.1f deg/s at "
                          "20 m/s. Felt as a snap; no G reading will show it, because felt G "
                          "has no roll-rate term.",
                          RateStep, i + 1, RateStep * 20.0 * 180.0 / 3.14159265358979323846);
            Out.push_back(Make(ETrackIssue::RollRateStep, false, i, Buf));
        }
    }

    return Out;
}

// ------------------------------------------------------------- roll rate

// What banking costs the rider, expressed as something a human can judge.
//
// PHASE0_FINDINGS records two facts that only make sense together. The rider is
// modelled as a POINT at the heartline, so roll-rate terms vanish exactly — the
// felt-G numbers are correct and will never show this. But a real head sits
// about 0.5 m above the heartline, and the omitted centripetal term there is
// w^2 * r: measured at 0.031 g for 45 deg/s, 0.126 g at 90, and 0.503 g at 180.
// A fast barrel roll that reads perfectly smooth at the heartline is half a G
// of head snap.
//
// So this is not a redundant view of felt G. It is the part felt G structurally
// cannot contain, and until the train has length it is the only way to see it.
struct FRollRateReport
{
    double PeakRateRadPerM = 0.0;
    double PeakRateDegPerSec = 0.0; // at the speed passed in
    std::size_t PeakSegment = 0;

    double WorstStepRadPerM = 0.0;
    double WorstStepDegPerSec = 0.0;
    std::size_t WorstStepJoint = 0; // joint between this segment and the next

    // w^2 * r / g at the peak rate, for a rider RiderOffset above the
    // heartline. The G the model does not report.
    double HeadSnapG = 0.0;
};

// Measures the AUTHORED roll rate — what was typed, over the length it was
// typed across. For PathRelative segments that is the whole story. A WorldBank
// segment holding a constant bank through a hill reads zero here and is still
// twisting, because the frame is moving underneath it; use
// AnalyseResolvedRollRate for the rate a rider actually gets.
inline FRollRateReport AnalyseRollRate(const std::vector<FTrackSegment>& Segments, double SpeedMs,
                                       double RiderOffset = 0.5)
{
    const double RadToDeg = 180.0 / 3.14159265358979323846;
    FRollRateReport R;

    for (std::size_t i = 0; i < Segments.size(); ++i)
    {
        const FTrackSegment& S = Segments[i];
        if (!(S.Length > 0.0) || !std::isfinite(S.RollStart) || !std::isfinite(S.RollEnd))
        {
            continue;
        }
        const double Rate = std::fabs((S.RollEnd - S.RollStart) / S.Length);
        if (Rate > R.PeakRateRadPerM)
        {
            R.PeakRateRadPerM = Rate;
            R.PeakSegment = i;
        }
    }

    for (std::size_t i = 0; i + 1 < Segments.size(); ++i)
    {
        const FTrackSegment& A = Segments[i];
        const FTrackSegment& B = Segments[i + 1];
        if (!(A.Length > 0.0) || !(B.Length > 0.0))
        {
            continue;
        }
        const double Step = std::fabs((A.RollEnd - A.RollStart) / A.Length
                                      - (B.RollEnd - B.RollStart) / B.Length);
        if (Step > R.WorstStepRadPerM)
        {
            R.WorstStepRadPerM = Step;
            R.WorstStepJoint = i;
        }
    }

    R.PeakRateDegPerSec = R.PeakRateRadPerM * SpeedMs * RadToDeg;
    R.WorstStepDegPerSec = R.WorstStepRadPerM * SpeedMs * RadToDeg;

    const double Omega = R.PeakRateRadPerM * SpeedMs; // rad/s
    R.HeadSnapG = Omega * Omega * RiderOffset / GravityMs2;
    return R;
}

// The same question asked of the built track instead of the authored list, so
// the answer includes the roll the GEOMETRY contributes and not just the roll
// somebody typed.
//
// This is the check ERollMode::WorldBank needs to exist. A constant bank held
// through a climbing turn is a constant number in the data and a continuously
// rotating rider in the world; the segment-list version is structurally unable
// to see that, in the same way felt G is structurally unable to see roll rate
// at all. Two blind spots, stacked — hence two reports.
//
// It also covers WorldBank's failure mode without needing a special case for
// it: as the tangent approaches vertical the level reference swings, and the
// peak rate here goes through the roof. That is not an artefact of the solve,
// it is what banking against a horizon you are pointing at costs.
struct FResolvedRollReport
{
    double PeakRateRadPerM = 0.0;
    double PeakRateDegPerSec = 0.0; // at the speed passed in
    double PeakS = 0.0;             // arc length, metres, where the peak sits
    double HeadSnapG = 0.0;         // w^2 * r / g at the peak, RiderOffset above heartline
};

inline FResolvedRollReport AnalyseResolvedRollRate(const FTrack& Track, double SpeedMs,
                                                   double RiderOffset = 0.5, double StepM = 0.25)
{
    const double Pi = 3.14159265358979323846;
    FResolvedRollReport R;

    const double Total = Track.TotalLength();
    if (!(Total > 0.0) || !(StepM > 0.0))
    {
        return R;
    }

    // Walked with AdvanceFrom rather than repeated EvaluateAt: same reason the
    // train uses it, and it also keeps every sample on one continuous
    // integration so consecutive frames are directly comparable.
    const int N = static_cast<int>(std::ceil(Total / StepM));
    FTrackFrame Prev = Track.EvaluateAt(0.0);
    double PrevS = 0.0;

    for (int i = 1; i <= N; ++i)
    {
        const double Here = (i == N) ? Total : i * StepM;
        const FTrackFrame F = Track.AdvanceFrom(Prev, PrevS, Here);
        const double Span = Here - PrevS;
        if (Span > 0.0)
        {
            // Unwrapped. Resolved roll carries an atan2 term, so a bank passing
            // through +/-pi jumps by 2pi with nothing physical happening — and
            // that fake step would otherwise be the reported peak every time.
            double D = F.Roll - Prev.Roll;
            while (D > Pi)
            {
                D -= 2.0 * Pi;
            }
            while (D < -Pi)
            {
                D += 2.0 * Pi;
            }
            const double Rate = std::fabs(D / Span);
            if (Rate > R.PeakRateRadPerM)
            {
                R.PeakRateRadPerM = Rate;
                R.PeakS = (PrevS + Here) * 0.5;
            }
        }
        Prev = F;
        PrevS = Here;
    }

    R.PeakRateDegPerSec = R.PeakRateRadPerM * SpeedMs * 180.0 / Pi;
    const double Omega = R.PeakRateRadPerM * SpeedMs;
    R.HeadSnapG = Omega * Omega * RiderOffset / GravityMs2;
    return R;
}

// ponytail: finite-differenced over StepM, so a genuine roll STEP at a joint
// reports as one very large rate over one step rather than as an infinity. That
// is the right shape for a warning and the wrong shape for a precise number —
// ValidateTrack is what locates steps exactly.

// ------------------------------------------------------------ self-clearance

// Does the track pass through itself?
//
// The third member of the family this file keeps finding. Felt G cannot see
// roll rate. The authored segment list cannot see the roll the geometry
// contributes. And NOTHING in either can see that two parts of the track occupy
// the same space — every segment is individually perfect, every joint is
// continuous, every G reading is plausible, and the rider is passing through
// solid steel.
//
// Found the hard way: the vertical slice's loop is built from pure pitch
// curvature, which makes it exactly planar, so its ascending and descending legs
// close to 0.189 m of each other. The rails alone are 1.215 m wide. It took
// riding it to notice.
//
// Compared rail centre to rail centre, so the threshold to judge against is
// TrackWidth() at minimum — and a good deal more once a rider, a support
// structure and a maintenance walkway are in the way.
//
// THE COLLISION CORRIDOR is that "good deal more", given a number: how close
// two pieces of track may pass, rail centre to rail centre, before a train on
// one sweeps space a train on the other needs. Each track brings an envelope —
// a 1.40 m body, riders' reach past it, spine and ties below — and support
// practice already holds 1.5 m clear of the track's swept width
// (FSupportSettings::ClearanceM); two tracks each bring one, so the corridor is
// twice that. Vertically the envelope is TALLER than it is wide (a rider above
// the heartline, structure below the rails), so 3 m is the floor of what a
// crossing needs, not a generous figure. The Sidewinder's level helix shipped
// crossing itself at 0.109 m because nothing compared against this.
// ponytail: one number for a swept volume; an oriented envelope (lateral vs
// vertical corridors judged separately) is the upgrade when a legitimate
// near-miss layout — racing tracks, a flyover — needs the distinction.
constexpr double CollisionCorridorM = 3.0;

struct FClearanceReport
{
    double ClosestApproach = 1e9; // metres, rail centre to rail centre
    double AtS = 0.0;             // one of the two places
    double AndS = 0.0;            // the other
    bool bStructureOverlaps = false; // closer than the track is wide: definitely wrong
};

// IgnoreWithinM skips pairs of samples close together ALONG the track, which are
// trivially near each other in space and are not what this is looking for. It
// has to exceed the tightest turn's diameter or a hairpin reports itself.
//
// bCircuit measures that separation THE SHORT WAY ROUND. On a closed circuit the
// last sample and the first are the same piece of track, so a linear separation
// calls them TotalLength apart, never skips them, and reports the closure itself
// as a 0.00 m self-intersection — the one place a circuit is guaranteed to touch.
// Measured on the closed two-train layout: 0.00 m at S=0.0 against S=1717.0, with
// nothing wrong anywhere. Off by default, because a point-to-point layout's two
// ends genuinely are far apart and hiding that would be the opposite error.
inline FClearanceReport AnalyseSelfClearance(const FTrack& Track, const FTrackProfile& Profile,
                                             double StepM = 0.5, double IgnoreWithinM = 12.0,
                                             bool bCircuit = false)
{
    FClearanceReport R;
    const double Total = Track.TotalLength();
    if (!(Total > 0.0) || !(StepM > 0.0))
    {
        R.ClosestApproach = 0.0;
        return R;
    }

    std::vector<FVec3> Rail;
    std::vector<double> Arc;
    FTrackFrame Walk = Track.EvaluateAt(0.0);
    double S = 0.0;
    for (;;)
    {
        Rail.push_back(CrossSectionAt(Walk, Track.GetHeartlineHeight(), Profile).RailCentre);
        Arc.push_back(S);
        if (S >= Total)
        {
            break;
        }
        const double Next = S + StepM < Total ? S + StepM : Total;
        Walk = Track.AdvanceFrom(Walk, S, Next);
        S = Next;
    }

    // ponytail: O(n^2) over samples — 1,100 samples on a 550 m track at 0.5 m is
    // about 600k distance tests, which is milliseconds and runs offline. A
    // uniform spatial grid is the upgrade when a 5 km layout or a live editor
    // check makes this show up in a profile. Measure before building it.
    for (std::size_t i = 0; i < Rail.size(); ++i)
    {
        for (std::size_t j = i + 1; j < Rail.size(); ++j)
        {
            const double Along = bCircuit
                ? std::min(Arc[j] - Arc[i], Total - (Arc[j] - Arc[i]))
                : Arc[j] - Arc[i];
            if (Along < IgnoreWithinM)
            {
                continue;
            }
            const double D = Length(Rail[j] - Rail[i]);
            if (D < R.ClosestApproach)
            {
                R.ClosestApproach = D;
                R.AtS = Arc[i];
                R.AndS = Arc[j];
            }
        }
    }
    if (R.ClosestApproach > 1e8)
    {
        R.ClosestApproach = 0.0; // nothing was far enough apart along the track to compare
    }
    R.bStructureOverlaps = R.ClosestApproach < TrackWidth(Profile);
    return R;
}

// Is this list safe to build an FTrack from? Warnings do not block.
inline bool HasErrors(const std::vector<FTrackDiagnostic>& Diagnostics)
{
    for (const FTrackDiagnostic& D : Diagnostics)
    {
        if (D.bIsError)
        {
            return true;
        }
    }
    return false;
}

// ponytail: no severity beyond error/warning, and no localisation. Two levels
// is what the caller actually branches on — build or refuse — and a third would
// just be a warning nobody reads. Messages are English literals because the
// only consumer is a solo developer's editor; wrap them when there is a UI with
// a language setting.
