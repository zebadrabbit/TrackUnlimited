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
        std::snprintf(Buf, sizeof(Buf), "Length is %.6g; it must be greater than zero.",
                      Seg.Length);
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

        const double RollStep = std::fabs(A.RollEnd - B.RollStart);
        if (RollStep > Limits.RollJointTolerance)
        {
            std::snprintf(Buf, sizeof(Buf),
                          "Roll steps by %.6g rad (%.2f degrees) into segment %zu — the track "
                          "twists instantaneously.",
                          RollStep, RollStep * 180.0 / 3.14159265358979323846, i + 1);
            Out.push_back(Make(ETrackIssue::RollStep, false, i, Buf));
        }
    }

    return Out;
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
