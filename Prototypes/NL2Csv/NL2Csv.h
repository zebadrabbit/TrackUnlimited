// TrackUnlimited Phase 0 prototype: NoLimits 2 CSV spline import/export.
// Plain C++17, no engine dependency — same shape as the other prototypes.
// Format reference: https://nolimitscoaster.com/nolimits2/help/pages/fileformats.html
//
// WHY THIS EXISTS. Docs/PHASE0_FINDINGS.md ends with two open unknowns that are
// blocked on the same missing thing — real track data and a reference simulator
// to check against:
//
//   * "Calibration against a real ride ... nothing has been compared to a
//      measured speed or G trace."
//   * "Lateral-G sign versus NoLimits 2 ... rests on a documentation reading
//      only ... it is one negation now and expensive later."
//
// NL2 exports its track spline as a tab-separated table of sampled frames —
// position plus front/left/up unit vectors, one row per sample. That is
// literally an FTrackFrame sequence, so reading it is a strtod loop rather than a
// parser project, and it turns both unknowns into measurements.
//
// WHAT THIS IS NOT. This is a validation and test-fixture path, NOT an
// authoring path, and it does not soften CLAUDE.md constraint #1. An imported
// track is thousands of derived micro-segments fitted to sampled data; the
// original segment vocabulary (which parts were straight, arc, clothoid) is
// gone and cannot be recovered. Nobody edits that numerically. Track authoring
// stays typed parametric segments, and this file must never grow into a
// back door around that.
//
// Units: metres, radians, seconds — same as TrackSpline.

#pragma once

#include "../TrackSpline/TrackSpline.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// ------------------------------------------------------------------ axis map

// NL2's world is right-handed with +Y up; the prototypes are right-handed with
// +Z up. This is a ROTATION (determinant +1), not a handedness flip: cross
// products survive it untouched, so NL2's `Front x Left = Up` lands exactly on
// `Tangent x Lateral = Up` with no sign correction anywhere.
//
// Do NOT copy the UE5 port rule (findings, "UE5 port checklist" item 1) into
// this file. UE needs `Right = -M(Lateral)` because UE is LEFT-handed. NL2 is
// not, and applying that rule here would mirror the whole track into geometry
// that still looks self-consistent.
//
// The docs give the world handedness but never state the triad ordering, so
// ValidateSamples asserts Front x Left == Up on the actual data rather than
// trusting the reading. If real files ever disagree, the fix is one negation
// on Lateral HERE, at the boundary — never inside the math.
inline FVec3 FromNL2(const FVec3& V) { return {V.X, -V.Z, V.Y}; }
inline FVec3 ToNL2(const FVec3& V) { return {V.X, V.Z, -V.Y}; }

// ------------------------------------------------------------------- samples

// One exported frame, already converted to prototype axes.
struct FNL2Sample
{
    FVec3 Position;
    FVec3 Front; // tangent
    FVec3 Left;  // rider's left, BANKED (NL2 exports the car frame)
    FVec3 Up;    // rider's up, banked
};

struct FNL2Result
{
    std::vector<FNL2Sample> Samples;
    std::string Error; // empty on success

    bool Ok() const { return Error.empty(); }
};

// -------------------------------------------------------------------- parsing

namespace NL2CsvDetail
{

// The docs say tab-separated; strtod skips any leading whitespace, so this also
// accepts space-separated files for free. It rejects a short row, a non-numeric
// field, a NaN/Inf spelled out as text (strtod parses those happily), and a row
// with EXTRA columns — that last one would mean the format drifted under us,
// which is worth failing on rather than silently ignoring.
inline bool ParseRow(const char* Line, double* Out)
{
    const char* P = Line;
    for (int i = 0; i < 13; ++i)
    {
        char* End = nullptr;
        const double V = std::strtod(P, &End);
        if (End == P || !std::isfinite(V))
        {
            return false;
        }
        Out[i] = V;
        P = End;
    }
    while (*P == ' ' || *P == '\t' || *P == '\r' || *P == '\n')
    {
        ++P;
    }
    return *P == '\0';
}

inline bool IsBlank(const std::string& S)
{
    for (const char C : S)
    {
        if (C != ' ' && C != '\t' && C != '\r' && C != '\n')
        {
            return false;
        }
    }
    return true;
}

} // namespace NL2CsvDetail

// Column order per the format docs:
//   No. PosX PosY PosZ FrontX FrontY FrontZ LeftX LeftY LeftZ UpX UpY UpZ
// The index column is read and discarded — arc length comes from the geometry,
// not from a row number we would have to trust.
inline FNL2Result ParseNL2Csv(const std::string& Text)
{
    FNL2Result Result;
    std::istringstream Stream(Text);
    std::string Line;
    std::size_t LineNo = 0;
    bool bPastHeader = false;

    while (std::getline(Stream, Line))
    {
        ++LineNo;
        if (NL2CsvDetail::IsBlank(Line))
        {
            continue;
        }

        double F[13];
        if (!NL2CsvDetail::ParseRow(Line.c_str(), F))
        {
            // Only the first non-blank line gets to be a header row. Any later
            // unparseable line is a real problem, not a stray label.
            if (!bPastHeader)
            {
                bPastHeader = true;
                continue;
            }
            char Msg[128];
            std::snprintf(Msg, sizeof(Msg), "line %zu: expected 13 numeric columns", LineNo);
            Result.Error = Msg;
            return Result;
        }
        bPastHeader = true;

        FNL2Sample S;
        S.Position = FromNL2({F[1], F[2], F[3]});
        S.Front = FromNL2({F[4], F[5], F[6]});
        S.Left = FromNL2({F[7], F[8], F[9]});
        S.Up = FromNL2({F[10], F[11], F[12]});
        Result.Samples.push_back(S);
    }

    if (Result.Samples.size() < 2)
    {
        Result.Error = "need at least 2 sample rows";
    }
    return Result;
}

inline FNL2Result ReadNL2CsvFile(const std::string& Path)
{
    std::ifstream In(Path.c_str());
    if (!In)
    {
        FNL2Result Result;
        Result.Error = "cannot open " + Path;
        return Result;
    }
    std::ostringstream Buffer;
    Buffer << In.rdbuf();
    return ParseNL2Csv(Buffer.str());
}

// ----------------------------------------------------------------- validation

// An imported file is external input, so it gets checked rather than absorbed.
// Returns an empty string when the samples are usable.
//
// Tolerance is loose on purpose: NL2 writes rounded decimal text, so a unit
// vector arrives a few ulps short of unit and the triad a few urad off square.
inline std::string ValidateSamples(const std::vector<FNL2Sample>& Samples, double Tolerance = 1e-3)
{
    if (Samples.size() < 2)
    {
        return "need at least 2 samples";
    }

    char Msg[192];
    for (std::size_t i = 0; i < Samples.size(); ++i)
    {
        const FNL2Sample& S = Samples[i];
        const double LenF = Length(S.Front);
        const double LenL = Length(S.Left);
        const double LenU = Length(S.Up);
        if (std::fabs(LenF - 1.0) > Tolerance || std::fabs(LenL - 1.0) > Tolerance
            || std::fabs(LenU - 1.0) > Tolerance)
        {
            std::snprintf(Msg, sizeof(Msg),
                          "sample %zu: basis vectors not unit length (%.6f, %.6f, %.6f)",
                          i, LenF, LenL, LenU);
            return Msg;
        }
        if (std::fabs(Dot(S.Front, S.Left)) > Tolerance || std::fabs(Dot(S.Front, S.Up)) > Tolerance
            || std::fabs(Dot(S.Left, S.Up)) > Tolerance)
        {
            std::snprintf(Msg, sizeof(Msg), "sample %zu: basis vectors not orthogonal", i);
            return Msg;
        }
        // The handedness check the docs do not give us. If this ever fires on a
        // genuine NL2 export, NL2's triad is ordered the other way and the fix
        // is to negate Left in FromNL2 — not to patch the reconstruction.
        if (Length(Cross(S.Front, S.Left) - S.Up) > Tolerance)
        {
            std::snprintf(Msg, sizeof(Msg),
                          "sample %zu: Front x Left != Up (file is not the expected handedness)", i);
            return Msg;
        }
    }

    // FTrack always begins at the origin with T=+X, L=+Y, U=+Z, so a
    // reconstruction is the original translated and YAWED about world up. Yaw
    // and translation leave the gravity relationship untouched, which is why
    // that is harmless — but only while the first TANGENT is horizontal. A
    // pitched start would be rotated flat, tilting the entire track relative to
    // gravity with no visible symptom, and no amount of care downstream can
    // recover it because the start frame is not ours to choose.
    //
    // A BANKED start is fine and is not rejected: roll does not move the
    // tangent, and TrackFromSamples measures the opening roll against world up
    // rather than assuming zero.
    if (std::fabs(Dot(Samples[0].Front, FVec3{0.0, 0.0, 1.0})) > Tolerance)
    {
        return "first sample is not level, and the samples are not a closed circuit with a "
               "level sample to rotate to — export a full circuit, or start the export on "
               "level track; otherwise the reconstruction would be tilted relative to gravity";
    }
    return std::string();
}

// ------------------------------------------------------- samples -> FTrack

// Signed angle from A to B measured about Axis (right-hand positive).
inline double SignedAngleAbout(const FVec3& A, const FVec3& B, const FVec3& Axis)
{
    return std::atan2(Dot(Cross(A, B), Axis), Dot(A, B));
}

// NL2 begins a circuit export wherever its spline happens to start, which is
// almost never the station — the first real export tried here opened 22 degrees
// into a drop. That is unusable as-is, because FTrack pins its own start frame
// level and would tilt the whole track relative to gravity (see ValidateSamples).
//
// On a CLOSED circuit the fix is free: rotate the sample list to begin at a
// level sample. It is the same track — only the arc-length origin moves — so
// nothing about the geometry or the physics changes. Note that S = 0 in the
// reconstruction is therefore NOT row 1 of the file, and positions are yaw
// rotated and translated besides; imported tracks are not correlatable to NL2's
// world coordinates by index, only by arc length along the reconstruction.
//
// Returns false when the data is not a closed circuit, or when no sample
// anywhere on it is level — in which case a pitched start really is unusable
// and ValidateSamples will say so.
inline bool RotateToLevelStart(std::vector<FNL2Sample>& Samples, double Tolerance = 1e-3)
{
    const std::size_t N = Samples.size();
    if (N < 3 || std::fabs(Samples[0].Front.Z) <= Tolerance)
    {
        return true; // already level, or too short to say anything about
    }

    double Sum = 0.0;
    for (std::size_t i = 0; i + 1 < N; ++i)
    {
        Sum += Length(Samples[i + 1].Position - Samples[i].Position);
    }
    const double MeanStep = Sum / (N - 1);
    if (!(MeanStep > 0.0))
    {
        return false;
    }

    // Some exports repeat the opening sample at the end to mark closure. Drop it,
    // or the rotation would splice a coincident pair into the middle.
    const bool bDuplicateEnd = Length(Samples[N - 1].Position - Samples[0].Position) < 0.01 * MeanStep;
    std::vector<FNL2Sample> Cycle(Samples.begin(), Samples.end() - (bDuplicateEnd ? 1 : 0));
    const std::size_t M = Cycle.size();
    if (M < 3)
    {
        return false;
    }

    // Closed means the last sample is about one step from the first AND heading
    // the same way. Position alone would accept a track that doubles back.
    if (Length(Cycle[M - 1].Position - Cycle[0].Position) > 2.0 * MeanStep
        || Dot(Cycle[M - 1].Front, Cycle[0].Front) < 0.99)
    {
        return false;
    }

    std::size_t Best = 0;
    double BestPitch = 2.0;
    for (std::size_t i = 0; i < M; ++i)
    {
        const double P = std::fabs(Cycle[i].Front.Z);
        if (P < BestPitch)
        {
            BestPitch = P;
            Best = i;
        }
    }
    if (BestPitch > Tolerance)
    {
        return false;
    }

    // Rotate, then repeat the opening sample at the end so the reconstruction
    // still spans the whole circuit including the wrap-around step.
    std::vector<FNL2Sample> Out;
    Out.reserve(M + 1);
    for (std::size_t i = 0; i < M; ++i)
    {
        Out.push_back(Cycle[(Best + i) % M]);
    }
    Out.push_back(Cycle[Best]);
    Samples.swap(Out);
    return true;
}

// Fit a curvature-profile track to sampled frames.
//
// Per interval: the rotation carrying Front[i] to Front[i+1] gives the turn
// angle and axis, hence the Darboux vector; the path (unbanked) frame is
// carried along by the same rotation, which is exactly the rotation-minimising
// transport FTrack::Integrate performs — so this reconstructs the model rather
// than approximating it. Roll is then whatever angle separates the transported
// path up from the file's up.
//
// Returns an empty string on success and leaves OutTrack untouched on failure.
inline std::string TrackFromSamples(const std::vector<FNL2Sample>& InSamples, FTrack& OutTrack,
                                    double Tolerance = 1e-3)
{
    // Copied so the rotation cannot surprise a caller who still wants their own
    // sample list. Trivial next to the per-sample fit that follows.
    std::vector<FNL2Sample> Samples = InSamples;
    RotateToLevelStart(Samples, Tolerance); // no-op unless the start is pitched

    const std::string Bad = ValidateSamples(Samples, Tolerance);
    if (!Bad.empty())
    {
        return Bad;
    }

    const std::size_t N = Samples.size();
    std::vector<double> ArcLen(N - 1);
    std::vector<double> IntervalYaw(N - 1);
    std::vector<double> IntervalPitch(N - 1);
    std::vector<double> Roll(N, 0.0);

    // The opening path frame is WORLD-referenced, not copied from the first
    // sample — FTrack pins its own start to PathLateral=+Y, PathUp=+Z, so
    // copying a banked sample frame here would quietly absorb the bank into the
    // path and tilt every curvature decomposition after it. ValidateSamples has
    // already guaranteed the first tangent is horizontal, so world up is a legal
    // path up, and `T x L = U` gives the lateral.
    const FVec3 WorldUp{0.0, 0.0, 1.0};
    FVec3 PathU = WorldUp;
    FVec3 PathL = Cross(WorldUp, Samples[0].Front);
    Roll[0] = -SignedAngleAbout(PathU, Samples[0].Up, Samples[0].Front);
    char Msg[128];

    for (std::size_t i = 0; i + 1 < N; ++i)
    {
        const FVec3& T0 = Samples[i].Front;
        const FVec3& T1 = Samples[i + 1].Front;

        const double Chord = Length(Samples[i + 1].Position - Samples[i].Position);
        if (!(Chord > 0.0))
        {
            std::snprintf(Msg, sizeof(Msg), "samples %zu and %zu are coincident", i, i + 1);
            return Msg;
        }

        FVec3 Axis = Cross(T0, T1);
        const double AxisLen = Length(Axis);

        // atan2 of the cross against the dot, NOT acos of the dot. Consecutive
        // tangents differ by a tiny angle and acos is ill-conditioned exactly
        // there — d(theta)/d(cos) = -1/sin(theta) amplifies the last digits of
        // the file's rounded decimal text by 1/theta. Measured on an R=25 arc
        // sampled at 0.2 m, acos left a curvature error of 4.9e-07 1/m and a
        // 3.0e-05 m endpoint error; atan2 is well conditioned at both ends,
        // costs the same, and drops the clamp acos needed to survive a dot
        // product a hair outside [-1, 1].
        const double Theta = std::atan2(AxisLen, Dot(T0, T1));

        // A chord under-measures the arc it subtends. The exact circular
        // correction is one line and removes the leading O(k^2 d^2) term, which
        // would otherwise accumulate as arc-length drift along the whole track
        // — and the physics integrates speed against arc length, so that drift
        // would land straight in the ride timing.
        const double Arc = Theta > 1e-12 ? Chord * (Theta * 0.5) / std::sin(Theta * 0.5) : Chord;
        ArcLen[i] = Arc;

        if (AxisLen > 1e-12)
        {
            Axis = Axis * (1.0 / AxisLen);
            const FVec3 Omega = Axis * (Theta / Arc);

            // Decomposed against the path basis at the START of the interval,
            // and that is EXACT rather than first-order: Omega is parallel to
            // Axis, and rotation about Axis preserves Dot(Axis, PathU) and
            // Dot(Axis, PathL). Sampling at the midpoint instead would change
            // nothing — do not "improve" this.
            // Sign convention matches Integrate's `Omega = U*Yaw - L*Pitch`.
            IntervalYaw[i] = Dot(Omega, PathU);
            IntervalPitch[i] = -Dot(Omega, PathL);

            PathL = RotateAbout(PathL, Axis, Theta);
            PathU = RotateAbout(PathU, Axis, Theta);
        }
        else
        {
            IntervalYaw[i] = 0.0;
            IntervalPitch[i] = 0.0;
        }

        // FTrack::Finish builds the rider frame as RotateAbout(PathUp, T, -Roll),
        // so recovering roll is that inversion — derived from the model, not
        // guessed at. Unwrapping to the branch nearest the previous sample keeps
        // a barrel roll continuous through +/-pi instead of snapping sign.
        const double Raw = -SignedAngleAbout(PathU, Samples[i + 1].Up, T1);
        const double TwoPi = 2.0 * 3.14159265358979323846;
        Roll[i + 1] = Raw + TwoPi * std::floor((Roll[i] - Raw) / TwoPi + 0.5);
    }

    // Measured curvature is a per-INTERVAL average, but a segment needs a value
    // at each end. Averaging the two adjacent intervals gives a continuous ramp,
    // so the reconstruction passes IsCurvatureContinuous by construction.
    //
    // ponytail: this smooths slightly — a segment's mean curvature comes out as
    // a (1,2,1)/4 kernel over the intervals rather than the interval value
    // itself, so total turn angle drifts a little at genuine discontinuities.
    // The exactly-conservative alternative is the recurrence
    // End[i] = 2*Interval[i] - Start[i], which is continuous too but has gain -1
    // per step: any noise turns into a sawtooth that never decays. Smoothing is
    // the right trade on sampled data. Revisit only if a measured round-trip
    // residual says so.
    std::vector<double> Yaw(N), Pitch(N);
    for (std::size_t i = 0; i < N; ++i)
    {
        if (i == 0)
        {
            Yaw[i] = IntervalYaw[0];
            Pitch[i] = IntervalPitch[0];
        }
        else if (i + 1 == N)
        {
            Yaw[i] = IntervalYaw[N - 2];
            Pitch[i] = IntervalPitch[N - 2];
        }
        else
        {
            Yaw[i] = 0.5 * (IntervalYaw[i - 1] + IntervalYaw[i]);
            Pitch[i] = 0.5 * (IntervalPitch[i - 1] + IntervalPitch[i]);
        }
    }

    FTrack Built;
    for (std::size_t i = 0; i + 1 < N; ++i)
    {
        FTrackSegment Seg;
        Seg.Length = ArcLen[i];
        Seg.YawCurvatureStart = Yaw[i];
        Seg.YawCurvatureEnd = Yaw[i + 1];
        Seg.PitchCurvatureStart = Pitch[i];
        Seg.PitchCurvatureEnd = Pitch[i + 1];
        Seg.RollStart = Roll[i];
        Seg.RollEnd = Roll[i + 1];
        if (!Built.AddSegment(Seg))
        {
            std::snprintf(Msg, sizeof(Msg), "segment %zu rejected (length %.9g)", i, Seg.Length);
            return Msg;
        }
    }

    OutTrack = Built;
    return std::string();
}

// Convenience: text -> FTrack in one call.
inline std::string TrackFromNL2Csv(const std::string& Text, FTrack& OutTrack)
{
    const FNL2Result Parsed = ParseNL2Csv(Text);
    if (!Parsed.Ok())
    {
        return Parsed.Error;
    }
    return TrackFromSamples(Parsed.Samples, OutTrack);
}

// ------------------------------------------------------- FTrack -> samples

// NL2's own importer only requires the up column (it re-derives front from the
// position sequence and left from up x front), but all 13 are written because
// the round-trip test needs them.
//
// Walks with AdvanceFrom rather than EvaluateAt: EvaluateAt is O(track length)
// per call, so sampling a 600 m track at 0.25 m would be quadratic.
inline std::string WriteNL2Csv(const FTrack& Track, double Spacing)
{
    std::string Out =
        "No.\tPosX\tPosY\tPosZ\tFrontX\tFrontY\tFrontZ\tLeftX\tLeftY\tLeftZ\tUpX\tUpY\tUpZ\n";

    const double Total = Track.TotalLength();
    if (!(Spacing > 0.0) || !(Total > 0.0))
    {
        return Out;
    }

    // Walk by row INDEX, not by accumulating S. Accumulating drifts, and the
    // drift lands on the last row: a final clamped step of ~1e-12 m emits a row
    // that rounds to the same decimal text as its predecessor, which a reader
    // can only see as two coincident samples. Uniform fractions of the total
    // have neither problem and put a row exactly on both ends.
    //
    // The clamp is not a trust boundary — Spacing is developer-supplied, not
    // file input. It exists so a fat-fingered value produces an absurdly large
    // file rather than an overflowed, negative row count.
    const double Wanted = std::ceil(Total / Spacing) + 1.0;
    const int Rows = Wanted < 2.0 ? 2 : (Wanted > 4.0e6 ? 4000000 : static_cast<int>(Wanted));

    FTrackFrame F = Track.EvaluateAt(0.0);
    double S = 0.0;
    char Row[512];

    for (int Index = 0; Index < Rows; ++Index)
    {
        const double Next = Total * Index / (Rows - 1);
        if (Next > S)
        {
            F = Track.AdvanceFrom(F, S, Next);
            S = Next;
        }

        const FVec3 P = ToNL2(F.Position);
        const FVec3 Fr = ToNL2(F.Tangent);
        const FVec3 Le = ToNL2(F.Lateral);
        const FVec3 Up = ToNL2(F.Up);
        std::snprintf(Row, sizeof(Row),
                      "%d\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\t%.9f\n",
                      // "The first data point starts with 1" — the docs are
                      // explicit, and NL2 is the one reading this back.
                      Index + 1, P.X, P.Y, P.Z, Fr.X, Fr.Y, Fr.Z, Le.X, Le.Y, Le.Z, Up.X, Up.Y,
                      Up.Z);
        Out += Row;
    }
    return Out;
}

inline bool WriteNL2CsvFile(const FTrack& Track, double Spacing, const std::string& Path)
{
    std::ofstream Out(Path.c_str());
    if (!Out)
    {
        return false;
    }
    Out << WriteNL2Csv(Track, Spacing);
    return Out.good();
}

// ponytail: which line does NL2 export — the heartline or the rail centreline?
// The format docs do not say, and it changes nothing for a round-trip through
// OUR writer (same line in, same line out) but it does change what an imported
// NL2 track means: read the rail centreline as a heartline and every banked
// turn's felt G is subtly wrong. Settle it from the NL2 export dialog before
// quoting any G number taken from an imported file, and record the answer in
// Docs/PHASE0_FINDINGS.md.
