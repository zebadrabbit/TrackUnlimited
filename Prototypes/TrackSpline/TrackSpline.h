// TrackUnlimited Phase 0 prototype: curvature-continuous track spline math.
// Plain C++, no engine dependencies — designed to port into UE5 C++ later.
// Design reference: Docs/PROJECT_PLAN.md Section 5, "Track representation".
//
// The track is an ordered list of segments, each defined as a *curvature
// profile over arc length* rather than as control points. Geometry comes from
// integrating a moving frame along the curve.
//
// This is why: C2 continuity becomes a property of the data instead of
// something fitted after the fact. If curvature is continuous across a joint,
// the geometry is — no solving, no post-hoc smoothing. A clothoid (curvature
// linear in s) is exactly what makes a continuous joint between a straight
// (k = 0) and an arc (k = 1/R) possible, which is why real track alignment is
// designed this way and why NL2's track feels smooth where a naive
// position-only spline does not.
//
// Roll (banking) is carried on the same curvilinear parameter but integrated
// separately: banking rotates the rails *around* the heartline and must not
// perturb the path. The integrated curve IS the heartline; rail centreline is
// derived from it (see FTrack::RailCentreAt).
//
// Units: metres, radians, seconds. UE uses centimetres — convert at the port
// boundary, not in here.
// Frame: right-handed, Tangent x Lateral = Up. +Lateral is the rider's left.

#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

// ---------------------------------------------------------------- vector math

struct FVec3
{
    double X = 0.0, Y = 0.0, Z = 0.0;
};

inline FVec3 operator+(const FVec3& A, const FVec3& B) { return {A.X + B.X, A.Y + B.Y, A.Z + B.Z}; }
inline FVec3 operator-(const FVec3& A, const FVec3& B) { return {A.X - B.X, A.Y - B.Y, A.Z - B.Z}; }
inline FVec3 operator*(const FVec3& A, double S) { return {A.X * S, A.Y * S, A.Z * S}; }
inline double Dot(const FVec3& A, const FVec3& B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
inline FVec3 Cross(const FVec3& A, const FVec3& B)
{
    return {A.Y * B.Z - A.Z * B.Y, A.Z * B.X - A.X * B.Z, A.X * B.Y - A.Y * B.X};
}
inline double Length(const FVec3& A) { return std::sqrt(Dot(A, A)); }

// Rotate V about a unit axis by Angle radians (Rodrigues).
inline FVec3 RotateAbout(const FVec3& V, const FVec3& UnitAxis, double Angle)
{
    const double C = std::cos(Angle);
    const double S = std::sin(Angle);
    return V * C + Cross(UnitAxis, V) * S + UnitAxis * (Dot(UnitAxis, V) * (1.0 - C));
}

// ------------------------------------------------------------------- segments

// What a roll value is measured FROM.
//
// This was Docs/DEFERRED_DECISIONS.md item 1, and the answer is "both, per
// segment", because neither is defined everywhere a coaster goes.
enum class ERollMode
{
    // Roll relative to the rotation-minimising path frame. Roll 0 means "no
    // twist beyond whatever the track's own shape carried here" — which on
    // non-planar track is NOT level with the horizon. It is well defined
    // everywhere, including vertical and inverted track, so it stays the
    // default and it is the only thing the integrator ever sees.
    PathRelative,

    // Roll relative to the horizon: the value is the bank angle a spirit level
    // would read. This is what an author means by "bank this turn 30 degrees".
    //
    // Resolved per sample against the walked frame, NOT baked in when it is
    // typed — so editing an upstream hill cannot silently change a downstream
    // bank. The stored number keeps meaning what the author said.
    //
    // UNDEFINED where the tangent is vertical: there is no horizontal direction
    // perpendicular to straight up, so there is nothing to be level with. Near
    // vertical the reference swings fast and the bank costs a real, large roll
    // rate — see AnalyseResolvedRollRate. Author inversions as PathRelative.
    WorldBank,
};

// One parametric segment. The curvature VECTOR is what varies over arc length:
// its magnitude ramps linearly, and it may additionally rotate about the
// tangent at a constant rate. That covers the whole authored vocabulary:
//   straight  — yaw = pitch = 0
//   arc       — yaw constant (= 1/R)
//   clothoid  — yaw linear from one curvature to another
//   helix     — yaw constant, plus a constant Torsion
// Roll likewise varies linearly across the segment.
//
// ponytail: linear magnitude ramps and constant torsion only. Between them they
// cover every segment type in the Phase 1 editor vocabulary; add a cubic (or
// arbitrary) profile when an authoring case actually needs one.
struct FTrackSegment
{
    double Length = 0.0;

    // 1/m. +ve yaw turns left; +ve pitch raises the nose. These are the values
    // BEFORE the torsion rotation below — read them as the curvature vector at
    // the segment's own start, not as what a rider meets partway through.
    double YawCurvatureStart = 0.0;
    double YawCurvatureEnd = 0.0;
    double PitchCurvatureStart = 0.0;
    double PitchCurvatureEnd = 0.0;

    // 1/m. Rate at which the curvature vector rotates about the tangent as arc
    // length advances. This is what makes a helix expressible, and it is the
    // representation gap PHASE0_FINDINGS.md recorded as disproved.
    //
    // The path frame is rotation-minimising, and the Frenet frame turns about
    // the tangent at exactly the torsion rate relative to it. So a curve of
    // constant curvature and constant torsion — a helix — appears in THIS frame
    // as `Yaw = k*cos(tau*s)`, `Pitch = k*sin(tau*s)`. One extra field buys
    // that; a general non-linear profile is not needed.
    //
    // Zero for straight, arc and clothoid, which leaves them bit-identical to
    // the pre-torsion behaviour.
    double Torsion = 0.0;

    // Radians. +ve banks *into* a +ve (left) turn: the rider's up-vector tilts
    // toward the centre of the turn, the way a motorcycle leans. What these are
    // measured from is RollMode's job.
    double RollStart = 0.0;
    double RollEnd = 0.0;
    ERollMode RollMode = ERollMode::PathRelative;
};

// The curvature vector at local arc length U along a segment: linear ramp
// first, then rotated about the tangent by Torsion*U.
//
// Rotating about the tangent takes Lateral toward Up (right-hand rule, since
// Tangent x Lateral = Up), which is why the yaw component leaks into pitch with
// this sign and not the other. Getting it backwards produces a mirror-image
// helix that still looks like a helix.
inline void CurvatureAt(const FTrackSegment& Seg, double U, double& OutYaw, double& OutPitch)
{
    const double A = Seg.Length > 0.0 ? U / Seg.Length : 0.0;
    const double Yaw = Seg.YawCurvatureStart + (Seg.YawCurvatureEnd - Seg.YawCurvatureStart) * A;
    const double Pitch =
        Seg.PitchCurvatureStart + (Seg.PitchCurvatureEnd - Seg.PitchCurvatureStart) * A;

    if (Seg.Torsion == 0.0)
    {
        OutYaw = Yaw;
        OutPitch = Pitch;
        return;
    }
    const double Phi = Seg.Torsion * U;
    const double C = std::cos(Phi);
    const double S = std::sin(Phi);
    OutYaw = Yaw * C - Pitch * S;
    OutPitch = Yaw * S + Pitch * C;
}

// Hand the curvature vector from one segment to the next, so a RUN of torsioned
// segments chains continuously instead of stepping at every joint.
//
// This is needed because torsion's phase is measured from each segment's own
// start: a segment exits with its curvature vector rotated by Torsion*Length,
// and the next one begins its rotation again from zero. Author three torsioned
// segments in a row without this and every joint steps — the geometry looks
// plausible and `IsCurvatureContinuous` correctly says no.
//
// Sets B's START only. What B's END should be is the author's intent and not
// something to guess: equal to the start for constant magnitude, or zero to ramp
// out. Both in one run: the side-stepping loop in test_trackspline.cpp.
//
// NOTHING IN THE BUILD PATH CALLS THIS. It is reachable from prototype C++ only —
// BuildSegment maps one authored row to one segment with no neighbour awareness,
// so a chained value can only reach a document by being typed into a Raw segment,
// which stores a DERIVED number and goes stale the moment the neighbour is
// edited. Measured: chain a helix exit by hand, then change the helix radius
// 20 m -> 22 m, and the joint reopens to 4.523e-03 1/m with the file still
// claiming to be valid. Wiring this into the build path is a real change with a
// save-format question attached, not a tidy-up — see Docs/PHASE0_FINDINGS.md,
// "The helix composes, but only from C++".
inline void ChainCurvature(const FTrackSegment& A, FTrackSegment& B)
{
    CurvatureAt(A, A.Length, B.YawCurvatureStart, B.PitchCurvatureStart);
}

inline FTrackSegment MakeStraight(double InLength, double Roll = 0.0)
{
    FTrackSegment Seg;
    Seg.Length = InLength;
    Seg.RollStart = Seg.RollEnd = Roll;
    return Seg;
}

// Radius +ve = left turn, -ve = right turn.
inline FTrackSegment MakeArc(double InLength, double Radius, double Roll = 0.0)
{
    FTrackSegment Seg = MakeStraight(InLength, Roll);
    Seg.YawCurvatureStart = Seg.YawCurvatureEnd = 1.0 / Radius;
    return Seg;
}

// The transition curve. Curvature ramps linearly, so it joins a straight to an
// arc (or two arcs) with no curvature step — the whole point of the exercise.
inline FTrackSegment MakeClothoid(double InLength, double CurvatureStart, double CurvatureEnd,
                                  double RollStart = 0.0, double RollEnd = 0.0)
{
    FTrackSegment Seg;
    Seg.Length = InLength;
    Seg.YawCurvatureStart = CurvatureStart;
    Seg.YawCurvatureEnd = CurvatureEnd;
    Seg.RollStart = RollStart;
    Seg.RollEnd = RollEnd;
    return Seg;
}

// A true helix — constant curvature AND constant torsion — from the parameters
// anyone actually authors. Radius +ve = left, ClimbAngle +ve = ascending, Turns
// is revolutions about the axis.
//
// For a helix of radius R climbing at angle a:
//   curvature = cos^2(a) / R,  torsion = sin(a)cos(a) / R,  length = 2*pi*R/cos(a) per turn.
//
// IMPORTANT — the axis is inherited, not specified. This segment produces a
// curve of constant curvature and torsion, which is a helix, but its axis is
// whatever the incoming frame implies. **The track must already be pitched at
// ClimbAngle when this segment starts**, or the axis comes out tilted. Real
// track needs that transition anyway. See Docs/DEFERRED_DECISIONS.md item 2 on
// whether the editor should insert it for you.
//
// Note also that the curvature vector has ROTATED by Torsion*Length by the exit
// (2*pi*sin(a) per turn), so a helix does not generally end on pure yaw. That
// is not an error — it is the geometry — but it means the exit transition has
// to match a curvature vector with a pitch component in it.
inline FTrackSegment MakeHelix(double Radius, double ClimbAngle, double Turns, double Roll = 0.0)
{
    const double Pi = 3.14159265358979323846;
    const double C = std::cos(ClimbAngle);

    FTrackSegment Seg;
    Seg.Length = Turns * 2.0 * Pi * std::fabs(Radius) / C;
    Seg.YawCurvatureStart = Seg.YawCurvatureEnd = (C * C) / Radius;
    Seg.Torsion = std::sin(ClimbAngle) * C / Radius;
    Seg.RollStart = Seg.RollEnd = Roll;
    return Seg;
}

// ---------------------------------------------------------------------- frame

// Two bases, and the difference between them is the whole heartline idea:
// the path frame is what the track *does*, the rider frame is what the rider
// *feels*. Banking rotates the second around the first.
struct FTrackFrame
{
    FVec3 Position;    // heartline position
    FVec3 Tangent;     // shared by both bases — roll is about this axis
    FVec3 Lateral;     // rider's left, banked
    FVec3 Up;          // rider's up, banked
    FVec3 PathLateral; // unbanked left — the direction the path actually bends
    FVec3 PathUp;      // unbanked up
    // Always PathRelative, whatever mode the segment authored it in — a
    // WorldBank segment is resolved before it lands here. Everything
    // downstream (FeltG, the rail cross-section, the ride camera) therefore
    // needs to know nothing about roll modes.
    double Roll = 0.0;
    double YawCurvature = 0.0;
    double PitchCurvature = 0.0;
};

constexpr FVec3 WorldUpAxis{0.0, 0.0, 1.0};

// How far the path frame has already twisted away from level at this point —
// equivalently, the PathRelative roll that would put the rider level with the
// horizon. Zero on flat ground; it is the accumulated holonomy elsewhere.
//
// This is the whole of ERollMode::WorldBank. The rider's lateral ends up at
// (Psi - Roll) about the tangent from the level reference, so asking for a bank
// of B means resolving to Roll = Psi + B.
//
// Returns 0 where the tangent is vertical — there is no horizontal direction
// perpendicular to straight up, so there is no reference to be level with. That
// keeps the geometry finite and continuous rather than producing a NaN, and it
// degrades to PathRelative for that instant. It is not a fix: a WorldBank
// segment that goes near vertical is authored wrong, and the symptom is a large
// roll rate, which AnalyseResolvedRollRate reports.
inline double LevelRollOffset(const FVec3& Tangent, const FVec3& PathLateral)
{
    const FVec3 Ref = Cross(WorldUpAxis, Tangent);
    const double RefLen = Length(Ref);
    if (RefLen < 1e-12)
    {
        return 0.0;
    }
    const FVec3 LevelLateral = Ref * (1.0 / RefLen);
    // Both arguments are perpendicular to the tangent, so their cross product
    // is exactly parallel to it and this atan2 is the exact signed angle.
    return std::atan2(Dot(Cross(LevelLateral, PathLateral), Tangent),
                      Dot(LevelLateral, PathLateral));
}

// The bank a spirit level would read at this frame: 0 is level, +ve leans the
// rider toward their left, matching the sign of RollStart/RollEnd. The inverse
// of what WorldBank authoring asks for, and how a test checks it.
inline double WorldBankOf(const FTrackFrame& F)
{
    return -LevelRollOffset(F.Tangent, F.Lateral);
}

// -------------------------------------------------------------------- gravity

constexpr double GravityMs2 = 9.80665;

// Felt acceleration in G, as the rider experiences it, at a given frame and
// speed. Tangential (accelerating/braking) force is the physics prototype's
// job; this is the geometric part — the bit banking has to get right.
// Signs follow one rule on both axes: the value is the felt vector resolved
// onto a rider axis, and the rider is pressed the *opposite* way. Vertical +1
// is sitting on level track, pressed down into the seat; Lateral +1 likewise
// presses the rider toward their right.
struct FGForces
{
    double Lateral = 0.0;
    double Vertical = 0.0;
};

inline FGForces FeltG(const FTrackFrame& F, double SpeedMs)
{
    const double V2 = SpeedMs * SpeedMs;
    // The path bends along the *unbanked* axes — banking rotates the rider, not
    // the trajectory. Resolving onto the banked axes instead is the classic way
    // to get a banked turn that never cancels its own lateral G.
    const FVec3 PathAccel = F.PathLateral * (V2 * F.YawCurvature) + F.PathUp * (V2 * F.PitchCurvature);
    const FVec3 Felt = PathAccel + FVec3{0.0, 0.0, GravityMs2};
    return {Dot(Felt, F.Lateral) / GravityMs2, Dot(Felt, F.Up) / GravityMs2};
}

// ---------------------------------------------------------------------- track

class FTrack
{
public:
    // Height of the heartline above the rail centreline. ~1.1 m puts it at
    // roughly rider chest height for a sit-down train.
    explicit FTrack(double InHeartlineHeight = 1.1)
        : HeartlineHeight(InHeartlineHeight)
    {
    }

    // Rejects degenerate segments rather than storing them; returns false if the
    // segment was refused. This is the one shared entry point, so guarding here
    // covers every way a bad segment could get in. A zero-length segment whose
    // endpoint curvatures happen to match its neighbours would otherwise satisfy
    // IsCurvatureContinuous at both joints while bridging a real discontinuity,
    // and a negative length silently desynchronises S from arc length. The
    // comparison form also rejects NaN.
    bool AddSegment(const FTrackSegment& Seg)
    {
        if (!(Seg.Length > 0.0))
        {
            return false;
        }
        Segments.push_back(Seg);
        return true;
    }

    std::size_t NumSegments() const { return Segments.size(); }

    // Exposed so a cross-section can be placed against the same reference line
    // RailCentreAt uses, instead of a caller keeping its own copy that can drift.
    double GetHeartlineHeight() const { return HeartlineHeight; }

    double TotalLength() const
    {
        double Sum = 0.0;
        for (const FTrackSegment& Seg : Segments)
        {
            Sum += Seg.Length;
        }
        return Sum;
    }

    // Evaluate the heartline frame at arc length S (clamped to the track).
    //
    // At an exact joint the ENDING segment supplies roll and curvature. That
    // only matters on curvature-discontinuous data, and it is not reliable in
    // floating point anyway — with authored lengths that do not sum exactly,
    // roughly a quarter of joint queries land on the following segment instead.
    // Do not depend on which side you get; sample either side of a joint.
    //
    // Use the returned Tangent for direction; do NOT finite-difference Position
    // to recover it. Steps = ceil(Span/MaxStep) re-discretises per call, so
    // position is not smooth in S below ~1e-9 m and a difference quotient stops
    // converging under h ~ 1e-4. The Tangent is exact to 1e-15 rad — free and
    // better.
    //
    // ponytail: re-integrates from the start on every call — O(track length),
    // about 0.2 ms on a 1 km track. Fine for tests and offline analysis; build a
    // cached arc-length sample table when the editor needs interactive scrubbing
    // or the physics tick needs this every frame.
    FTrackFrame EvaluateAt(double S) const
    {
        const double Total = TotalLength();
        S = S < 0.0 ? 0.0 : (S > Total ? Total : S);

        // Unrolled path frame: banking must not perturb the path, so roll is
        // applied only at the end, as a rotation about the tangent.
        FVec3 P{0.0, 0.0, 0.0};
        FVec3 T{1.0, 0.0, 0.0};
        FVec3 L{0.0, 1.0, 0.0};
        FVec3 U{0.0, 0.0, 1.0};

        double Remaining = S;
        std::size_t Index = 0;
        double Local = 0.0;
        for (std::size_t i = 0; i < Segments.size(); ++i)
        {
            Index = i;
            const double Span = Remaining < Segments[i].Length ? Remaining : Segments[i].Length;
            Integrate(Segments[i], 0.0, Span, P, T, L, U);
            Local = Span;
            Remaining -= Span;
            if (Remaining <= 0.0)
            {
                break;
            }
        }

        return Finish(Index, Local, P, T, L, U);
    }

    // Continue an already-evaluated frame forward to ToS, instead of starting
    // over at the track beginning. Cost is O(ToS - FromS), not O(ToS).
    //
    // This is the fast path a ticking train wants, and the reason it exists is
    // measured: a 425 m layout is ~42,500 integrator steps per EvaluateAt, so
    // one call per frame at 60 Hz does not fit in a frame. Advancing 0.33 m —
    // one tick at 20 m/s — is about 33 steps instead.
    //
    // `From` must be the frame this track produced at FromS; its PathLateral
    // and PathUp carry the unrolled basis the integrator needs. The result is
    // not bit-identical to EvaluateAt(ToS), because the steps land in different
    // places, but both are the same second-order integration of the same curve
    // and the difference is far below the 0.027 mm/km accumulation either way.
    FTrackFrame AdvanceFrom(const FTrackFrame& From, double FromS, double ToS) const
    {
        const double Total = TotalLength();
        const double Target = ToS < 0.0 ? 0.0 : (ToS > Total ? Total : ToS);
        if (Target == FromS)
        {
            // Already there — `From` IS the answer. Worth special-casing: a
            // stalled train asks this every tick, and falling through to a full
            // evaluation would make standing still the most expensive thing the
            // simulation does.
            return From;
        }
        if (Target < FromS || Segments.empty())
        {
            return EvaluateAt(Target);
        }

        FVec3 P = From.Position;
        FVec3 T = From.Tangent;
        FVec3 L = From.PathLateral;
        FVec3 U = From.PathUp;

        std::size_t Index = 0;
        double Local = 0.0;
        Locate(FromS, Index, Local);

        double Remaining = Target - FromS;
        while (Remaining > 0.0 && Index < Segments.size())
        {
            const FTrackSegment& Seg = Segments[Index];
            const double Span = std::min(Remaining, Seg.Length - Local);
            if (Span > 0.0)
            {
                Integrate(Seg, Local, Local + Span, P, T, L, U);
                Local += Span;
                Remaining -= Span;
            }
            if (Remaining <= 0.0)
            {
                break;
            }
            ++Index;
            Local = 0.0;
        }
        if (Index >= Segments.size())
        {
            Index = Segments.size() - 1;
            Local = Segments[Index].Length;
        }

        return Finish(Index, Local, P, T, L, U);
    }

    // The rails hang below the heartline; banking swings them around it, which
    // is the entire reason the heartline model exists.
    FVec3 RailCentreAt(double S) const
    {
        const FTrackFrame F = EvaluateAt(S);
        return F.Position - F.Up * HeartlineHeight;
    }

    // Are all segment joints free of curvature (and roll) steps? A step in
    // curvature is a step in lateral G — the jolt clothoids exist to remove.
    //
    // ponytail: checks curvature and roll value, not roll *rate*. A roll-rate
    // step is a jerk rather than a jolt — worth flagging once the physics
    // prototype can measure whether it's felt.
    bool IsCurvatureContinuous(double Tolerance = 1e-9) const
    {
        for (std::size_t i = 0; i + 1 < Segments.size(); ++i)
        {
            const FTrackSegment& A = Segments[i];
            const FTrackSegment& B = Segments[i + 1];

            // Compared through CurvatureAt, not against the raw End fields. A
            // segment with torsion exits with its curvature vector ROTATED by
            // Torsion*Length, so the raw fields describe a joint that is not
            // the one a rider crosses. Reading them directly would call a
            // genuinely stepped helix exit continuous.
            double AYaw = 0.0, APitch = 0.0, BYaw = 0.0, BPitch = 0.0;
            CurvatureAt(A, A.Length, AYaw, APitch);
            CurvatureAt(B, 0.0, BYaw, BPitch);

            if (std::fabs(AYaw - BYaw) > Tolerance || std::fabs(APitch - BPitch) > Tolerance)
            {
                return false;
            }

            // Roll values are only comparable when both sides measure from the
            // same thing. Across a mode change the two numbers are in different
            // units of meaning, and whether the resolved roll actually steps
            // depends on the frame at the joint — which this data-only check
            // does not have. Refusing is the conservative answer: it can cry
            // wolf on a joint that happens to be level, and the alternative is
            // reporting a real instantaneous twist as continuous.
            if (A.RollMode != B.RollMode)
            {
                return false;
            }
            if (std::fabs(A.RollEnd - B.RollStart) > Tolerance)
            {
                return false;
            }
        }
        return true;
    }

private:
    static double Lerp(double A, double B, double T) { return A + (B - A) * T; }

    // Which segment owns arc length S, and how far into it. At an exact joint
    // the ENDING segment wins, matching EvaluateAt.
    void Locate(double S, std::size_t& OutIndex, double& OutLocal) const
    {
        double Acc = 0.0;
        for (std::size_t i = 0; i < Segments.size(); ++i)
        {
            if (S <= Acc + Segments[i].Length || i + 1 == Segments.size())
            {
                OutIndex = i;
                OutLocal = S - Acc;
                return;
            }
            Acc += Segments[i].Length;
        }
        OutIndex = 0;
        OutLocal = 0.0;
    }

    // Turn an integrated path frame into the rider frame. Shared by EvaluateAt
    // and AdvanceFrom so the roll convention can only ever be defined once.
    FTrackFrame Finish(std::size_t Index, double Local,
                  const FVec3& P, const FVec3& T, const FVec3& L, const FVec3& U) const
    {
        FTrackFrame Out;
        Out.Position = P;
        Out.Tangent = T;
        if (!Segments.empty())
        {
            const FTrackSegment& Seg = Segments[Index];
            const double A = Seg.Length > 0.0 ? Local / Seg.Length : 0.0;
            Out.Roll = Lerp(Seg.RollStart, Seg.RollEnd, A);
            if (Seg.RollMode == ERollMode::WorldBank)
            {
                // Solved here, against the frame we just walked to, rather than
                // baked in at authoring time. That is the whole point: roll is
                // the one authored quantity whose meaning depends on everything
                // upstream of it, so resolving it late is what stops an edit to
                // a hill from silently unbanking a turn a hundred metres later.
                Out.Roll += LevelRollOffset(T, L);
            }
            // Through CurvatureAt, so a helix reports the curvature actually
            // acting here rather than its unrotated start value. FeltG reads
            // these two against PathLateral and PathUp, so getting it wrong
            // would misreport G everywhere torsion is non-zero.
            CurvatureAt(Seg, Local, Out.YawCurvature, Out.PitchCurvature);
        }
        Out.PathLateral = L;
        Out.PathUp = U;
        // Negated: a right-hand rotation about the tangent drops the rider's
        // right side, so the sign is flipped to make +ve roll bank *into* a
        // +ve (left) turn, which is what an author means by "bank left".
        Out.Lateral = RotateAbout(L, T, -Out.Roll);
        Out.Up = RotateAbout(U, T, -Out.Roll);
        return Out;
    }

    // Exponential-midpoint integration: half-rotate the frame, translate along
    // the tangent, half-rotate again. Symmetric, second order, and keeps the
    // frame orthonormal by construction (it only ever applies rotations),
    // unlike a componentwise RK4 which drifts and needs re-orthonormalising.
    // Integrates the sub-range [From, To] of a segment's own arc length, so
    // that a partial advance samples the curvature ramp at the right place.
    static void Integrate(const FTrackSegment& Seg, double From, double To,
                          FVec3& P, FVec3& T, FVec3& L, FVec3& U)
    {
        const double Span = To - From;
        if (Span <= 0.0)
        {
            return;
        }
        const double MaxStep = 0.01; // metres
        int Steps = static_cast<int>(std::ceil(Span / MaxStep));
        if (Steps < 1)
        {
            Steps = 1;
        }
        const double Ds = Span / Steps;

        for (int i = 0; i < Steps; ++i)
        {
            const double Mid = From + (i + 0.5) * Ds;
            double Yaw = 0.0, Pitch = 0.0;
            CurvatureAt(Seg, Mid, Yaw, Pitch);

            // Darboux vector, no tangential (roll) component: roll is applied
            // at evaluation time so it cannot bend the path.
            const FVec3 Omega = U * Yaw - L * Pitch;
            const double Rate = Length(Omega);

            if (Rate > 0.0)
            {
                const FVec3 Axis = Omega * (1.0 / Rate);
                const double Half = Rate * Ds * 0.5;
                T = RotateAbout(T, Axis, Half);
                L = RotateAbout(L, Axis, Half);
                U = RotateAbout(U, Axis, Half);
                P = P + T * Ds;
                T = RotateAbout(T, Axis, Half);
                L = RotateAbout(L, Axis, Half);
                U = RotateAbout(U, Axis, Half);
            }
            else
            {
                P = P + T * Ds;
            }
        }
    }

    // The helix used to be a representation gap. It is not any more — see
    // MakeHelix and FTrackSegment::Torsion — but the trap that made it one is
    // still worth knowing, because it is the obvious way to author a helix and
    // it silently produces the wrong curve.
    //
    // "Constant radius + constant climb" is NOT constant pitch curvature: it is
    // zero pitch curvature at a fixed pitch angle, yawing about the *world*
    // vertical rather than the body up-axis. Authored that way you get a tilted
    // planar circle that returns to its starting height after a full turn
    // (measured out-of-plane deviation 1.9e-12 m over 300 m — genuinely flat,
    // not a bad helix). A real helix needs the curvature vector to ROTATE about
    // the tangent at the torsion rate, which is what Torsion does.

    double HeartlineHeight;
    std::vector<FTrackSegment> Segments;
};
