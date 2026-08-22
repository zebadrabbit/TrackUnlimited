// TrackUnlimited Phase 3.5: the runtime camera.
// Plain C++17, no engine dependency.
//
// Today the viewport is Unreal's editor viewport, which does not exist in a
// packaged build. All of it has to be built — and the parts worth testing are
// arithmetic rather than input handling, so they live here and the UE layer is
// thin.
//
// ===================== WHAT ACTUALLY GOES WRONG =====================
//
// FRAMING CHECKS ONE AXIS. The distance that fits a bounding box vertically is
// not the distance that fits it horizontally, and which one binds depends on the
// aspect ratio and the shape of the thing. Check only the vertical and a long
// low layout — which is every out-and-back ever built — hangs off both sides of
// the screen. Check only the horizontal and a lift hill goes off the top.
//
// THE NEAR PLANE. This project needs a 90 m lift hill and a 2 cm bolt on a rail
// in the same scene. A near plane set for the bolt destroys depth precision at
// the top of the hill; one set for the hill clips your own hands off the
// restraint. The ratio far/near is what decides depth precision, and it is a
// number worth computing rather than guessing.
//
// PITCH WRAPS. An orbit camera that reaches straight down and keeps going flips
// the world over, and every user who has done it accidentally never trusts the
// camera again.
//
// Units: metres and DEGREES — degrees because this is the layer a person types
// into, and the same rule the authored track data follows.

#pragma once

#include <cmath>
#include <cstddef>

struct FCamVec
{
    double X = 0.0, Y = 0.0, Z = 0.0;
};

inline FCamVec operator+(const FCamVec& A, const FCamVec& B) { return {A.X + B.X, A.Y + B.Y, A.Z + B.Z}; }
inline FCamVec operator-(const FCamVec& A, const FCamVec& B) { return {A.X - B.X, A.Y - B.Y, A.Z - B.Z}; }
inline FCamVec operator*(const FCamVec& A, double S) { return {A.X * S, A.Y * S, A.Z * S}; }
inline double CamLength(const FCamVec& A) { return std::sqrt(A.X * A.X + A.Y * A.Y + A.Z * A.Z); }

// An axis-aligned box, which is all a framing calculation needs. Built by
// sweeping the track's own walked frames, so it costs nothing extra.
struct FCamBounds
{
    FCamVec Min{1e30, 1e30, 1e30};
    FCamVec Max{-1e30, -1e30, -1e30};

    void Add(const FCamVec& P)
    {
        Min.X = P.X < Min.X ? P.X : Min.X;
        Min.Y = P.Y < Min.Y ? P.Y : Min.Y;
        Min.Z = P.Z < Min.Z ? P.Z : Min.Z;
        Max.X = P.X > Max.X ? P.X : Max.X;
        Max.Y = P.Y > Max.Y ? P.Y : Max.Y;
        Max.Z = P.Z > Max.Z ? P.Z : Max.Z;
    }
    bool IsValid() const { return Max.X >= Min.X; }
    FCamVec Centre() const { return (Min + Max) * 0.5; }
    FCamVec Extent() const { return (Max - Min) * 0.5; }

    // The radius of the sphere that contains it. Used for framing because it is
    // ROTATION-INDEPENDENT: frame by the box's own extents and the layout goes
    // off screen the moment you orbit, which is a bug that only shows up when
    // somebody moves the camera and is therefore reported as "the camera is
    // broken" rather than "framing is wrong".
    double Radius() const
    {
        return IsValid() ? CamLength(Extent()) : 0.0;
    }
};

constexpr double CamPi = 3.14159265358979323846;
inline double CamRad(double Deg) { return Deg * CamPi / 180.0; }

// ===================== FRAMING =====================
//
// The distance at which a sphere of this radius fills the view, checking BOTH
// axes and taking the larger.
//
// `FovDegrees` is the HORIZONTAL field of view, because that is what Unreal's
// camera component takes and converting at the boundary rather than here is the
// same rule as every other unit in this project.
inline double DistanceToFrame(double Radius, double FovDegrees, double AspectWidthOverHeight,
                              double Margin = 1.1)
{
    if (!(Radius > 0.0) || !(FovDegrees > 0.0) || !(AspectWidthOverHeight > 0.0))
    {
        return 0.0;
    }
    const double HalfH = CamRad(FovDegrees) * 0.5;
    // The vertical half-angle is NOT the horizontal one. Deriving it is one line
    // and skipping it is the bug: on a 21:9 monitor the vertical field is barely
    // half the horizontal, so a layout framed by the horizontal alone loses its
    // lift hill off the top.
    const double HalfV = std::atan(std::tan(HalfH) / AspectWidthOverHeight);
    const double ByH = Radius / std::sin(HalfH);
    const double ByV = Radius / std::sin(HalfV);
    return (ByH > ByV ? ByH : ByV) * Margin;
}

// ===================== NEAR AND FAR =====================
//
// A 90 m lift hill and a 2 cm bolt in one scene. The ratio far/near is what
// decides depth precision, so it is computed rather than guessed — and REPORTED,
// because a ratio this layer cannot fix is a ratio somebody should know about.
struct FDepthRange
{
    double Near = 0.0;
    double Far = 0.0;
    double Ratio() const { return Near > 0.0 ? Far / Near : 0.0; }

    // Above this, a 24-bit depth buffer starts z-fighting on coplanar surfaces at
    // distance — which on a coaster is rails against their own ties.
    bool IsPrecisionRisky() const { return Ratio() > 100000.0; }
};

inline FDepthRange DepthRangeFor(double DistanceToSubject, double SceneRadius)
{
    FDepthRange R;
    // The near plane FOLLOWS THE CAMERA rather than being a constant. Standing
    // off a whole layout it can be metres; sitting in a seat it has to be
    // centimetres, or the restraint in front of the rider is clipped away. A
    // fixed value cannot serve both and picking one is picking which view to
    // break.
    R.Near = DistanceToSubject * 0.002;
    if (R.Near < 0.02) { R.Near = 0.02; }     // 2 cm: the bolt
    if (R.Near > 5.0)  { R.Near = 5.0; }
    R.Far = (DistanceToSubject + SceneRadius) * 4.0;
    if (R.Far < 1000.0) { R.Far = 1000.0; }
    return R;
}

// ===================== THE ORBIT =====================

struct FOrbitState
{
    FCamVec Focus;
    double Distance = 50.0;
    double YawDeg = -45.0;
    double PitchDeg = -25.0;   // negative looks down, matching Unreal

    // PITCH IS CLAMPED, NOT WRAPPED. An orbit camera that reaches straight down
    // and keeps going flips the world over, and everybody who has done that
    // accidentally stops trusting the camera. Just short of vertical, because
    // exactly vertical is where the up vector becomes undefined.
    static constexpr double PitchLimit = 89.0;

    void AddYaw(double D) { YawDeg = Wrap(YawDeg + D); }
    void AddPitch(double D)
    {
        PitchDeg += D;
        if (PitchDeg > PitchLimit) { PitchDeg = PitchLimit; }
        if (PitchDeg < -PitchLimit) { PitchDeg = -PitchLimit; }
    }

    // Zoom is MULTIPLICATIVE. Additive zoom crawls when you are far away and
    // slams into the subject when you are close — the same wheel notch has to
    // mean the same proportion of the distance, or the control is unusable at one
    // end of its range.
    void Zoom(double Factor, double MinDistance = 1.0, double MaxDistance = 20000.0)
    {
        if (!(Factor > 0.0)) { return; }
        Distance *= Factor;
        if (Distance < MinDistance) { Distance = MinDistance; }
        if (Distance > MaxDistance) { Distance = MaxDistance; }
    }

    // Panning moves the FOCUS, in the camera's own plane, and scales with
    // distance — dragging across the screen should move the world by about the
    // same fraction of the view whatever the zoom.
    void Pan(double ScreenRight, double ScreenUp)
    {
        const double Y = CamRad(YawDeg);
        const double P = CamRad(PitchDeg);
        const FCamVec Right{-std::sin(Y), std::cos(Y), 0.0};
        const FCamVec Up{std::cos(Y) * std::sin(P), std::sin(Y) * std::sin(P), std::cos(P)};
        const double Scale = Distance * 0.001;
        Focus = Focus + Right * (ScreenRight * Scale) + Up * (ScreenUp * Scale);
    }

    FCamVec Position() const
    {
        const double Y = CamRad(YawDeg);
        const double P = CamRad(PitchDeg);
        // NEGATIVE PITCH LOOKS DOWN, which is what the field's comment always
        // said and what the sign here did not do: the default -25 deg framed
        // every track from underneath, looking up at its own spine. Asserted now.
        const FCamVec Back{-std::cos(Y) * std::cos(P), -std::sin(Y) * std::cos(P), std::sin(P)};
        return Focus - Back * Distance;
    }

    void Frame(const FCamBounds& B, double FovDegrees, double Aspect)
    {
        if (!B.IsValid()) { return; }
        Focus = B.Centre();
        Distance = DistanceToFrame(B.Radius(), FovDegrees, Aspect);
    }

private:
    static double Wrap(double D)
    {
        while (D > 180.0) { D -= 360.0; }
        while (D < -180.0) { D += 360.0; }
        return D;
    }
};

// ===================== SMOOTHING =====================
//
// Framing a segment should GLIDE rather than snap, because a snap loses the
// person's sense of where they were — which is the whole reason a validation
// warning that jumps you somewhere is disorienting rather than helpful.
//
// FRAME-RATE INDEPENDENT, which the obvious `Current += (Target - Current) * K`
// is not: it converges faster at high frame rates, so the camera feels different
// on a better machine. The exponential form is one `exp` and is correct at any
// step, which matters here for the same reason the fixed scan period does.
inline double Smoothed(double Current, double Target, double HalfLifeSeconds, double DeltaSeconds)
{
    if (!(HalfLifeSeconds > 0.0)) { return Target; }
    const double T = std::exp(-DeltaSeconds * 0.6931471805599453 / HalfLifeSeconds);
    return Target + (Current - Target) * T;
}

inline FCamVec Smoothed(const FCamVec& Current, const FCamVec& Target,
                        double HalfLifeSeconds, double DeltaSeconds)
{
    return {Smoothed(Current.X, Target.X, HalfLifeSeconds, DeltaSeconds),
            Smoothed(Current.Y, Target.Y, HalfLifeSeconds, DeltaSeconds),
            Smoothed(Current.Z, Target.Z, HalfLifeSeconds, DeltaSeconds)};
}

// ===================== PER-MODE MEMORY =====================
//
// Switching Build to Ride to Build must not lose your place. Obvious, cheap, and
// the sort of thing that is never added later because it is never the most
// urgent bug — but it is the difference between a viewport somebody works in and
// one they fight.
class FCameraRigs
{
public:
    enum { NumModes = 5 };   // matches EAppMode's count

    FOrbitState& For(int Mode)
    {
        const int I = Mode >= 0 && Mode < NumModes ? Mode : 0;
        return Rig[I];
    }
    const FOrbitState& For(int Mode) const
    {
        const int I = Mode >= 0 && Mode < NumModes ? Mode : 0;
        return Rig[I];
    }

private:
    FOrbitState Rig[NumModes];
};

// ponytail: no free-fly integration here — that is velocity times delta against
// held keys, which is engine input handling and has nothing to test. No FOV
// animation, no collision, no camera shake. The parts above are the ones with an
// answer that can be wrong in a way nobody notices until it is shipped.
