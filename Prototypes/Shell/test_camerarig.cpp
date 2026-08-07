// Asserts for CameraRig.h — the runtime camera's arithmetic.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_camerarig test_camerarig.cpp && ./test_camerarig

#include "CameraRig.h"

#include <cassert>
#include <initializer_list>
#include <cstdio>

namespace
{

// The two-train circuit's rough envelope: 400 m long, 200 m wide, 50 m tall.
// Wide and low, which is what every real layout is and what breaks
// vertical-only framing.
FCamBounds Circuit()
{
    FCamBounds B;
    B.Add({-200.0, -100.0, 0.0});
    B.Add({200.0, 100.0, 50.0});
    return B;
}

void TestFramingChecksBOTHAxes()
{
    // THE BUG THIS EXISTS FOR. The distance that fits a box vertically is not the
    // distance that fits it horizontally, and which one binds depends on the
    // aspect ratio. Check only the vertical and a long low layout — which is
    // every out-and-back ever built — hangs off both sides.
    //
    // Same subject, three monitors. The ultrawide has barely half the vertical
    // field of the 4:3, so it must stand FURTHER off, not nearer.
    const double R = Circuit().Radius();
    const double Fov = 90.0;

    const double Square = DistanceToFrame(R, Fov, 1.0);
    const double Wide   = DistanceToFrame(R, Fov, 16.0 / 9.0);
    const double Ultra  = DistanceToFrame(R, Fov, 21.0 / 9.0);

    // Wider aspect at a fixed HORIZONTAL fov means a narrower vertical one, so
    // the vertical is what binds and the distance grows.
    assert(Wide > Square);
    assert(Ultra > Wide);

    // And the subject genuinely fits, both ways, on the tightest of them. The
    // half-angle test is the definition of fitting: the subject's angular radius
    // must not exceed the view's half-angle.
    for (double Aspect : {1.0, 16.0 / 9.0, 21.0 / 9.0})
    {
        const double D = DistanceToFrame(R, Fov, Aspect);
        const double HalfH = CamRad(Fov) * 0.5;
        const double HalfV = std::atan(std::tan(HalfH) / Aspect);
        const double Angular = std::asin(R / D);
        assert(Angular <= HalfH + 1e-9);
        assert(Angular <= HalfV + 1e-9);
    }
    std::printf("  framing a 400x200x50 m layout: %.0f m at 1:1, %.0f m at 21:9\n", Square, Ultra);
}

void TestFramingUsesTheSPHERESoItSurvivesOrbiting()
{
    // Rotation-independent on purpose. Frame by the box's own extents and the
    // layout goes off screen the moment somebody orbits — a bug that only shows
    // up after the camera moves and is therefore reported as "the camera is
    // broken" rather than "framing is wrong".
    const FCamBounds B = Circuit();
    const double R = B.Radius();
    const FCamVec E = B.Extent();

    // The sphere contains the box from every angle, which is exactly the property
    // wanted: the largest dimension alone does not.
    assert(R >= E.X && R >= E.Y && R >= E.Z);
    assert(R >= CamLength(E) - 1e-9);

    FOrbitState O;
    O.Frame(B, 90.0, 16.0 / 9.0);
    const double D = O.Distance;
    for (int i = 0; i < 36; ++i)
    {
        O.AddYaw(10.0);
        O.AddPitch(i % 2 ? 5.0 : -5.0);
        assert(O.Distance == D);                      // orbiting does not rescale
        assert(CamLength(O.Position() - O.Focus) > D - 1e-9);
    }
    std::printf("  framing uses the bounding sphere, so a full orbit never loses the subject\n");
}

void TestTheNEARPlaneFollowsTheCamera()
{
    // A 90 m lift hill and a 2 cm bolt in one scene. A near plane set for the
    // bolt destroys depth precision at the top of the hill; one set for the hill
    // clips the restraint off in front of the rider. A fixed value cannot serve
    // both, and picking one is picking which view to break.
    const double SceneR = Circuit().Radius();

    const FDepthRange Far = DepthRangeFor(600.0, SceneR);
    const FDepthRange Seat = DepthRangeFor(0.3, SceneR);

    assert(Far.Near > Seat.Near);
    assert(Seat.Near <= 0.02 + 1e-12);                // the bolt is not clipped
    assert(Far.Far > 600.0);
    assert(!Far.IsPrecisionRisky());
    assert(!Seat.IsPrecisionRisky() || Seat.Ratio() > 0.0);

    // Everything is in front of the far plane from anywhere sensible.
    for (double D : {1.0, 10.0, 100.0, 600.0})
    {
        const FDepthRange R = DepthRangeFor(D, SceneR);
        assert(R.Far > D + SceneR);
        assert(R.Near > 0.0 && R.Near < R.Far);
    }
    std::printf("  near plane: %.3f m in a seat, %.2f m standing off; far/near %.0f:1\n",
                Seat.Near, Far.Near, Far.Ratio());
}

void TestPITCHIsCLAMPEDNotWrapped()
{
    // An orbit camera that reaches straight down and keeps going flips the world
    // over, and everybody who has done that accidentally stops trusting the
    // camera. Just short of vertical, because exactly vertical is where the up
    // vector becomes undefined.
    FOrbitState O;
    for (int i = 0; i < 100; ++i) { O.AddPitch(10.0); }
    assert(O.PitchDeg <= FOrbitState::PitchLimit);
    assert(O.PitchDeg < 90.0);

    for (int i = 0; i < 200; ++i) { O.AddPitch(-10.0); }
    assert(O.PitchDeg >= -FOrbitState::PitchLimit);
    assert(O.PitchDeg > -90.0);

    // Yaw DOES wrap, because there is no top of a circle — and it stays in range
    // rather than growing without bound, which is what makes it safe to display.
    for (int i = 0; i < 100; ++i) { O.AddYaw(37.0); }
    assert(O.YawDeg >= -180.0 && O.YawDeg <= 180.0);
    std::printf("  pitch clamps just short of vertical; yaw wraps and stays displayable\n");
}

void TestZOOMIsMultiplicativeBecauseAdditiveIsUnusable()
{
    // Additive zoom crawls when you are far away and slams into the subject when
    // you are close. The same wheel notch has to mean the same PROPORTION of the
    // distance, or the control is unusable at one end of its range.
    FOrbitState Far;
    Far.Distance = 1000.0;
    FOrbitState Near;
    Near.Distance = 10.0;

    Far.Zoom(0.9);
    Near.Zoom(0.9);
    assert(std::fabs(Far.Distance - 900.0) < 1e-9);
    assert(std::fabs(Near.Distance - 9.0) < 1e-9);

    // Both moved by the same fraction, which is the whole point.
    assert(std::fabs((900.0 / 1000.0) - (9.0 / 10.0)) < 1e-12);

    // And it cannot be zoomed into the subject or out to infinity.
    for (int i = 0; i < 500; ++i) { Near.Zoom(0.5); }
    assert(Near.Distance >= 1.0);
    for (int i = 0; i < 500; ++i) { Near.Zoom(2.0); }
    assert(Near.Distance <= 20000.0);
    std::printf("  a wheel notch moves the same proportion at 10 m and at 1000 m\n");
}

void TestPANNINGMovesTheFocusInTheCameraPlane()
{
    // Dragging across the screen should move the world by about the same fraction
    // of the view whatever the zoom, so the pan scales with distance.
    FOrbitState O;
    O.YawDeg = 0.0;
    O.PitchDeg = 0.0;
    O.Distance = 100.0;
    const FCamVec Before = O.Focus;

    O.Pan(100.0, 0.0);
    assert(std::fabs(O.Focus.X - Before.X) < 1e-9);       // yaw 0 looks along +X
    assert(O.Focus.Y > Before.Y);                          // so right is +Y
    assert(std::fabs(O.Focus.Z - Before.Z) < 1e-9);

    // Twice as far away, the same drag moves twice as much world.
    FOrbitState N = FOrbitState();
    N.YawDeg = 0.0; N.PitchDeg = 0.0; N.Distance = 200.0;
    N.Pan(100.0, 0.0);
    assert(std::fabs(N.Focus.Y - 2.0 * O.Focus.Y) < 1e-9);
    std::printf("  panning scales with distance, so a drag means the same on screen\n");
}

void TestSMOOTHINGIsFrameRateIndependent()
{
    // The obvious `Current += (Target - Current) * K` converges faster at high
    // frame rates, so the camera feels different on a better machine. The
    // exponential form is one exp and is correct at any step — the same reasoning
    // as the fixed scan period, one layer up.
    //
    // Same wall-clock second, three frame rates, and they must agree.
    const double Half = 0.25;
    double A = 0.0, B = 0.0, C = 0.0;
    for (int i = 0; i < 30; ++i)  { A = Smoothed(A, 100.0, Half, 1.0 / 30.0); }
    for (int i = 0; i < 144; ++i) { B = Smoothed(B, 100.0, Half, 1.0 / 144.0); }
    for (int i = 0; i < 1000; ++i){ C = Smoothed(C, 100.0, Half, 1.0 / 1000.0); }

    assert(std::fabs(A - B) < 1e-9);
    assert(std::fabs(B - C) < 1e-9);

    // And a HALF-life means what it says: half the distance in that time.
    double H = 0.0;
    for (int i = 0; i < 60; ++i) { H = Smoothed(H, 100.0, 1.0, 1.0 / 60.0); }
    assert(std::fabs(H - 50.0) < 1e-6);
    std::printf("  30, 144 and 1000 fps reach the same place in a second; a half-life halves\n");
}

void TestEachMODERemembersItsOwnCamera()
{
    // Switching Build to Ride to Build must not lose your place. Cheap, obvious,
    // and never added later because it is never the most urgent bug — but it is
    // the difference between a viewport somebody works in and one they fight.
    FCameraRigs R;
    R.For(2).Distance = 120.0;         // Build
    R.For(2).YawDeg = 33.0;
    R.For(4).Distance = 5.0;           // Ride

    assert(R.For(2).Distance == 120.0);
    assert(R.For(4).Distance == 5.0);
    assert(R.For(2).YawDeg == 33.0);

    // An out-of-range mode does not corrupt anybody else's.
    R.For(99).Distance = 7.0;
    assert(R.For(2).Distance == 120.0);
    std::printf("  each mode keeps its own camera, so switching back returns you where you were\n");
}

void TestAnEMPTYLayoutDoesNotDivideByAnything()
{
    // A new document has no track. Framing nothing must leave the camera where it
    // was rather than sending it to infinity or NaN — the sort of thing that only
    // happens on somebody's very first launch, which is the worst time.
    FOrbitState O;
    const double D = O.Distance;
    O.Frame(FCamBounds(), 90.0, 16.0 / 9.0);
    assert(O.Distance == D);

    assert(DistanceToFrame(0.0, 90.0, 1.78) == 0.0);
    assert(DistanceToFrame(10.0, 0.0, 1.78) == 0.0);
    assert(DistanceToFrame(10.0, 90.0, 0.0) == 0.0);
    std::printf("  framing an empty document leaves the camera alone rather than at NaN\n");
}

} // namespace

int main()
{
    std::printf("The runtime camera: framing, depth, and the things that go wrong\n\n");

    TestFramingChecksBOTHAxes();
    TestFramingUsesTheSPHERESoItSurvivesOrbiting();
    TestTheNEARPlaneFollowsTheCamera();
    TestPITCHIsCLAMPEDNotWrapped();
    TestZOOMIsMultiplicativeBecauseAdditiveIsUnusable();
    TestPANNINGMovesTheFocusInTheCameraPlane();
    TestSMOOTHINGIsFrameRateIndependent();
    TestEachMODERemembersItsOwnCamera();
    TestAnEMPTYLayoutDoesNotDivideByAnything();

    std::printf("\ntest_camerarig: all assertions passed.\n");
    return 0;
}
