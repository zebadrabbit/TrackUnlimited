// The one runnable check Seat.h leaves behind.
#include "Seat.h"
#include <cassert>
#include <cmath>
#include <cstdio>

static bool Near(double A, double B, double Tol = 1e-9) { return std::fabs(A - B) <= Tol; }

int main()
{
    // Five 3 m cars: nose at +7.5, tail at -7.5. Front row of car 0 is 1.5 m
    // behind the nose; car 4 is 1.5 m ahead of the tail.
    FSeat Front; Front.Car = 0;
    FSeat Back;  Back.Car = 4;
    assert(Near(SeatOffsetAlongM(Front, 5, 3.0), 6.0));
    assert(Near(SeatOffsetAlongM(Back, 5, 3.0), -6.0));
    // Two rows per car split the car; row 1 of car 0 is behind row 0 of car 0.
    FSeat R0 = Front, R1 = Front; R1.Row = 1;
    assert(SeatOffsetAlongM(R1, 5, 3.0, 2) < SeatOffsetAlongM(R0, 5, 3.0, 2));
    assert(Near(SeatOffsetAlongM(R0, 5, 3.0, 2), 7.5 - 0.75));
    // Out-of-range seats clamp rather than leave the train.
    FSeat Far; Far.Car = 99;
    assert(Near(SeatOffsetAlongM(Far, 5, 3.0), -6.0));

    // The frame: off the heartline by the seat's own offsets, axes kept.
    FTrackFrame H;
    H.Position = {10.0, 0.0, 5.0};
    H.Tangent = {1.0, 0.0, 0.0};
    H.Lateral = {0.0, 1.0, 0.0};
    H.Up = {0.0, 0.0, 1.0};
    H.PathLateral = H.Lateral; H.PathUp = H.Up;
    FSeat Wing; Wing.LateralM = 1.2; Wing.VerticalM = 0.25;
    const FTrackFrame W = SeatFrame(H, Wing);
    assert(Near(W.Position.Y, 1.2) && Near(W.Position.Z, 5.25) && Near(W.Position.X, 10.0));
    assert(Near(W.Tangent.X, 1.0) && Near(W.Lateral.Y, 1.0));
    // Backward facing reverses tangent and lateral together; up is untouched.
    FSeat Rev = Wing; Rev.FacingSign = -1;
    const FTrackFrame B = SeatFrame(H, Rev);
    assert(Near(B.Tangent.X, -1.0) && Near(B.Lateral.Y, -1.0) && Near(B.Up.Z, 1.0));
    // Still right-handed: Tangent x Lateral = Up.
    const double CrossZ = B.Tangent.X * B.Lateral.Y - B.Tangent.Y * B.Lateral.X;
    assert(Near(CrossZ, 1.0));

    // Stepping: 5 cars is 10 seats, left first, wrapping.
    assert(SeatByIndex(0, 5, 0.4).Car == 0 && Near(SeatByIndex(0, 5, 0.4).LateralM, 0.4));
    assert(Near(SeatByIndex(1, 5, 0.4).LateralM, -0.4));
    assert(SeatByIndex(9, 5, 0.4).Car == 4);
    assert(SeatByIndex(10, 5, 0.4).Car == 0);
    assert(SeatByIndex(-1, 5, 0.4).Car == 4);

    std::printf("seat: ok\n");
    return 0;
}
