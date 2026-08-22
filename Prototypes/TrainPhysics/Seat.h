#pragma once
//
// A SEAT IS DATA, AND THE FRAME IS DERIVED FROM IT.
//
// The design (Docs/TRAIN_DESIGN.md, "the camera does not want a bone, it wants
// a seat") is that a rider's eye and a rider's felt G are the same point, so a
// seat must be one answer with several consumers: the camera sits in it, the
// mesh will put the restraint there, and the envelope judges the G there. A
// socket on a mesh would be a second source of truth for where the rider is.
//
// Engine-free, like everything in this directory, and it needs nothing of the
// train beyond GetFrameAt: a seat is an offset ALONG the train, which the
// physics already answers, plus an offset ACROSS and UP from the heartline,
// which is one vector sum.
//
// LateralM is the wing-coaster prerequisite: a rider offset from the heartline
// feels roll-rate x offset that the centre seat never does, and that applies to
// the outer seats of every wide train. The term itself is not computed here --
// it needs roll rate, which is the envelope's business -- but the number it
// needs now exists.
//
// ponytail: one row per car. RowsPerCar is a parameter so a two-row car is a
// number rather than a rewrite; nothing shipped has one.

#include "../TrackSpline/TrackSpline.h"

struct FSeat
{
    int Car = 0;             // 0 is the leading car
    int Row = 0;             // 0 is the front row of that car
    double LateralM = 0.0;   // +left, the frame's own sign
    double VerticalM = 0.25; // eye above the heartline; the camera's old constant
    int FacingSign = 1;      // -1 for a backward-facing seat
};

// Where along the train a seat is, as the offset GetFrameAt takes: +ahead of
// the train's centre, -behind. Car 0 is at the nose, which is the same
// reference PlaceCars and the stop marks use.
inline double SeatOffsetAlongM(const FSeat& Seat, int CarCount, double CarLengthM, int RowsPerCar = 1)
{
    if (CarCount <= 0 || !(CarLengthM > 0.0)) { return 0.0; }
    const int Car = Seat.Car < 0 ? 0 : (Seat.Car >= CarCount ? CarCount - 1 : Seat.Car);
    const int Rows = RowsPerCar < 1 ? 1 : RowsPerCar;
    const int Row = Seat.Row < 0 ? 0 : (Seat.Row >= Rows ? Rows - 1 : Seat.Row);
    const double Half = CarLengthM * static_cast<double>(CarCount) * 0.5;
    // Rows sit at the centres of equal slices of the car.
    const double RowInCar = CarLengthM * (static_cast<double>(Row) + 0.5) / static_cast<double>(Rows);
    return Half - CarLengthM * static_cast<double>(Car) - RowInCar;
}

// The seat's frame from the heartline frame at its arc length. Position moves
// off the heartline; the axes are the heartline's, reversed for a backward
// facing seat -- tangent AND lateral together, or the frame stops being
// right-handed, which is the rule RunRideProfile's FacingSign already follows.
inline FTrackFrame SeatFrame(const FTrackFrame& Heart, const FSeat& Seat)
{
    FTrackFrame F = Heart;
    F.Position = Heart.Position + Heart.Lateral * Seat.LateralM + Heart.Up * Seat.VerticalM;
    if (Seat.FacingSign < 0)
    {
        F.Tangent = Heart.Tangent * -1.0;
        F.Lateral = Heart.Lateral * -1.0;
        F.PathLateral = Heart.PathLateral * -1.0;
    }
    return F;
}

// Seats in the order a key would step through them: car by car from the nose,
// left then right within a car. N cars with one row each is 2N seats.
inline FSeat SeatByIndex(int Index, int CarCount, double HalfWidthM, int RowsPerCar = 1)
{
    const int Rows = RowsPerCar < 1 ? 1 : RowsPerCar;
    const int PerCar = Rows * 2;
    const int Total = (CarCount < 1 ? 1 : CarCount) * PerCar;
    int I = Index % Total;
    if (I < 0) { I += Total; }
    FSeat S;
    S.Car = I / PerCar;
    S.Row = (I % PerCar) / 2;
    S.LateralM = (I % 2 == 0) ? HalfWidthM : -HalfWidthM;
    return S;
}
