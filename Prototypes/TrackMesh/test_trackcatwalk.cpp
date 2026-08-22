// Asserts for TrackCatwalk.h — the deck and guardrail an evacuation route is
// actually walked along.
//
//   clang++ -std=c++17 -O2 -Wall -Wextra -o test_trackcatwalk test_trackcatwalk.cpp
//
// The evacuation model has decided whether a stopped train can be reached since it
// was written, over a walkway nobody could see. These are the checks that make the
// drawn version worth trusting: which SIDE it is on, that a gap in the authoring
// is a gap in the deck, and that track nobody could walk on is REPORTED rather
// than quietly skipped.

#include "TrackCatwalk.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <utility>
#include <vector>

namespace
{

FTrack Straight(double Len = 40.0)
{
    FTrack T;
    T.AddSegment(MakeStraight(Len));
    return T;
}

// RADIANS AT THIS LAYER. The geometry is metres, radians and seconds throughout;
// degrees are an authoring-side unit and the conversion happens once, at the
// boundary. Written out because the first version of this test passed 10 and 50
// straight in and was quietly asking for 573 and 2865 degrees of bank.
FTrack Banked(double Degrees)
{
    FTrack T;
    T.AddSegment(MakeStraight(40.0, Degrees * 3.14159265358979323846 / 180.0));
    return T;
}

std::vector<FWalkwaySpan> OneSpan(double A, double B, EWalkway Side)
{
    FWalkwaySpan W;
    W.StartS = A;
    W.EndS = B;
    W.Side = Side;
    return {W};
}

// Every edge shared by exactly two triangles, welded by position because the UV
// seam splits vertices that are geometrically one point.
std::size_t BoundaryEdges(const FMeshBuffer& M)
{
    std::vector<std::uint32_t> Weld(M.Position.size());
    for (std::size_t i = 0; i < M.Position.size(); ++i)
    {
        Weld[i] = static_cast<std::uint32_t>(i);
        for (std::size_t j = 0; j < i; ++j)
        {
            if (Length(M.Position[i] - M.Position[j]) < 1e-9) { Weld[i] = Weld[j]; break; }
        }
    }
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> Edges;
    for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
    {
        for (int e = 0; e < 3; ++e)
        {
            const std::uint32_t A = Weld[M.Index[t + static_cast<std::size_t>(e)]];
            const std::uint32_t B = Weld[M.Index[t + static_cast<std::size_t>((e + 1) % 3)]];
            Edges[{std::min(A, B), std::max(A, B)}] += 1;
        }
    }
    std::size_t Boundary = 0;
    for (const auto& E : Edges) { if (E.second != 2) { ++Boundary; } }
    return Boundary;
}

double SignedVolume(const FMeshBuffer& M)
{
    double V6 = 0.0;
    for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
    {
        V6 += Dot(M.Position[M.Index[t]],
                  Cross(M.Position[M.Index[t + 1]], M.Position[M.Index[t + 2]]));
    }
    return V6 / 6.0;
}

// ===================== WHICH SIDE, AND IT IS THE ONE THING THAT SILENTLY
// MIRRORS AN ENTIRE RIDE =====================
//
// +Lateral is the rider's LEFT throughout this project, and getting it backwards
// produces a catwalk that looks perfectly correct in isolation and is on the
// wrong side of every ride ever built with it. The same class of error as the
// handedness flip at the port boundary, which is why that one is asserted too.
void TestTheDeckIsOnTheSIDEThatWasAskedFor()
{
    std::printf("A catwalk is on the side that was asked for\n");

    const FTrack T = Straight();
    const std::vector<FTrackFrame> F = WalkTrack(T, 0.5);

    const FCatwalkMesh L = BuildCatwalks(F, OneSpan(5.0, 30.0, EWalkway::Left), FTrackProfile());
    const FCatwalkMesh R = BuildCatwalks(F, OneSpan(5.0, 30.0, EWalkway::Right), FTrackProfile());
    assert(!L.Deck.Position.empty() && !R.Deck.Position.empty());

    // On a straight running along +X with no roll, the rider's left is +Y.
    double LeftY = 0.0, RightY = 0.0;
    for (const FVec3& P : L.Deck.Position) { LeftY += P.Y; }
    for (const FVec3& P : R.Deck.Position) { RightY += P.Y; }
    LeftY /= static_cast<double>(L.Deck.Position.size());
    RightY /= static_cast<double>(R.Deck.Position.size());
    std::printf("  left deck at y = %+.2f m, right deck at y = %+.2f m\n", LeftY, RightY);
    assert(LeftY > 0.0 && "the rider's left is +Lateral, which on this track is +Y");
    assert(RightY < 0.0);

    // AND BOTH IS BOTH, rather than one of them twice.
    const FCatwalkMesh B = BuildCatwalks(F, OneSpan(5.0, 30.0, EWalkway::Both), FTrackProfile());
    assert(B.Deck.Index.size() == L.Deck.Index.size() + R.Deck.Index.size());
    std::printf("  both sides is both, not one of them twice\n");
}

// ===================== A CLOSED SLAB, NOT A ONE-SIDED STRIP =====================
//
// A catwalk is looked at from UNDERNEATH more often than from above — that is
// what a lift hill is from the queue — so a single quad strip would be invisible
// exactly where people look at it. Closing it also buys the signed-volume check,
// which is what found the track mesh inside out on the first try.
void TestTheDeckIsCLOSEDAndENCLOSESVolume()
{
    std::printf("The deck is closed and encloses volume\n");

    const FTrack T = Straight();
    const std::vector<FTrackFrame> F = WalkTrack(T, 0.5);
    FCatwalkSettings S;
    const FCatwalkMesh M = BuildCatwalks(F, OneSpan(5.0, 30.0, EWalkway::Left), FTrackProfile(), S);

    assert(BoundaryEdges(M.Deck) == 0 && "a capped deck run has no open rim");

    const double Volume = SignedVolume(M.Deck);
    // A slab of deck: width by thickness by however much of the span the sampling
    // actually covered, which is a whole number of half-metre steps and therefore
    // a little short of the authored 25 m at each end.
    const double PerMetre = S.DeckWidthM * S.DeckThicknessM;
    std::printf("  volume %.6f m^3, which is %.2f m of a %.2f x %.2f m slab\n",
                Volume, Volume / PerMetre, S.DeckWidthM, S.DeckThicknessM);
    // FLUSHED, because the assertion below aborts and a block-buffered stdout
    // loses the number that would say why — the one moment it matters.
    std::fflush(stdout);
    assert(Volume > 0.0 && "an outward-wound closed mesh has positive signed volume");
    // Between 24 and 25 m of slab: the run cannot be longer than the span asked
    // for, and cannot be more than one sample step short of it at each end.
    assert(Volume / PerMetre >= 24.0 - 1e-9 && Volume / PerMetre <= 25.0 + 1e-9);

    // AND THE CHECK BITES — which is not a claim, it is what happened. Both end
    // caps were wound inside out on the first version, WATERTIGHTNESS PASSED
    // ANYWAY (edge sharing is blind to orientation, so a mesh with reversed caps
    // still counts every edge exactly twice), and this reported 8 m of slab for a
    // 24 m run. That is the whole reason this project checks volume rather than
    // winding.
    //
    // Asserted as the position-independent property: reverse EVERY triangle and
    // the volume negates exactly. Reversing one is not a valid check — a single
    // triangle's contribution is signed by where it sits relative to the origin,
    // so flipping it can just as easily make the total larger.
    FMeshBuffer Flipped = M.Deck;
    for (std::size_t t = 0; t + 2 < Flipped.Index.size(); t += 3)
    {
        std::swap(Flipped.Index[t + 1], Flipped.Index[t + 2]);
    }
    assert(std::fabs(SignedVolume(Flipped) + Volume) < 1e-12);
    std::printf("  reversing every triangle negates it exactly\n");
}

// ===================== A GAP IN THE AUTHORING IS A GAP IN THE DECK =====================
//
// The evacuation model's whole job is deciding whether a stopped train can be
// REACHED, and the answer turns on where the catwalk stops. A mesher that
// stitched a quad across the space between two spans would draw a continuous
// walkway over a gap the safety model knows about — the drawing and the model
// disagreeing about the thing the model exists to decide.
void TestASpanBoundaryENDSTheDeck()
{
    std::printf("A gap between spans is a gap in the deck\n");

    const FTrack T = Straight(60.0);
    const std::vector<FTrackFrame> F = WalkTrack(T, 0.5);

    std::vector<FWalkwaySpan> Two;
    FWalkwaySpan A; A.StartS = 5.0;  A.EndS = 20.0; A.Side = EWalkway::Left;
    FWalkwaySpan B; B.StartS = 40.0; B.EndS = 55.0; B.Side = EWalkway::Left;
    Two.push_back(A);
    Two.push_back(B);

    const FCatwalkMesh M = BuildCatwalks(F, Two, FTrackProfile());
    // Two closed runs: still no boundary edges, and no deck vertex in the gap.
    assert(BoundaryEdges(M.Deck) == 0 && "each run is capped, so two runs are still closed");
    for (const FVec3& P : M.Deck.Position)
    {
        assert(!(P.X > 22.0 && P.X < 38.0) && "nothing is drawn across the gap");
    }
    std::printf("  two runs, both capped, and nothing between them\n");
}

// ===================== TRACK NOBODY COULD WALK ON IS REPORTED =====================
//
// A catwalk follows the track's own frame, so it banks with it — correct, and
// exactly why real rides put catwalks on lift hills and brake runs rather than
// through banked turns. Past some angle it stops being walkable.
//
// REPORTED, NOT REFUSED: the author asked for it and may have a reason, and a
// mesher that silently dropped it would leave a hole in an evacuation route with
// nothing saying why. Report, never repair — the same rule as everywhere else.
void TestUnwalkableBankIsREPORTEDNotDropped()
{
    std::printf("A catwalk too steep to walk on says so\n");

    const std::vector<FTrackFrame> Gentle = WalkTrack(Banked(10.0), 0.5);
    const FCatwalkMesh Fine = BuildCatwalks(Gentle, OneSpan(5.0, 30.0, EWalkway::Left),
                                            FTrackProfile());
    assert(Fine.Finding.empty() && "ten degrees is a ramp, not a finding");

    const std::vector<FTrackFrame> Steep = WalkTrack(Banked(50.0), 0.5);
    const FCatwalkMesh Hard = BuildCatwalks(Steep, OneSpan(5.0, 30.0, EWalkway::Left),
                                            FTrackProfile());
    assert(Hard.Finding.size() == 1);
    // STILL DRAWN. The finding is the point; dropping the geometry would hide the
    // very thing being complained about.
    assert(!Hard.Deck.Index.empty() && "reported, not repaired: it is still there");
    std::printf("  %s\n", Hard.Finding[0].What.c_str());

    // THE SILENCE IS THE RESULT. A checker that fires on ordinary track is one
    // somebody switches off, so a level catwalk is asserted to say nothing.
    const std::vector<FTrackFrame> Level = WalkTrack(Straight(), 0.5);
    assert(BuildCatwalks(Level, OneSpan(5.0, 30.0, EWalkway::Both), FTrackProfile())
               .Finding.empty());
    std::printf("  and level track is silent\n");
}

// ===================== THE DECK IS LEVEL, WHATEVER THE TRACK DOES =====================
//
// The version before this one built the deck from the frame's own Lateral and
// Up, so it rolled and twisted with the rails. It looked defensible in the
// header and indefensible in the engine: a walkway that corkscrews is not
// somewhere anybody stands. A floor is flat; only the track banks.
//
// ASSERTED ON THE GEOMETRY RATHER THAN ON THE SETTING, because "flat" is a
// property of the vertices and a flag can be true while the deck twists anyway.
void TestTheDeckStaysLEVELThroughABankedTurn()
{
    std::printf("The deck is level even where the track is not\n");

    // 50 degrees of bank: far past anything walkable, which is the point — if
    // the deck followed the track at all, it would show here more than anywhere.
    const std::vector<FTrackFrame> Steep = WalkTrack(Banked(50.0), 0.5);
    const FCatwalkMesh M = BuildCatwalks(Steep, OneSpan(5.0, 30.0, EWalkway::Left),
                                         FTrackProfile());
    assert(!M.Deck.Position.empty());

    // Every ring is inner-top, outer-top, outer-bottom, inner-bottom — so the
    // first two of each four are the walking surface, and they must be at the
    // same height as each other however hard the track is rolling.
    double WorstCross = 0.0;
    for (std::size_t k = 0; k + 1 < M.Deck.Position.size(); k += 4)
    {
        const double D = std::fabs(M.Deck.Position[k].Z - M.Deck.Position[k + 1].Z);
        WorstCross = D > WorstCross ? D : WorstCross;
    }
    std::printf("  worst cross-fall over a %.2f m deck: %.9f m\n",
                FCatwalkSettings().DeckWidthM, WorstCross);
    assert(WorstCross < 1e-9 && "the walking surface is level across, exactly");

    // AND THE TRACK REALLY IS BANKED THERE. Without this the assertion above
    // passes just as well on straight level track, which is the way a check like
    // this quietly stops meaning anything.
    double WorstUpZ = 1.0;
    for (const FTrackFrame& F : Steep)
    {
        WorstUpZ = F.Up.Z < WorstUpZ ? F.Up.Z : WorstUpZ;
    }
    assert(WorstUpZ < 0.7 && "the fixture must actually bank, or this proves nothing");

    // THE HANDRAIL STANDS UP TOO. A level floor under a leaning rail is worse
    // than either mistake alone, and the rail is a separate buffer built in a
    // separate loop — so it is a separate thing to get wrong.
    double MinZ = M.Rail.Position[0].Z, MaxZ = M.Rail.Position[0].Z;
    for (const FVec3& P : M.Rail.Position)
    {
        MinZ = P.Z < MinZ ? P.Z : MinZ;
        MaxZ = P.Z > MaxZ ? P.Z : MaxZ;
    }
    // Deck top to top rail, plus the tube's own radius either end. A rail leaning
    // with 50 degrees of bank would span far more than its own height.
    const FCatwalkSettings S;
    const double Span = MaxZ - MinZ;
    std::printf("  handrail spans %.3f m of height for a %.2f m rail\n", Span, S.RailHeightM);
    assert(Span < S.RailHeightM + S.RailDiameterM + 1e-6);
    std::printf("  and the posts are vertical\n");
}

// Nothing asked for is nothing built — not an empty run, not a degenerate quad.
void TestNoWalkwayIsNoGeometry()
{
    std::printf("No walkway is no geometry\n");
    const std::vector<FTrackFrame> F = WalkTrack(Straight(), 0.5);
    const FCatwalkMesh None = BuildCatwalks(F, {}, FTrackProfile());
    assert(None.Deck.Index.empty() && None.Rail.Index.empty() && None.Finding.empty());

    const FCatwalkMesh Off = BuildCatwalks(F, OneSpan(5.0, 30.0, EWalkway::None), FTrackProfile());
    assert(Off.Deck.Index.empty() && Off.Rail.Index.empty());
    std::printf("  an unwanted side builds nothing at all\n");
}

} // namespace

int main()
{
    std::printf("Catwalks: the deck an evacuation route is walked along\n\n");

    TestTheDeckIsOnTheSIDEThatWasAskedFor();
    TestTheDeckIsCLOSEDAndENCLOSESVolume();
    TestASpanBoundaryENDSTheDeck();
    TestUnwalkableBankIsREPORTEDNotDropped();
    TestTheDeckStaysLEVELThroughABankedTurn();
    TestNoWalkwayIsNoGeometry();

    std::printf("\ntest_trackcatwalk: all assertions passed.\n");
    return 0;
}
