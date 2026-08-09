// Asserts for TrackSupports.h — getting the track off the ground.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_tracksupports test_tracksupports.cpp && ./test_tracksupports
//
// The card calls support placement a genuinely hard sub-problem, and it is. What
// is asserted here is not that the placement is right — it is that the placer
// REFUSES the cases it cannot get right, and says which, rather than producing
// geometry that is obviously wrong to anybody who looks at the ride.

#include "TrackSupports.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace
{

FTrack Straight(double Len)
{
    FTrack T;
    T.AddSegment(MakeStraight(Len));
    return T;
}

// A hill: up, over, down. Straight track at ground level would never test the
// height rules.
FTrack Hill()
{
    FTrack T;
    T.AddSegment(MakeStraight(20.0));
    FTrackSegment Up = MakeStraight(30.0);
    Up.PitchCurvatureStart = 0.02;
    Up.PitchCurvatureEnd = 0.02;
    T.AddSegment(Up);
    T.AddSegment(MakeStraight(30.0));
    return T;
}

FTrack VerticalLoop(double R = 8.0)
{
    FTrack T;
    T.AddSegment(MakeStraight(20.0));
    FTrackSegment Loop = MakeStraight(TrackMeshTwoPi * R);
    Loop.PitchCurvatureStart = 1.0 / R;
    Loop.PitchCurvatureEnd = 1.0 / R;
    T.AddSegment(Loop);
    T.AddSegment(MakeStraight(10.0));
    return T;
}

FSupportPlan Plan(const FTrack& T, const FSupportSettings& S,
                  const FGroundHeight& G, double Spacing = 0.5)
{
    return PlanSupports(WalkTrack(T, Spacing), T.GetHeartlineHeight(),
                        FTrackProfile(), S, G);
}

void TestSPACINGIsASPANNotASampleCount()
{
    // Steel track spans a DISTANCE. One support every N samples puts them 0.5 m
    // apart on a tight helix and 40 m apart on a straight — the same rule that
    // makes tie spacing a distance rather than a count.
    //
    // Deep enough underground that the whole hill is well clear of it.
    const FTrack T = Hill();
    FSupportSettings S;
    S.SpanM = 9.0;

    const FSupportPlan A = Plan(T, S, FlatGround(-40.0));
    // 80 m of track at 9 m spacing is nine bents counting the one at S = 0.
    assert(A.Leg.size() == 9);
    for (std::size_t i = 1; i < A.Leg.size(); ++i)
    {
        const double Gap = A.Leg[i].S - A.Leg[i - 1].S;
        assert(Gap >= 9.0 - 0.6 && Gap <= 9.0 + 0.6);   // within one walk sample
    }

    // Halving the span roughly doubles the count, and CHANGING THE WALK SPACING
    // does not change it at all — which is the property being asserted.
    S.SpanM = 4.5;
    assert(Plan(T, S, FlatGround(-40.0)).Leg.size() >= 17);
    S.SpanM = 9.0;
    assert(Plan(T, S, FlatGround(-40.0), 0.25).Leg.size() == A.Leg.size());
    assert(Plan(T, S, FlatGround(-40.0), 1.0).Leg.size() == A.Leg.size());
    std::printf("  %zu bents over 80 m at a 9 m span, and the walk spacing does not change it\n",
                A.Leg.size());
}

void TestTrackBELOWGroundGetsNoColumn()
{
    // A column of negative length is the kind of thing that looks fine in a list
    // of numbers and absurd in a viewport. Track at or under grade is in a trench
    // or a tunnel and gets nothing.
    const FTrack T = Straight(60.0);
    FSupportSettings S;

    // Ground well above the track.
    const FSupportPlan Under = Plan(T, S, FlatGround(50.0));
    assert(Under.Leg.empty());
    assert(!Under.Finding.empty());
    assert(Under.Finding[0].What.find("below ground") != std::string::npos);

    // And every leg that IS placed has a positive height, on every layout here.
    const FSupportPlan Over = Plan(Hill(), S, FlatGround(-40.0));
    for (const FSupportLeg& L : Over.Leg)
    {
        assert(L.Height() >= S.MinHeightM);
        assert(L.Foot.Z < L.Top.Z);
    }
    std::printf("  track under grade gets no column, and no placed leg has negative height\n");
}

void TestATGRADEIsNotAFindingBecauseThatIsAStation()
{
    // A station sits on the ground and wants a footer plate rather than a tower.
    // Reporting that as a problem would bury the real findings under one per bent
    // on every ride ever built.
    // THE HEARTLINE ORIGIN IS RIDER HEIGHT, NOT TRACK HEIGHT, which is the thing
    // to keep hold of here: z = 0 in track space puts the spine 1.55 m BELOW it,
    // being 1.1 m of heartline plus 0.45 m of spine drop. So "at grade" is ground
    // at about -2, not at 0 — and ground at 0 really is a tunnel.
    const FTrack T = Straight(60.0);
    FSupportSettings S;
    const FSupportPlan P = Plan(T, S, FlatGround(-2.0));

    assert(P.Leg.empty());
    assert(P.Finding.empty());

    // AND IT GETS THE FOOTER PLATE THE RULE ALWAYS PROMISED. The MinHeightM branch
    // has said "wants a footer plate, not a tower" since it was written, and then
    // placed nothing — which was invisible while nothing was drawn and obvious the
    // moment legs were: the lowest run of a layout floated with no column and no
    // pad, which is the one part of a real coaster nobody has ever seen
    // unsupported.
    //
    // SILENT STILL MEANS SILENT. The two assertions above are about REPORTING and
    // are unchanged: a station getting a pad is not a placement failure.
    assert(!P.Footing.empty());
    for (const FSupportFooting& F : P.Footing)
    {
        // The pad reaches from the track down past the ground it sits on, so track
        // a few centimetres up gets a footing rather than a film.
        assert(F.Thickness > 0.1);
        assert(F.Width > S.LegDiameterM);
    }
    std::printf("  at-grade track is silent and gets %zu footer plates, no columns\n",
                P.Footing.size());
}

void TestINVERTEDTrackIsREFUSEDRatherThanSpeared()
{
    // WHERE A NAIVE PLACER MAKES ITS WORST GEOMETRY. `SpineDrop` puts the spine
    // BELOW the rails, so through an inversion the spine is ABOVE the track — and
    // a column coming up from underneath would have to pass through the rails to
    // reach it.
    //
    // Detected from the frame rather than guessed at: the spine is above the rail
    // centre exactly when the track's own up vector points down.
    const FTrack T = VerticalLoop();
    FSupportSettings S;
    S.SpanM = 4.0;
    const FSupportPlan P = Plan(T, S, FlatGround(-5.0));

    std::size_t Inverted = 0;
    for (const FSupportFinding& F : P.Finding)
    {
        if (F.What.find("inverted") != std::string::npos) { ++Inverted; }
    }
    assert(Inverted > 0);

    // And nothing was placed on an inverted frame: every leg's top is at or below
    // the heartline it carries, which is only true the right way up.
    const std::vector<FTrackFrame> Path = WalkTrack(T, 0.5);
    for (const FSupportLeg& L : P.Leg)
    {
        double Best = 1e9;
        std::size_t At = 0;
        for (std::size_t i = 0; i < Path.size(); ++i)
        {
            const double D = Length(Path[i].Position - L.Top);
            if (D < Best) { Best = D; At = i; }
        }
        assert(Path[At].Up.Z > 0.0);
    }
    std::printf("  a loop refuses %zu inverted placements and spears nothing\n", Inverted);
}

// A path built by hand: out at ground level, up, and back OVER ITSELF at height.
//
// PlanSupports takes FRAMES rather than an FTrack, which is the seam that lets
// this be constructed exactly rather than hoped for out of a preset. Every frame
// is upright, so nothing here is refused for being inverted — the only reason a
// column can be refused on this path is that it would foul the lower leg.
std::vector<FTrackFrame> OverItself()
{
    std::vector<FTrackFrame> Path;
    auto Add = [&Path](double X, double Z)
    {
        FTrackFrame F;
        F.Position = {X, 0.0, Z};
        F.Tangent = {1.0, 0.0, 0.0};
        F.Lateral = {0.0, 1.0, 0.0};
        F.Up = {0.0, 0.0, 1.0};
        F.PathLateral = F.Lateral;
        F.PathUp = F.Up;
        Path.push_back(F);
    };
    for (double X = 0.0; X <= 40.0; X += 0.5) { Add(X, 0.0); }          // out, at grade
    for (double Z = 0.5; Z <= 20.0; Z += 0.5) { Add(40.0, Z); }         // up the far end
    for (double X = 39.5; X >= 0.0; X -= 0.5) { Add(X, 20.0); }         // back, overhead
    return Path;
}

void TestAColumnThatWouldFOULTheTrackIsREFUSED()
{
    // THE CLASSIC FAILURE, and the one that looks worst: a footer under a high
    // point with the column running straight up through a LOWER piece of track it
    // happens to sit over. Every layout with a return leg above its outbound one
    // has this, and so does every helix.
    //
    // The lower leg gets its bents. The upper leg gets NOTHING, because a column
    // from the ground to 20 m would pass straight through the track below it —
    // and a real ride solves that with a bent standing beside the lower leg and
    // reaching across, which is structure this layer does not yet build. So it
    // refuses rather than guesses.
    FSupportSettings S;
    S.SpanM = 6.0;
    const FSupportPlan P = PlanSupports(OverItself(), 1.1, FTrackProfile(), S,
                                        FlatGround(-6.0));

    std::size_t Fouled = 0;
    for (const FSupportFinding& F : P.Finding)
    {
        if (F.What.find("foul") != std::string::npos) { ++Fouled; }
    }
    assert(Fouled > 0);

    // Every bent that WAS placed is on the lower leg or the riser, never the
    // return. Nothing is speared.
    for (const FSupportLeg& L : P.Leg) { assert(L.Top.Z < 19.0); }
    assert(!P.Leg.empty());

    // The finding says WHERE it fouls, because "cannot place a support" without a
    // location is somewhere to start looking rather than somewhere to go — and it
    // is MERGED into runs, so a 40 m stretch that cannot take one is one problem
    // with a length rather than seven copies of the same sentence.
    bool bNamesAPlace = false;
    double LongestRun = 0.0;
    for (const FSupportFinding& F : P.Finding)
    {
        if (F.What.find(" m") != std::string::npos) { bNamesAPlace = true; }
        if (F.LengthM() > LongestRun) { LongestRun = F.LengthM(); }
    }
    assert(bNamesAPlace);
    assert(LongestRun > 20.0);
    std::printf("  a return leg over its own outbound: %zu bents below, and %.0f m above"
                " refused as one finding\n", P.Leg.size(), LongestRun);
}

void TestAColumnDoesNOTFoulTheTrackItIsHOLDINGUP()
{
    // The other half, and the one that would make the check useless: a column
    // obviously passes close to the track at its own attachment point. Excluding
    // the neighbourhood is what makes the test about OTHER track.
    //
    // On a plain hill nothing is above anything, so nothing may be refused.
    const FTrack T = Hill();
    FSupportSettings S;
    const FSupportPlan P = Plan(T, S, FlatGround(-40.0));

    for (const FSupportFinding& F : P.Finding)
    {
        assert(F.What.find("foul") == std::string::npos);
    }
    assert(P.Leg.size() > 5);
    std::printf("  a plain hill refuses nothing: a column may touch the track it carries\n");
}

void TestTheLONGESTGAPIsReportedBecauseRefusalsLeaveHoles()
{
    // THE NUMBER AN ENGINEER WOULD ACTUALLY ASK FOR. A placer that reported only
    // what it placed would say nothing about the sixty metres of unsupported track
    // its own refusals left behind — and those refusals are the whole point of
    // this layer, so the hole they leave has to be visible.
    const FTrack T = VerticalLoop();
    FSupportSettings S;
    S.SpanM = 4.0;
    const FSupportPlan P = Plan(T, S, FlatGround(-5.0));

    assert(!P.Finding.empty());
    // The loop is 50 m of track that can take nothing at all, so the gap is at
    // least that.
    assert(P.LongestGapM > 40.0);

    // A layout with no refusals has a gap of about one span, which is the
    // healthy reading.
    const FSupportPlan Clean = Plan(Hill(), S, FlatGround(-40.0));
    assert(Clean.Finding.empty());
    assert(Clean.LongestGapM < S.SpanM * 1.5);
    std::printf("  the loop leaves a %.1f m unsupported run; the hill's worst is %.1f m\n",
                P.LongestGapM, Clean.LongestGapM);
}

void TestGROUNDIsAFunctionSoTerrainCostsNothingLater()
{
    // Phase 5 adds terrain sculpting. A flat default that had become an assumption
    // baked into every support ever placed would be a retrofit; a callback is the
    // calibration knob that makes it a swap.
    const FTrack T = Hill();
    FSupportSettings S;

    // A slope: ground rising a metre every ten metres of X.
    const FGroundHeight Slope = [](double X, double) { return -40.0 + X * 0.1; };
    const FSupportPlan P = PlanSupports(WalkTrack(T, 0.5), T.GetHeartlineHeight(),
                                        FTrackProfile(), S, Slope);
    assert(!P.Leg.empty());
    for (const FSupportLeg& L : P.Leg)
    {
        // Every footer sits ON the ground, not on the plane the first one found.
        assert(std::fabs(L.Foot.Z - Slope(L.Foot.X, L.Foot.Y)) < 1e-9);
        assert(L.Height() > 0.0);
    }
    // AND THE RELATIONSHIP IS EXACT, which is worth more than "they get shorter"
    // — the first version asserted that and it was simply false, because this
    // hill climbs faster than the slope does. Against a flat ground at the same
    // datum, every sloped leg is shorter by exactly the ground's rise under it.
    const FSupportPlan Flat = PlanSupports(WalkTrack(T, 0.5), T.GetHeartlineHeight(),
                                           FTrackProfile(), S, FlatGround(-40.0));
    assert(Flat.Leg.size() == P.Leg.size());
    for (std::size_t i = 0; i < P.Leg.size(); ++i)
    {
        const double Rise = P.Leg[i].Foot.X * 0.1;
        assert(std::fabs((Flat.Leg[i].Height() - P.Leg[i].Height()) - Rise) < 1e-9);
    }
    std::printf("  footers land on a sloped ground and the legs shorten as it rises\n");
}

} // namespace

// ===================== THE SUPPORTS AS GEOMETRY =====================
//
// Placement has been asserted since this file was written. What was never checked
// is the step that turns a plan into something you can see, because nothing did
// it: the actor read Plan.Finding and dropped Plan.Leg on the floor every rebuild.
//
// Held to the same bar as the track, because it is the same failure if it is
// wrong: a closed mesh encloses positive volume only when every triangle faces
// outward, and a column standing on the ground with an open bottom is the ties'
// open-pipe bug with a better view of it.
void TestSupportsBecomeGeometry()
{
    std::printf("Supports become geometry\n");

    // A plain hill, high enough that every span wants a column.
    const FSupportSettings S;
    const FSupportPlan Built = Plan(Hill(), S, FlatGround(-40.0));
    assert(!Built.Leg.empty() && "a hill above the ground wants columns");

    const int Sides = 8;
    const FMeshBuffer M = BuildSupportMesh(Built, Sides);

    // ONE CAPPED TUBE PER LEG AND PER FOOTING, and the count is exact rather than
    // "about right": a tube of N sides is 2N wall triangles plus two N-triangle
    // cap fans.
    const std::size_t PerTube = static_cast<std::size_t>(Sides) * 4;
    const std::size_t Tubes = Built.Leg.size() + Built.Footing.size();
    assert(M.Index.size() / 3 == Tubes * PerTube);

    // EVERY LEG LANDS ON SOMETHING. A column that stops in the dirt is what made
    // this worth building, so it is asserted rather than assumed: at least one
    // footing per leg, and more wherever at-grade track has a pad and no column.
    assert(Built.Footing.size() >= Built.Leg.size());
    std::printf("  %zu legs, %zu footings -> %zu triangles\n",
                Built.Leg.size(), Built.Footing.size(), M.Index.size() / 3);

    // ---- WATERTIGHT. Every edge shared by exactly two triangles, welded by
    // position because the UV seam splits vertices that are geometrically one.
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> Edges;
    std::vector<std::uint32_t> Weld(M.Position.size());
    for (std::size_t i = 0; i < M.Position.size(); ++i)
    {
        Weld[i] = static_cast<std::uint32_t>(i);
        for (std::size_t j = 0; j < i; ++j)
        {
            if (Length(M.Position[i] - M.Position[j]) < 1e-9) { Weld[i] = Weld[j]; break; }
        }
    }
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
    assert(Boundary == 0 && "a column has no open end — including the one on the ground");

    // ---- OUTWARD-WOUND, by signed volume, and compared against what the legs
    // should enclose: an N-gon prism per leg of its own height.
    double V6 = 0.0;
    for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
    {
        V6 += Dot(M.Position[M.Index[t]],
                  Cross(M.Position[M.Index[t + 1]], M.Position[M.Index[t + 2]]));
    }
    const double Volume = V6 / 6.0;

    auto Prism = [&](double Radius, double Height)
    {
        return 0.5 * Sides * Radius * Radius * std::sin(TrackMeshTwoPi / Sides) * Height;
    };
    double Expected = 0.0;
    for (const FSupportLeg& L : Built.Leg) { Expected += Prism(L.Diameter * 0.5, L.Height()); }
    for (const FSupportFooting& F : Built.Footing) { Expected += Prism(F.Width * 0.5, F.Thickness); }
    std::printf("  volume %.6f m^3, %d-gon prisms %.6f m^3\n", Volume, Sides, Expected);
    assert(Volume > 0.0 && "an outward-wound closed mesh has positive signed volume");
    assert(std::fabs(Volume - Expected) < 1e-6);

    // ---- AND THE CHECK BITES. Reversing one triangle drops the volume, which is
    // the property that makes this stronger than a winding check: a test that
    // cannot fail is decoration.
    FMeshBuffer Flipped = M;
    std::swap(Flipped.Index[1], Flipped.Index[2]);
    double FlipV6 = 0.0;
    for (std::size_t t = 0; t + 2 < Flipped.Index.size(); t += 3)
    {
        FlipV6 += Dot(Flipped.Position[Flipped.Index[t]],
                      Cross(Flipped.Position[Flipped.Index[t + 1]],
                            Flipped.Position[Flipped.Index[t + 2]]));
    }
    assert(FlipV6 / 6.0 < Volume);

    // ---- A REFUSED PLACEMENT PRODUCES NO COLUMN, which is the whole point of the
    // placement rules surviving into the geometry: track at grade is a station and
    // gets a footer rather than a tower, so a plan with no legs is an empty mesh
    // rather than a degenerate one.
    FSupportPlan Empty;
    const FMeshBuffer None = BuildSupportMesh(Empty, Sides);
    assert(None.Index.empty() && None.Position.empty());
    std::printf("  a plan with no legs builds nothing at all\n");
}

int main()
{
    std::printf("Support placement: where a column goes, and where it must not\n\n");

    TestSPACINGIsASPANNotASampleCount();
    TestTrackBELOWGroundGetsNoColumn();
    TestATGRADEIsNotAFindingBecauseThatIsAStation();
    TestINVERTEDTrackIsREFUSEDRatherThanSpeared();
    TestAColumnThatWouldFOULTheTrackIsREFUSED();
    TestAColumnDoesNOTFoulTheTrackItIsHOLDINGUP();
    TestTheLONGESTGAPIsReportedBecauseRefusalsLeaveHoles();
    TestGROUNDIsAFunctionSoTerrainCostsNothingLater();
    TestSupportsBecomeGeometry();

    std::printf("\ntest_tracksupports: all assertions passed.\n");
    return 0;
}
