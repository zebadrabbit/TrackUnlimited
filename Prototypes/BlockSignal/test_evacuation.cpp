// Asserts for Evacuation.h.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_evacuation test_evacuation.cpp && ./test_evacuation

#include "Evacuation.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{

FWalkwaySpan W(double A, double B, EWalkway Side = EWalkway::Both)
{
    FWalkwaySpan S; S.StartS = A; S.EndS = B; S.Side = Side; return S;
}
FStoppedTrain T(double Rear, double Front, int Index = 0)
{
    FStoppedTrain X; X.RearS = Rear; X.FrontS = Front; X.Index = Index; return X;
}

void TestATrainFullyBesideACatwalkIsFine()
{
    const FEvacVerdict V = CheckEvacuation({W(100.0, 200.0)}, {T(120.0, 140.0)}, 500.0);
    assert(V.bEveryoneCanWalkOff);
    assert(V.Findings.empty());
    assert(std::fabs(V.TrackCoverageM - 100.0) < 1e-9);
    std::printf("  a train standing on a catwalk is reachable\n");
}

void TestATrainOnOpenCourseIsNot()
{
    // The case the E-stop measurement could never see. "Every train came to rest
    // at a holding device, no violation" is a statement about SIGNALLING; it says
    // nothing about whether anybody can walk to them.
    const FEvacVerdict V = CheckEvacuation({W(100.0, 200.0)}, {T(300.0, 320.0, 2)}, 500.0);
    assert(!V.bEveryoneCanWalkOff);
    assert(V.Findings.size() == 1);
    assert(V.Findings[0].Train == 2);
    assert(std::fabs(V.Findings[0].UnservedMetres - 20.0) < 1e-9);
    std::printf("  a train on open course is not — 20.0 m unserved\n");
}

void TestPartlyBesideItIsStillAProblem()
{
    // THE METRIC IS THE WORST GAP, NOT A PERCENTAGE. A 40 m train with 4 m
    // hanging off the end of the catwalk is not "90% fine" — it is a back car
    // full of people that somebody has to reach, and the back car is exactly the
    // one that gets forgotten. That is why the train is given as nose and tail
    // rather than as a position.
    const FEvacVerdict V = CheckEvacuation({W(100.0, 200.0)}, {T(196.0, 236.0)}, 500.0);
    assert(!V.bEveryoneCanWalkOff);
    assert(std::fabs(V.Findings[0].UnservedMetres - 36.0) < 1e-9);
    assert(std::fabs(V.Findings[0].WorstGapStartS - 200.0) < 1e-9);
    std::printf("  a train hanging off the end reports the 36.0 m that is unserved\n");
}

void TestAGapInTheMiddleIsFound()
{
    // Two catwalks with a hole between them, and a train straddling the hole.
    // The cursor walk has to notice an interior gap, not just the ends.
    const FEvacVerdict V = CheckEvacuation({W(0.0, 100.0), W(130.0, 300.0)},
                                           {T(80.0, 150.0)}, 300.0);
    assert(!V.bEveryoneCanWalkOff);
    assert(std::fabs(V.Findings[0].UnservedMetres - 30.0) < 1e-9);
    assert(std::fabs(V.Findings[0].WorstGapStartS - 100.0) < 1e-9);
    std::printf("  a 30 m hole between two catwalks is found under a straddling train\n");
}

void TestAbuttingSpansOfDIFFERENTSidesAreOneWalkway()
{
    // Real layouts swap which side the catwalk runs down. Requiring one side
    // end-to-end would fail almost every real ride, and a person can step across
    // at the join — so reachability merges them and the SIDE is reported rather
    // than required to match.
    const FEvacVerdict V = CheckEvacuation({W(0.0, 100.0, EWalkway::Left),
                                            W(100.0, 200.0, EWalkway::Right)},
                                           {T(90.0, 110.0)}, 200.0);
    assert(V.bEveryoneCanWalkOff);
    std::printf("  a left catwalk abutting a right one is one continuous route\n");
}

void TestOverlappingAndUnsortedSpansAreHandled()
{
    // Spans come from a segment walk and there is no reason they arrive sorted
    // or disjoint — a launch with walkways authored per segment produces plenty
    // of both.
    const FEvacVerdict V = CheckEvacuation({W(150.0, 250.0), W(0.0, 60.0), W(40.0, 160.0)},
                                           {T(10.0, 240.0)}, 300.0);
    assert(V.bEveryoneCanWalkOff);
    assert(std::fabs(V.TrackCoverageM - 250.0) < 1e-9);
    std::printf("  unsorted, overlapping spans merge to 250.0 m of continuous route\n");
}

void TestNoWalkwaysAtAllFailsEveryTrain()
{
    const FEvacVerdict V = CheckEvacuation({}, {T(0.0, 20.0, 0), T(100.0, 120.0, 1)}, 500.0);
    assert(!V.bEveryoneCanWalkOff);
    assert(V.Findings.size() == 2);
    assert(V.TrackCoverageM == 0.0);
    std::printf("  a layout with no catwalks fails every train, which is correct\n");
}

void TestASideOfNONEIsNotAWalkway()
{
    // A segment that carries the field but has it set to None must not count.
    // The span list comes from a walk over every segment, so this is the common
    // case rather than an edge one.
    const FEvacVerdict V = CheckEvacuation({W(0.0, 500.0, EWalkway::None)},
                                           {T(100.0, 120.0)}, 500.0);
    assert(!V.bEveryoneCanWalkOff);
    assert(V.TrackCoverageM == 0.0);
    std::printf("  a span authored None is not a catwalk\n");
}

void TestATrainStraddlingTheSEAM()
{
    // THE TRAP THIS PROJECT KEEPS MEETING, one layer at a time. On a circuit a
    // train over the wrap has its REAR at a greater arc length than its FRONT.
    // That is a real state, not a caller error, and the first version of this
    // walk terminated immediately on it and reported NO GAP — which is the worst
    // possible answer, because it says a stranded train is fine.
    //
    // Found by wiring the check to the real E-stop measurement rather than by
    // thinking about it.
    const double Total = 600.0;

    // Rear 590, front 10: the train spans the seam, and NOTHING is beside it.
    {
        const FEvacVerdict V = CheckEvacuation({W(100.0, 200.0)},
                                               {T(590.0, 10.0)}, Total);
        assert(!V.bEveryoneCanWalkOff);
        assert(std::fabs(V.Findings[0].UnservedMetres - 20.0) < 1e-9);
    }

    // Now catwalk it, on both sides of the seam. A walkway ending at 600 and one
    // starting at 0 are CONTINUOUS on a ring; left unmerged the join reads as a
    // zero-width hole and the train comes back stranded by nothing.
    {
        const FEvacVerdict V = CheckEvacuation({W(560.0, 600.0), W(0.0, 40.0)},
                                               {T(590.0, 10.0)}, Total);
        assert(V.bEveryoneCanWalkOff);
    }

    // And a gap that STRADDLES the seam is one gap, not two. Split in halves it
    // would report 10 m twice; it is 20 m of train somebody has to reach.
    {
        const FEvacVerdict V = CheckEvacuation({W(500.0, 590.0), W(10.0, 100.0)},
                                               {T(585.0, 15.0)}, Total);
        assert(!V.bEveryoneCanWalkOff);
        assert(std::fabs(V.Findings[0].UnservedMetres - 20.0) < 1e-9);
    }
    std::printf("  a train over the seam is measured as one train, not two halves\n");
}

void TestWorstFirst()
{
    // An author fixes the biggest hole, not the earliest.
    const FEvacVerdict V = CheckEvacuation({}, {T(0.0, 10.0, 0), T(100.0, 180.0, 1)}, 500.0);
    assert(V.Findings[0].Train == 1);
    assert(V.Findings[0].UnservedMetres > V.Findings[1].UnservedMetres);
    std::printf("  findings are worst-first\n");
}

} // namespace

int main()
{
    std::printf("Evacuation: can everybody get off?\n\n");

    TestATrainFullyBesideACatwalkIsFine();
    TestATrainOnOpenCourseIsNot();
    TestPartlyBesideItIsStillAProblem();
    TestAGapInTheMiddleIsFound();
    TestAbuttingSpansOfDIFFERENTSidesAreOneWalkway();
    TestOverlappingAndUnsortedSpansAreHandled();
    TestNoWalkwaysAtAllFailsEveryTrain();
    TestASideOfNONEIsNotAWalkway();
    TestATrainStraddlingTheSEAM();
    TestWorstFirst();

    std::printf("\ntest_evacuation: all assertions passed.\n");
    return 0;
}
