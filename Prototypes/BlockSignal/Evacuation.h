// TrackUnlimited: can everybody get off?
// Plain C++17, no dependencies — spans in, a verdict out.
//
// An evacuation catwalk is a solid decked walkway with handrails running
// alongside the track, on one side or both. Real ones are fitted to almost any
// segment but cluster where a train is expected to be stopped and where staff
// have to walk anyway: brake runs, launches, shuttle sections, lifts.
//
// ===================== IT IS NOT A ZONE, AND NOT A BLOCK =====================
//
// Exactly the shape bAntiRollback already has, for exactly the same reasons. A
// ZONE is a control device: it has a speed, an authority, and something
// commanding it every frame. A catwalk has none of those. It cannot release a
// train, so it is no more a place to park one than a trim brake is, and it is
// not a block boundary. And it OVERLAPS zones freely — a launch section with
// walkways down both sides is both at once, which is precisely the photograph
// this was modelled from.
//
// So it is a property of TRACK, derived into spans, and it changes no physics
// whatsoever. What it changes is what the layout can be asked.
//
// ===================== THE QUESTION IT ANSWERS =====================
//
// After an emergency stop, is every train somewhere people can walk to?
//
// Nothing in this project could ask that before. The E-stop was measured as
// "every train comes to rest at a holding device, no violation" — which is a
// statement about SIGNALLING and says nothing about whether the riders can then
// be got out on foot rather than with a cherry picker.
//
// It is a property of the LAYOUT rather than of the physics, checkable at edit
// time, and it belongs to the same family as the acceleration envelope and the
// failed-brake question: not "does this ride work" but "what happens to it when
// something does not".

#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

// Which side of the track a rider steps out onto. Sides matter because a train
// is boarded and evacuated from a specific side, and a catwalk on the far side
// of the track from the restraints is a walkway for staff rather than an
// evacuation route.
enum class EWalkway
{
    None,
    Left,
    Right,
    Both,
};

struct FWalkwaySpan
{
    double StartS = 0.0;
    double EndS = 0.0;
    EWalkway Side = EWalkway::None;
};

// Where a train came to rest, and how long it is. Nose and tail rather than a
// centre, because a train is only reachable if the WHOLE of it is: riders sit
// along its length and the ones in the back car are the ones who get forgotten.
struct FStoppedTrain
{
    double RearS = 0.0;
    double FrontS = 0.0;
    int Index = 0;
};

struct FEvacFinding
{
    int Train = 0;
    double RearS = 0.0;
    double FrontS = 0.0;
    // The longest stretch of this train with no walkway beside it, in metres.
    // Zero means fully served; anything else is how far somebody would have to
    // be carried, or how much of the train a ladder has to reach.
    double UnservedMetres = 0.0;
    double WorstGapStartS = 0.0;
};

struct FEvacVerdict
{
    bool bEveryoneCanWalkOff = true;
    std::vector<FEvacFinding> Findings;   // one per train that is not fully served
    double TrackCoverageM = 0.0;          // how much of the layout has a walkway
};

namespace EvacuationDetail
{
    // Merge overlapping spans, ignoring side. Two catwalks that abut are one
    // walkway for reachability purposes even if one is left and one is right —
    // a person can step across at the join. Sides are reported, not required to
    // match, because requiring a single side end-to-end would fail every real
    // layout that swaps which side the walkway runs down.
    inline std::vector<std::pair<double, double>> Merge(std::vector<FWalkwaySpan> In)
    {
        std::vector<std::pair<double, double>> Out;
        std::sort(In.begin(), In.end(),
                  [](const FWalkwaySpan& A, const FWalkwaySpan& B) { return A.StartS < B.StartS; });
        for (const FWalkwaySpan& S : In)
        {
            if (S.Side == EWalkway::None || !(S.EndS > S.StartS)) { continue; }
            if (!Out.empty() && S.StartS <= Out.back().second + 1e-9)
            {
                Out.back().second = std::max(Out.back().second, S.EndS);
            }
            else
            {
                Out.push_back({S.StartS, S.EndS});
            }
        }
        return Out;
    }
}

// Is every stopped train walkable-to, along its whole length?
//
// Deliberately takes STOPPED trains rather than working them out: where a train
// comes to rest after an E-stop is the signalling layer's answer, and this
// should not have a second opinion about it. Feed it the positions the E-stop
// measurement already produces.
inline FEvacVerdict CheckEvacuation(const std::vector<FWalkwaySpan>& Walkways,
                                    const std::vector<FStoppedTrain>& Trains,
                                    double TotalLength)
{
    FEvacVerdict V;
    const std::vector<std::pair<double, double>> W = EvacuationDetail::Merge(Walkways);

    for (const auto& S : W)
    {
        V.TrackCoverageM += std::min(S.second, TotalLength) - std::max(S.first, 0.0);
    }

    for (const FStoppedTrain& T : Trains)
    {
        // Walk the train's span and find the longest stretch with nothing beside
        // it. A train half-served is still a train somebody has to be carried
        // out of, so the metric is the WORST gap rather than a percentage — a
        // percentage would call a 40 m train with 4 m unreachable "90% fine".
        double Cursor = T.RearS;
        double Worst = 0.0;
        double WorstAt = T.RearS;

        for (const auto& S : W)
        {
            if (S.second <= Cursor) { continue; }
            if (S.first >= T.FrontS) { break; }
            if (S.first > Cursor)
            {
                const double Gap = S.first - Cursor;
                if (Gap > Worst) { Worst = Gap; WorstAt = Cursor; }
            }
            Cursor = std::max(Cursor, S.second);
            if (Cursor >= T.FrontS) { break; }
        }
        if (Cursor < T.FrontS)
        {
            const double Gap = T.FrontS - Cursor;
            if (Gap > Worst) { Worst = Gap; WorstAt = Cursor; }
        }

        if (Worst > 1e-9)
        {
            FEvacFinding F;
            F.Train = T.Index;
            F.RearS = T.RearS;
            F.FrontS = T.FrontS;
            F.UnservedMetres = Worst;
            F.WorstGapStartS = WorstAt;
            V.Findings.push_back(F);
            V.bEveryoneCanWalkOff = false;
        }
    }

    std::sort(V.Findings.begin(), V.Findings.end(),
              [](const FEvacFinding& A, const FEvacFinding& B)
              { return A.UnservedMetres > B.UnservedMetres; });
    return V;
}

// ponytail: no ladders, no cherry pickers, no height rule. A catwalk 40 m up a
// lift hill is treated the same as one at ground level, and in reality they are
// very different evacuations. Height is available — the track knows it — and
// adding it means deciding a threshold, which is a judgement nobody here is
// entitled to make yet. Reported as reachable-on-foot or not, and no further.
