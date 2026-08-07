// TrackUnlimited Tier 2: the GENERATED DEFAULT PROGRAM.
// Plain C++17, no dependencies beyond PlcExpr/PlcImage.
//
// The single most important usability decision in this architecture, and it comes
// straight from watching NoLimits 2 get it wrong.
//
// ===================== WHAT NL2 DID =====================
//
// NL2 has scripting, and its fatal decision is that ENTERING SCRIPTED MODE
// DISABLES AUTOMATIC BLOCK MODE ENTIRELY. You then hand-code every brake, block,
// transport and dispatch. Community reports put a station departure plus the first
// brake run at 500+ lines, and a transfer table at 1000-1500. Adoption stayed low
// outside cosmetic uses, and the API is widely described as near-unreusable
// between coasters.
//
// The failure is not the language. It is that customising ONE BRAKE costs you the
// whole ride.
//
// ===================== SO: OVERRIDE IS PER BLOCK, NEVER A MODE =====================
//
// The generated, data-driven block system is ALWAYS running. An override attaches
// to ONE block and is an extra AND on that block's permissive. There is no global
// switch that trades "everything works" for "you now own all of it", and if a
// future change would ever require somebody to hand-write their whole block
// system to customise one brake, that change is wrong.
//
// ===================== AND THE GENERATED PROGRAM IS THE TUTORIAL =====================
//
// `CLAUDE.md` § "The control layer is a LAYER OF CHOICE": the best boilerplate is
// the default program itself — readable, for YOUR track, in the syntax you would
// write an override in. It beats a blank editor and it beats a manual, because it
// is already correct and already about the ride in front of you.
//
// That is why this emits TEXT rather than only structure. The text is the feature.

#pragma once

#include "PlcExpr.h"
#include "PlcImage.h"

#include <string>
#include <vector>

// One device on the track, as the generator needs to see it. Deliberately a plain
// description rather than an FTrain or an FCircuit: the generator walks the same
// ordered list the mesh, the physics and the control panel walk, and taking it as
// data is what keeps this testable without a ride.
struct FProgramZone
{
    std::string Name;              // "STATION", "LAUNCH", "MID-COURSE BRAKE"
    std::size_t Block = 0;         // the block this device bounds
    double ReleaseSpeed = 0.0;     // m/s, what it is authored to release at
    bool bHolding = false;         // pads AND tyres: can stop a train and start one
    int Platform = -1;             // index into platform[], or -1
    int NextDevice = -1;           // the zone a train is dispatched INTO, or -1
    std::vector<std::size_t> Ahead;// blocks that must be clear to release
};

// One block's permissive: what was generated, and what the author replaced it with.
//
// A POU in IEC 61131-3 terms, though a small one — Tier 2 is expressions, so a
// "program organisation unit" here is a named boolean over the process image.
struct FPlcPou
{
    std::string Name;              // "DISPATCH_BLOCK_3"
    std::string Comment;           // why it says what it says
    std::string Generated;         // the default, always regenerated
    std::string Override;          // the author's, never regenerated over
    bool bHasOverride = false;

    // A custom POU that fails to parse falls back to the generated one WITH A LOUD
    // WARNING rather than leaving the block unhandled. An unhandled block is a
    // train nobody is holding.
    std::string OverrideError;
    bool bOverrideFellBack = false;

    // What actually runs.
    const std::string& Effective() const
    {
        return bHasOverride && OverrideError.empty() ? Override : Generated;
    }
};

class FPlcProgram
{
public:
    // ===================== GENERATE, FROM THE SAME WALK AS EVERYTHING ELSE =====================
    //
    // One POU per HOLDING device, because those are the only places a train is
    // ever released from — a trim brake cannot start a train and a launch cannot
    // stop one, so neither has a dispatch permissive to override.
    //
    // Regeneration NEVER touches an override. A track edit that changes the block
    // count needs an explicit reconcile, which is the one thing that must not be
    // silent: somebody's work is in those strings.
    void Generate(const std::vector<FProgramZone>& Zones)
    {
        std::vector<FPlcPou> Fresh;
        for (const FProgramZone& Z : Zones)
        {
            if (!Z.bHolding) { continue; }
            Fresh.push_back(MakePou(Z));
        }

        // Carry every override across by NAME rather than by index, so inserting a
        // block earlier in the layout does not silently move somebody's override
        // onto a different brake. Names are derived from the block, so a block that
        // still exists keeps its override and one that does not is reported.
        for (FPlcPou& New : Fresh)
        {
            for (const FPlcPou& Old : Pou)
            {
                if (Old.Name == New.Name && Old.bHasOverride)
                {
                    New.Override = Old.Override;
                    New.bHasOverride = true;
                    New.OverrideError = Old.OverrideError;
                    New.bOverrideFellBack = Old.bOverrideFellBack;
                }
            }
        }

        // ORPHANS ARE REPORTED, NEVER DROPPED SILENTLY. An override whose block no
        // longer exists is somebody's work about to vanish, and the only honest
        // thing is to say so and make them decide.
        Orphaned.clear();
        for (const FPlcPou& Old : Pou)
        {
            if (!Old.bHasOverride) { continue; }
            bool bStillThere = false;
            for (const FPlcPou& New : Fresh)
            {
                if (New.Name == Old.Name) { bStillThere = true; break; }
            }
            if (!bStillThere) { Orphaned.push_back(Old); }
        }

        Pou = Fresh;
    }

    std::size_t Num() const { return Pou.size(); }
    const FPlcPou& At(std::size_t i) const { return Pou[i]; }

    // Overrides carried across a regeneration whose block has gone. Non-empty means
    // a reconcile is owed to somebody.
    const std::vector<FPlcPou>& OrphanedOverrides() const { return Orphaned; }

    // ===================== SET AN OVERRIDE ON ONE BLOCK =====================
    //
    // Parsed and bound immediately, against the image it will actually run on. A
    // failure does NOT reject the edit — the author keeps their text, the block
    // keeps running its generated default, and the error is attached where the
    // editor can show it. Rejecting the text would lose work; running an unparsed
    // block would lose a train.
    bool SetOverride(std::size_t Index, const std::string& Source, const FProcessImage& Image)
    {
        if (Index >= Pou.size()) { return false; }
        FPlcPou& P = Pou[Index];
        P.Override = Source;
        P.bHasOverride = !Source.empty();
        P.OverrideError.clear();
        P.bOverrideFellBack = false;
        if (!P.bHasOverride) { return true; }

        const FPlcExpr E = FPlcExpr::Parse(Source, Image);
        if (!E.IsValid())
        {
            P.OverrideError = E.Error();
            P.bOverrideFellBack = true;
            return false;
        }
        if (E.ResultType() != EPlcType::Bool)
        {
            P.OverrideError = "an override must be a permissive and yield BOOL";
            P.bOverrideFellBack = true;
            return false;
        }
        return true;
    }

    void ClearOverride(std::size_t Index)
    {
        if (Index < Pou.size())
        {
            Pou[Index] = FPlcPou{Pou[Index].Name, Pou[Index].Comment, Pou[Index].Generated,
                                 "", false, "", false};
        }
    }

    std::size_t NumOverridden() const
    {
        std::size_t N = 0;
        for (const FPlcPou& P : Pou) { if (P.bHasOverride) { ++N; } }
        return N;
    }
    std::size_t NumFellBack() const
    {
        std::size_t N = 0;
        for (const FPlcPou& P : Pou) { if (P.bOverrideFellBack) { ++N; } }
        return N;
    }

    // ===================== THE PROGRAM, AS TEXT =====================
    //
    // The tutorial. Readable, for this track, in the syntax an override is written
    // in — which beats a blank editor and beats a manual, because it is already
    // correct and already about the ride in front of you.
    std::string Text() const
    {
        std::string Out =
            "(* Generated control program.\n"
            "   One permissive per holding device, derived from this layout.\n"
            "   Everything here is regenerated when the track changes; an override\n"
            "   attaches to ONE block and is an extra AND on that block's chain. *)\n";
        for (const FPlcPou& P : Pou)
        {
            Out += "\n(* " + P.Comment + " *)\n";
            Out += P.Name + " :=\n    " + P.Generated + ";\n";
            if (P.bHasOverride)
            {
                if (P.OverrideError.empty())
                {
                    Out += "(* OVERRIDDEN by this layout's author: *)\n";
                    Out += P.Name + "_OVERRIDE :=\n    " + P.Override + ";\n";
                }
                else
                {
                    // LOUD, because a block silently running its default when the
                    // author believes it is running their override is the worst of
                    // the three outcomes.
                    Out += "(* !! OVERRIDE WILL NOT LOAD: " + P.OverrideError + "\n";
                    Out += "   !! THIS BLOCK IS RUNNING ITS GENERATED DEFAULT.\n";
                    Out += "   !! " + P.Override + " *)\n";
                }
            }
        }
        return Out;
    }

private:
    static std::string Speed(double V)
    {
        // One decimal, because these are m/s and nobody needs a millimetre per
        // second in a permissive.
        const long long Tenths = static_cast<long long>(V * 10.0 + (V >= 0 ? 0.5 : -0.5));
        return std::to_string(Tenths / 10) + "." + std::to_string(Tenths % 10 < 0 ? -(Tenths % 10)
                                                                                  : Tenths % 10);
    }

    static FPlcPou MakePou(const FProgramZone& Z)
    {
        FPlcPou P;
        P.Name = "DISPATCH_BLOCK_" + std::to_string(Z.Block);
        P.Comment = Z.Name + " (block " + std::to_string(Z.Block)
                  + ", releases at " + Speed(Z.ReleaseSpeed) + " m/s)";

        // THE PERMISSIVE IS AN AND, and it is the same one ServeHolds builds — this
        // is not a simplified illustration. Blocks clear, the station says go, and
        // the device about to take the train is ready.
        std::string E;
        for (std::size_t B : Z.Ahead)
        {
            if (!E.empty()) { E += " AND "; }
            E += "block[" + std::to_string(B) + "].clear";
        }
        if (E.empty()) { E = "TRUE"; }

        if (Z.Platform >= 0)
        {
            const std::string Pf = "platform[" + std::to_string(Z.Platform) + "].";
            // Three contacts and a walk-round, spelled out rather than hidden
            // behind one "ready" flag, because seeing them is half the point of
            // reading this.
            E += "\n    AND " + Pf + "restraints_locked AND " + Pf + "gates_closed"
                 "\n    AND " + Pf + "in_position AND " + Pf + "ready";
        }
        if (Z.NextDevice >= 0)
        {
            // PRE-LAUNCH: clear is not the same as ready. A block with a launch in
            // it can be empty and still refuse a train, because the launch has not
            // armed.
            E += "\n    AND zone[" + std::to_string(Z.NextDevice) + "].ready";
        }

        // And the two terms every permissive on every ride carries.
        E += "\n    AND NOT ride.estop AND ride.outputs_enabled";

        P.Generated = E;
        return P;
    }

    std::vector<FPlcPou> Pou;
    std::vector<FPlcPou> Orphaned;
};

// ponytail: the program is emitted as text and parsed back for validation, rather
// than being a tree the editor manipulates directly. That is a round trip nobody
// needs yet — there is no editor — and text is what a person reads, diffs and
// pastes into a forum post when they are asking why their ride will not dispatch.
