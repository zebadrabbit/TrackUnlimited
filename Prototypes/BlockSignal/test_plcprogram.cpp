// Asserts for PlcProgram.h — the generated default control program.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_plcprogram test_plcprogram.cpp && ./test_plcprogram
//
// The card this implements calls itself the single most important usability
// decision in the architecture. What is asserted here is the shape of that
// decision: the default always runs, an override is per block, and nothing
// silently loses somebody's work or leaves a block unhandled.

#include "PlcProgram.h"

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

namespace
{

FProcessImage Circuit()
{
    FProcessImage I;
    I.Declare(6, 6, 1, 2);
    return I;
}

// The two-train circuit, as the generator sees it: station, launch, mid-course
// brake, outer brake, transfer tyres, inner brake.
std::vector<FProgramZone> Layout()
{
    return {
        {"STATION",           0, 1.5,  true,  0, 1, {1}},
        {"LAUNCH",            1, 38.0, false, -1, -1, {}},
        {"MID-COURSE BRAKE",  2, 20.0, true,  -1, 3, {3}},
        {"OUTER BRAKE",       3, 6.0,  true,  -1, 4, {4}},
        {"TRANSFER TYRES",    4, 4.0,  true,  -1, 5, {5}},
        {"INNER BRAKE",       5, 2.0,  true,  -1, 0, {0}},
    };
}

void TestAPOUPerHOLDINGDeviceAndNoOthers()
{
    // One permissive per HOLDING device, because those are the only places a train
    // is ever released from. A trim brake cannot start a train and a launch cannot
    // stop one, so neither has a dispatch permissive to override — and generating
    // one for the launch would be inviting somebody to write an override that can
    // never fire.
    FPlcProgram P;
    P.Generate(Layout());
    assert(P.Num() == 5);                    // six zones, five of them holding
    for (std::size_t i = 0; i < P.Num(); ++i)
    {
        assert(P.At(i).Name.find("DISPATCH_BLOCK_") == 0);
        assert(P.At(i).Name != "DISPATCH_BLOCK_1");   // the launch has no permissive
    }
    std::printf("  five POUs for six zones: the launch has nothing to release from\n");
}

void TestEveryGeneratedPermissivePARSESAndYieldsBOOL()
{
    // THE ONE THAT MATTERS MOST. The generated program is the tutorial — somebody
    // learning this reads the one their own coaster produced — so a program that
    // does not parse in its own language teaches the wrong thing and, worse, would
    // make the round trip a lie.
    const FProcessImage I = Circuit();
    FPlcProgram P;
    P.Generate(Layout());

    for (std::size_t i = 0; i < P.Num(); ++i)
    {
        const FPlcExpr E = FPlcExpr::Parse(P.At(i).Generated, I);
        if (!E.IsValid())
        {
            std::printf("  GENERATED PROGRAM DOES NOT PARSE (%s): %s\n",
                        P.At(i).Name.c_str(), E.Error().c_str());
            assert(false);
        }
        assert(E.ResultType() == EPlcType::Bool);
    }
    std::printf("  every generated permissive parses and binds in its own language\n");
}

void TestTheGeneratedProgramIsTheREALPermissiveNotAnIllustration()
{
    // It has to be the same AND that ServeHolds builds, or reading it teaches
    // something that is not what the ride does. Blocks clear, the station's three
    // contacts and its walk-round, the device ahead ready, and the two terms every
    // permissive on every ride carries.
    FPlcProgram P;
    P.Generate(Layout());

    const FPlcPou& Station = P.At(0);
    assert(Station.Generated.find("block[1].clear") != std::string::npos);
    assert(Station.Generated.find("platform[0].restraints_locked") != std::string::npos);
    assert(Station.Generated.find("platform[0].gates_closed") != std::string::npos);
    assert(Station.Generated.find("platform[0].in_position") != std::string::npos);
    assert(Station.Generated.find("zone[1].ready") != std::string::npos);   // pre-launch
    assert(Station.Generated.find("NOT ride.estop") != std::string::npos);

    // And a brake with no platform does not grow platform terms out of nowhere.
    const FPlcPou& Outer = P.At(2);
    assert(Outer.Name == "DISPATCH_BLOCK_3");
    assert(Outer.Generated.find("platform[") == std::string::npos);
    assert(Outer.Generated.find("zone[4].ready") != std::string::npos);
    std::printf("  the station's permissive carries all three contacts and the pre-launch term\n");
}

void TestAnOverrideIsPERBLOCKAndTheRestKeepsRunning()
{
    // THE DECISION THIS CARD IS ABOUT. NL2's fatal choice was that entering
    // scripted mode disables automatic block mode entirely, so customising ONE
    // BRAKE costs you the whole ride — 500+ lines for a station and a brake run.
    //
    // Here an override attaches to one block. Everything else is still generated,
    // still running, and still regenerates when the track changes.
    const FProcessImage I = Circuit();
    FPlcProgram P;
    P.Generate(Layout());

    assert(P.SetOverride(2, "block[4].clear AND block[5].clear", I));
    assert(P.NumOverridden() == 1);

    // The other four are untouched and still generated.
    for (std::size_t i = 0; i < P.Num(); ++i)
    {
        if (i == 2) { continue; }
        assert(!P.At(i).bHasOverride);
        assert(P.At(i).Effective() == P.At(i).Generated);
    }
    assert(P.At(2).Effective() == "block[4].clear AND block[5].clear");
    std::printf("  one block overridden, four still generated and running\n");
}

void TestABrokenOverrideFALLSBACKLoudlyRatherThanLeavingTheBlockUnhandled()
{
    // Three possible outcomes and only one is defensible.
    //
    //   Reject the edit          — loses the author's work.
    //   Run the unparsed block   — loses a train.
    //   Keep the text, run the   — the author keeps everything, the ride keeps
    //   default, say so LOUDLY     working, and somebody is told.
    //
    // The third, and "loudly" is the load-bearing word: a block silently running
    // its default while its author believes their override is live is the worst
    // of the three, because it looks like the first two never happened.
    const FProcessImage I = Circuit();
    FPlcProgram P;
    P.Generate(Layout());

    assert(!P.SetOverride(1, "block[9].clear", I));           // no such block
    assert(P.At(1).bOverrideFellBack);
    assert(!P.At(1).OverrideError.empty());
    assert(P.At(1).Effective() == P.At(1).Generated);         // the ride still runs
    assert(P.At(1).Override == "block[9].clear");             // and the text is kept

    // A REAL is not a permissive, and that is refused for the same reason.
    assert(!P.SetOverride(3, "zone[0].output", I));
    assert(P.At(3).bOverrideFellBack);
    assert(P.NumFellBack() == 2);

    // And it is loud in the text, which is the thing a person actually reads.
    const std::string T = P.Text();
    assert(T.find("!! OVERRIDE WILL NOT LOAD") != std::string::npos);
    assert(T.find("RUNNING ITS GENERATED DEFAULT") != std::string::npos);
    std::printf("  a broken override keeps the text, runs the default, and says so loudly\n");
}

void TestREGENERATIONNeverSilentlyOverwritesAnOverride()
{
    // A track edit regenerates every default. Somebody's work is in those other
    // strings, and losing it to a geometry tweak would be unforgivable in a way
    // that is very easy to ship.
    const FProcessImage I = Circuit();
    FPlcProgram P;
    P.Generate(Layout());
    assert(P.SetOverride(3, "block[5].clear AND NOT ride.estop", I));

    // The author lengthens the mid-course brake. Same blocks, different speeds.
    std::vector<FProgramZone> Edited = Layout();
    Edited[2].ReleaseSpeed = 18.0;
    P.Generate(Edited);

    assert(P.NumOverridden() == 1);
    assert(P.At(3).Override == "block[5].clear AND NOT ride.estop");
    // POU index 1 is the mid-course brake: the POU list holds only HOLDING zones,
    // so it is not the zone list with the same indices.
    assert(P.At(1).Name == "DISPATCH_BLOCK_2");
    assert(P.At(1).Comment.find("18.0") != std::string::npos);
    std::printf("  a regeneration updates every default and touches no override\n");
}

void TestOverridesFollowTheBLOCKNotTheINDEX()
{
    // Insert a device earlier in the layout and every index after it shifts. An
    // override carried across by POSITION would silently land on a different
    // brake — which is a change nobody made, applied to a device nobody chose.
    //
    // So they are carried by NAME, and the name is derived from the block.
    const FProcessImage I = Circuit();
    FPlcProgram P;
    P.Generate(Layout());
    assert(P.SetOverride(4, "block[0].clear", I));            // the inner brake, block 5
    assert(P.At(4).Name == "DISPATCH_BLOCK_5");

    // A new holding device is authored at the front. Blocks keep their identities;
    // the POU list gets one longer.
    std::vector<FProgramZone> Grown = Layout();
    Grown.insert(Grown.begin(), {"PRE-STATION HOLD", 9, 2.0, true, -1, 0, {0}});
    P.Generate(Grown);

    assert(P.Num() == 6);
    assert(P.At(0).Name == "DISPATCH_BLOCK_9");
    assert(!P.At(0).bHasOverride);                            // the new one is plain
    assert(P.At(5).Name == "DISPATCH_BLOCK_5");
    assert(P.At(5).bHasOverride);                             // and block 5 kept its own
    assert(P.At(5).Override == "block[0].clear");
    std::printf("  an inserted device shifts every index and moves nobody's override\n");
}

void TestAnORPHANEDOverrideIsREPORTEDNotDropped()
{
    // The block an override was attached to is deleted. That is somebody's work
    // about to vanish, and the only honest thing is to say so and let them decide
    // — the explicit reconcile the card asks for.
    const FProcessImage I = Circuit();
    FPlcProgram P;
    P.Generate(Layout());
    assert(P.SetOverride(2, "block[4].clear", I));
    assert(P.At(2).Name == "DISPATCH_BLOCK_3");

    std::vector<FProgramZone> Shrunk = Layout();
    Shrunk.erase(Shrunk.begin() + 3);                         // the outer brake goes
    P.Generate(Shrunk);

    assert(P.Num() == 4);
    assert(P.OrphanedOverrides().size() == 1);
    assert(P.OrphanedOverrides()[0].Name == "DISPATCH_BLOCK_3");
    assert(P.OrphanedOverrides()[0].Override == "block[4].clear");
    std::printf("  deleting an overridden block reports the orphan rather than eating it\n");
}

void TestTheTEXTIsTheTutorial()
{
    // `CLAUDE.md` § "The control layer is a LAYER OF CHOICE": the best boilerplate
    // is the default program itself — readable, for YOUR track, in the syntax you
    // would write an override in. It beats a blank editor and it beats a manual,
    // because it is already correct and already about the ride in front of you.
    FPlcProgram P;
    P.Generate(Layout());
    const std::string T = P.Text();

    assert(T.find("STATION") != std::string::npos);
    assert(T.find("MID-COURSE BRAKE") != std::string::npos);
    assert(T.find("DISPATCH_BLOCK_0 :=") != std::string::npos);
    assert(T.find("(*") != std::string::npos);                // ST comments, not //
    assert(T.size() > 500);

    std::printf("\n%s\n", T.c_str());
}

} // namespace

int main()
{
    std::printf("The generated default control program\n\n");

    TestAPOUPerHOLDINGDeviceAndNoOthers();
    TestEveryGeneratedPermissivePARSESAndYieldsBOOL();
    TestTheGeneratedProgramIsTheREALPermissiveNotAnIllustration();
    TestAnOverrideIsPERBLOCKAndTheRestKeepsRunning();
    TestABrokenOverrideFALLSBACKLoudlyRatherThanLeavingTheBlockUnhandled();
    TestREGENERATIONNeverSilentlyOverwritesAnOverride();
    TestOverridesFollowTheBLOCKNotTheINDEX();
    TestAnORPHANEDOverrideIsREPORTEDNotDropped();
    TestTheTEXTIsTheTutorial();

    std::printf("test_plcprogram: all assertions passed.\n");
    return 0;
}
