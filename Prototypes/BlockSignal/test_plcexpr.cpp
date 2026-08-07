// Asserts for PlcImage.h and PlcExpr.h — Tier 2's process image and override
// expression.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_plcexpr test_plcexpr.cpp && ./test_plcexpr

#include "PlcExpr.h"
#include "PlcImage.h"

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

namespace
{

// The two-train circuit's shape: six blocks, six zones, one platform, two trains.
FProcessImage Circuit()
{
    FProcessImage I;
    I.Declare(6, 6, 1, 2);
    I.SetRide(ERideField::Trains, FPlcValue::Number(2.0));
    I.SetRide(ERideField::EStop, FPlcValue::Boolean(false));
    I.SetRide(ERideField::OutputsEnabled, FPlcValue::Boolean(true));
    for (std::size_t b = 0; b < 6; ++b)
    {
        I.SetBlock(b, EBlockField::Clear, FPlcValue::Boolean(true));
    }
    for (std::size_t z = 0; z < 6; ++z)
    {
        I.SetZone(z, EZoneField::Output, FPlcValue::Number(0.0));
        I.SetZone(z, EZoneField::Ready, FPlcValue::Boolean(true));
        I.SetZone(z, EZoneField::Holding, FPlcValue::Boolean(z != 1));
    }
    I.SetPlatform(0, EPlatformField::Ready, FPlcValue::Boolean(true));
    I.SetTrain(0, ETrainField::Speed, FPlcValue::Number(0.0));
    I.SetTrain(1, ETrainField::Speed, FPlcValue::Number(26.4));
    return I;
}

bool Eval(const std::string& Src, const FProcessImage& I)
{
    const FPlcExpr E = FPlcExpr::Parse(Src, I);
    assert(E.IsValid());
    return E.Evaluate(I).AsBool();
}

std::string WhyNot(const std::string& Src, const FProcessImage& I)
{
    return FPlcExpr::Parse(Src, I).Error();
}

// ---------------------------------------------------------------- the image

void TestTheImageIsDECLAREDFromTheLayout()
{
    // A layout with six blocks has block[0] through block[5] and nothing else.
    // That is what makes an override written against a different track fail to
    // BIND rather than silently read a slot that now means something different —
    // the same argument as the PLC refusing to RUN a program built elsewhere.
    const FProcessImage I = Circuit();
    std::size_t Slot = 0;
    EPlcType T = EPlcType::Bool;
    assert(I.Find("block[5].clear", Slot, T));
    assert(T == EPlcType::Bool);
    assert(!I.Find("block[6].clear", Slot, T));      // no seventh block on this ride
    assert(!I.Find("block[0].warm", Slot, T));       // no such field anywhere

    // Case-insensitive, because IEC 61131-3 identifiers are.
    assert(I.Find("BLOCK[5].CLEAR", Slot, T));
    assert(I.Find("Zone[2].Output", Slot, T));
    assert(T == EPlcType::Real);
    std::printf("  the image is declared from the layout, and an unknown name does not bind\n");
}

void TestTheSHAPEIsGeneratedNotMaintained()
{
    // THE ON-RAMP REQUIREMENT from CLAUDE.md § "The control layer is a LAYER OF
    // CHOICE": autocomplete and the variable reference enumerate the image at
    // runtime. A hand-kept list drifts the first time somebody adds a field, and
    // the drift is silent.
    //
    // So the shape is asserted to be self-describing: every slot Describe()
    // reports must bind by the name it reports, to itself, with the type it
    // reports. That is the property a generated reference needs and a hand-kept
    // one cannot have.
    const FProcessImage I = Circuit();
    const std::vector<FProcessImage::FSlotInfo> All = I.Describe();
    assert(All.size() == I.NumSlots());
    assert(All.size() == 4 + 6 * 3 + 6 * 7 + 1 * 5 + 2 * 2);

    for (const FProcessImage::FSlotInfo& S : All)
    {
        std::size_t Slot = 0;
        EPlcType T = EPlcType::Bool;
        assert(I.Find(S.Name, Slot, T));
        assert(Slot == S.Index);
        assert(T == S.Type);
    }
    std::printf("  all %zu slots describe themselves and bind back to themselves\n", All.size());
}

void TestAWriteOfTheWrongTYPEIsRefused()
{
    // Types belong to the SHAPE, not to whatever was last written. A slot that
    // changed type when the scan wrote it would make a bound expression's type
    // check meaningless — it would have been checked against a type that no
    // longer holds.
    FProcessImage I = Circuit();
    I.SetBlock(0, EBlockField::Clear, FPlcValue::Number(1.0));    // REAL into a BOOL
    std::size_t Slot = 0;
    EPlcType T = EPlcType::Bool;
    assert(I.Find("block[0].clear", Slot, T));
    assert(I.At(Slot).Type == EPlcType::Bool);
    assert(I.At(Slot).bBool);                                     // unchanged, not coerced
    std::printf("  a write of the wrong type is refused rather than coerced\n");
}

// ---------------------------------------------------------------- the language

void TestSTRICTSTSpelling()
{
    // A SUBSET WITH THE WRONG OPERATORS IS NOT A SUBSET. If this taught != and
    // ? : then a later Structured Text would be a replacement rather than an
    // extension, and everything somebody learned here would be worth nothing
    // outside — which is the whole argument for the verbatim-standards choices.
    const FProcessImage I = Circuit();

    assert(Eval("block[0].clear AND block[1].clear", I));
    assert(Eval("NOT ride.estop", I));
    assert(Eval("block[0].clear OR ride.estop", I));
    assert(Eval("ride.trains = 2", I));                 // ONE equals, not two
    assert(Eval("ride.trains <> 3", I));                // <> , not !=
    assert(Eval("train[1].speed > 20.0", I));
    assert(Eval("TRUE XOR FALSE", I));

    // And the C spellings are NOT accepted, which is the half that keeps it a
    // subset rather than a dialect.
    assert(!WhyNot("ride.trains != 3", I).empty());
    assert(!WhyNot("block[0].clear && block[1].clear", I).empty());
    assert(!WhyNot("ride.estop ? 1 : 2", I).empty());
    std::printf("  strict ST spelling, and the C spellings are refused\n");
}

void TestSELTakesITSArgumentsInTHESTANDARDSOrder()
{
    // SEL(G, IN0, IN1) yields IN0 when G is FALSE. That is IEC 61131-3's order and
    // it reads backwards to almost everybody, which is exactly why it is spelled
    // the standard's way instead of a friendlier one.
    const FProcessImage I = Circuit();
    assert(Eval("SEL(FALSE, TRUE, FALSE)", I));         // G false -> IN0
    assert(!Eval("SEL(TRUE, TRUE, FALSE)", I));         // G true  -> IN1

    const FPlcExpr E = FPlcExpr::Parse("SEL(ride.estop, 4.0, 26.0)", I);
    assert(E.IsValid() && E.ResultType() == EPlcType::Real);
    assert(E.Evaluate(I).AsReal() == 4.0);              // not stopped -> IN0

    // MIN and MAX are built on SEL rather than being new opcodes: two fewer cases
    // in the evaluator is two fewer places to be wrong.
    assert(FPlcExpr::Parse("MIN(3.0, 7.0)", I).Evaluate(I).AsReal() == 3.0);
    assert(FPlcExpr::Parse("MAX(3.0, 7.0)", I).Evaluate(I).AsReal() == 7.0);
    std::printf("  SEL(G, IN0, IN1) yields IN0 when G is FALSE, as the standard has it\n");
}

void TestTypesAreCHECKEDAtParseTime()
{
    // ST is strongly typed, and more to the point `zone[0].output AND ...` almost
    // certainly means the author wanted a comparison and forgot it. Caught when
    // the override is loaded rather than becoming a permissive that is quietly
    // always true.
    const FProcessImage I = Circuit();
    assert(!WhyNot("zone[0].output AND block[0].clear", I).empty());
    assert(!WhyNot("NOT ride.trains", I).empty());
    assert(!WhyNot("block[0].clear > block[1].clear", I).empty());   // ordered on BOOL
    assert(!WhyNot("block[0].clear = ride.trains", I).empty());      // BOOL vs REAL
    assert(WhyNot("block[0].clear = block[1].clear", I).empty());    // BOOL vs BOOL is fine
    assert(WhyNot("zone[0].output <= train[1].speed", I).empty());

    // The error says WHERE, because "type error" without an offset in a 200
    // character expression is somewhere to start looking rather than somewhere to
    // go.
    const FPlcExpr Bad = FPlcExpr::Parse("block[0].clear AND zone[0].output", I);
    assert(!Bad.IsValid());
    assert(Bad.ErrorAt() > 0);
    std::printf("  types are checked at parse time: \"%s\"\n", Bad.Error().c_str());
}

void TestAnUnknownNameFAILSToLoad()
{
    // Not FALSE for ever, which is what a permissive would become if a misspelt
    // block quietly bound to nothing. An override must fail to load.
    const FProcessImage I = Circuit();
    assert(!WhyNot("blok[0].clear", I).empty());
    assert(!WhyNot("block[9].clear", I).empty());
    assert(!WhyNot("SQRT(4.0)", I).empty());            // not in the function set
    std::printf("  an unknown name or function fails to load rather than reading FALSE\n");
}

// ---------------------------------------------------------------- the safety rule

void TestAnOverrideCanONLYRestrict()
{
    // THE SAFETY RULE, AND IT IS STRUCTURAL RATHER THAN POLICY.
    //
    // Restrict() is an AND onto the existing chain, so there is no expression —
    // however written, however hostile — that turns a denied permissive into a
    // granted one. That is what makes an override from a downloaded track safe
    // BY CONSTRUCTION rather than by anybody reading it.
    const FProcessImage I = Circuit();

    const FPlcExpr AlwaysTrue = FPlcExpr::Parse("TRUE", I);
    const FPlcExpr AlwaysFalse = FPlcExpr::Parse("FALSE", I);
    assert(AlwaysTrue.IsValid() && AlwaysFalse.IsValid());

    // It can take a permission away.
    assert(!AlwaysFalse.Restrict(true, I));
    // It cannot hand one out. This is the assertion the whole design exists for.
    assert(!AlwaysTrue.Restrict(false, I));
    assert(!AlwaysFalse.Restrict(false, I));
    assert(AlwaysTrue.Restrict(true, I));

    // Nor can anything more inventive, because there is no syntax for it: the
    // expression's value is one operand of an AND and the permissive is the other.
    const char* Hostile[] = {
        "TRUE OR FALSE",
        "NOT ride.estop",
        "ride.trains > -1.0",
        "SEL(TRUE, FALSE, TRUE)",
        "block[0].clear = block[0].clear",
    };
    for (const char* Src : Hostile)
    {
        const FPlcExpr E = FPlcExpr::Parse(Src, I);
        assert(E.IsValid());
        assert(E.Evaluate(I).AsBool());      // every one of them is TRUE
        assert(!E.Restrict(false, I));       // and not one of them grants anything
    }
    std::printf("  an override can only ever restrict — five hostile expressions, none grants\n");
}

void TestABROKENOverrideDoesNotStopTheRide()
{
    // A broken override returns the permissive UNCHANGED rather than FALSE.
    // Otherwise a typo in a downloaded track is a denial of service, and the
    // failure is reported through Error() where somebody can read it — the same
    // "report, never repair" rule as everywhere else here.
    const FProcessImage I = Circuit();
    const FPlcExpr Broken = FPlcExpr::Parse("block[0].clear AND", I);
    assert(!Broken.IsValid());
    assert(Broken.Restrict(true, I));        // the ride runs
    assert(!Broken.Restrict(false, I));      // and a denial stays denied

    // And an override that parses but yields a REAL is not a permissive at all.
    const FPlcExpr Wrong = FPlcExpr::Parse("train[1].speed", I);
    assert(Wrong.IsValid() && Wrong.ResultType() == EPlcType::Real);
    assert(Wrong.Restrict(true, I));
    std::printf("  a broken override leaves the permissive alone rather than stopping the ride\n");
}

void TestNOStateAcrossScans()
{
    // What no-state buys: determinism for free. The same image gives the same
    // answer for ever, so an override drops into FSimDigest unchanged and there is
    // nothing to record or replay.
    //
    // Asserted by evaluating one expression against two images and back again: if
    // anything were remembered, the third answer would differ from the first.
    FProcessImage A = Circuit();
    FProcessImage B = Circuit();
    B.SetBlock(2, EBlockField::Clear, FPlcValue::Boolean(false));

    const FPlcExpr E = FPlcExpr::Parse("block[2].clear AND NOT ride.estop", A);
    assert(E.IsValid());
    assert(E.Evaluate(A).AsBool());
    assert(!E.Evaluate(B).AsBool());
    assert(E.Evaluate(A).AsBool());
    for (int i = 0; i < 1000; ++i)
    {
        assert(E.Evaluate(A).AsBool());
        assert(!E.Evaluate(B).AsBool());
    }
    std::printf("  a pure expression over a snapshot: 1000 alternating evaluations, no drift\n");
}

// ---------------------------------------------------------------- the on-ramp

void TestEVERYDocumentedExampleParsesAndBinds()
{
    // THE ON-RAMP REQUIREMENT, and the same pattern as reference_figures.cpp: an
    // example in the documentation that has gone stale wastes somebody's evening,
    // so every one of them is parsed and bound against a real layout HERE and
    // fails the build rather than the reader.
    //
    // These are the overrides a person would plausibly write, and each is a real
    // answer to "the default will not do what I want".
    const FProcessImage I = Circuit();
    struct FExample { const char* What; const char* Src; };
    const FExample Examples[] = {
        {"hold the station until the mid-course brake is clear as well",
         "block[2].clear"},
        {"do not dispatch while any train is still moving on the ride",
         "NOT train[0].moving AND NOT train[1].moving"},
        {"extra headway: two blocks clear ahead, not one",
         "block[3].clear AND block[4].clear"},
        {"do not launch into a drive that is not up to speed",
         "zone[1].ready AND zone[1].output >= 30.0"},
        {"hold if the transfer drive is pulling more than 90% torque",
         "zone[4].load < 0.9"},
        {"a comment, and an ST block comment at that",
         "(* the lift must be turning *) zone[1].ready"},
        {"a line comment",
         "zone[1].ready // and nothing after this matters"},
        {"belt and braces: never while the ride is stopped",
         "NOT ride.estop AND ride.outputs_enabled"},
        {"speed-dependent, using SEL in the standard's argument order",
         "train[1].speed <= SEL(block[3].clear, 6.0, 26.0)"},
        {"and the platform's own three contacts, restated as an override",
         "platform[0].restraints_locked AND platform[0].gates_closed AND platform[0].in_position"},
    };

    for (const FExample& Ex : Examples)
    {
        const FPlcExpr E = FPlcExpr::Parse(Ex.Src, I);
        if (!E.IsValid())
        {
            std::printf("  STALE EXAMPLE (%s): %s -- %s\n", Ex.What, Ex.Src, E.Error().c_str());
            assert(false);
        }
        // Every documented example must be a PERMISSIVE, or it cannot be used as
        // one and the documentation is teaching something that will not load.
        assert(E.ResultType() == EPlcType::Bool);
    }
    std::printf("  all %zu documented examples parse, bind and yield BOOL\n",
                sizeof(Examples) / sizeof(Examples[0]));
}

void TestTheCostIsKnownAtParseTime()
{
    // No loops and no recursion, so the cost of an override is the size of its
    // tree — countable when it is loaded. That is what replaces an execution
    // budget, and it is why nothing here needs a watchdog of its own.
    const FProcessImage I = Circuit();
    const FPlcExpr Small = FPlcExpr::Parse("block[0].clear", I);
    const FPlcExpr Big = FPlcExpr::Parse(
        "block[0].clear AND block[1].clear AND block[2].clear AND block[3].clear"
        " AND train[1].speed < 30.0 AND NOT ride.estop", I);
    assert(Small.NodeCount() == 1);
    assert(Big.NodeCount() > Small.NodeCount());
    assert(Big.NodeCount() < 32);
    std::printf("  cost is tree size and it is known at load: %zu nodes for six terms\n",
                Big.NodeCount());
}

} // namespace

int main()
{
    std::printf("Tier 2: the process image, and the override expression over it\n\n");

    TestTheImageIsDECLAREDFromTheLayout();
    TestTheSHAPEIsGeneratedNotMaintained();
    TestAWriteOfTheWrongTYPEIsRefused();

    TestSTRICTSTSpelling();
    TestSELTakesITSArgumentsInTHESTANDARDSOrder();
    TestTypesAreCHECKEDAtParseTime();
    TestAnUnknownNameFAILSToLoad();

    TestAnOverrideCanONLYRestrict();
    TestABROKENOverrideDoesNotStopTheRide();
    TestNOStateAcrossScans();

    TestEVERYDocumentedExampleParsesAndBinds();
    TestTheCostIsKnownAtParseTime();

    std::printf("\ntest_plcexpr: all assertions passed.\n");
    return 0;
}
