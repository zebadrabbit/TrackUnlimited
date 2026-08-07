// TrackUnlimited: the device audit.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_deviceaudit test_deviceaudit.cpp
//
// What is asserted here is mostly that each check FIRES ON THE RIGHT THING and
// stays silent on a healthy layout. A checker that reports everything is as
// useless as one that reports nothing, and the second failure is the one that
// gets shipped because it looks like success.

#include "DeviceAudit.h"

#include <cassert>
#include <cstdio>

namespace
{

FDeviceSpan BlockBrake(double Start, double End, const char* Name = "block brake")
{
    FDeviceSpan D;
    D.StartS = Start;
    D.EndS = End;
    D.bCanHold = true;
    D.bCanRelease = true;      // brakes AND drive tyres
    D.bIsBlockBoundary = true;
    D.Name = Name;
    return D;
}

bool Has(const std::vector<FDeviceFinding>& F, EDeviceProblem P)
{
    for (const FDeviceFinding& X : F) { if (X.Problem == P) { return true; } }
    return false;
}

} // namespace

// A HEALTHY DEVICE REPORTS NOTHING, and this is the assertion that keeps the
// rest honest. Every check below fires on a deliberately broken layout; without
// this one they could all be firing all the time and every test would still pass.
void TestAHealthyBlockBrakeIsSILENT()
{
    std::printf("A healthy block brake is silent\n");

    FDeviceAuditSettings S;
    const std::vector<FDeviceSpan> D{BlockBrake(400.0, 530.0)};

    // 130 m, arriving at 20 m/s: needs 66.7 m at 3 m/s^2. Comfortable.
    const auto F = AuditDevices(D, S, [](double) { return 20.0; });
    for (const FDeviceFinding& X : F) { std::printf("  UNEXPECTED: %s\n", X.What.c_str()); }
    assert(F.empty());
    std::printf("  OK\n\n");
}

void TestADeviceShorterThanItsTrainSaysWHERETheMarkLands()
{
    std::printf("A device shorter than its train says where the stop mark lands\n");

    FDeviceAuditSettings S;
    S.TrainLengthM = 15.0;
    const std::vector<FDeviceSpan> D{BlockBrake(100.0, 110.0)};   // 10 m for a 15 m train

    const auto F = AuditDevices(D, S, [](double) { return 2.0; });
    assert(Has(F, EDeviceProblem::ShorterThanTrain));
    for (const FDeviceFinding& X : F) { std::printf("  %s\n", X.What.c_str()); }

    // THE MESSAGE CARRIES THE NUMBERS, which is the whole point of this pass —
    // "too short" is a rule somebody has to go and look up, and 5 m past the far
    // end is the failure. Asserted rather than admired, or the sentence rots.
    bool bNamesTheOverrun = false;
    for (const FDeviceFinding& X : F)
    {
        if (X.Problem == EDeviceProblem::ShorterThanTrain)
        {
            bNamesTheOverrun = X.What.find("5.0 m PAST") != std::string::npos
                            && X.What.find("Needs 16.0 m") != std::string::npos;
        }
    }
    assert(bNamesTheOverrun);
    std::printf("  OK\n\n");
}

void TestABrakeThatCannotStopSaysWhatItLEAVESAt()
{
    std::printf("A brake that cannot stop says what it leaves at\n");

    FDeviceAuditSettings S;
    S.ServiceDecelMs2 = 3.0;
    const std::vector<FDeviceSpan> D{BlockBrake(400.0, 530.0)};   // 130 m

    // 30.4 m/s needs 154 m. The device has 129 m usable, so it exits moving.
    const auto F = AuditDevices(D, S, [](double) { return 30.4; });
    assert(Has(F, EDeviceProblem::CannotStopArrival));
    for (const FDeviceFinding& X : F) { std::printf("  %s\n", X.What.c_str()); }

    // v^2 - 2aL = 924.16 - 774 = 150.16 -> 12.25 m/s. Checked by hand, because a
    // number this pass INVENTS is the one somebody will act on.
    const double Exit = std::sqrt(30.4 * 30.4 - 2.0 * 3.0 * 129.0);
    assert(std::fabs(Exit - 12.25) < 0.01);
    bool bNamesTheExit = false;
    for (const FDeviceFinding& X : F)
    {
        if (X.Problem == EDeviceProblem::CannotStopArrival)
        {
            bNamesTheExit = X.What.find("12.3 m/s") != std::string::npos;
        }
    }
    assert(bNamesTheExit);
    std::printf("  OK\n\n");
}

void TestATRIMAtABlockBoundaryIsREPORTED()
{
    std::printf("A trim used as a block boundary is reported\n");

    // The single most likely bad edit, and the reason the enum has two brake
    // kinds: a trim can slow a train to a stand and can NEVER start one again.
    FDeviceSpan Trim = BlockBrake(400.0, 530.0, "trim");
    Trim.bCanRelease = false;

    const auto F = AuditDevices({Trim}, FDeviceAuditSettings(), [](double) { return 10.0; });
    assert(Has(F, EDeviceProblem::HoldsButCannotRelease));
    for (const FDeviceFinding& X : F) { std::printf("  %s\n", X.What.c_str()); }

    // AND A LAUNCH IS THE OTHER HALF of the same mistake, so it is asserted the
    // other way round rather than assumed to be symmetrical.
    FDeviceSpan Launch = BlockBrake(400.0, 530.0, "launch");
    Launch.bCanHold = false;
    const auto G = AuditDevices({Launch}, FDeviceAuditSettings(), [](double) { return 10.0; });
    assert(Has(G, EDeviceProblem::ReleasesButCannotHold));
    assert(!Has(G, EDeviceProblem::HoldsButCannotRelease));
    for (const FDeviceFinding& X : G) { std::printf("  %s\n", X.What.c_str()); }
    std::printf("  OK\n\n");
}

void TestATRIMThatIsNOTABlockBoundaryIsSILENT()
{
    std::printf("A trim that is not a block boundary is silent\n");

    // TRIMMING IS WHAT A TRIM IS FOR. It is only a defect when the interlocking
    // has been told to park a train on it — and a checker that complained about
    // every trim on every ride would be turned off within a day, which is how a
    // useful check becomes no check.
    FDeviceSpan Trim = BlockBrake(400.0, 420.0, "trim");
    Trim.bCanRelease = false;
    Trim.bIsBlockBoundary = false;

    const auto F = AuditDevices({Trim}, FDeviceAuditSettings(), [](double) { return 30.0; });
    for (const FDeviceFinding& X : F) { std::printf("  UNEXPECTED: %s\n", X.What.c_str()); }
    assert(F.empty());
    std::printf("  OK\n\n");
}

void TestTheAuditReadsTheProfileATTheDeviceStart()
{
    std::printf("The audit reads the speed at each device, not one speed for the ride\n");

    // Two identical brakes at different points on the same ride. Only the fast
    // one is a problem, and a pass that took a single speed would either report
    // both or neither.
    const std::vector<FDeviceSpan> D{BlockBrake(100.0, 230.0, "upper brake"),
                                     BlockBrake(400.0, 530.0, "lower brake")};

    const auto F = AuditDevices(D, FDeviceAuditSettings(),
        [](double S) { return S < 200.0 ? 8.0 : 30.4; });

    int Reported = 0;
    for (const FDeviceFinding& X : F)
    {
        if (X.Problem == EDeviceProblem::CannotStopArrival)
        {
            ++Reported;
            assert(X.What.find("lower brake") != std::string::npos);
            std::printf("  %s\n", X.What.c_str());
        }
    }
    assert(Reported == 1);
    std::printf("  OK\n\n");
}

int main()
{
    std::printf("DeviceAudit: what a layout's devices will actually do\n\n");

    TestAHealthyBlockBrakeIsSILENT();
    TestADeviceShorterThanItsTrainSaysWHERETheMarkLands();
    TestABrakeThatCannotStopSaysWhatItLEAVESAt();
    TestATRIMAtABlockBoundaryIsREPORTED();
    TestATRIMThatIsNOTABlockBoundaryIsSILENT();
    TestTheAuditReadsTheProfileATTheDeviceStart();

    std::printf("test_deviceaudit: all assertions passed.\n");
    return 0;
}
