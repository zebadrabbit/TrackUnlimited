#include "TUCoasterRide.h"

#include "ProceduralMeshComponent.h"
#include "RenderCore.h"                 // SetNearClipPlaneGlobals
#include "Blueprint/UserWidget.h"
#include "UI/TUFrameWidget.h"
#include "UI/TUMenuWidget.h"
#include "UI/TUSegmentEditorWidget.h"
#include "UI/TUPaintedPanelWidget.h"
#include "Framework/Application/SlateApplication.h"
#include "Fonts/FontMeasure.h"
#include "UI/TUStyle.h"
#include "Framework/Application/NavigationConfig.h"
#include "Framework/Application/SlateApplication.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"

#include "Shell/SettingsSchema.h"
#include "TrackSpline/TrackClose.h"
#include "TrackSpline/TrackValidate.h"

#include "Camera/CameraComponent.h"
#include "CanvasItem.h"
#include "Components/InputComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Debug/DebugDrawService.h"
#include "DrawDebugHelpers.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#if WITH_EDITOR
// EDITOR ONLY, and the include is guarded for the same reason the call is: the
// module does not exist in a packaged build, so an unguarded include is a link
// error in the one configuration nobody compiles until release day.
#include "DesktopPlatformModule.h"
#include "Framework/Application/SlateApplication.h"
#include "IDesktopPlatform.h"
#endif
#include "Misc/App.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Sound/SoundBase.h"
#include "Sound/SoundAttenuation.h"
#include "UObject/ConstructorHelpers.h"

// The ride's own event stream. Its own category so it can be filtered on its
// own — with transitions on it is the loudest thing in the log by a wide margin,
// and mixed into LogTemp it would bury everything else.
DEFINE_LOG_CATEGORY(LogTUEvents);

// The authored side -> the model's side. One place, so the two enums cannot
// drift into disagreeing about what Left means.
static EWalkway ToWalkwaySide(ETUWalkway In)
{
	switch (In)
	{
	case ETUWalkway::Left:  return EWalkway::Left;
	case ETUWalkway::Right: return EWalkway::Right;
	case ETUWalkway::Both:  return EWalkway::Both;
	default:                return EWalkway::None;
	}
}


namespace
{
	// The prototypes work in metres; Unreal works in centimetres.
	constexpr double MetresToUU = 100.0;

	constexpr double Pi = 3.14159265358979323846;
	double Deg(double D) { return D * Pi / 180.0; }
}

ATUCoasterRide::ATUCoasterRide()
{
	PrimaryActorTick.bCanEverTick = true;
	AutoPossessPlayer = EAutoReceiveInput::Player0;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	Cars = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("Cars"));
	Cars->SetupAttachment(Root);
	Cars->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Cars->SetCastShadow(false);
	// The shell's frame, found the same way the cars find their cube: by path, so
	// a fresh actor has a UI without anybody wiring one up. Overridable in the
	// Details panel, and null is a valid state.
	static ConstructorHelpers::FClassFinder<UTUFrameWidget> FrameBP(
		TEXT("/Game/UI/WBP_TUFrame"));
	if (FrameBP.Succeeded())
	{
		FrameWidgetClass = FrameBP.Class;
	}

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Cars->SetStaticMesh(CubeMesh.Object);
	}

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Root);

	// PHASE 4. Three components rather than one: rails, spine and ties are three
	// materials — running rail is polished where the wheels touch it, spine is
	// painted structure, ties are neither — and a track style may want to replace
	// one of them without touching the others.
	//
	// COLLISION OFF on all three. A quarter of a million triangles of collision
	// geometry costs a great deal and buys nothing: the train is constrained to
	// the spline analytically and does not collide with anything, which is the
	// whole reason FTrain is 1D rather than a rigid body.
	auto MakeTrackMesh = [this](const TCHAR* Name)
	{
		UProceduralMeshComponent* C = CreateDefaultSubobject<UProceduralMeshComponent>(Name);
		C->SetupAttachment(Root);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->bUseAsyncCooking = true;
		return C;
	};
	RailMesh = MakeTrackMesh(TEXT("RailMesh"));
	SpineMesh = MakeTrackMesh(TEXT("SpineMesh"));
	TieMesh = MakeTrackMesh(TEXT("TieMesh"));
	SupportMesh = MakeTrackMesh(TEXT("SupportMesh"));
	CatwalkDeckMesh = MakeTrackMesh(TEXT("CatwalkDeckMesh"));
	CatwalkRailMesh = MakeTrackMesh(TEXT("CatwalkRailMesh"));
	DeviceSteelMesh = MakeTrackMesh(TEXT("DeviceSteelMesh"));
	DeviceRubberMesh = MakeTrackMesh(TEXT("DeviceRubberMesh"));
	StationConcreteMesh = MakeTrackMesh(TEXT("StationConcreteMesh"));
	StationSteelMesh = MakeTrackMesh(TEXT("StationSteelMesh"));
	StationStripeMesh = MakeTrackMesh(TEXT("StationStripeMesh"));

	// AND THE TRAIN. Four more, for the same reason and on the same terms: four
	// materials, no collision. These are the only ones that move every frame,
	// which is why RebuildTrainMesh updates their vertices in place rather than
	// recreating the sections.
	TrainBodyMesh = MakeTrackMesh(TEXT("TrainBodyMesh"));
	TrainChassisMesh = MakeTrackMesh(TEXT("TrainChassisMesh"));
	TrainWheelMesh = MakeTrackMesh(TEXT("TrainWheelMesh"));
	TrainCouplerMesh = MakeTrackMesh(TEXT("TrainCouplerMesh"));

	// Seeded so a freshly placed actor has a ride in it. Everything about that
	// ride is data in the Details panel rather than code, which is the whole
	// point — Preset + bLoadPreset puts a known-good one back if an edit goes
	// wrong, or swaps in a different worked example to take apart.
	Segments = PresetLayout(Preset);

	// AND THE SESSION STARTS IN BUILD, not in Boot.
	//
	// This matters more than it looks: EditsAllowed() is false everywhere except
	// Build, and PostEditChangeProperty now refuses on it — so a session left in
	// Boot would silently reject every number typed into the Details panel, which
	// is the whole editing surface today.
	//
	// Boot exists to discover a crash sidecar before anything can overwrite it,
	// and an actor placed in a level has already skipped that: the level IS the
	// document. The menu is for the packaged shell, which starts somewhere else.
	Session.Enter(EAppMode::MainMenu);
	Session.Enter(EAppMode::Build);
}

// The drawing palette and its two primitives, HOISTED ABOVE EVERY PANEL THAT
// USES THEM. There are four now — the mode banner, the profile graph, the
// diagnostics list and the control panel — and a palette declared beside the
// last one only compiles for the last one.
//
// Shared deliberately rather than duplicated: the panels are meant to look like
// one instrument seen from different angles, and two copies of a colour drift
// the first time somebody adjusts one.
namespace
{
	// ===================== ONE PALETTE, ONE TYPE =====================
	//
	// These are FTUStyle's colours, not a second set that happened to be near
	// them. The frame is UMG and every panel is the debug canvas, and with two
	// palettes the screen read as a designed header stapled onto a developer
	// overlay. Alpha on the ground is the only thing the canvas adds.
	const FLinearColor PanelGround(FTUStyle::Panel.R, FTUStyle::Panel.G, FTUStyle::Panel.B, 0.92f);
	const FLinearColor PanelRule = FTUStyle::Border;
	const FLinearColor PanelText = FTUStyle::TextPrimary;
	const FLinearColor PanelDim = FTUStyle::TextSecondary;
	const FLinearColor PanelAmber = FTUStyle::LampOccupied;
	const FLinearColor PanelCyan = FTUStyle::LampMeasured;
	const FLinearColor PanelGreen = FTUStyle::LampClear;
	const FLinearColor PanelRed = FTUStyle::LampFault;

	// THE FRAME'S BODY SIZE, as near as a UFont gets. The canvas drew everything
	// in the engine's "small" font, which is the one size smaller than anything
	// else on screen -- that, more than colour, is what made the panels look
	// like debug output.
	UFont* PanelFont() { return GEngine->GetMediumFont(); }

	// ===================== THE RECORDER =====================
	//
	// While UTUPaintedPanelWidget is painting, the four primitives below append
	// to this list instead of drawing; the widget replays it. Null otherwise, and
	// then they draw on the canvas exactly as before. See that widget's header.
	TArray<FTUPanelCmd>* GPanelRecord = nullptr;
	float GPanelSizeY = 0.f;
	float PanelSizeY(UCanvas* C) { return GPanelRecord ? GPanelSizeY : static_cast<float>(C->SizeY); }

	// MEASURED, NOT ESTIMATED. Every tile behind a line of text was sized as
	// `Len * 6.2`, which was a guess for one font and is wrong for any other.
	float PanelTextWidth(UCanvas* C, const FString& S)
	{
		if (GPanelRecord)
		{
			// Measured in the font the widget will DRAW in, which is the only
			// measurement that makes a tile fit the text on it.
			return static_cast<float>(FSlateApplication::Get().GetRenderer()->GetFontMeasureService()
				->Measure(S, FTUStyle::Get().GetFontStyle("Font.Small")).X);
		}
		float W = 0.f, H = 0.f;
		C->TextSize(PanelFont(), S, W, H);
		return W;
	}

	void PanelTile(UCanvas* C, float X, float Y, float W, float H, const FLinearColor& Col)
	{
		if (GPanelRecord) { GPanelRecord->Add({FTUPanelCmd::Tile, FVector2D(X, Y), FVector2D(W, H), Col}); return; }
		FCanvasTileItem Tile(FVector2D(X, Y), FVector2D(W, H), Col);
		Tile.BlendMode = SE_BLEND_Translucent;
		C->DrawItem(Tile);
	}

	void PanelLabel(UCanvas* C, float X, float Y, const FString& S, const FLinearColor& Col)
	{
		if (GPanelRecord) { GPanelRecord->Add({FTUPanelCmd::Label, FVector2D(X, Y), FVector2D::ZeroVector, Col, S}); return; }
		FCanvasTextItem Item(FVector2D(X, Y), FText::FromString(S), PanelFont(), Col);
		Item.EnableShadow(FLinearColor::Black);
		C->DrawItem(Item);
	}

	// The large size, for the three things on a console that are read from
	// across a room. Everything else stays at the body size.
	void PanelLabelBig(UCanvas* C, float X, float Y, const FString& S, const FLinearColor& Col)
	{
		if (GPanelRecord) { GPanelRecord->Add({FTUPanelCmd::BigLabel, FVector2D(X, Y), FVector2D::ZeroVector, Col, S}); return; }
		FCanvasTextItem Item(FVector2D(X, Y), FText::FromString(S), PanelFont(), Col);
		Item.Scale = FVector2D(1.6f, 1.6f);   // the engine's "large" font is barely larger
		Item.EnableShadow(FLinearColor::Black);
		C->DrawItem(Item);
	}

	// The third primitive, and it arrived with the browser's plan-view thumbnails.
	// A tile per sample was the alternative -- it needs no new primitive and it
	// draws a dotted line, which on a 30-pixel thumbnail is the difference between
	// a layout you recognise and a smear.
	void PanelLine(UCanvas* C, float X0, float Y0, float X1, float Y1,
		const FLinearColor& Col, float Thickness = 1.f)
	{
		if (GPanelRecord) { GPanelRecord->Add({FTUPanelCmd::Line, FVector2D(X0, Y0), FVector2D(X1, Y1), Col, FString(), Thickness}); return; }
		FCanvasLineItem Item(FVector2D(X0, Y0), FVector2D(X1, Y1));
		Item.SetColor(Col);
		Item.LineThickness = Thickness;
		C->DrawItem(Item);
	}

	// Which flank a walkway is on, in the rider's terms rather than the enum's —
	// "the rider's right" is unambiguous where "Right" alone is a question about
	// which way you are facing.
	const TCHAR* WalkwaySideName(ETUWalkway Side)
	{
		switch (Side)
		{
		case ETUWalkway::Left:  return TEXT("rider's left");
		case ETUWalkway::Right: return TEXT("rider's right");
		case ETUWalkway::Both:  return TEXT("both sides");
		default:                return TEXT("no side");
		}
	}

	// ===================== FOUR KINDS, AND RAW IS NOT ONE =====================
	//
	// The kinds the panel can actually show fields for. RAW IS DELIBERATELY NOT A
	// DESTINATION: it is a sampled curvature profile the NL2 importer produces, it
	// has no authored parameters to offer, and cycling INTO it would hand somebody
	// a segment they can neither edit nor leave by any means the panel offers. A
	// raw segment cycles OUT to Straight, which is an explicit edit with undo
	// behind it rather than something that happens to an imported track quietly.
	//
	// ONE ANSWER, because the smoke test walks it too. A cycle written inline in
	// the click handler could only be checked by a test that re-implemented it,
	// which is a test that agrees with itself.
	ETUSegmentKind NextAuthorableKind(ETUSegmentKind K)
	{
		static const ETUSegmentKind Authorable[] = {
			ETUSegmentKind::Straight, ETUSegmentKind::Arc,
			ETUSegmentKind::Clothoid, ETUSegmentKind::Helix};
		int32 At = 0;
		for (int32 k = 0; k < UE_ARRAY_COUNT(Authorable); ++k)
		{
			if (Authorable[k] == K) { At = k + 1; break; }
		}
		return Authorable[At % UE_ARRAY_COUNT(Authorable)];
	}

	// What a segment IS, for the editor row. RAW is listed and is NOT something
	// the runtime editor will cycle INTO -- see CycleSegmentKind -- but an imported
	// track can already be full of them and a row that showed one as "Straight"
	// would be lying about the geometry under the camera.
	const TCHAR* SegmentKindName(ETUSegmentKind K)
	{
		switch (K)
		{
		case ETUSegmentKind::Straight: return TEXT("straight");
		case ETUSegmentKind::Arc:      return TEXT("arc");
		case ETUSegmentKind::Clothoid: return TEXT("clothoid");
		case ETUSegmentKind::Helix:    return TEXT("helix");
		case ETUSegmentKind::Raw:      return TEXT("raw");
		}
		return TEXT("-");
	}

	// What a zone IS, for the module heading. FTrackZone drops the kind because the
	// physics does not care; the panel is the one place that has to say it.
	const TCHAR* ZoneKindName(ETUSegmentZone Kind)
	{
		switch (Kind)
		{
		case ETUSegmentZone::Lift:          return TEXT("LIFT");
		case ETUSegmentZone::Launch:        return TEXT("LAUNCH");
		case ETUSegmentZone::Brake:         return TEXT("TRIM");
		case ETUSegmentZone::BlockBrake:    return TEXT("BLOCK BRAKE");
		case ETUSegmentZone::Station:       return TEXT("STATION");
		case ETUSegmentZone::StationUnload: return TEXT("UNLOAD");
		case ETUSegmentZone::StationLoad:   return TEXT("LOAD");
		default:                            return TEXT("-");
		}
	}

	const TCHAR* PhaseName(EStationPhase P)
	{
		switch (P)
		{
		case EStationPhase::Empty:     return TEXT("EMPTY");
		case EStationPhase::Arriving:  return TEXT("ARRIVING");
		case EStationPhase::Unloading: return TEXT("UNLOADING");
		case EStationPhase::Loading:   return TEXT("LOADING");
		case EStationPhase::Securing:  return TEXT("SECURING");
		case EStationPhase::Ready:     return TEXT("READY");
		default:                       return TEXT("DEPARTING");
		}
	}
}

TArray<FTUTrackSegment> ATUCoasterRide::PresetLayout(ETUPresetLayout Which)
{
	switch (Which)
	{
	case ETUPresetLayout::FlatRig:         return FlatRigLayout();
	case ETUPresetLayout::OutAndBack:      return OutAndBackLayout();
	case ETUPresetLayout::TwoTrainCircuit: return TwoTrainCircuitLayout();
	case ETUPresetLayout::SmallBatch:      return SmallBatchLayout();
	case ETUPresetLayout::Showcase:        return ShowcaseLayout();
	default:                               return ReferenceLayout();
	}
}

void ATUCoasterRide::ApplyPresetWalkways()
{
	// ===================== A PRESET CATWALKS ITS DEVICES =====================
	//
	// `ETUWalkway` is authored, and every shipped preset authored NONE — so the
	// evacuation model, whose whole job is deciding whether a stopped train can be
	// reached, had no route to reason about on any ride this project ships.
	//
	// DERIVED FROM THE ZONES RATHER THAN TYPED AS ARC LENGTHS, and that is the
	// better answer as well as the cheaper one. A real ride catwalks its lift, its
	// launch, its brake runs and its station — the places a train stops and staff
	// need to reach it — which is exactly the set of powered runs. Absolute arc
	// lengths typed into a preset would also drift the first time a segment
	// upstream changed length, which is the cost the FTUWalkway header already
	// warns about.
	//
	// Authoring is untouched: this only ever runs on an explicit preset load, the
	// same rule the train follows, so somebody who has drawn their own walkways
	// keeps them.
	Walkways.Reset();

	// ===================== AND NONE MEANS NONE =====================
	//
	// Returning here rather than authoring spans with a side of None leaves the
	// list genuinely EMPTY, which is what makes this the honest way to say a ride
	// has no walkways: a span that exists but draws nothing is still a route as
	// far as anything reasoning about reachability is concerned, and that is
	// exactly the lie `bBuildCatwalks` cannot avoid telling.
	if (PresetWalkwaySide == ETUWalkway::None)
	{
		return;
	}

	double S = 0.0;
	double RunStart = 0.0;
	bool bInRun = false;
	for (int32 i = 0; i < Segments.Num(); ++i)
	{
		const bool bWants = Segments[i].Zone != ETUSegmentZone::None;
		if (bWants && !bInRun) { RunStart = S; bInRun = true; }
		S += static_cast<double>(Segments[i].Length);
		const bool bEnds = bInRun && (!bWants || i == Segments.Num() - 1);
		if (bEnds)
		{
			FTUWalkway W;
			W.StartS = static_cast<float>(RunStart);
			// A run that ended because the NEXT segment is unpowered stops at the
			// start of this one; one that ended at the last segment runs to the end.
			W.EndS = static_cast<float>(bWants ? S : S - static_cast<double>(Segments[i].Length));
			// THE SIDE IS A SETTING NOW, and it was hardcoded to Both. See the
			// property's own comment for why one side is the better default: the
			// old version put a deck and a handrail down both flanks of every
			// powered run on every shipped ride, which is twice the geometry
			// saying the same thing.
			W.Side = PresetWalkwaySide;
			if (W.EndS > W.StartS) { Walkways.Add(W); }
			bInRun = false;
		}
	}
}

void ATUCoasterRide::ApplyPresetTrainSetup(ETUPresetLayout Which)
{
	// A PRESET IS A WORKED EXAMPLE, AND THE TRAIN IS PART OF THE EXAMPLE. A
	// small-batch operation has small vehicles, and its platform positions are
	// sized for them — 10 m each, which fits a 6 m train and its clearance and
	// nothing longer. Loading that layout with the 15 m train every other preset
	// uses puts the stop mark PAST the end of every position, where nothing trips
	// it, and a train sent to park there crawls out of its block into the next
	// one. Measured: seven signalling violations inside four seconds.
	//
	// So the train comes with the layout. Only on an explicit preset load, never
	// on a rebuild — an author who has since chosen a different train keeps it.
	switch (Which)
	{
	case ETUPresetLayout::SmallBatch:
		CarCount = 2; CarLengthM = 3.f;   // 6 m, exactly as before
		TrainCount = 5;
		break;
	case ETUPresetLayout::Showcase:
		// FIVE, WHICH IS SmallBatch'S MEASURED NUMBER, and it was six for an hour on
		// an argument rather than a measurement: that the trim splits a block off the
		// return leg and so buys headway for one more. It ran six and got a
		// SIGNALLING VIOLATION.
		//
		// The argument was wrong in a way this project has already written down.
		// "Trains = holding places - 1" is asserted for a ring where every block can
		// HOLD; a trim cannot, so splitting one off the return leg puts a block in
		// the ring that a train may not stop in, and a train dispatched into it is
		// committed all the way to the next device. That is headway SPENT, not
		// bought. The panel still offers 7 because the formula counts places, which
		// is necessary and not sufficient -- the same gap the old blocks/(1+lookahead)
		// rule had, recorded on TwoTrainCircuitLayout and repeated here anyway.
		//
		// So it takes the number that was MEASURED on this geometry rather than the
		// one that was reasoned about. Six is reachable with [+ RETURN] for anybody
		// who wants to watch it trip.
		CarCount = 2; CarLengthM = 3.f;
		TrainCount = 5;
		break;
	case ETUPresetLayout::TwoTrainCircuit:
		CarCount = 5; CarLengthM = 3.f;   // 15 m, exactly as before
		TrainCount = 2;
		break;
	default:
		CarCount = 5; CarLengthM = 3.f;   // 15 m, exactly as before
		TrainCount = 1;   // every other preset has one holding place
		break;
	}
}

// Shared shapes for the layouts below. ReferenceLayout keeps its own copies
// deliberately: it is the fixture every published figure is measured from, and
// re-pointing it at new helpers would mean re-verifying all of them to save a
// dozen lines.
namespace
{

// Returns nothing on purpose. Handing back a reference into a TArray invites
// somebody to hold it across the next Add, which reallocates.
void AddStraight(TArray<FTUTrackSegment>& Out, double Length,
	ETUSegmentZone Zone = ETUSegmentZone::None, float ZoneSpeed = 0.f)
{
	FTUTrackSegment S;
	S.Kind = ETUSegmentKind::Straight;
	S.Length = static_cast<float>(Length);
	S.Zone = Zone;
	// SET ONLY WHEN THERE IS A DEVICE. A speed on unpowered track is inert —
	// the zone walk never reads it — but the SAVE FORMAT writes anything that is
	// not at its default, so an unconditional 0 here put `"zoneSpeed": 0` on
	// every plain segment of every preset: a number nobody typed, in a file whose
	// whole claim is that it holds the author's decisions and nothing else.
	if (Zone != ETUSegmentZone::None) { S.ZoneSpeed = ZoneSpeed; }
	Out.Add(S);
}

// A vertical curve with no curvature step at either end: pitch curvature ramps
// 0 -> peak -> 0, so both joints stay continuous. Two Raw segments, because the
// authored vocabulary is still yaw-only and no Make* helper builds pitch.
void AddEasedPitch(TArray<FTUTrackSegment>& Out, double PitchDelta, double PeakCurvature,
	ETUSegmentZone Zone = ETUSegmentZone::None, float ZoneSpeed = 0.f)
{
	const double K = PitchDelta >= 0.0 ? PeakCurvature : -PeakCurvature;
	const double L = FMath::Abs(PitchDelta) / PeakCurvature;

	FTUTrackSegment In;
	In.Kind = ETUSegmentKind::Raw;
	In.Length = static_cast<float>(L);
	In.PitchCurvatureEnd = static_cast<float>(K);
	In.Zone = Zone;
	if (Zone != ETUSegmentZone::None) { In.ZoneSpeed = ZoneSpeed; }
	Out.Add(In);

	FTUTrackSegment Tail;
	Tail.Kind = ETUSegmentKind::Raw;
	Tail.Length = static_cast<float>(L);
	Tail.PitchCurvatureStart = static_cast<float>(K);
	Tail.Zone = Zone;
	if (Zone != ETUSegmentZone::None) { Tail.ZoneSpeed = ZoneSpeed; }
	Out.Add(Tail);
}

// Clothoid in, constant-radius hold, clothoid out, with roll ramping across the
// clothoids so neither curvature nor roll steps at a joint.
//
// Roll RATE still steps, and measurably: roll varies LINEARLY across a segment,
// so its rate is constant within one and changes abruptly at the ends. Every
// banked turn in this vocabulary has that, the reference layout included — the
// validator reports four such steps on it. Removing them needs roll rate itself
// to ease, which the authored vocabulary cannot currently say.
void AddBankedTurn(TArray<FTUTrackSegment>& Out, double Radius, double ArcLength,
	double EaseLength, double BankDegrees)
{
	FTUTrackSegment In;
	In.Kind = ETUSegmentKind::Clothoid;
	In.Length = static_cast<float>(EaseLength);
	In.CurvatureStart = 0.f;
	In.CurvatureEnd = static_cast<float>(1.0 / Radius);
	In.RollEndDegrees = static_cast<float>(BankDegrees);
	Out.Add(In);

	FTUTrackSegment Hold;
	Hold.Kind = ETUSegmentKind::Arc;
	Hold.Length = static_cast<float>(ArcLength);
	Hold.Radius = static_cast<float>(Radius);
	Hold.RollStartDegrees = Hold.RollEndDegrees = static_cast<float>(BankDegrees);
	Out.Add(Hold);

	FTUTrackSegment OutEase;
	OutEase.Kind = ETUSegmentKind::Clothoid;
	OutEase.Length = static_cast<float>(EaseLength);
	OutEase.CurvatureStart = static_cast<float>(1.0 / Radius);
	OutEase.CurvatureEnd = 0.f;
	OutEase.RollStartDegrees = static_cast<float>(BankDegrees);
	Out.Add(OutEase);
}

// The bank that cancels lateral G at a given speed on a given radius.
double BankDegreesFor(double SpeedMs, double Radius)
{
	return FMath::RadiansToDegrees(
		FMath::Atan((SpeedMs * SpeedMs) / (GravityMs2 * Radius)));
}

} // namespace

TArray<FTUTrackSegment> ATUCoasterRide::FlatRigLayout()
{
	// Three straights. Dead level, so gravity contributes nothing and what the
	// train does is purely launch, rolling resistance and drag — which is exactly
	// why this shape was the rig the RollingResistance correction was measured
	// on against NoLimits 2.
	//
	// Measured: 160 m, ends at station height, C2, 30.0 km/h, a flat +1.00 g
	// throughout, no validation issues at all.
	TArray<FTUTrackSegment> Out;
	AddStraight(Out, 20.0, ETUSegmentZone::Launch, 8.333f);  // 30 km/h
	AddStraight(Out, 90.0);
	AddStraight(Out, 50.0, ETUSegmentZone::Brake, 0.f);
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::OutAndBackLayout()
{
	// Measured: 20 segments, 631.4 m, ends within a millimetre of station height,
	// C2 to 1e-9, 96.9 km/h, vertical -0.10..+2.04 g, lateral 0.34, closest
	// self-approach 7.77 m. Gentler than the reference layout and it does not
	// pass through itself, which the reference does at 0.19 m.
	const double Up = Deg(25.0);
	const double Drop = Deg(-32.0);

	// Solved, not eyeballed: this is the drop that brings the ride back to
	// station height at the end. 18 m left it 4.64 m in the air.
	const double DropLength = 26.758;

	TArray<FTUTrackSegment> Out;
	AddStraight(Out, 20.0, ETUSegmentZone::Station, 4.f);      // station
	AddEasedPitch(Out, Up, 0.03, ETUSegmentZone::Lift, 4.f);   // into the climb
	AddStraight(Out, 62.0, ETUSegmentZone::Lift, 4.f);         // lift climb

	// Only the FIRST of the crest pair is powered, for the reason spelled out in
	// ReferenceLayout: the chain must run over the top and let go just past it.
	const int32 CrestFirst = Out.Num();
	AddEasedPitch(Out, Drop - Up, 0.05);
	Out[CrestFirst].Zone = ETUSegmentZone::Lift;
	Out[CrestFirst].ZoneSpeed = 4.f;

	AddStraight(Out, DropLength);
	AddEasedPitch(Out, -Drop, 0.015);                          // pull-out to level

	// The airtime hill: a lazy 14 degree rise into a sharp 30 degree fall, which
	// puts the whole ride's minimum vertical G here at -0.10 g.
	//
	// MEASURED, because the obvious claim about it turned out to be false: the
	// front car sees -0.14 g and the back -0.12, so on THIS hill the front gets
	// marginally the better airtime, not the back. Asymmetry alone does not buy
	// the back-row effect — PHASE0_FINDINGS records a crest where it is worth
	// 0.73 g, so the shape matters and this one is not that shape. Left as it is
	// because the ride is good; treat tuning it into a back-row hill as an open
	// exercise rather than a fix.
	//
	// The largest front-to-back spread on this layout is 0.55 g and it is in the
	// banked turn, not here — the cars are on different curvature there, which is
	// a different mechanism entirely.
	AddEasedPitch(Out, Deg(14.0), 0.006);
	AddEasedPitch(Out, Deg(-30.0), 0.030);
	AddEasedPitch(Out, Deg(16.0), 0.015);                      // back to level

	// 180 degree turnaround, banked for the speed it is actually taken at.
	AddBankedTurn(Out, 26.0, Pi * 26.0, 22.0, BankDegreesFor(20.0, 26.0));

	AddStraight(Out, 40.0);
	AddStraight(Out, 60.0, ETUSegmentZone::Brake, 0.f);        // brake run
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::TwoTrainCircuitLayout()
{
	// The only preset with enough blocks to run more than one train.
	//
	// Blocks are DERIVED: a boundary falls where a powered run starts or ends,
	// because that is the only place a device exists that can hold a train.
	//
	// CAPACITY IS NOT blocks/(1 + lookahead), which an earlier version of this
	// comment claimed and which is neither necessary nor sufficient — it allows
	// four trains at lookahead 1, where they collided, and forbids three at
	// lookahead 2, which run. The rule is
	//
	//     trains = (blocks that can STOP a train and LET IT GO) - 1
	//
	// and the minus one is the whole thing: one has to stay empty or every train
	// is standing exactly where the train behind it needs to go and the ring
	// cannot rotate. Asserted for rings of 3 to 8 in test_ridesignals.cpp, which
	// drives it on bare numbers because capacity is a signalling property with
	// nothing to do with geometry.
	//
	// It is also why a high-throughput ride is built with MORE block sections
	// rather than a longer train: sections buy trains, and trains buy capacity.
	// This preset has five and runs four; every other one has a station and a trim
	// brake — and a trim cannot let a train go again — so they have ONE, and one
	// place is one train, always.
	//
	// Do NOT read a real ride's brake count as its block count. Three different
	// things sit on the track and all look like a brake: BLOCK BRAKES, which are
	// these; EVACUATION ZONES, which are about getting riders off and need a
	// walkway rather than just a way to stop; and SAFETY CATCHES for rollback or a
	// defect. A large ride can have ~25 of the second and a handful of the third
	// while running far fewer trains than either count. Only block brakes feed the
	// formula, and neither of the other two is modelled at all — see SIGNALLING.md.
	//
	// The approach is TWO brake blocks with drive tyres between, not one long
	// brake, and both halves of that matter. Two adjacent Brake runs MERGE into a
	// single block, so "one long brake" holds exactly one train — and worse, the
	// second run's authored ZoneSpeed is silently discarded (measured: 6 m/s then
	// 2 m/s becomes one zone at 6). And a friction brake can hold a train but
	// cannot start one, also measured, so moving a stopped train from the outer
	// brake to the inner one needs powered track either way.
	//
	// ---- IT IS A CLOSED CIRCUIT, AND THE SHAPE IS WHY ----
	//
	// An OVAL closes analytically, with no solver: both turns the same way, each
	// EXACTLY 180 degrees, same radius, same easement. Then the two lateral
	// displacements (+2R, -2R) cancel, the along-track ones cancel, heading sums to
	// 360, and one scalar condition is left — the return leg's horizontal extent
	// must equal the two collinear outbound legs'. A 180 degree turn with easements
	// of length E needs arc = pi*R - E, because each easement turns E/(2R).
	//
	// The turnaround sits at the TOP of the hill, and that is not decoration. The
	// closure condition means every metre in the outbound leg is paid for TWICE, so
	// the first attempt — whole hill outbound, turnarounds at each end — came to
	// 1717 m and every variant of it STALLED, because softening the crest to tame
	// the G lengthens it (crest length is angle/curvature), which lengthens the
	// circuit, which costs more energy than a longer launch buys. Splitting the
	// hill across the turn is cheaper in three currencies at once: the train is
	// slowest at the top so the turn costs least lateral G, a small radius will do,
	// and the drop's horizontal extent lands in the return leg where it is needed.
	//
	// MEASURED, every figure re-derived through the authored path: 30 segments,
	// 1288.00 m, 8 blocks, C2 to 1e-9, closest self-approach 11.68 m, top
	// 136.8 km/h, vertical -0.53..+3.08 g, lateral 0.15 g, crest 48.5 m, 105 s,
	// peak roll rate 17.4 deg/s, ZERO curvature-step and ZERO roll-rate
	// diagnostics. The seam closes to 0.000000 m, 0.000084 degrees of heading and
	// 0.000000 degrees of roll.
	//
	// Those figures are the ride with every signal GREEN, which is what the ride
	// profile measures and the right thing for it to measure. Held at a red the
	// train stops wherever the block brake is, and none of the numbers above move
	// — a holding device only ever removes energy the layout had already spent.
	const double Up = Deg(26.0);       // pull-up out of the launch
	const double Dn = Deg(32.0);       // the drop out of the turnaround

	// The turn. Both are this, exactly, or the circuit does not close.
	const double R = 35.0;
	const double Ease = 50.0;          // 50, not 26: roll rate 34.0 -> 17.4 deg/s
	const double Arc = Pi * R - Ease;  // exactly 180 degrees, easements included

	// SOLVED, not eyeballed. The drop straight is the height lever — more of it
	// ends lower, monotonically — and the fill is then the horizontal one, which
	// cannot disturb the height because it is level.
	const double DropLen = 15.6847323;
	const double FillLen = 75.5024975;

	TArray<FTUTrackSegment> Out;

	// ---- LEG A, outbound. Station, launch, and the climb to the turnaround.
	AddStraight(Out, 26.0, ETUSegmentZone::Station, 1.5f);    // 1 STATION, drive tyres
	AddStraight(Out, 150.0, ETUSegmentZone::Launch, 38.f);    // 2 LAUNCH

	// Launch length matters more than the target: at the fixed 6 m/s^2 grip a
	// launch caps at sqrt(2*grip*length) whatever is asked for, so 70 m could
	// never exceed 29 m/s and three different targets all produced the same top
	// speed. 150 m is what makes 38 m/s actually reachable.
	AddEasedPitch(Out, Up, 0.0130);    // the +Gz peak lives HERE — highest v^2
	AddStraight(Out, 40.0);            // curvature on the track. 0.0130 holds it
	AddEasedPitch(Out, -Up, 0.0130);   // to +3.08 g. Levels off 48.5 m up.

	// ANTI-ROLLBACK over the launch and the whole climb, which is where a real
	// launched coaster puts it: a train that fails to make the top comes back down
	// this stretch and arrives in the station at speed, backwards.
	//
	// It changes NOTHING while the ride works — the train crests at 12.14 m/s with
	// margin — and that is exactly the property a safety device should have. Detune
	// the launch and it is the difference between a held train and a loose one.
	for (int32 i = 1; i < Out.Num(); ++i)
	{
		Out[i].bAntiRollback = true;
	}

	// ---- TURN 1, level, at the top, taken at 12.1 m/s. Being slow here is the
	// whole point of putting it at the top: a turnaround at launch speed would
	// need R ~ 119 m to stay under 2 g, and that radius doubles into the circuit.
	AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(14.2, R));

	// ---- LEG B, the return. Drop, airtime, then the mid-course block brake.
	AddEasedPitch(Out, -Dn, 0.0150);
	AddStraight(Out, DropLen);
	AddEasedPitch(Out, Dn, 0.0150);

	// One airtime hill, which is where the -0.53 g comes from. Symmetric, so it
	// returns to the height and the pitch it started at: up, over, level out.
	AddEasedPitch(Out, Deg(20.0), 0.024);
	AddEasedPitch(Out, Deg(-40.0), 0.024);
	AddEasedPitch(Out, Deg(20.0), 0.024);

	AddStraight(Out, FillLen);

	// 4 MID-COURSE BLOCK BRAKE, and closing the circuit is what made it one. It
	// takes the train at 26.40 m/s and stopping that at 6 m/s^2 needs 58.1 m — so
	// the 45 m it used to have could only ever trim, and 130 m can HOLD. That is a
	// third queueing position, on the far side of the circuit from the station.
	AddStraight(Out, 130.0, ETUSegmentZone::BlockBrake, 20.f);

	// ---- TURN 2, level, at station height, taken at 18.1 m/s.
	AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(18.1, R));

	// ---- LEG C, the approach. Collinear with leg A and ending exactly where the
	// station begins, which is what closes the circuit.
	AddStraight(Out, 24.0);

	// 6/7/8 THE APPROACH. The outer brake is where a second train waits while the
	// first is in the station; the inner one holds short of the station and is
	// what clears the station-entry signal once its train has stopped.
	//
	// All three are hold-capable, which is what makes this preset run two trains:
	// blocks 5, 6 and 7 can each stop a train AND let it go again, so a queue can
	// form outside the station instead of on the course. Measured arrival speeds
	// are 15.50, 5.77 and 2.80 m/s, needing 20.0, 2.8 and 0.7 m to stop — every one
	// inside its own block, and now so is the mid-course brake.
	//
	// The authored speeds are the RELEASE speeds. A holding device rests closed
	// and is commanded to these only while its permissive is granted, which is
	// why the two brakes may sensibly trim to 6 and 2 m/s and still come to a
	// dead stop when the signal is red.
	AddStraight(Out, 37.5, ETUSegmentZone::BlockBrake, 6.f);  // outer
	AddStraight(Out, 27.0, ETUSegmentZone::Lift, 4.f);        // transfer tyres
	AddStraight(Out, 37.5, ETUSegmentZone::BlockBrake, 2.f);  // inner
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::SmallBatchLayout()
{
	// THE SAME OVAL, RE-ZONED. Derived from TwoTrainCircuitLayout rather than
	// copied, because the closure is a property of the LEG LENGTHS and a copy
	// could drift from the shape it depends on with nothing to notice — the seam
	// would miss by a metre, look fine from the cockpit, and teleport the train
	// once a lap.
	//
	// Leg A opens with a 26 m station and a 150 m launch: 176 m of flat. Replace
	// that with 40 m of platform and a 136 m launch and it is the same 176 m, so
	// the circuit still closes to 0.000000 m and every G figure still holds. The
	// launch reaches its 38 m/s in 120 m either way.
	//
	// Verified in Prototypes/TrainPhysics/test_twotrains.cpp against the two-train
	// layout: identical total length, C2, seam within 1e-3 m, and it still gets
	// round on a 6 m train — which a shorter train is not guaranteed to do, since
	// less of it straddles each crest and it pays more of the height.
	TArray<FTUTrackSegment> Out = TwoTrainCircuitLayout();
	Out.RemoveAt(0, 2);

	// 10 m positions: a 6 m train plus its 1 m nose clearance, with room to spare.
	// Any shorter and the stop mark falls past the end of the device, where no
	// train can ever trip it — which the build now warns about.
	TArray<FTUTrackSegment> Head;
	AddStraight(Head, 10.0, ETUSegmentZone::StationUnload, 1.5f);  // riders off
	AddStraight(Head, 10.0, ETUSegmentZone::StationLoad, 1.5f);    // position 3, rear
	AddStraight(Head, 10.0, ETUSegmentZone::StationLoad, 1.5f);    // position 2
	AddStraight(Head, 10.0, ETUSegmentZone::StationLoad, 1.5f);    // position 1, front
	AddStraight(Head, 136.0, ETUSegmentZone::Launch, 38.f);

	// Identical kind AND identical speed, so nothing the zone walk can see tells
	// these three apart — they really are the same machine three times over, and
	// are still three machines. Without this they are one 30 m zone holding one
	// train, which is a platform that loads one at a time.
	Head[2].bStartsNewDevice = true;
	Head[3].bStartsNewDevice = true;

	Out.Insert(Head, 0);
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::ShowcaseLayout()
{
	// ===================== THE MILESTONE RIDE =====================
	//
	// One track that exercises as much of what has been built as a single layout
	// honestly can, so the project has something to SHOW rather than a list of
	// headers to describe.
	//
	// DERIVED FROM SmallBatchLayout, NOT AUTHORED FRESH, and that is the whole
	// reason it is safe to add. This oval closes to 0.000000 m because of its LEG
	// LENGTHS -- and closure is the expensive property here: the first hand-drawn
	// attempt at this shape came to 1717 m and stalled in every variant. Every
	// change below either leaves a length alone or splits one in place, so the
	// seam, the C2 continuity and every G figure come along already measured.
	//
	// What it adds is DEVICES, which is where most of the engineering went and
	// none of which any preset was exercising:
	//
	//   - A FRICTION PAD on the mid-course brake. Until now `ZoneBrakeDecel` was 0
	//     on every shipped preset, so the "a block brake is TWO machines" model --
	//     a pad that can only remove energy, and drive tyres that push and hold --
	//     was tested and shipped with no ride using it. Here the pad bites at
	//     8 m/s^2 and the tyres convey at 1.5, which is an ordinary specification
	//     and was inexpressible before the two were separated.
	//
	//   - A TRIM BRAKE, which the closed circuit has never had. Split out of the
	//     fill straight rather than added to it, so the leg keeps its length. It
	//     bounds a block and cannot hold a train, which is exactly the distinction
	//     the signalling layer draws and nothing here demonstrated.
	//
	//   - DEVICES THAT ARE NOT ALL THE SAME MACHINE. Every preset ran every device
	//     at the default 6 m/s^2 for both accel and decel, which is a chain hauling
	//     at 0.61 g. A launch is punchy, a chain is slow and relentless, transfer
	//     tyres creep.
	//
	//   - ANTI-ROLLBACK ON THE LAUNCH, which SmallBatch quietly lost: the two-train
	//     layout sets it over leg A, and SmallBatch replaces exactly those segments
	//     to build its platform, so its launch and climb had none. Set explicitly
	//     here rather than by index, because that is how it went missing.
	//
	// AND WHAT IT DELIBERATELY DOES NOT ADD: no new geometry. A helix or an
	// inversion is a closure problem and a G problem, not an afternoon -- and the
	// authored vocabulary cannot express a rideable loop, which is a recorded
	// deferral with five measured failed fixes behind it. A showcase that stalled
	// its train would show the wrong thing.
	TArray<FTUTrackSegment> Out = SmallBatchLayout();

	// ---- The launch and the climb out, which is where a failed launch comes back
	// down. Everything from the launch to the first turn.
	for (FTUTrackSegment& S : Out)
	{
		if (S.Zone == ETUSegmentZone::Launch)
		{
			S.bAntiRollback = true;
			// An LSM launch is roughly 1 g. At the old 6 m/s^2 the 136 m run needs
			// 120 m to reach 38 m/s; at 10 it is there in 72 and holds the rest,
			// which is what a real launch does and what the drive panel shows.
			S.ZoneAccel = 10.f;
		}
	}

	// ---- The devices, each specified as itself.
	for (int32 i = 0; i < Out.Num(); ++i)
	{
		FTUTrackSegment& S = Out[i];
		if (S.Zone == ETUSegmentZone::BlockBrake && S.Length > 100.f)
		{
			// THE MID-COURSE BRAKE, AND THE ONE PLACE THE TWO MACHINES SHOW. A
			// train arrives here fast; the pad is what stops it, and the tyres only
			// ever truck it forward to the stop mark afterwards.
			S.ZoneBrakeDecel = 8.f;
			S.ZoneDecel = 1.5f;
			S.ZoneAccel = 1.5f;
		}
		else if (S.Zone == ETUSegmentZone::Lift)
		{
			// Transfer tyres. They creep, and a rider feels the pickup.
			S.ZoneAccel = 1.0f;
			S.ZoneDecel = 1.0f;
		}
		else if (S.Zone == ETUSegmentZone::StationLoad
			|| S.Zone == ETUSegmentZone::StationUnload)
		{
			S.ZoneAccel = 1.0f;
			S.ZoneDecel = 1.0f;
		}
	}

	// ---- KICKER TYRES OUT OF THE MID-COURSE ----------------------------------
	//
	// A train restarting from a standing hold at the mid-course leaves it with a
	// few metres of tyre push and nothing else, which is why an MCBR on a real
	// ride either sits high with a drop after it or gets drive tyres bolted to
	// its exit. Ours is at grade, so it gets the tyres: the turn's own entry
	// easement carries a Launch zone -- a zone on EXISTING geometry, so closure
	// cannot move, and a Launch cannot hold a train, so the capacity table cannot
	// either. A Launch also has no braking authority whatever is typed, so a
	// train already above 22 m/s passes untouched and every green-signal figure
	// is unmoved.
	//
	// ---- AND WHY THERE IS NO HELIX HERE, TWICE OVER --------------------------
	//
	// A full circle added to this turn (Length += 2*pi*R) was shipped as a helix
	// finale and is NOT a helix: an Arc is level, so the extra turn lies on top
	// of the track it just came round. MEASURED, once the showcase test was made
	// to ask the geometric question at all -- closest self-approach 0.09 m at
	// 1105.5 / 1325.5 m, exactly one turn apart, against 11.68 m without it.
	//
	// It cannot be fixed in plan. The property that made it free is the property
	// that breaks it: ANY closure-neutral addition of turning returns to its own
	// start point and heading, and therefore has to touch itself. Only vertical
	// separation answers it, and that breaks the hand-solved closure this layout
	// exists to inherit.
	//
	// The real version, if it is ever worth it: reduce DropLen so the mid-course
	// sits Dh high, and let a descending helix be the drop back to station
	// height -- which is what the paragraph above says real MCBRs do, and it
	// would make these tyres unnecessary. That is a two-variable closure re-solve
	// and a re-measurement of every published figure, not a line.
	for (int32 i = Out.Num() - 1; i >= 0; --i)
	{
		if (Out[i].Kind != ETUSegmentKind::Arc || !(Out[i].Radius > 0.f)) { continue; }
		// The easement INTO this turn, which AddBankedTurn authored immediately
		// before the arc. Guarded rather than assumed, because a layout edit that
		// reordered the turn would otherwise zone the wrong piece of track.
		if (i > 0 && Out[i - 1].Kind == ETUSegmentKind::Clothoid)
		{
			Out[i - 1].Zone = ETUSegmentZone::Launch;
			Out[i - 1].ZoneSpeed = 22.f;
			Out[i - 1].ZoneAccel = 10.f;
		}
		break;
	}

	// ---- The trim, SPLIT OUT of the fill straight so the leg keeps its length.
	//
	// Found by its length rather than its index: this list has already been
	// rebuilt twice by the two derivations above it, and an index would be a
	// number that silently means something else the day either one changes.
	for (int32 i = 0; i < Out.Num(); ++i)
	{
		if (Out[i].Zone != ETUSegmentZone::None
			|| Out[i].Kind != ETUSegmentKind::Straight
			|| Out[i].Length < 70.f || Out[i].Length > 80.f)
		{
			continue;
		}
		const float TrimLen = 40.f;
		Out[i].Length -= TrimLen;

		FTUTrackSegment Trim;
		Trim.Kind = ETUSegmentKind::Straight;
		Trim.Length = TrimLen;
		Trim.Zone = ETUSegmentZone::Brake;
		// A TRIM IS A PAD AND NOTHING ELSE -- it has no tyres, so it can shave
		// speed off a train and can never start one. Authored above the arrival
		// speed on purpose: a trim that stopped the train would be a block brake
		// that cannot let go, which is a wedged ride rather than a trimmed one.
		Trim.ZoneSpeed = 24.f;
		Trim.ZoneBrakeDecel = 3.f;
		Out.Insert(Trim, i);
		break;
	}
	return Out;
}

TArray<FTUTrackSegment> ATUCoasterRide::ReferenceLayout()
{
	// The reference ride, as authored data rather than as code. Tuned in the
	// standalone harness first; these numbers give a 50.1 m crest, 105.2 km/h,
	// +0.66..+4.19 vertical G, 0.28 peak lateral and +1.13 G over the loop apex,
	// with the track ending at station height to within a millimetre.
	//
	// This used to be a sequence of AddSegment calls, which meant the layout was
	// only editable by recompiling. It is a list now because that is what the
	// project said track authoring should be from the start.
	const double Lift = Deg(25.0);
	const double Drop = Deg(-34.0);
	const double LoopRadius = 9.0;
	const double LoopEase = 54.0;
	const double TurnRadius = 32.0;
	const double BankDeg = FMath::RadiansToDegrees(
		FMath::Atan((26.5 * 26.5) / (GravityMs2 * TurnRadius)));

	// Both solved rather than eyeballed, and they do different jobs.
	//
	// The DROP sets the ride. Everything the train has at the loop comes from
	// the descent below the crest, so this is the only lever that changes how it
	// feels. 12.0 m was tuned when RollingResistance was 0.006 — a figure
	// justified against steel-on-steel, which is a railway rather than a coaster.
	// At the corrected 0.024 for polyurethane wheels, 12 m leaves the train
	// cresting the loop at +0.13 g: hanging on the track rather than held to it.
	// 24 m restores +1.13 g and the original G profile almost exactly.
	//
	// The LIFT sets the height and nothing else. Raising it changes no G number
	// on this layout at all — measured, across 75 to 115 m — because the chain
	// delivers the train to the crest at 4 m/s whatever height the crest is. So
	// it is free to use purely to close the ride back to station level, which is
	// what 90.99 does: deepening the drop by 12 m put the ending 6.710 m low.
	const double LiftClimb = 90.99;
	const double DropLength = 24.0;

	TArray<FTUTrackSegment> Out;

	auto Straight = [&Out](double Length, ETUSegmentZone Zone = ETUSegmentZone::None,
		float ZoneSpeed = 0.f) -> FTUTrackSegment&
	{
		FTUTrackSegment S;
		S.Kind = ETUSegmentKind::Straight;
		S.Length = static_cast<float>(Length);
		S.Zone = Zone;
		if (Zone != ETUSegmentZone::None) { S.ZoneSpeed = ZoneSpeed; }
		return Out[Out.Add(S)];
	};

	// A vertical curve with no curvature step at either end: pitch curvature
	// ramps 0 -> peak -> 0, so both joints stay continuous. Two Raw segments,
	// because no Make* helper builds pitch curvature — the authored vocabulary
	// is still yaw-only, which is exactly what ESegmentKind::Raw records.
	auto EasedPitch = [&Out](double PitchDelta, double PeakCurvature,
		ETUSegmentZone Zone = ETUSegmentZone::None, float ZoneSpeed = 0.f)
	{
		const double K = PitchDelta >= 0.0 ? PeakCurvature : -PeakCurvature;
		const double L = FMath::Abs(PitchDelta) / PeakCurvature;

		FTUTrackSegment In;
		In.Kind = ETUSegmentKind::Raw;
		In.Length = static_cast<float>(L);
		In.PitchCurvatureEnd = static_cast<float>(K);
		In.Zone = Zone;
		if (Zone != ETUSegmentZone::None) { In.ZoneSpeed = ZoneSpeed; }
		Out.Add(In);

		FTUTrackSegment Tail;
		Tail.Kind = ETUSegmentKind::Raw;
		Tail.Length = static_cast<float>(L);
		Tail.PitchCurvatureStart = static_cast<float>(K);
		Tail.Zone = Zone;
		if (Zone != ETUSegmentZone::None) { Tail.ZoneSpeed = ZoneSpeed; }
		Out.Add(Tail);
	};

	Straight(20.0, ETUSegmentZone::Station, 4.f);      // station
	EasedPitch(Lift, 0.03, ETUSegmentZone::Lift, 4.f); // into the climb
	Straight(LiftClimb, ETUSegmentZone::Lift, 4.f);    // lift climb

	// The chain has to run OVER the crest and stop AT it, and the window is
	// narrower than it looks in both directions.
	//
	// Release at the top of the climb and the train is stranded: the straight
	// tops out while the track is still rising. Carry on down the far side and
	// it is worse — MakeLift holds a fast train back as well as pulling a slow
	// one, so the chain sits on the train at 4 m/s down a 34-degree drop and
	// takes the ride's energy with it. Measured: powering both halves of the
	// crest instead of the first costs 10 km/h of top speed and flattens the
	// loop apex from +1.34 g to nothing.
	//
	// So only the FIRST of the pair is powered — the segment carrying the track
	// from +25 degrees down through level. Pitch crosses zero 17.4 m into a
	// 20.6 m segment, so the chain lets go just past the top.
	const int32 CrestFirst = Out.Num();
	EasedPitch(Drop - Lift, 0.05);
	Out[CrestFirst].Zone = ETUSegmentZone::Lift;
	Out[CrestFirst].ZoneSpeed = 4.f;

	Straight(DropLength);                              // drop
	EasedPitch(-Drop, 0.012);                             // pull-out

	// Teardrop loop: curvature eases in and out rather than stepping, so the
	// radius is large where the train is fastest. A circular loop at this speed
	// would pull over 9 G at the bottom, which is why real loops are not circles.
	//
	// Known defect, measured and left alone deliberately: being planar, its two
	// legs pass 0.19 m from each other. See PHASE0_FINDINGS.md — every cheap fix
	// was tried and costs more than the defect does.
	{
		FTUTrackSegment EaseIn;
		EaseIn.Kind = ETUSegmentKind::Raw;
		EaseIn.Length = static_cast<float>(LoopEase);
		EaseIn.PitchCurvatureEnd = static_cast<float>(1.0 / LoopRadius);
		Out.Add(EaseIn);

		FTUTrackSegment Crown;
		Crown.Kind = ETUSegmentKind::Raw;
		Crown.Length = static_cast<float>(2.0 * Pi * LoopRadius - LoopEase);
		Crown.PitchCurvatureStart = static_cast<float>(1.0 / LoopRadius);
		Crown.PitchCurvatureEnd = static_cast<float>(1.0 / LoopRadius);
		Out.Add(Crown);

		FTUTrackSegment EaseOut;
		EaseOut.Kind = ETUSegmentKind::Raw;
		EaseOut.Length = static_cast<float>(LoopEase);
		EaseOut.PitchCurvatureStart = static_cast<float>(1.0 / LoopRadius);
		Out.Add(EaseOut);
	}

	// Banked turn, clothoid in and out so neither curvature nor roll steps.
	// Path-relative, matching what this layout has always been: it follows an
	// inversion, so the frame arrives carrying whatever twist the loop left it,
	// and re-reading these numbers as bank-from-horizon would change the ride.
	{
		FTUTrackSegment In;
		In.Kind = ETUSegmentKind::Clothoid;
		In.Length = 26.f;
		In.CurvatureStart = 0.f;
		In.CurvatureEnd = static_cast<float>(1.0 / TurnRadius);
		In.RollEndDegrees = static_cast<float>(BankDeg);
		Out.Add(In);

		FTUTrackSegment Hold;
		Hold.Kind = ETUSegmentKind::Arc;
		Hold.Length = 55.f;
		Hold.Radius = static_cast<float>(TurnRadius);
		Hold.RollStartDegrees = Hold.RollEndDegrees = static_cast<float>(BankDeg);
		Out.Add(Hold);

		FTUTrackSegment OutEase;
		OutEase.Kind = ETUSegmentKind::Clothoid;
		OutEase.Length = 26.f;
		OutEase.CurvatureStart = static_cast<float>(1.0 / TurnRadius);
		OutEase.CurvatureEnd = 0.f;
		OutEase.RollStartDegrees = static_cast<float>(BankDeg);
		Out.Add(OutEase);
	}

	Straight(70.0, ETUSegmentZone::Brake, 0.f);           // brake run
	return Out;
}

void ATUCoasterRide::RebuildFromSegments()
{
	++SegmentsRevision;
	// A REBUILD IS NOT A TRANSITION. After this, channel 4 may be a different
	// block from the channel 4 being watched, so every stale baseline is a
	// transition that never happened — on most channels at once. Reseed.
	StateWatch.Forget();
	// And the show layer's own baseline, for the same reason: a layout that has
	// just gained or lost blocks has channels that no longer mean what they meant,
	// and carrying them across fires every cue on the rebuild.
	ShowPublisher.Rebuilt();

	FTrackDocument Doc;
	Doc.HeartlineHeight = 1.1;
	Doc.Segments.reserve(static_cast<std::size_t>(Segments.Num()));
	for (const FTUTrackSegment& S : Segments)
	{
		Doc.Segments.push_back(ToAuthored(S));
	}

	// Validate the AUTHORED list before building anything from it — the whole
	// reason TrackValidate exists is that the geometry cannot tell you a radius
	// was typed into a curvature field, it can only produce the consequences.
	//
	// KEPT, not only logged. The findings have been correct since Phase 0 and
	// invisible outside a log for just as long; the panel is what makes them
	// somewhere to go rather than something to scroll back for.
	LastDiagnostics = ValidateTrack(BuildSegments(Doc));
	for (const FTrackDiagnostic& D : LastDiagnostics)
	{
		UE_LOG(LogTemp, Warning, TEXT("Track segment %d: %s"),
			static_cast<int32>(D.SegmentIndex), UTF8_TO_TCHAR(D.Message.c_str()));
	}

	// THE SESSION SEES WHAT THE DOCUMENT NOW IS. Here rather than in each of the
	// dozen callers that change a segment, because a rebuild is what every one of
	// them ends with — an edit that did not rebuild changed nothing anybody can
	// see. `IsDirty` is a comparison, so this one call is the whole of it.
	//
	// The text is built from `Doc`, which is the same list just walked, so this
	// costs a serialisation of a few dozen segments per rebuild and no walk of its
	// own. A rebuild is already the expensive operation on this actor.
	//
	// AND HISTORY IS DELIBERATELY NOT PUSHED HERE, however tidy it would look
	// beside this. A rebuild is not an edit: it also happens for a preset, an
	// open and an undo, and committing here would make undo record the undone
	// state as a new step — so the second press would go forward again.
	// `PushHistory` is called from the two places an edit is FINISHED.
	{
		std::string DocText, DocError;
		Session.Observe(WriteTrackJson(Doc, DocText, DocError) ? DocText : std::string());
	}

	Track = ::BuildTrack(Doc);
	if (Track.NumSegments() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("TrackUnlimited: no usable segments; nothing to ride."));
		// Dropped rather than left pointing at the track that just failed to
		// build. An FTrain holds a reference to it, so keeping one would be a
		// train riding a track with no segments in it.
		Trains.Reset();
		StoppedForS.Reset();
		Signals.Reset();
		StopMarks.Reset();
		Drives.Reset();
		BlockSensors.Reset();
		Counter.Reset();
		// THE MESH GOES TOO. Returning here skipped the mesh rebuild at the bottom,
		// so an empty document -- the menu, the Blank template -- kept drawing the
		// last track that was open, rails, supports, trains and all.
		RebuildTrackMesh();
		RebuildTrainMesh();
		if (Cars) { Cars->ClearInstances(); }
		return;
	}

	// Built once and handed to every train, rather than walked per train: the
	// zones ARE the track, and a second train that derived its own could disagree
	// with the first about where the brakes are.
	TArray<FTrackZone> Zones;
	ZoneReleaseSpeed.Reset();
	ZoneBrakeDecel.Reset();
	ZoneSpans.Reset();

	// Zones come from contiguous runs of segments carrying the same kind, so a
	// lift is however many segments in a row say "Lift" and moving one is an
	// edit rather than a recompile. Arc lengths are accumulated over the
	// segments the track actually ACCEPTED, so a rejected degenerate segment
	// cannot silently shift every zone boundary downstream of it.
	{
		ETUSegmentZone Open = ETUSegmentZone::None;
		double OpenS = 0.0;
		float OpenSpeed = 0.f;
		float OpenAccel = 6.f;
		float OpenDecel = 6.f;
		float OpenPad = 0.f;
		double AccS = 0.0;

		// Block boundaries fall out of this SAME walk, because they are the same
		// fact: a boundary is only meaningful where there is a device that can
		// hold a train, so one falls wherever a powered run starts or ends. No
		// new authored field, no new enumerator, and every track already placed
		// in a level gets blocks without being edited.
		//
		// It also inherits the accepted-geometry filter below for free, so a
		// rejected degenerate segment cannot shift a block boundary any more than
		// it can shift a zone boundary.
		std::vector<double> BlockStarts{0.0};
		auto AddBoundary = [&BlockStarts](double S)
		{
			// Ascending and unique. A zero-length block is one nothing can ever
			// be inside, and two boundaries at the same S would make one.
			if (S > BlockStarts.back())
			{
				BlockStarts.push_back(S);
			}
		};

		// Where each zone's stop mark gets bolted down. Filled in the same walk and
		// in the same order as ZoneReleaseSpeed, so the index is the zone's own.
		std::vector<double> StopMarkS;

		auto Close = [this, &Zones, &Open, &OpenS, &OpenSpeed, &OpenAccel, &OpenDecel,
			&OpenPad, &StopMarkS](double EndS)
		{
			if (Open == ETUSegmentZone::None || !(EndS > OpenS))
			{
				return;
			}
			// AUTHORED NOW, not one 6.0 for every device on every ride. That was
			// the same defect as the merged zone speeds before it: a number
			// nobody typed standing in for one they should have. NL2 exposes
			// exactly these two per device and it is right to — a chain hauling
			// at 0.61 g is nothing like a real chain.
			//
			// The kind still decides WHICH authority exists; these give the
			// magnitude of the ones it has.
			const double Accel = FMath::IsFinite(OpenAccel) && OpenAccel > 0.f
				? static_cast<double>(OpenAccel) : 6.0;
			const double Decel = FMath::IsFinite(OpenDecel) && OpenDecel > 0.f
				? static_cast<double>(OpenDecel) : 6.0;

			// THE PAD, if this device has one. Zero is "no pad", which is every
			// preset until somebody authors a rate -- and the zone then keeps the
			// negative BrakeLimit that means the second device is simply absent.
			const double Pad = FMath::IsFinite(OpenPad) && OpenPad > 0.f
				? static_cast<double>(OpenPad) : 0.0;

			// Sanitised HERE rather than trusted, because AddZone REFUSES a
			// malformed zone instead of storing it — and a refusal would leave the
			// train one zone shorter than ZoneReleaseSpeed, sliding every release
			// speed after it onto the wrong device. The field is clamped in the
			// editor, so this only catches a value that never came from the panel;
			// the point is that the two lists then cannot disagree at all, rather
			// than disagreeing rarely and silently.
			const double Speed = FMath::IsFinite(OpenSpeed) && OpenSpeed > 0.f
				? static_cast<double>(OpenSpeed) : 0.0;

			switch (Open)
			{
			case ETUSegmentZone::Lift:
				Zones.Add(FTrackZone{OpenS, EndS, Speed, Accel, Decel});
				break;
			case ETUSegmentZone::Launch:
				// NO BRAKING AUTHORITY, whatever was typed. That is what the
				// enumerator MEANS, and a number must not grant an authority the
				// device does not have -- a launch that could stop a train is a
				// different machine, and the dispatcher would park trains on it.
				Zones.Add(FTrackZone{OpenS, EndS, Speed, Accel, 0.0});
				break;
			case ETUSegmentZone::Brake:
				// And no tractive authority, for the same reason: a trim cannot
				// start a train, which is the whole distinction from a block brake.
				Zones.Add(FTrackZone{OpenS, EndS, Speed, 0.0, Decel});
				break;
			case ETUSegmentZone::BlockBrake:
			case ETUSegmentZone::Station:
			case ETUSegmentZone::StationUnload:
			case ETUSegmentZone::StationLoad:
				// Brakes AND drive tyres, so identical in shape to a lift chain.
				// The separate enumerators exist for the block boundary and for
				// the Details panel, not for the physics — what makes them hold a
				// train is having BOTH authorities, which is exactly what
				// FTrain::FindHoldZoneAt looks for.
				Zones.Add(FTrackZone{OpenS, EndS, Speed, Accel, Decel});
				break;
			default:
				return;
			}
			// A TRIM ALREADY IS A PAD, so it does not get a second one. Its whole
			// definition is a ceiling with no tractive authority, which is what
			// BrakeLimit means -- giving it another would be one device counted
			// twice and braking at the sum of its own two rates.
			if (Pad > 0.0 && Open != ETUSegmentZone::Brake && Zones.Num() > 0)
			{
				Zones.Last().BrakeLimit = Speed;
				Zones.Last().BrakeDecel = Pad;
			}
			ZoneBrakeDecel.Add(Open == ETUSegmentZone::Brake ? 0.0 : Pad);

			// The authored number is the RELEASE speed. A holding device spends
			// most of its life commanded to zero, so this is the only surviving
			// record of what it should open to.
			ZoneReleaseSpeed.Add(Speed);
			ZoneSpans.Add(FTUZoneSpan{OpenS, EndS, Open});

			// THE STOP MARK, surveyed rather than computed. Its nose clearance is
			// measured back from the far end of the device, because what the margin
			// exists to prevent is a train protruding into the next zone through a
			// defect. Clamped so a device barely longer than the train still puts
			// the mark where the whole train fits behind it.
			//
			// This is the ONE place train length is allowed to touch the holding
			// logic. Everything downstream reads a boolean.
			const double Mark = FMath::Max(OpenS + static_cast<double>(TrainLengthM),
				EndS - HoldNoseClearanceM);
			StopMarkS.push_back(Mark);

			// A DEVICE SHORTER THAN ITS TRAIN CANNOT HOLD IT, and the symptom is
			// vicious rather than obvious: the mark lands PAST the far end, no train
			// ever trips it, so a train sent to park there crawls straight out of
			// its own block into the next one and collides with whatever is
			// standing in it. Measured — a 10 m platform position and a 15 m train
			// gave seven signalling violations inside four seconds.
			//
			// Reported, not repaired, like every other authored-value check here:
			// clamping the mark back inside would produce a device that stops a
			// train with its nose hanging over the boundary, which is worse because
			// it looks like it worked.
			if (Mark > EndS && (Open == ETUSegmentZone::Station
				|| Open == ETUSegmentZone::StationLoad
				|| Open == ETUSegmentZone::StationUnload
				|| Open == ETUSegmentZone::BlockBrake || Open == ETUSegmentZone::Lift))
			{
				UE_LOG(LogTemp, Warning,
					TEXT("TrackUnlimited: the device at %.1f–%.1f m is %.1f m long and cannot "
						"hold a %.1f m train — its stop mark falls at %.1f m, past the end, so "
						"nothing will ever trip it. Lengthen the device or shorten the train."),
					OpenS, EndS, EndS - OpenS, TrainLengthM, Mark);
			}
		};

		// Anti-rollback runs, walked in the SAME pass and by the same rule as
		// zones — contiguous segments carrying the flag become one span — but kept
		// in their own list, because a catch is not a zone. It has no speed and
		// nothing commands it, and it overlaps zones freely: a lift hill is a
		// powered run and a ratchet at the same time, which a single enumerator
		// could not say.
		//
		// Deliberately NOT a block boundary either. A catch cannot release a train,
		// so it is no more a place to park one than a trim brake is.
		TArray<TPair<double, double>> CatchSpans;
		bool bCatchOpen = false;
		double CatchStartS = 0.0;

		for (int32 i = 0; i < Segments.Num(); ++i)
		{
			const double SegLength =
				BuildSegment(Doc.Segments[static_cast<std::size_t>(i)]).Length;
			if (!(SegLength > 0.0))
			{
				continue; // AddSegment refused it, so it occupies no arc length
			}
			if (Segments[i].bAntiRollback != bCatchOpen)
			{
				if (bCatchOpen)
				{
					CatchSpans.Add(TPair<double, double>(CatchStartS, AccS));
				}
				bCatchOpen = Segments[i].bAntiRollback;
				CatchStartS = AccS;
			}
			// A RUN ENDS WHERE THE DEVICE CHANGES, and the device is its kind AND
			// its speed. The speed half used to be missing: a run was defined by
			// kind alone, so a brake at 6 m/s followed immediately by one at 2 m/s
			// became a single zone targeting 6 and the 2 never existed. That is a
			// typed number the build throws away, which is exactly what this
			// project validates against everywhere else — and it was logged as a
			// warning rather than fixed, with the suggested workaround being to
			// wedge a different zone kind between the two.
			//
			// Splitting on it instead is both the smaller rule and the honest one:
			// two speeds is two devices, so it is two zones and two BLOCKS. That is
			// also what lets several holding devices be authored in a row at all —
			// a queue of brake sections keeping trains fed to a platform is not a
			// missing concept, it is block brakes, and until now ten of them in a
			// row authored as one block holding one train.
			//
			// No preset changes: every run in all four is a single speed already.
			const bool bKindChanged = Segments[i].Zone != Open;
			// AND ITS RATES, by exactly the argument that added speed. A brake
			// biting at 8 m/s^2 and one at 2 are two different machines, and
			// merging them discards a typed number to target whichever was
			// authored first -- which this project treats as a defect everywhere
			// else it appears.
			const bool bSpeedChanged = Open != ETUSegmentZone::None
				&& (!FMath::IsNearlyEqual(Segments[i].ZoneSpeed, OpenSpeed)
					|| !FMath::IsNearlyEqual(Segments[i].ZoneAccel, OpenAccel)
					|| !FMath::IsNearlyEqual(Segments[i].ZoneDecel, OpenDecel)
					|| !FMath::IsNearlyEqual(Segments[i].ZoneBrakeDecel, OpenPad));
			// And the author saying so outright, for devices that are identical in
			// every respect the walk can see and are still separate machines —
			// three loading positions on one platform, a queue of brake sections.
			const bool bDeclared = Segments[i].bStartsNewDevice
				&& Segments[i].Zone != ETUSegmentZone::None;
			if (bKindChanged || bSpeedChanged || bDeclared)
			{
				Close(AccS);
				AddBoundary(AccS);
				Open = Segments[i].Zone;
				OpenS = AccS;
				OpenSpeed = Segments[i].ZoneSpeed;
				OpenAccel = Segments[i].ZoneAccel;
				OpenDecel = Segments[i].ZoneDecel;
				OpenPad = Segments[i].ZoneBrakeDecel;
			}
			AccS += SegLength;
		}
		Close(AccS);
		if (bCatchOpen)
		{
			CatchSpans.Add(TPair<double, double>(CatchStartS, AccS));
		}
		CatchSpanList = CatchSpans;

		// THE AUTHORED WALKWAYS, converted rather than derived. A person placed
		// these; this only turns start/stop/side into what the evacuation check
		// reads, and rejects the two ways a typed pair can be meaningless.
		WalkwaySpans.clear();
		for (const FTUWalkway& W : Walkways)
		{
			if (W.Side == ETUWalkway::None || !(W.EndS > W.StartS))
			{
				// Reported, never repaired, like every other authored-value check
				// here. Silently swapping a reversed pair would hide a typo behind
				// a catwalk that looks placed and is not where anybody put it.
				UE_LOG(LogTemp, Warning,
					TEXT("TrackUnlimited: walkway ignored — %.1f to %.1f m is not a span, "
						"or its side is None."), W.StartS, W.EndS);
				continue;
			}
			FWalkwaySpan Span;
			Span.StartS = W.StartS;
			Span.EndS = FMath::Min(static_cast<double>(W.EndS), AccS);
			Span.Side = ToWalkwaySide(W.Side);
			WalkwaySpans.push_back(Span);
		}

		// AN INVENTORY OF WHAT IS ACTUALLY ON THIS TRACK. The devices are the part
		// of a layout you cannot see, and the coloured rails only help once you are
		// looking at the right thing — so the build says what it found.
		//
		// It exists because of a specific confusion that is easy to hit and hard to
		// diagnose from the viewport: a station authored as a Lift is a lift. It
		// runs, it holds a train, it is drawn green, and it is wrong in nothing
		// except what it MEANS — no station process, no boarding sequence, and the
		// dispatch permissive missing its whole other half. A level saved before a
		// zone kind existed keeps the old kind until its preset is reloaded, and
		// nothing about that announces itself.
		{
			int32 ByKind[8] = {};
			for (const FTUZoneSpan& Z : ZoneSpans)
			{
				const int32 K = static_cast<int32>(Z.Kind);
				if (K >= 0 && K < 8) { ++ByKind[K]; }
			}
			const int32 Platforms_ =
				ByKind[static_cast<int32>(ETUSegmentZone::Station)]
				+ ByKind[static_cast<int32>(ETUSegmentZone::StationUnload)]
				+ ByKind[static_cast<int32>(ETUSegmentZone::StationLoad)];

			UE_LOG(LogTemp, Log,
				TEXT("TrackUnlimited: devices — %d station, %d unload, %d load, %d lift, "
					"%d launch, %d trim brake, %d block brake"),
				ByKind[static_cast<int32>(ETUSegmentZone::Station)],
				ByKind[static_cast<int32>(ETUSegmentZone::StationUnload)],
				ByKind[static_cast<int32>(ETUSegmentZone::StationLoad)],
				ByKind[static_cast<int32>(ETUSegmentZone::Lift)],
				ByKind[static_cast<int32>(ETUSegmentZone::Launch)],
				ByKind[static_cast<int32>(ETUSegmentZone::Brake)],
				ByKind[static_cast<int32>(ETUSegmentZone::BlockBrake)]);

			if (Platforms_ == 0 && ZoneSpans.Num() > 0)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("TrackUnlimited: no station on this track, so nothing runs a boarding "
						"sequence and the dispatch permissive is the interlocking alone. If it "
						"HAS a station, it is authored as a lift or a block brake — which works "
						"and is drawn green rather than blue. Reload the preset to fix a layout "
						"saved before the Station kind existed."));
			}
		}

		// Reported before it is used, not repaired silently — FRideSignals will
		// repair it either way, but a walk that produced something malformed is a
		// bug upstream and should say so rather than be absorbed.
		if (!FRideSignals::IsWellFormed(BlockStarts))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("TrackUnlimited: derived block boundaries are malformed; repairing."));
		}
		// A train needs somewhere to STAND: a device that can both stop it and
		// start it again. Drive tyres and block brakes qualify; a trim brake can
		// hold a train and never release it, and a launch can release one and
		// never hold it. So the number of trains the layout can run is the number
		// of hold-capable zones, and asking for more is refused rather than
		// granted into open course.
		// A TRAIN IS ITS CARS. Derived here rather than kept in step by hand: two
		// fields that must agree are one field and a bug waiting for whichever gets
		// written second.
		TrainLengthM = FMath::Max(0.f, CarCount * CarLengthM);

		TArray<double> HoldMidS;
		HoldZoneIndices.Reset();
		ShortestHoldM = 0.0;
		for (int32 z = 0; z < Zones.Num(); ++z)
		{
			if (Zones[z].MaxAccel > 0.0 && Zones[z].MaxDecel > 0.0)
			{
				// The MIDDLE, because that is where the dispatcher parks a held
				// train and where a whole train fits inside one block. Placed at a
				// zone's start instead, a 15 m train hangs back over the boundary —
				// and for the station, whose start is the seam, that means a train
				// placed there collides with one placed in the LAST block.
				HoldMidS.Add(0.5 * (Zones[z].StartS + Zones[z].EndS));
				HoldZoneIndices.Add(z);
				const double Len = Zones[z].EndS - Zones[z].StartS;
				ShortestHoldM = (ShortestHoldM <= 0.0) ? Len : FMath::Min(ShortestHoldM, Len);
			}
		}

		// ONE HOLDING PLACE MUST STAY FREE, or nothing can ever move: every train
		// is standing where the train behind it needs to go, and the ride gridlocks
		// without a single violation to show for it. MEASURED on this circuit,
		// which has five: four trains run clean and five never move at all.
		// ZERO IS A REAL ANSWER, and the floor of 1 refused it. Taking every train
		// off for maintenance is an ordinary state of a real ride, and the layout
		// is still perfectly valid with nothing on it -- the no-train case is
		// already handled everywhere, because a track being built has none either.
		const int32 Wanted = FMath::Max(0, TrainCount);
		// ===================== NOWHERE TO PUT A TRAIN IS A REAL LAYOUT =====================
		//
		// N holding places run N-1 trains, because one has to stay free for anything
		// to move -- and the floor of 1 was there so a layout with exactly one place
		// still runs a train rather than none.
		//
		// BUT IT APPLIED WITH ZERO PLACES TOO, and then Place(HoldMidS[0]) indexed an
		// empty array. A blank track with one straight on it is exactly that layout,
		// which is what somebody gets from the menu's Blank template and their first
		// [I] -- so the very first thing a new author does crashed the editor.
		//
		// The floor is conditional now: no holding device means no train, which is
		// the honest answer rather than a special case. Track with nowhere to park is
		// track somebody is still building.
		const int32 Places = HoldMidS.Num();
		HoldingPlaces = Places;   // so the panel can say what the ceiling is
		const int32 Capacity = Places == 0 ? 0 : FMath::Max(1, Places - 1);
		const int32 Running = FMath::Min(Wanted, Capacity);
		if (Places == 0)
		{
			// SAID PLAINLY AND WITHOUT ALARM. This is the normal state of a track
			// being built, not a fault -- so it names the next step rather than
			// reporting a failure.
			UE_LOG(LogTemp, Log,
				TEXT("TrackUnlimited: no train yet — nowhere to park one. Give a segment a "
					"station or a block brake and a train appears on it."));
		}
		else if (Running < Wanted)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("TrackUnlimited: %d trains asked for, %d run — the layout has %d place(s) "
					"that can both hold a train and release it, and one has to stay free for "
					"anything to move. Add a block brake or drive-tyre run to park another."),
				Wanted, Running, HoldMidS.Num());
		}

		// MEASURED, not authored. A layout either comes back to where it started
		// or it does not, and only the geometry knows — so nobody gets to tick a
		// "this is a circuit" box and be wrong about it. All three have to hold:
		// position, because a gap is a gap; HEADING, because a track that returns
		// to the right place pointing the wrong way is not joined; and ROLL,
		// because the rider would be flipped at the seam.
		//
		// A millimetre and a hundredth of a degree are far below anything a rider
		// or a mesh can resolve, and far above the ~1e-5 m that float-stored
		// authored values actually leave.
		{
			const double End = Track.TotalLength();
			const FTrackFrame A = Track.EvaluateAt(0.0);
			const FTrackFrame B = Track.EvaluateAt(End);
			const double Gap = Length(B.Position - A.Position);
			const double Head = FMath::RadiansToDegrees(FMath::Acos(
				FMath::Clamp(Dot(B.Tangent, A.Tangent), -1.0, 1.0)));
			const double Roll = FMath::RadiansToDegrees(FMath::Abs(B.Roll - A.Roll));
			bTrackIsCircuit = Gap < 1e-3 && Head < 0.01 && Roll < 0.01;

			UE_LOG(LogTemp, Log,
				TEXT("TrackUnlimited: seam %.6f m, %.6f deg heading, %.6f deg roll — %s"),
				Gap, Head, Roll,
				bTrackIsCircuit ? TEXT("CLOSED CIRCUIT, trains run laps")
								: TEXT("open layout, trains return to the station by teleport"));
		}

		// The switches go in with the track. Rebuilt wholesale for the same reason
		// the blocks are: move a brake run and you have moved its stop mark.
		StopMarks = MakeUnique<FTrackSensors>(StopMarkS);

		// And the motors, one per zone. A friction-only trim brake has no motor but
		// keeps its slot: an index that means the same thing in the zone list and
		// the drive list is worth more than the empty entry.
		//
		// ponytail: no ramp configured, so every output follows its command
		// instantly and nothing measured before drives existed moves. Give a device
		// a real FDriveSpec when a gentle station release is worth authoring, not
		// before — and note that a ramp slower than the zone's grip is the only kind
		// that changes anything.
		Drives = MakeUnique<FTrackDrives>(static_cast<std::size_t>(ZoneReleaseSpeed.Num()));
		ReportedDriveFault.Reset();   // new motors, so old faults are not this ride's

		// A platform for every station zone. One position each — a split operation
		// authors its unload and load as separate zones, which is why they are
		// separate kinds, and each gets its own sequence here.
		Platforms.Reset();
		for (int32 z = 0; z < ZoneSpans.Num(); ++z)
		{
			// NOT "Role": AActor has one, it is the network role, and shadowing it
			// is a warning-as-error here. The same class of collision CLAUDE.md
			// records for FFrame and FField, met on a member name instead of a type.
			EStationRole PlatformRole;
			switch (ZoneSpans[z].Kind)
			{
			case ETUSegmentZone::Station:       PlatformRole = EStationRole::Combined; break;
			case ETUSegmentZone::StationUnload: PlatformRole = EStationRole::Unload;   break;
			case ETUSegmentZone::StationLoad:   PlatformRole = EStationRole::Load;     break;
			default: continue;
			}
			FTUPlatform P;
			P.Zone = z;
			P.Process = FStationProcess(PlatformRole);
			P.Crew.UnloadSeconds = UnloadSeconds;
			P.Crew.LoadSeconds = LoadSeconds;
			P.Crew.SecureSeconds = RestraintCheckSeconds;
			P.Crew.Restraints.TravelSeconds = RestraintTravelSeconds;
			P.Crew.Restraints.Groups = FMath::Max(1, RestraintGroups);
			// Airgates: the same device in a different place, and they travel
			// faster than a restraint bar because they carry nothing.
			P.Crew.Gates.TravelSeconds = RestraintTravelSeconds * 0.5f;
			P.Crew.Gates.Groups = FMath::Max(1, RestraintGroups);
			Platforms.Add(P);
		}

		Signals = MakeUnique<FRideSignals>(BlockStarts, BlockBufferSeconds,
			static_cast<std::size_t>(FMath::Max(1, DispatchLookahead)),
			static_cast<std::size_t>(Running), bTrackIsCircuit);

		// Where those boundaries actually are on the track, walked once now rather
		// than integrated eleven times a frame to draw eleven posts.
		BuildBlockMarks();

		// THE PROGRAM'S IDENTITY, DERIVED FROM THE LAYOUT IT IS FOR.
		//
		// A digest of the derived blocks and zones — not of the authored segment
		// list, because two different segment lists that produce the same control
		// structure genuinely are the same program, and a geometry tweak that
		// moves no boundary should not invalidate it.
		//
		// The program is rebuilt here, so it matches by construction. What that
		// buys is the check: if the loaded program's identity ever DISAGREES with
		// the layout it is installed on, the controller refuses to run. That is
		// the detector for "I changed the code and the editor is still doing the
		// old thing" — a class that has bitten this project and had nothing
		// watching for it.
		{
			FSimDigest Id;
			for (double B : BlockStarts) { Id.Add(B); }
			for (const FTUZoneSpan& Z : ZoneSpans)
			{
				Id.Add(static_cast<int>(Z.Kind));
				Id.Add(Z.StartS);
				Id.Add(Z.EndS);
			}
			Plc.SetLayoutIdentity(Id.Value());
			Plc.LoadProgram(Id.Value());
		}

		// The second detection method: a switch at every block boundary, and a
		// counter that derives occupancy from their trips and nothing else.
		// Circuits only — see the header for why the ring wrap is a lie on an open
		// layout. Seeded once the trains are placed, further down.
		if (bTrackIsCircuit)
		{
			BlockSensors = MakeUnique<FTrackSensors>(BlockStarts);
			Counter = MakeUnique<FBlockCounter>(*BlockSensors);
		}
		else
		{
			BlockSensors.Reset();
			Counter.Reset();
		}

		// THE BRAKING-DISTANCE RULE, expressed at last. Tell the signalling which
		// blocks contain a device that can stop a train and let it go again, and
		// its permissive clears all the way to the next one of those rather than a
		// fixed count of blocks — because a train let into a block with nothing in
		// it is COMMITTED until it reaches somewhere it can stop.
		//
		// MEASURED, and this is not a refinement: with a fixed count, four trains
		// on this circuit COLLIDE — 14 violations at lookahead 1, 18 at lookahead 2
		// — because a train is granted a free block and finds the one beyond it
		// occupied on arrival. No single count can be right for a layout whose free
		// runs are 696 m and 184 m. With this, all four run clean.
		{
			std::vector<bool> CanHold(BlockStarts.size(), false);
			for (std::size_t b = 0; b < BlockStarts.size(); ++b)
			{
				const double S0 = BlockStarts[b];
				const double S1 = (b + 1 < BlockStarts.size()) ? BlockStarts[b + 1] : AccS;
				const double Mid = 0.5 * (S0 + S1);
				for (const FTrackZone& Z : Zones)
				{
					if (Mid >= Z.StartS && Mid <= Z.EndS
						&& Z.MaxAccel > 0.0 && Z.MaxDecel > 0.0)
					{
						CanHold[b] = true;
						break;
					}
				}
			}
			Signals->SetHoldingBlocks(CanHold);
		}

		{
			FString Where;
			for (const double S : Signals->Boundaries())
			{
				Where += FString::Printf(TEXT("%.1f  "), S);
			}
			UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: %d blocks, boundaries at %sm; "
				"lookahead %d, overlap %.1f s, %d train(s), %d holding place(s)"),
				static_cast<int32>(Signals->NumBlocks()), *Where,
				static_cast<int32>(Signals->Lookahead()), BlockBufferSeconds,
				Running, HoldMidS.Num());
		}

		FTrainConfig TrainConfig;
		TrainConfig.TrainLength = TrainLengthM;
		TrainConfig.bAllowRollback = bAllowRollback;

		Trains.Reset();
		StoppedForS.Init(0.f, Running);
		for (int32 t = 0; t < Running; ++t)
		{
			TUniquePtr<FTrain> New = MakeUnique<FTrain>(Track, TrainConfig);
			for (const FTrackZone& Z : Zones)
			{
				New->AddZone(Z);
			}
			for (const TPair<double, double>& C : CatchSpans)
			{
				New->AddAntiRollback(C.Key, C.Value);
			}
			// Mid-device, every one of them including the station — the same place
			// the dispatcher parks a held train, and the only placement that puts
			// a whole train inside a single block.
			New->Place(HoldMidS[t], 0.0);
			Trains.Add(MoveTemp(New));
		}

		BrakeStartS = AccS;
		for (int32 i = Segments.Num() - 1; i >= 0; --i)
		{
			if (Segments[i].Zone != ETUSegmentZone::Brake
				&& Segments[i].Zone != ETUSegmentZone::BlockBrake)
			{
				break;
			}
			BrakeStartS -= BuildSegment(Doc.Segments[static_cast<std::size_t>(i)]).Length;
		}
	}

	// Seed occupancy from where the trains actually are, before anything asks a
	// permissive. Without this the station block reads CLEAR with a train sitting
	// in it, and the first dispatch is granted against a lie.
	if (Signals)
	{
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			Signals->Update(static_cast<std::size_t>(t), Trains[t]->GetRearS(),
				Trains[t]->GetFrontS());
		}
	}

	// And SEED THE COUNTER the same way, which is the operator's sweep: walk the
	// ride, confirm where every train is, zero the counters. Without it the counter
	// is right in the middle of a run and wrong at the start of one — a train
	// beginning inside block 0 sends the block BEHIND it to minus one the moment
	// its tail leaves the first sensor, counted out of somewhere it was never
	// counted into. Real rides are swept for exactly this reason.
	if (BlockSensors && Signals)
	{
		// SCAN FIRST, THEN BUILD THE COUNTER. Its constructor snapshots the
		// sensors' edge totals, so a counter built afterwards starts from what the
		// switches already read and does not process the rising edges a train
		// parked on a boundary produced when the sensors were first covered. Built
		// before the scan, that train gets counted in AND seeded in — two trains in
		// a block holding one.
		ScanBlockSensors();
		Counter = MakeUnique<FBlockCounter>(*BlockSensors);

		// Seeded by SPAN rather than by centre block, because a train straddling a
		// boundary really is in both and the interlocking already says so. Asking
		// which blocks it holds is the only way to seed the two to agree — which is
		// the entire point of running them side by side.
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			for (std::size_t b = 0; b < Counter->NumBlocks(); ++b)
			{
				if (Signals->OccupiedBy(static_cast<std::size_t>(t), b))
				{
					Counter->Seed(b);
				}
			}
		}
	}

	// Where the ride's lowest structural point sits relative to the heartline
	// origin, so ToWorld can lift the whole thing onto the ground.
	//
	// The heartline is RIDER height, not track height: at the station the rails
	// hang 1.1 m below it and the spine another 0.45 m below those, so track
	// z = 0 is about 1.7 m above the bottom of the structure. Place the actor at
	// world z = 0 without accounting for that and the station buries itself —
	// the rails go under, and the camera, which rides the heartline exactly,
	// gets cut in half by the ground plane.
	//
	// Computed, not typed, and that is the point. The offset depends on the
	// whole layout: a banked or inverted section puts the structure somewhere
	// else entirely relative to the heartline, and every edit to the track moves
	// it again. A constant here would be a number someone has to remember to
	// re-derive, which is exactly how this ended up as a hand-tuned actor Z
	// twice already.
	{
		const double SpineRadius = Profile.SpineDiameter * 0.5;
		const double Total = Track.TotalLength();
		double Lowest = 0.0;

		FTrackFrame Walk = Track.EvaluateAt(0.0);
		double S = 0.0;
		for (;;)
		{
			const FTrackCrossSection Section =
				CrossSectionAt(Walk, Track.GetHeartlineHeight(), Profile);
			Lowest = FMath::Min3(Lowest, FMath::Min(Section.LeftRail.Z, Section.RightRail.Z),
				Section.SpineCentre.Z - SpineRadius);
			if (S >= Total)
			{
				break;
			}
			// 1 m steps. This is looking for a minimum over a smooth curve, not
			// resolving geometry — the sag between samples is far below the
			// clearance any real support structure needs anyway.
			const double Next = FMath::Min(S + 1.0, Total);
			Walk = Track.AdvanceFrom(Walk, S, Next);
			S = Next;
		}
		GroundOffsetM = -Lowest;
	}

	// Everything an author needs to know about the edit they just made, without
	// riding it. Each of these was a defect that took riding the track to find:
	// a discontinuity, an underground back half, and a loop passing through
	// itself. None of them are visible in a G trace.
	// Run the whole ride now, at edit time. An author needs to know a hill is
	// too tall BEFORE watching a train fail to crest it, and the extremes are
	// worth more with a position attached: "4.25 g" is a number, "4.25 g at
	// S=310 m" is somewhere to go and look.
	//
	// Run on train 0 with every holding device still OPEN, which is deliberate:
	// the profile answers "what does this layout do to a rider", not "what does
	// the signalling do to a timetable". A profile measured through a red would
	// report a stall at the first block brake and call the ride broken.
	//
	// AND THERE MAY BE NO TRAIN AT ALL, which is not an error: a track with no
	// station or block brake has nowhere to park one, and that is the ordinary
	// state of a layout somebody is halfway through building. `Trains[0]` on that
	// crashed, which meant the first segment placed on a blank track took the
	// editor down with it.
	//
	// A DEFAULT PROFILE IS THE RIGHT ANSWER rather than a skipped assignment: it
	// carries bCompleted = false, which every reader already handles, because a
	// ride that did not happen is a case the graph and the diagnostics panel were
	// built around. There is nothing new to teach them.
	if (Trains.IsEmpty() || !Trains[0].IsValid())
	{
		Profile_ = FRideProfile();
		LastDeviceFindings.clear();
		BuildDiagnostics();
		// Same as above: the track is still drawn, the trains that are not there
		// are not.
		RebuildTrackMesh();
		RebuildTrainMesh();
		if (Cars) { Cars->ClearInstances(); }
		return;
	}
	Profile_ = RunRideProfile(*Trains[0], Track, 1.0);

	// ===================== WHAT THE DEVICES WILL ACTUALLY DO =====================
	//
	// HERE rather than in BuildDiagnostics, because this is the one place the
	// derived zones and the ride profile both exist. Deriving the zones a second
	// time to ask these questions would be a second derivation that can disagree
	// with the first, which this project treats as a defect wherever it appears.
	//
	// THE AUTHORITIES COME FROM THE ZONE, NOT FROM THE SEGMENT ENUM, and they come
	// from the SAME TEST the interlocking uses one screen down — MaxAccel > 0 and
	// MaxDecel > 0. Reading the enum here instead would be a second opinion about
	// what a block boundary is, and the day the two disagreed the panel would be
	// reassuring somebody about a layout the signalling was refusing to run.
	{
		LastDeviceFindings.clear();
		std::vector<FDeviceSpan> Devices;
		Devices.reserve(static_cast<std::size_t>(Zones.Num()));
		for (const FTrackZone& Z : Zones)
		{
			FDeviceSpan D;
			D.StartS = Z.StartS;
			D.EndS = Z.EndS;
			D.CommandedSpeed = Z.TargetSpeed;
			D.bCanHold = Z.MaxDecel > 0.0;
			D.bCanRelease = Z.MaxAccel > 0.0;
			// THE ZONE'S OWN RATE. The audit had a global service deceleration and
			// it did not match the Grip these are built with, so it was predicting
			// stopping distances the physics would never produce.
			// WHAT ACTUALLY STOPS THE TRAIN is the harder of the two devices, and
			// on a section with a pad that is the pad. Reading MaxDecel alone
			// would report a brake as too short that stops comfortably.
			D.DecelMs2 = FMath::Max(Z.MaxDecel, Z.BrakeDecel);
			D.bIsBlockBoundary = D.bCanHold && D.bCanRelease;

			// The name is only for the message. FTrackZone drops the kind on
			// purpose — a station, a block brake and a lift chain are the same
			// physics — so it is looked up by midpoint from the list that kept it.
			const double Mid = 0.5 * (Z.StartS + Z.EndS);
			D.Name = "device";
			for (const FTUZoneSpan& Span : ZoneSpans)
			{
				if (Mid >= Span.StartS && Mid <= Span.EndS)
				{
					D.Name = TCHAR_TO_UTF8(ZoneKindName(Span.Kind));
					break;
				}
			}
			Devices.push_back(D);
		}

		FDeviceAuditSettings Audit;
		Audit.TrainLengthM = static_cast<double>(TrainLengthM);
		Audit.NoseClearanceM = HoldNoseClearanceM;

		// A RIDE THAT DID NOT COMPLETE HAS NO SPEEDS TO READ, and reporting a
		// braking distance against a profile that stopped at 46 m would be the
		// envelope suite's own old failure — a verdict on a ride that did not
		// happen. The length and authority checks need no speed and still run;
		// this reports 0 past the end, which is silent rather than wrong.
		const FRideProfile& P = Profile_;
		LastDeviceFindings = AuditDevices(Devices, Audit, [&P](double S) -> double
		{
			if (!P.bCompleted || P.Samples.empty()) { return 0.0; }
			// Samples are ordered in S. A handful of devices against a few
			// thousand samples does not need a binary search to find them.
			double Best = 0.0;
			for (const FRideSample& Sm : P.Samples)
			{
				if (Sm.S > S) { break; }
				Best = Sm.Speed;
			}
			return Best;
		});
	}

	// BACK TO ITS HOLDING POSITION, not to zero. The profile run leaves train 0
	// wherever the ride ended, so it has to be put back — but putting it at 0.0
	// puts it ON THE SEAM, where a train straddles the boundary and legitimately
	// holds the first block and the last. That was harmless while nothing checked,
	// and the moment the counter cross-check arrived it tripped the ride on frame
	// one: the interlocking held blocks 0 and 10, the counter had been seeded for
	// block 0 only, and the two disagreed exactly as they are supposed to when one
	// of them is wrong.
	//
	// Which is the cross-check doing its job on its first run, against a line that
	// had been quietly wrong since before any of this existed.
	// NO TRAINS IS NOW REACHABLE WITH HOLDING PLACES PRESENT, which it never was.
	// The old floor of 1 meant a track with somewhere to park always had something
	// parked, so this block could index Trains[0] safely; SHED in the maintenance
	// panel takes the last one off and leaves the places behind. Same shape as the
	// blank-template crash -- a guard that was true by accident rather than by
	// construction.
	if (!Trains.IsEmpty() && Trains[0].IsValid())
	{
		// Mid of the first holding device, which is where it was placed to begin
		// with. Recomputed from HoldZoneIndices rather than reaching for the local
		// the placement loop used, which is long out of scope by here.
		double Home = 0.0;
		if (HoldZoneIndices.Num() > 0)
		{
			const FTrackZone Z =
				Trains[0]->GetZone(static_cast<std::size_t>(HoldZoneIndices[0]));
			Home = 0.5 * (Z.StartS + Z.EndS);
		}
		Trains[0]->Place(Home, 0.0);
	}

	// NOW shut them, once the profile has been taken. Brakes-on is the resting
	// state of real ride control, and the alternative — open until a dispatcher
	// notices — is open for exactly one frame every time, which is one frame of a
	// train being pushed through a red.
	//
	// And NOW close the circuit, for the same reason and in the same breath: the
	// profile walks arc length forwards and stops at the end, so on a lapping
	// train it would never stop and its samples would overwrite each other. The
	// ride profile is one lap, measured open; the ride itself laps.
	// Shut at the DRIVE now, not on each train. Every drive is preset — commanded
	// and already at its output, because a ride opens with its motors running and
	// one that ramps up from zero on the first frame of the session is a lift chain
	// standing still when the first train reaches it. Holding devices preset to
	// zero; everything else to what it was authored at.
	if (Drives)
	{
		for (int32 z = 0; z < ZoneReleaseSpeed.Num(); ++z)
		{
			Drives->Preset(static_cast<std::size_t>(z),
				HoldZoneIndices.Contains(z) ? 0.0 : ZoneReleaseSpeed[z]);
		}
	}
	for (const TUniquePtr<FTrain>& T : Trains)
	{
		for (const int32 Zi : HoldZoneIndices)
		{
			T->SetZoneTargetSpeed(static_cast<std::size_t>(Zi), 0.0);
		}
		T->SetCircuit(bTrackIsCircuit);
	}

	if (!Profile_.bCompleted)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrackUnlimited: the train does not get round — it %s at %.1f m, %.1f m up. ")
			TEXT("Everything past that point is unridden."),
			Profile_.bRolledBack ? TEXT("ROLLS BACK") : TEXT("stalls"),
			Profile_.StalledAtS, Profile_.StalledHeight);
	}
	else
	{
		UE_LOG(LogTemp, Log,
			TEXT("TrackUnlimited ride: %.0f s | top %.1f km/h at %.0f m | vertical %+.2f at %.0f m ")
			TEXT("to %+.2f at %.0f m | lateral %.2f at %.0f m | roll rate %.0f deg/s at %.0f m"),
			Profile_.Duration, Profile_.TopSpeed * 3.6, Profile_.TopSpeedS, Profile_.MinVerticalG,
			Profile_.MinVerticalGS, Profile_.MaxVerticalG, Profile_.MaxVerticalGS,
			Profile_.MaxAbsLateralG, Profile_.MaxAbsLateralGS, Profile_.MaxAbsRollRate,
			Profile_.MaxAbsRollRateS);
	}

	const FClosureGap Gap = MeasureClosure(Track, HeightTarget(Track));
	const FClearanceReport Clear = AnalyseSelfClearance(Track, Profile, 1.0, 12.0);
	UE_LOG(LogTemp, Log,
		TEXT("TrackUnlimited: %d segments, %.1f m, C2=%s | ends %+.2f m vs station | ")
		TEXT("closest approach %.2f m%s | sits %.2f m above the heartline origin"),
		static_cast<int32>(Track.NumSegments()), Track.TotalLength(),
		Track.IsCurvatureContinuous() ? TEXT("yes") : TEXT("NO"), Gap.HeightError,
		Clear.ClosestApproach,
		Clear.bStructureOverlaps ? TEXT(" (TRACK PASSES THROUGH ITSELF)") : TEXT(""),
		GroundOffsetM);

	// AFTER GroundOffsetM is known, because the port lifts every vertex by it —
	// the heartline origin is RIDER height and z = 0 in track space is about
	// 1.7 m above the bottom of the spine. Built here rather than beside
	// DrawTrack so both the editor preview and BeginPlay get it from one place.
	// THE STYLE IS THE SECTION AS WELL AS THE COLOUR, and it is applied HERE
	// rather than inside RebuildTrackMesh — which returns early when the mesh is
	// off, and would then leave the wireframe and the support planner reading a
	// gauge nobody chose. Everything downstream shares one Profile, so it has to
	// be set before any of them run.
	//
	// Changing a gauge without changing the paint would give a different track
	// that looks the same, which is exactly the confusion a "style" prevents — so
	// one pick moves both.
	{
		const FTUTrackStyle Style = ActiveStyle();
		Profile.Gauge = Style.GaugeM;
		Profile.RailDiameter = Style.RailDiameterM;
		Profile.SpineDrop = Style.SpineDropM;
		Profile.SpineDiameter = Style.SpineDiameterM;
		Profile.TieSpacing = Style.TieSpacingM;
		Profile.TieDiameter = Style.TieDiameterM;
	}

	RebuildTrackMesh();

	// AFTER everything else, because it reads the ride profile and the derived
	// blocks as well as the validator — and a panel showing findings about a
	// track that has just been replaced is worse than an empty one.
	BuildDiagnostics();
}

#if WITH_EDITOR
void ATUCoasterRide::PostEditChangeProperty(FPropertyChangedEvent& Event)
{
	// Before Super, deliberately: Super reruns the construction script, and
	// OnConstruction is what rebuilds and redraws. Resetting afterwards would
	// leave the preview showing the layout that was just replaced.
	if (bLoadPreset)
	{
		bLoadPreset = false;
		Segments = PresetLayout(Preset);
		ApplyPresetTrainSetup(Preset);
		ApplyPresetWalkways();
	}

	// ===================== EDITS ARE A MODE QUESTION =====================
	//
	// CONSTRAINT 1, ONE LEVEL UP. A ride that is running is not a ride being
	// edited, and an edit landing mid-lap would change the geometry under a
	// train — which on a circuit means changing the track a train is standing on.
	//
	// REFUSED rather than deferred: applying it when the mode changes back would
	// mean somebody's edit taking effect minutes later with no memory of having
	// made it. The message says which mode to be in, because a field that
	// silently reverts is indistinguishable from one that is broken.
	if (!Session.EditsAllowed())
	{
		UE_LOG(LogTUEvents, Warning,
			TEXT("edit refused: the ride is in %s. [Tab] back to BUILD to change the track."),
			UTF8_TO_TCHAR(AppModeName(Session.Mode())));
		Super::PostEditChangeProperty(Event);
		return;
	}

	// Rebuild and redraw happen in OnConstruction, so a typed number gets an
	// answer immediately — the drawn track, plus total length, continuity, where
	// it ends up and whether it hits itself. The viewport stays a read-only
	// preview: this is feedback, not manipulation.
	Super::PostEditChangeProperty(Event);

	// ===================== THE DETAILS PANEL FEEDS THE SAME HISTORY =====================
	//
	// It has Unreal's transactions and does not need ours, so this looks like a
	// second undo stack for the same edits — and the reason it is right anyway is
	// the case where it is MISSING. A Details edit our history never saw is a
	// document [J] does not know about, so the next undo would restore a state
	// from before it and silently revert somebody's typing.
	//
	// A MERGE KEY HERE AND NOT IN THE RUNTIME EDITOR, because this is the control
	// that fires continuously: a spinner dragged across twenty values is twenty
	// callbacks and one edit. Keyed on the property, cleared by touching a
	// different one, which is exactly what TrackHistory asks the UI to supply.
	if (History)
	{
		const FName Changed = Event.GetPropertyName();
		PushHistory(FString::Printf(TEXT("%s in the Details panel"), *Changed.ToString()),
			Changed.ToString());
	}
}
#endif

FVector ATUCoasterRide::ToLocal(const FVec3& V) const
{
	// Mirror Y: the prototype frame is right-handed, Unreal is left-handed.
	// Lift by GroundOffsetM: the heartline origin is RIDER height, not track
	// height, so z = 0 in track space is about 1.7 m above the bottom of the
	// spine. See RebuildFromSegments for why this is computed rather than typed.
	return FVector(V.X, -V.Y, V.Z + GroundOffsetM) * MetresToUU;
}

FVector ATUCoasterRide::ToLocalDirection(const FVec3& V) const
{
	// The same mirror and NOTHING ELSE. A normal put through ToLocal would come
	// back as a point a metre and a half above the origin rather than a
	// direction, which reads as lighting that is subtly wrong everywhere.
	return FVector(V.X, -V.Y, V.Z);
}

FVector ATUCoasterRide::ToWorld(const FVec3& V) const
{
	return GetActorLocation() + ToLocal(V);
}

FQuat ATUCoasterRide::ToWorldRotation(const FTrackFrame& Frame) const
{
	// Measured, not guessed: mirroring Y flips handedness, so the rider's LEFT
	// must be negated to become Unreal's +Y (right). Using the mirrored lateral
	// directly gives an exactly inverted basis, and the track still looks
	// self-consistent while being mirrored — see Docs/PHASE0_FINDINGS.md.
	const FVector Forward(Frame.Tangent.X, -Frame.Tangent.Y, Frame.Tangent.Z);
	const FVector Right(-Frame.Lateral.X, Frame.Lateral.Y, -Frame.Lateral.Z);
	const FVector Up(Frame.Up.X, -Frame.Up.Y, Frame.Up.Z);

	// Built from the basis directly. The frame is already exactly orthonormal,
	// so going via FRotator angles would only reintroduce error.
	return FMatrix(Forward, Right, Up, FVector::ZeroVector).ToQuat();
}

void ATUCoasterRide::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	if (PlayerInputComponent == nullptr)
	{
		return;
	}

	// ponytail: direct key bindings, no Enhanced Input mapping context and no
	// input assets. Enhanced Input's real advantages — rebinding, modifiers,
	// layered contexts — only arrive with content assets to hold them, and
	// creating those objects transiently in code buys the machinery without
	// most of the benefit. This is ~20 lines and works. Move to Enhanced Input
	// when a shipped build needs users to rebind keys, which is a Phase 6
	// concern rather than a today one.
	PlayerInputComponent->BindKey(EKeys::C, IE_Pressed, this, &ATUCoasterRide::CycleCameraMode);

	// THE OPERATOR. Space is the dispatch button and is bound on BOTH edges,
	// because the release is half the safety rule — a control that only ever
	// reports "pressed" is a control that can be wedged. Backspace stops the ride
	// and End resets it, deliberately far apart on the keyboard: the two are
	// nowhere near each other on a real panel either.
	PlayerInputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this,
		&ATUCoasterRide::PressDispatch);
	PlayerInputComponent->BindKey(EKeys::SpaceBar, IE_Released, this,
		&ATUCoasterRide::ReleaseDispatch);
	// BACKSPACE IS WANTED TWICE — the emergency stop, and deleting a digit. It
	// dispatches on whether a field has focus rather than being bound twice,
	// because two handlers on one key both fire.
	PlayerInputComponent->BindKey(EKeys::BackSpace, IE_Pressed, this,
		&ATUCoasterRide::KeyBackspace);
	// MONITORED RESET: pressed AND released, both bound, because the reset happens
	// on the release. A key bound only on press is a taped button.
	PlayerInputComponent->BindKey(EKeys::End, IE_Pressed, this,
		&ATUCoasterRide::PressResetButton);
	PlayerInputComponent->BindKey(EKeys::End, IE_Released, this,
		&ATUCoasterRide::ReleaseResetButton);
	PlayerInputComponent->BindKey(EKeys::P, IE_Pressed, this,
		&ATUCoasterRide::CyclePanelView);
	// NOT [A] — that is strafe-left, and acknowledging a fault every time somebody
	// moved the free camera is exactly the kind of accidental press the whole
	// acknowledge/reset ordering exists to prevent.
	//
	// A real console separates ACKNOWLEDGE and RESET physically, onto different
	// coloured fields. On a keyboard that separation cannot be meaningful, so the
	// ordering is enforced in software instead: a reset is REFUSED while anything
	// is unacknowledged.
	PlayerInputComponent->BindKey(EKeys::Home, IE_Pressed, this,
		&ATUCoasterRide::AcknowledgeFaults);

	// ONLY THE MOUSE IS AN AXIS HERE. BindAxisKey asserts on IsAxis1D(), and a
	// letter key is a BUTTON — WASDQE were bound this way and were always wrong,
	// surviving only because `ensure` logs and carries on. They are polled in
	// PollMovementKeys instead, which is also six bindings and six methods less.
	PlayerInputComponent->BindAxisKey(EKeys::MouseX, this, &ATUCoasterRide::AxisLookYaw);
	PlayerInputComponent->BindAxisKey(EKeys::MouseY, this, &ATUCoasterRide::AxisLookPitch);

	// HOLD RIGHT TO LOOK. The cursor is free wherever there are rows to click, so
	// a camera that turned on every mouse move would make reaching for a row a
	// gamble on where it had gone by the time you got there. Right rather than
	// left because left is already select, and every editor with a viewport and
	// a panel side by side splits them exactly this way.
	PlayerInputComponent->BindKey(EKeys::RightMouseButton, IE_Pressed, this,
		&ATUCoasterRide::BeginLookDrag);
	PlayerInputComponent->BindKey(EKeys::RightMouseButton, IE_Released, this,
		&ATUCoasterRide::EndLookDrag);

	// ORBIT. [F] frames the whole track, which is the key you press constantly
	// once a validation warning points somewhere and you have no idea where. The
	// wheel zooms multiplicatively, so one notch means the same proportion of the
	// distance at 10 m and at 1000 m.
	PlayerInputComponent->BindKey(EKeys::F, IE_Pressed, this,
		&ATUCoasterRide::FrameWholeTrack);

	// THE RIDE PROFILE, as a graph you can read a number off. [G] cycles the
	// channel and [H] hides it — one at a time rather than four overlaid, because
	// four traces on one axis is a picture rather than a reading.
	PlayerInputComponent->BindKey(EKeys::G, IE_Pressed, this,
		&ATUCoasterRide::CycleProfileChannel);
	PlayerInputComponent->BindKey(EKeys::H, IE_Pressed, this,
		&ATUCoasterRide::ToggleProfileGraph);
	PlayerInputComponent->BindKey(EKeys::V, IE_Pressed, this,
		&ATUCoasterRide::ToggleDiagnostics);
	// [Z] frames the selected segment; clicking a diagnostics row selects one and
	// frames it in the same gesture, which is the whole point of a finding
	// carrying a place.
	PlayerInputComponent->BindKey(EKeys::Z, IE_Pressed, this,
		&ATUCoasterRide::FrameSelectedSegment);
	// ONE BINDING, dispatching by mode. Two handlers on one key both fire and
	// which one "wins" depends on registration order — the exact ambiguity
	// FInputMap's conflict test exists to warn about, and shipping it in the same
	// repository as that test would be poor form.
	PlayerInputComponent->BindKey(EKeys::LeftMouseButton, IE_Pressed, this,
		&ATUCoasterRide::ClickPrimary);
	// The RELEASE, which is where a drag can be told from a click.
	PlayerInputComponent->BindKey(EKeys::LeftMouseButton, IE_Released, this,
		&ATUCoasterRide::ReleasePrimary);
	// BUILD / OPERATE / RIDE. The mode decides the camera, the panels and whether
	// edits are accepted at all — which is what makes it a mode rather than a
	// label on a screen.
	PlayerInputComponent->BindKey(EKeys::Tab, IE_Pressed, this,
		&ATUCoasterRide::CycleAppMode);

	// ===================== NO FUNCTION KEYS. THE EDITOR OWNS THEM. =====================
	//
	// [U] — every overlay off, for a screenshot. It was F2, and F2 is the
	// editor's Unlit view mode; the settings key was F1, which is Wireframe.
	// Both fired BOTH bindings in PIE, so the overlay toggled and the world
	// changed rendering mode at the same time — which reads as the UI being
	// broken rather than as a key collision.
	//
	// The rule is not "those two were unlucky": UE binds F1-F8 to viewport view
	// modes, F8 to eject, F11 to immersive. In PIE the editor's bindings are live
	// alongside ours, so a function key is never ours to take. Every binding in
	// this file is now a letter, a bracket or a named key.
	//
	// [U] for UI, and it is not routed through IsTypingInField for the same
	// reason the others are not: a letter bound here is not part of a number the
	// segment editor accepts.
	PlayerInputComponent->BindKey(EKeys::U, IE_Pressed, this,
		&ATUCoasterRide::ToggleOverlays);

	// [O] — options, in the frame's content slot. A key rather than a button so
	// the screen is reachable before the main menu that will own it exists.
	//
	// NOT F1. The editor binds F1 to a viewport view mode, so pressing it in PIE
	// opened the settings AND flipped the viewport to wireframe — which read as
	// the settings screen being broken when it was drawing correctly over a
	// wireframe world. Same class of collision as [Backspace] being the E-stop
	// and the editor's own function keys are just as owned as ours.
	PlayerInputComponent->BindKey(EKeys::O, IE_Pressed, this,
		&ATUCoasterRide::ToggleSettings);

	// [T] — which train you are on. Not guarded against typing: a letter is not
	// part of a number the segment editor accepts, and the same is true of every
	// other letter bound here.
	PlayerInputComponent->BindKey(EKeys::T, IE_Pressed, this,
		&ATUCoasterRide::NextRiderTrain);
	// [N] -- the next seat: car by car from the nose, left then right.
	PlayerInputComponent->BindKey(EKeys::N, IE_Pressed, this,
		&ATUCoasterRide::NextRiderSeat);

	// [M] — back to the MENU, and [K] saves.
	//
	// NOT Ctrl+S, and that is the F1 lesson rather than a preference. The editor
	// binds Ctrl+S to Save Current Level and its bindings are live in PIE, so the
	// familiar chord would save the level AND the track from one press — two
	// different documents written by a keystroke aimed at one of them.
	//
	// [K] is a poor mnemonic and it is the honest one: every letter that spells
	// save is taken, [S] most of all — it is the movement key. A rebindable
	// control is the real answer and the Controls page already lists these.
	PlayerInputComponent->BindKey(EKeys::M, IE_Pressed, this,
		&ATUCoasterRide::OpenMainMenu);
	PlayerInputComponent->BindKey(EKeys::K, IE_Pressed, this,
		&ATUCoasterRide::SaveDocumentFromKey);

	// [J] UNDO and [L] REDO, either side of [K] save. Not Ctrl+Z, for the reason
	// [K] is not Ctrl+S: the editor owns that chord and its bindings are live in
	// PIE, so undo would run the EDITOR's undo stack as well as ours -- two
	// histories stepping back together, one of them through the level.
	//
	// J-K-L is a poor mnemonic and an honest one: it is three free adjacent keys,
	// back and forward either side of save, and every letter that spells any of
	// the three words is already taken.
	PlayerInputComponent->BindKey(EKeys::J, IE_Pressed, this,
		&ATUCoasterRide::UndoEdit);
	PlayerInputComponent->BindKey(EKeys::L, IE_Pressed, this,
		&ATUCoasterRide::RedoEdit);

	// [I] INSERT and [R] REMOVE. Unlike every other letter here these two guard on
	// IsTypingInField -- not because a letter could be part of a number, but
	// because they change the LIST somebody is typing into, and inserting a row
	// under a focused field is where "a letter is safe" stops being true.
	PlayerInputComponent->BindKey(EKeys::I, IE_Pressed, this,
		&ATUCoasterRide::InsertSegment);
	PlayerInputComponent->BindKey(EKeys::R, IE_Pressed, this,
		&ATUCoasterRide::RemoveSegment);

	// THE SEGMENT EDITOR. Numeric entry only — constraint 1 holds absolutely, and
	// a digit key IS typed entry rather than a stepper wearing its name.
	PlayerInputComponent->BindKey(EKeys::B, IE_Pressed, this,
		&ATUCoasterRide::ToggleSegmentEditor);
	// [ and ] walk the selection, and frame it in orbit — the answer to "which
	// index is that piece", which the list could not previously give.
	PlayerInputComponent->BindKey(EKeys::LeftBracket, IE_Pressed, this,
		&ATUCoasterRide::SelectPrevSegment);
	PlayerInputComponent->BindKey(EKeys::RightBracket, IE_Pressed, this,
		&ATUCoasterRide::SelectNextSegment);
	PlayerInputComponent->BindKey(EKeys::Enter, IE_Pressed, this,
		&ATUCoasterRide::CommitField);
	PlayerInputComponent->BindKey(EKeys::Escape, IE_Pressed, this,
		&ATUCoasterRide::CancelField);
	PlayerInputComponent->BindKey(EKeys::Decimal, IE_Pressed, this,
		&ATUCoasterRide::TypePoint);
	// The digits, main row and numpad. Ten methods rather than ten lambdas,
	// because BindKey wants a no-argument method either way.
	const FKey Row[10] = {EKeys::Zero, EKeys::One, EKeys::Two, EKeys::Three, EKeys::Four,
		EKeys::Five, EKeys::Six, EKeys::Seven, EKeys::Eight, EKeys::Nine};
	const FKey Pad[10] = {EKeys::NumPadZero, EKeys::NumPadOne, EKeys::NumPadTwo,
		EKeys::NumPadThree, EKeys::NumPadFour, EKeys::NumPadFive, EKeys::NumPadSix,
		EKeys::NumPadSeven, EKeys::NumPadEight, EKeys::NumPadNine};
	void (ATUCoasterRide::*Digit[10])() = {
		&ATUCoasterRide::Type0, &ATUCoasterRide::Type1, &ATUCoasterRide::Type2,
		&ATUCoasterRide::Type3, &ATUCoasterRide::Type4, &ATUCoasterRide::Type5,
		&ATUCoasterRide::Type6, &ATUCoasterRide::Type7, &ATUCoasterRide::Type8,
		&ATUCoasterRide::Type9};
	for (int32 i = 0; i < 10; ++i)
	{
		PlayerInputComponent->BindKey(Row[i], IE_Pressed, this, Digit[i]);
		PlayerInputComponent->BindKey(Pad[i], IE_Pressed, this, Digit[i]);
	}
	PlayerInputComponent->BindKey(EKeys::Hyphen, IE_Pressed, this,
		&ATUCoasterRide::TypeMinus);
	PlayerInputComponent->BindKey(EKeys::Subtract, IE_Pressed, this,
		&ATUCoasterRide::TypeMinus);

	// THE SIM CLOCK. Watching a buffer count down at quarter speed is how
	// somebody learns what the interlocking is doing, and stepping one scan at a
	// time is how they see a permissive drop the frame a restraint opens.
	PlayerInputComponent->BindKey(EKeys::Pause, IE_Pressed, this,
		&ATUCoasterRide::TogglePause);
	// And the full stop: one scan when the ride is what you are looking at, a
	// decimal point when a field is.
	PlayerInputComponent->BindKey(EKeys::Period, IE_Pressed, this,
		&ATUCoasterRide::KeyPeriod);
	PlayerInputComponent->BindKey(EKeys::Comma, IE_Pressed, this,
		&ATUCoasterRide::SlowDown);
	PlayerInputComponent->BindKey(EKeys::Slash, IE_Pressed, this,
		&ATUCoasterRide::SpeedUp);
	PlayerInputComponent->BindKey(EKeys::MouseScrollUp, IE_Pressed, this,
		&ATUCoasterRide::OrbitZoomIn);
	PlayerInputComponent->BindKey(EKeys::MouseScrollDown, IE_Pressed, this,
		&ATUCoasterRide::OrbitZoomOut);

	// ===================== THE ALPHABET, ONCE =====================
	//
	// Every other key above is bound one at a time, which is right for commands
	// and absurd for twenty-six letters — and the absence of any way to type one
	// is why save-as sat unbuilt while the rest of the shell shipped.
	//
	// AnyKey with the FKey-carrying handler is the engine's own answer. It fires
	// for EVERYTHING including the mouse, so the handler is the narrow part: it
	// appends only while a name is being typed, and only characters legal in a
	// file name. Bound last, so nothing above it changed shape to allow it.
	PlayerInputComponent->BindKey(EKeys::AnyKey, IE_Pressed, this,
		&ATUCoasterRide::OnAnyKeyTyped);

	PlayerInputComponent->BindKey(EKeys::LeftShift, IE_Pressed, this,
		&ATUCoasterRide::BoostOn);
	PlayerInputComponent->BindKey(EKeys::LeftShift, IE_Released, this,
		&ATUCoasterRide::BoostOff);

	// LAST, AND AGAINST THE LIST THAT WAS JUST BUILT. The settings page's Controls
	// tab is a second list beside everything above it, and a second list is a
	// second thing to keep true — it drifted once already, promising [F2] after F2
	// was given back to the editor.
	CheckBindingsAgainstInput(PlayerInputComponent);

	// ===================== WHICH BINDINGS ARE WHICH ACTION =====================
	//
	// Recorded ONCE, here, while every action still holds its default key and
	// the defaults are unique: an action is the set of KeyBindings entries on
	// its default key. That is what makes a later rebind unambiguous -- two
	// actions somebody has put on one key are still two different index sets,
	// so moving one does not move the other. Rebinding by key would.
	ActionBindingIndex.Reset();
	for (const FSettingEntry& E : SettingsSchema())
	{
		if (E.Kind != ESettingKind::Key) { continue; }
		const FName Named(UTF8_TO_TCHAR(E.Default.c_str()));
		TArray<int32>& Idx = ActionBindingIndex.FindOrAdd(UTF8_TO_TCHAR(E.Key.c_str()));
		for (int32 i = 0; i < PlayerInputComponent->KeyBindings.Num(); ++i)
		{
			const FInputKeyBinding& B = PlayerInputComponent->KeyBindings[i];
			if (B.Chord.Key.GetFName() == Named && !B.Chord.bShift && !B.Chord.bCtrl && !B.Chord.bAlt)
			{
				Idx.Add(i);
			}
		}
	}

	// The person's own keys, over the defaults, and applied to the component.
	LoadKeyBindings();
}

// ===================== REBINDING =====================
//
// The settings file must never carry a `key.` line (one home per fact), so the
// bindings have their own: Saved/TrackUnlimited.keys, one `action=Key` a line,
// only the ones somebody changed. A default is not a value here either.
FString ATUCoasterRide::KeyBindingsPath() const
{
	return FPaths::ConvertRelativePathToFull(
		FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("TrackUnlimited.keys")));
}

void ATUCoasterRide::LoadKeyBindings()
{
	FString Text;
	if (FFileHelper::LoadFileToString(Text, *KeyBindingsPath()))
	{
		TArray<FString> Lines;
		Text.ParseIntoArrayLines(Lines);
		for (const FString& L : Lines)
		{
			FString Action, Key;
			if (L.Split(TEXT("="), &Action, &Key))
			{
				RebindKey(Action.TrimStartAndEnd(), Key.TrimStartAndEnd(), false);
			}
		}
	}
}

void ATUCoasterRide::WriteKeyBindings() const
{
	FString Out;
	for (const FSettingEntry& E : SettingsSchema())
	{
		if (E.Kind != ESettingKind::Key) { continue; }
		const std::string Bound = Bindings.KeyFor(E.Key);
		if (!Bound.empty() && Bound != E.Default)
		{
			Out += FString::Printf(TEXT("%s=%s") LINE_TERMINATOR, UTF8_TO_TCHAR(E.Key.c_str()),
				UTF8_TO_TCHAR(Bound.c_str()));
		}
	}
	if (!FFileHelper::SaveStringToFile(Out, *KeyBindingsPath(),
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		UE_LOG(LogTUEvents, Warning, TEXT("controls: could not write %s"), *KeyBindingsPath());
	}
}

bool ATUCoasterRide::RebindKey(const FString& Action, const FString& KeyName, bool bPersist)
{
	// F1-F12 ARE THE EDITOR'S and their bindings are live in PIE -- the lesson
	// that moved Settings off F1. Refused here, so the one key that would make
	// the world change rendering at the same moment the UI did cannot be chosen.
	if (KeyName.Len() >= 2 && KeyName[0] == TEXT('F') && FChar::IsDigit(KeyName[1]))
	{
		UE_LOG(LogTUEvents, Warning, TEXT("controls: [%s] belongs to the editor; not bound"), *KeyName);
		return false;
	}
	const TArray<int32>* Idx = ActionBindingIndex.Find(Action);
	if (!Idx || !InputComponent)
	{
		return false;
	}
	const FKey Key(*KeyName);
	if (!Key.IsValid())
	{
		return false;
	}
	for (int32 i : *Idx)
	{
		if (InputComponent->KeyBindings.IsValidIndex(i))
		{
			InputComponent->KeyBindings[i].Chord.Key = Key;
		}
	}
	// The record the settings page reads. A conflict is stored, never refused:
	// refusing leaves somebody stuck half-way through a remap (FInputMap's own
	// comment), and the page shows the clash instead.
	Bindings.Bind(TCHAR_TO_UTF8(*Action), TCHAR_TO_UTF8(*KeyName));
	if (bPersist)
	{
		WriteKeyBindings();
	}
	return true;
}

void ATUCoasterRide::CycleCameraMode()
{
	switch (CameraMode)
	{
	case ETUCameraMode::Rider:   CameraMode = ETUCameraMode::Chase; break;
	case ETUCameraMode::Chase:   CameraMode = ETUCameraMode::Free; break;
	case ETUCameraMode::Free:    CameraMode = ETUCameraMode::Orbit; break;
	// ORBIT THEN CONSOLE, so the god's-eye view and the place you actually work
	// are adjacent: standing at the console gives up sight of the block ahead,
	// and one press back is how you get it when you want it.
	case ETUCameraMode::Orbit:   CameraMode = ETUCameraMode::Console; break;
	default: CameraMode = ETUCameraMode::Rider; break;
	}
	// Re-seed on the way in, so the free camera starts from wherever you were
	// just looking rather than teleporting you somewhere unrecognisable.
	bFreeInitialised = false;
}

EEditKind ATUCoasterRide::KindOf(ETUSegmentKind K)
{
	switch (K)
	{
	case ETUSegmentKind::Arc:      return EEditKind::Arc;
	case ETUSegmentKind::Clothoid: return EEditKind::Clothoid;
	case ETUSegmentKind::Helix:    return EEditKind::Helix;
	case ETUSegmentKind::Straight: return EEditKind::Straight;
	// RAW IS SHOWN AS A STRAIGHT'S FIELD SET, and that is a real answer rather
	// than a fallthrough: raw curvature is authored as a sampled profile by the
	// importer and there is no vocabulary in the panel for it, so the honest
	// thing is the shortest field list plus a row that says "raw" out loud.
	case ETUSegmentKind::Raw:      return EEditKind::Straight;
	}
	return EEditKind::Straight;
}

double ATUCoasterRide::ReadField(const FTUTrackSegment& S, EEditField F) const
{
	switch (F)
	{
	case EEditField::Length:         return S.Length;
	case EEditField::Radius:         return S.Radius;
	case EEditField::CurvatureStart: return S.CurvatureStart;
	case EEditField::CurvatureEnd:   return S.CurvatureEnd;
	case EEditField::ClimbAngle:     return S.ClimbAngleDegrees;
	case EEditField::Turns:          return S.Turns;
	case EEditField::Kind:           return static_cast<double>(KindOf(S.Kind));
	case EEditField::RollStart:      return S.RollStartDegrees;
	case EEditField::Roll:           return S.RollEndDegrees;
	case EEditField::ZoneSpeed:      return S.ZoneSpeed;
	case EEditField::ZoneAccel:      return S.ZoneAccel;
	case EEditField::ZoneDecel:      return S.ZoneDecel;
	case EEditField::ZoneBrakeDecel: return S.ZoneBrakeDecel;
	// CHOICES ARE NOT READ AS NUMBERS. The device and the tick box are drawn
	// from the segment directly, because "2.0" is not what a device is.
	case EEditField::ZoneKind:
	case EEditField::StartsNewDevice:
	case EEditField::Count:          break;
	}
	return 0.0;
}

void ATUCoasterRide::WriteField(FTUTrackSegment& S, EEditField F, double V)
{
	switch (F)
	{
	case EEditField::Length:         S.Length = static_cast<float>(V); break;
	case EEditField::Radius:         S.Radius = static_cast<float>(V); break;
	case EEditField::CurvatureStart: S.CurvatureStart = static_cast<float>(V); break;
	case EEditField::CurvatureEnd:   S.CurvatureEnd = static_cast<float>(V); break;
	case EEditField::ClimbAngle:     S.ClimbAngleDegrees = static_cast<float>(V); break;
	case EEditField::Turns:          S.Turns = static_cast<float>(V); break;
	case EEditField::RollStart:      S.RollStartDegrees = static_cast<float>(V); break;
	case EEditField::Roll:           S.RollEndDegrees = static_cast<float>(V); break;
	case EEditField::ZoneSpeed:      S.ZoneSpeed = static_cast<float>(V); break;
	case EEditField::ZoneAccel:      S.ZoneAccel = static_cast<float>(V); break;
	case EEditField::ZoneDecel:      S.ZoneDecel = static_cast<float>(V); break;
	case EEditField::ZoneBrakeDecel: S.ZoneBrakeDecel = static_cast<float>(V); break;
	// THE THREE CHOICES ARE CYCLED, NEVER TYPED, so there is no number on its
	// way here for any of them -- see ClickSegmentEditor. Listed rather than
	// defaulted so the next field added cannot go silently unwritten.
	case EEditField::Kind:
	case EEditField::ZoneKind:
	case EEditField::StartsNewDevice:
	case EEditField::Count:          break;
	}
}

void ATUCoasterRide::SelectSegment(int32 Index, bool bExtend)
{
	if (!Segments.IsValidIndex(Index))
	{
		return;
	}
	// A TYPED FIELD BELONGS TO THE SEGMENT IT WAS OPENED ON. Changing the
	// selection under a half-typed number would commit it somewhere else or lose
	// it, and both are worse than making somebody press Enter first.
	CancelField();

	if (!bExtend)
	{
		Selection.Reset();
		Selection.Add(Index);
		SelectedSegment = Index;
		return;
	}

	// SHIFT-CLICK TOGGLES, it does not only add. Removing one from a selection of
	// eight is otherwise start-again, which is how people end up not using
	// multi-select at all.
	const int32 At = Selection.Find(Index);
	if (At != INDEX_NONE)
	{
		Selection.RemoveAt(At);
		// The primary follows: it is what [Z] frames and what the panel scrolls
		// to, so leaving it pointing at something no longer selected would draw a
		// row nobody has chosen.
		SelectedSegment = Selection.Num() > 0 ? Selection.Last() : -1;
		return;
	}
	Selection.Add(Index);
	SelectedSegment = Index;
}

FSegmentEditor ATUCoasterRide::BuildSelectionEditor() const
{
	// BUILT OVER THE SELECTION ONLY, and compacted so the model's indices are
	// 0..n-1. A CSV import is four thousand segments and this runs to draw a
	// panel; converting all of them every frame to ask about six would be the O(n)
	// mistake somewhere nobody would think to look for it.
	std::vector<FEditSegment> Compact;
	std::vector<std::size_t> Sel;
	for (int32 Index : Selection)
	{
		if (!Segments.IsValidIndex(Index)) { continue; }
		const FTUTrackSegment& S = Segments[Index];

		FEditSegment E;
		E.Kind = KindOf(S.Kind);
		for (std::size_t f = 0; f < static_cast<std::size_t>(EEditField::Count); ++f)
		{
			const EEditField F = static_cast<EEditField>(f);
			E.Set(F, ReadField(S, F));
		}
		Sel.push_back(Compact.size());
		Compact.push_back(E);
	}

	FSegmentEditor Ed;
	Ed.SetSegments(Compact);
	Ed.Select(Sel);
	return Ed;
}

void ATUCoasterRide::ClickSegmentEditor()
{
	// THE WIDGET TAKES ITS OWN CLICKS; a press that reached here missed every
	// row, and a click outside every row cancels rather than committing, because
	// a half-typed number applied because somebody looked elsewhere is worse
	// than one lost.
	if (bShowSegmentEditor) { CancelField(); }
}

void ATUCoasterRide::EditorAction(int32 Action, bool bShift)
{
	if (!bShowSegmentEditor) { return; }
	// KEYS GO BACK TO THE GAME. A clicked button holds Slate focus, and the
	// digits somebody is about to type would land on the button, not the field.
	if (FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().SetAllUserFocusToGameViewport();
	}
	{
		if (Action <= -1000)
		{
			// SELECTING A DIFFERENT SEGMENT BREAKS THE EDIT RUN, or typing here,
			// clicking away and coming back would be ONE undo step covering both.
			//
			// SHIFT EXTENDS, and shift is already the camera boost. Two handlers on
			// one key both fire, which is the ambiguity FInputMap's conflict test
			// warns about — so this dispatches on CONTEXT, exactly as [Backspace]
			// and [.] do on a focused field. A shift that lands on a segment row is
			// a selection; one that does not is still a boost, and nobody is flying
			// the camera and clicking a row in the same gesture.
			SelectSegment(-1000 - Action, bShift);
			// FRAMED ONLY ON A PLAIN CLICK. Re-framing on every shift-click would
			// swing the camera across the layout while somebody is assembling a
			// selection, which is motion sickness rather than help.
			if (!bShift) { FrameSelectedSegment(); }
		}
		else if (Session.EditsAllowed())
		{
			const EEditField F = static_cast<EEditField>(Action);

			// A CHOICE CYCLES ON CLICK rather than taking focus, because there is
			// nothing to type into it. Committed immediately — unlike a number,
			// which waits for Enter because "3" on the way to "30" is a rebuild
			// nobody asked for. A pick has no half-typed state to protect.
			if (IsChoiceField(F))
			{
				CancelField();
				FTUTrackSegment& S = Segments[SelectedSegment];
				if (F == EEditField::Kind)
				{
					// ===================== FOUR KINDS, AND RAW IS NOT ONE =====================
					//
					// Straight, Arc, Clothoid, Helix -- the vocabulary the panel can
					// actually show fields for. RAW IS DELIBERATELY NOT A DESTINATION:
					// it is a sampled curvature profile the NL2 importer produces, it
					// has no authored parameters to offer, and cycling into it would
					// hand somebody a segment they cannot edit and cannot get out of
					// by any means the panel offers. A raw segment cycles OUT to
					// Straight, which is an explicit edit with undo behind it.
					//
					// NOTHING IS CLEARED. `hidden is not deleted` is the rule, and this
					// is the one edit that would break it if it were written as the
					// widget tidying up after itself: every field lives on the segment
					// whatever the kind, so a radius survives a trip through Helix.
					S.Kind = NextAuthorableKind(S.Kind);
				}
				else if (F == EEditField::StartsNewDevice)
				{
					S.bStartsNewDevice = !S.bStartsNewDevice;
				}
				else
				{
					// Wraps through every zone including None, so the way to
					// remove a device is the same gesture as adding one.
					const uint8 Count = static_cast<uint8>(ETUSegmentZone::StationLoad) + 1;
					S.Zone = static_cast<ETUSegmentZone>((static_cast<uint8>(S.Zone) + 1) % Count);

					// LEAVING A DEVICE CLEARS ITS FLAG. bStartsNewDevice on an
					// unpowered segment is a setting with nothing to act on, and
					// it would come back the moment a zone was set again — a
					// value somebody turned on for a device they have since
					// deleted, silently applying to its replacement.
					if (S.Zone == ETUSegmentZone::None) { S.bStartsNewDevice = false; }
				}
				RebuildFromSegments();
				return;
			}

			// Focusing a field starts EMPTY rather than pre-filled with the
			// current value. Pre-filling means the first keystroke has to be
			// select-all or the number becomes "3020" — and there is no
			// select-all here, so starting empty is the honest shape.
			FocusedField = F;
			FieldBuffer.Empty();
		}
	}
}

const TCHAR* ATUCoasterRide::KindNameOf(ETUSegmentKind K) { return SegmentKindName(K); }
const TCHAR* ATUCoasterRide::ZoneNameOf(ETUSegmentZone Z) { return ZoneKindName(Z); }

void ATUCoasterRide::KeyBackspace()
{
	// EDITING WINS WHILE EDITING, and the operator's stop wins the rest of the
	// time. Binding both to Backspace would fire both — and an operator typing a
	// radius would E-stop the ride, which is not a UI wart but the worst kind of
	// surprise this project can produce.
	if (IsTypingInField()) { TypeBackspace(); }
	else                   { PressEmergencyStop(); }
}

void ATUCoasterRide::KeyPeriod()
{
	if (IsTypingInField()) { TypePoint(); }
	else                   { StepOneScan(); }
}

void ATUCoasterRide::TypeDigit(int32 D)
{
	// WHILE NAMING, THE ANYKEY HANDLER HAS ALREADY TAKEN IT. Both bindings fire
	// for one press, so a digit appended in each place appears twice — which is
	// the whole reason these three guard rather than branch.
	if (bNamingSave || FocusedField == EEditField::Count) { return; }
	FieldBuffer.AppendChar(static_cast<TCHAR>('0' + FMath::Clamp(D, 0, 9)));
}

void ATUCoasterRide::TypePoint()
{
	// ONE decimal point. A second one makes a string no parser accepts, and
	// silently dropping it is what every numeric field has always done.
	// A FULL STOP IS NOT A NAME CHARACTER, deliberately: the extension is this
	// application's to add, and "Reference.track" typed in full would be saved as
	// Reference.track.track.
	if (bNamingSave) { return; }
	if (FocusedField == EEditField::Count || FieldBuffer.Contains(TEXT("."))) { return; }
	FieldBuffer.AppendChar('.');
}

void ATUCoasterRide::TypeMinus()
{
	// A minus is only a minus at the FRONT. Anywhere else it is a typo, and a
	// field that accepted "3-0" would produce a number nobody meant.
	if (bNamingSave) { return; }   // a hyphen anywhere is fine in a name; AnyKey has it
	if (FocusedField == EEditField::Count || FieldBuffer.Len() > 0) { return; }
	FieldBuffer.AppendChar('-');
}

void ATUCoasterRide::TypeBackspace()
{
	// THIS ONE BRANCHES RATHER THAN GUARDING, because AnyKey ignores Backspace —
	// deleting is what the key means, in either buffer.
	if (bNamingSave)
	{
		if (!NameBuffer.IsEmpty()) { NameBuffer.LeftChopInline(1); }
		NameError.Empty();
		return;
	}
	if (FocusedField == EEditField::Count || FieldBuffer.IsEmpty()) { return; }
	FieldBuffer.LeftChopInline(1);
}

void ATUCoasterRide::CommitField()
{
	// ENTER MEANS "DO THE THING IN FRONT OF ME", and while a name is being typed
	// that is the save rather than a segment field — which cannot be focused at
	// the same time, because the prompt is the only thing taking keys.
	if (bNamingSave) { CommitNameSave(); return; }
	if (FocusedField == EEditField::Count || !Session.EditsAllowed()
		|| SelectedSegment < 0 || SelectedSegment >= Segments.Num())
	{
		CancelField();
		return;
	}
	// AN EMPTY BUFFER IS A CANCEL, not a zero. Somebody who clicked a field and
	// pressed Enter meant "never mind", and writing 0 into a radius because of it
	// would be the most destructive possible reading.
	if (FieldBuffer.IsEmpty() || FieldBuffer == TEXT("-") || FieldBuffer == TEXT("."))
	{
		CancelField();
		return;
	}

	const double V = FCString::Atod(*FieldBuffer);

	// ===================== ONE NUMBER, EVERY SELECTED SEGMENT =====================
	//
	// This is the whole reason multi-select is worth having on a coaster: banking a
	// turn is one number typed into eight segments, and typing it eight times is
	// not tedium — it is eight chances to type a different one, and the result is a
	// bank that steps rather than ramps, which the roll-rate validator will then
	// quite correctly complain about.
	//
	// WRITTEN ONLY WHERE THE FIELD IS VISIBLE, from the tested intersection. A
	// selection holding an arc and a straight has no radius in common, and writing
	// one anyway would silently give the straight a radius it does not use — a
	// value that does nothing until somebody changes its kind, and then does
	// something nobody asked for.
	const FSegmentEditor Ed = BuildSelectionEditor();
	const std::vector<FFieldView> Fields = Ed.Fields();
	const std::size_t Fi = static_cast<std::size_t>(FocusedField);
	const bool bShared = Fi < Fields.size() && Fields[Fi].bVisible;

	int32 Written = 0;
	for (int32 Index : Selection)
	{
		if (!Segments.IsValidIndex(Index)) { continue; }
		if (Index != SelectedSegment && !bShared) { continue; }
		WriteField(Segments[Index], FocusedField, V);
		++Written;
	}
	if (Written == 0 && Segments.IsValidIndex(SelectedSegment))
	{
		// The primary always takes it, even if the selection somehow did not
		// include it — a typed number that lands nowhere is the worst outcome.
		WriteField(Segments[SelectedSegment], FocusedField, V);
		Written = 1;
	}

	// COMMITTED ON ENTER, NEVER PER KEYSTROKE. "3" on the way to "30" is a
	// segment 3 m long and a rebuild nobody asked for — and on a closed circuit
	// it is a rebuild that briefly reports the track as not closing.
	const EEditField Was = FocusedField;
	FocusedField = EEditField::Count;
	FieldBuffer.Empty();
	RebuildFromSegments();

	// ONE ENTER IS ONE UNDO STEP, and that falls out rather than needing a merge
	// key: this commits on Enter and never per keystroke, so "30.5" has already
	// been coalesced into one edit before history ever sees it. The key exists for
	// a control that fires continuously — a spinner drag in the Details panel —
	// and the runtime editor is not one.
	// The editor model's own name for the field, so the undo label and the panel
	// row cannot disagree about what a thing is called. The COUNT is in the label
	// because "roll on 8 segments" and "roll on segment 12" are different things
	// to be about to undo.
	PushHistory(Written > 1
		? FString::Printf(TEXT("%s on %d segments"), UTF8_TO_TCHAR(FieldName(Was)), Written)
		: FString::Printf(TEXT("%s on segment %d"), UTF8_TO_TCHAR(FieldName(Was)),
			SelectedSegment));
}

void ATUCoasterRide::InsertSegment()
{
	// ===================== AN EDITOR THAT CANNOT ADD A SEGMENT =====================
	//
	// The runtime editor could change numbers on segments that already existed and
	// nothing else, which makes it a tuning panel rather than an editor. The model
	// has had Insert, Remove and Duplicate since Phase 0.
	if (!Session.EditsAllowed() || IsTypingInField())
	{
		// NOT WHILE TYPING. A letter key is not part of a number, which is why the
		// others do not guard — but these two change the LIST somebody is typing
		// into, and inserting a row under a focused field is the one case where
		// "a letter is safe" stops being true.
		return;
	}

	// ONE KEY, AND IT COPIES RATHER THAN INSERTING A BLANK. Building track is
	// almost always "another one like that" — the new piece inherits the zone, the
	// roll and the kind, which is what somebody was about to type back in by hand.
	// A default straight is only what you get when there is nothing to copy, which
	// is the empty track and the first segment.
	FTUTrackSegment New;
	int32 At = Segments.Num();
	if (SelectedSegment >= 0 && SelectedSegment < Segments.Num())
	{
		New = Segments[SelectedSegment];
		At = SelectedSegment + 1;
	}
	Segments.Insert(New, At);

	// The new one becomes the selection, because the next thing anybody does is
	// edit it — and leaving the selection on the original means the first field
	// typed changes the wrong segment.
	SelectedSegment = At;
	RebuildFromSegments();
	PushHistory(FString::Printf(TEXT("insert segment %d"), At));
}

void ATUCoasterRide::RemoveSegment()
{
	if (!Session.EditsAllowed() || IsTypingInField()
		|| SelectedSegment < 0 || SelectedSegment >= Segments.Num())
	{
		return;
	}

	// DESTRUCTIVE AND UNCONFIRMED, which is only defensible because [J] undoes it.
	// A confirm on every delete is the dialog people learn to click through, and
	// the honest alternative to it is an undo that works — which landed first
	// deliberately, rather than this shipping with a prompt standing in for one.
	const int32 At = SelectedSegment;
	Segments.RemoveAt(At);
	// The segment that took its place, or the last one, or nothing on an empty
	// track. A selection past the end is an editor drawing a row that is not there.
	SelectedSegment = FMath::Clamp(At, -1, Segments.Num() - 1);
	RebuildFromSegments();
	PushHistory(FString::Printf(TEXT("remove segment %d"), At));
}

void ATUCoasterRide::CancelField()
{
	// ESCAPE BACKS OUT OF THE NAME PROMPT FIRST, which is the only thing on screen
	// when it is up. Nothing is saved and nothing is lost — the document is
	// exactly as dirty as it was.
	if (bNamingSave) { CancelNameSave(); return; }
	FocusedField = EEditField::Count;
	FieldBuffer.Empty();
}

FString ATUCoasterRide::SidecarPath() const
{
	// ONE FIXED NAME, not one per document. Recovery has to find it at boot with
	// nothing to go on — the whole point is that the application did not shut
	// down, so nobody wrote down which file was open — and a folder full of
	// sidecars named after documents is a search rather than a question.
	return FPaths::ConvertRelativePathToFull(
		FPaths::ProjectSavedDir() / TEXT("Recovered.track"));
}

void ATUCoasterRide::BootSession()
{
	// ===================== BOOT IS A REAL STATE =====================
	//
	// Not a loading screen: it is where a crashed session's sidecar is discovered,
	// and that HAS to happen before anything can be opened, because opening a
	// document is what would overwrite the evidence. `FSession::MayEnter` enforces
	// it — nothing leaves Boot while a recovery is pending.
	Session.Enter(EAppMode::Boot);

	FString Text;
	if (FFileHelper::LoadFileToString(Text, *SidecarPath()) && !Text.IsEmpty())
	{
		// OFFERED, NEVER APPLIED. A recovery that opened itself would silently
		// discard whatever somebody did deliberately after the crash — and the
		// case where they relaunch specifically to start again is not rare.
		Session.FoundSidecarAtBoot(TCHAR_TO_UTF8(*SidecarPath()), TCHAR_TO_UTF8(*Text));
		UE_LOG(LogTUEvents, Log, TEXT("boot: a recovered session is waiting"));
		return;
	}

	Session.Enter(EAppMode::MainMenu);
	RefreshTrackList();
}

void ATUCoasterRide::AnswerRecovery(bool bAccept)
{
	if (!Session.HasRecovery())
	{
		return;
	}
	if (bAccept)
	{
		const FString Text(UTF8_TO_TCHAR(Session.RecoveredText().c_str()));
		FTrackDocument Doc;
		std::string Error;
		if (ParseTrackJson(TCHAR_TO_UTF8(*Text), Doc, Error))
		{
			Segments.Reset(static_cast<int32>(Doc.Segments.size()));
			for (const FAuthoredSegment& A : Doc.Segments)
			{
				Segments.Add(FromAuthored(A));
			}
			// ACCEPTED IS DIRTY, because it is: what was recovered has never been
			// saved, and presenting it as clean would let somebody close it and
			// lose the same work a second time — the worst outcome available to a
			// feature whose entire job is not losing it. `AcceptRecovery` sets
			// that up; the rebuild below observes the text and the comparison does
			// the rest.
			Session.AcceptRecovery();
			RebuildFromSegments();
			ResetHistory();
			Session.Enter(EAppMode::MainMenu);
			Session.Enter(EAppMode::Build);
			ApplyAppMode(EAppMode::Build);
			FrameWholeTrack();
			UE_LOG(LogTUEvents, Log, TEXT("boot: recovered %d segments, unsaved"),
				Segments.Num());
			return;
		}
		// A SIDECAR THAT WILL NOT PARSE IS STILL EVIDENCE. Declining leaves it on
		// disc rather than deleting it, so somebody can look at the file itself.
		UE_LOG(LogTUEvents, Warning, TEXT("boot: the recovered session will not load — %s"),
			UTF8_TO_TCHAR(Error.c_str()));
	}

	Session.DeclineRecovery();
	// DELETED ONLY ON A DECLINE, and only after the person said so. Anything else
	// is the application deciding on their behalf what was worth keeping.
	if (!bAccept)
	{
		IPlatformFile& Files = FPlatformFileManager::Get().GetPlatformFile();
		Files.DeleteFile(*SidecarPath());
	}
	Session.Enter(EAppMode::MainMenu);
	RefreshTrackList();
}

void ATUCoasterRide::DrawRecoveryOffer(UCanvas* Canvas)
{
	if (!Canvas || !GEngine || !Session.HasRecovery())
	{
		return;
	}

	// THE FIRST THING ANYBODY SEES AFTER A CRASH, so it says what happened and
	// what the two answers do rather than asking a yes/no about a noun.
	const float Ox = 70.f;
	float Y = 110.f;
	PanelTile(Canvas, Ox - 18.f, Y - 16.f, 600.f, 150.f, PanelGround);
	PanelLabel(Canvas, Ox, Y, TEXT("A SESSION WAS NOT CLOSED"), PanelAmber);
	Y += 26.f;
	PanelLabel(Canvas, Ox, Y,
		TEXT("TrackUnlimited saved a copy of what you were working on."), PanelText);
	Y += 18.f;
	PanelLabel(Canvas, Ox, Y,
		FString::Printf(TEXT("%s"), *FPaths::GetCleanFilename(SidecarPath())), PanelDim);
	Y += 26.f;
	PanelLabel(Canvas, Ox, Y,
		TEXT("Recovering opens it UNSAVED, so save it somewhere you want it."), PanelDim);
	Y += 18.f;
	PanelLabel(Canvas, Ox, Y,
		TEXT("Discarding deletes the copy. Nothing else is touched."), PanelDim);
	Y += 26.f;

	MenuRowRects.Reset();
	MenuRowAction.Reset();
	MenuRowRects.Add(FVector4(Ox, Y, Ox + 150.f, Y + 20.f));
	MenuRowAction.Add(-6);
	PanelLabel(Canvas, Ox, Y, TEXT("[ Recover it ]"), PanelCyan);
	MenuRowRects.Add(FVector4(Ox + 180.f, Y, Ox + 320.f, Y + 20.f));
	MenuRowAction.Add(-7);
	PanelLabel(Canvas, Ox + 180.f, Y, TEXT("[ Discard it ]"), PanelAmber);
}

void ATUCoasterRide::DrawDragAnswer(UCanvas* Canvas)
{
	if (!Canvas || !GEngine || DragAnswerSeconds <= 0.0)
	{
		return;
	}

	// GREEDY WRAP AT A COLUMN, because the answer is written as paragraphs to be
	// READ and the canvas draws one line at a time. Not a general text layout
	// engine — this is the only prose in the application, and the day there is a
	// second one is the day it earns a helper.
	const int32 Columns = 78;
	TArray<FString> Lines;
	FString Source(UTF8_TO_TCHAR(WhyCannotIDragTheTrack()));
	Source.ReplaceInline(TEXT("\r"), TEXT(""));
	TArray<FString> Paragraphs;
	Source.ParseIntoArray(Paragraphs, TEXT("\n"), false);
	for (const FString& P : Paragraphs)
	{
		if (P.IsEmpty()) { Lines.Add(FString()); continue; }
		FString Line;
		TArray<FString> Words;
		P.ParseIntoArray(Words, TEXT(" "), true);
		for (const FString& Word : Words)
		{
			if (!Line.IsEmpty() && Line.Len() + 1 + Word.Len() > Columns)
			{
				Lines.Add(Line);
				Line.Empty();
			}
			if (!Line.IsEmpty()) { Line += TEXT(" "); }
			Line += Word;
		}
		if (!Line.IsEmpty()) { Lines.Add(Line); }
	}

	const float Ox = 70.f;
	const float RowH = 15.f;
	float Y = 120.f;
	PanelTile(Canvas, Ox - 18.f, Y - 16.f, 620.f, RowH * Lines.Num() + 62.f, PanelGround);
	PanelLabel(Canvas, Ox, Y, TEXT("WHY CAN'T I DRAG THE TRACK?"), PanelCyan);
	Y += 22.f;
	for (const FString& L : Lines)
	{
		PanelLabel(Canvas, Ox, Y, *L, PanelText);
		Y += RowH;
	}
	Y += 8.f;
	// WHAT TO DO INSTEAD, not just why not. An explanation that ends without a
	// next step is a refusal with paragraphs.
	PanelLabel(Canvas, Ox, Y,
		TEXT("[B] opens the segment list.  Click a number and type."), PanelAmber);
}

void ATUCoasterRide::DrawLeaveConfirm(UCanvas* Canvas)
{
	if (!Canvas || !GEngine || !bConfirmingMenu)
	{
		return;
	}

	// THE QUESTION THE SESSION ASKED, PUT ON SCREEN. It is deliberately the only
	// one in the shell: Build to Operate to Ride discards nothing and never asks,
	// which is what keeps this one worth reading rather than clicked through.
	const float W = 460.f;
	const float Ox = 80.f;
	float Y = 90.f;

	PanelTile(Canvas, Ox - 16.f, Y - 14.f, W + 32.f, 118.f, PanelGround);
	PanelLabel(Canvas, Ox, Y, TEXT("UNSAVED CHANGES"), PanelAmber);
	Y += 24.f;
	// WHAT WOULD BE LOST, named. "Are you sure?" is a question nobody can answer;
	// the document's name and the fact that going back discards it can be.
	PanelLabel(Canvas, Ox, Y,
		Session.HasPath()
			? *FString::Printf(TEXT("%s has changes that are not saved."),
				*FPaths::GetCleanFilename(UTF8_TO_TCHAR(Session.Path().c_str())))
			: TEXT("This track has never been saved."),
		PanelText);
	Y += 20.f;
	PanelLabel(Canvas, Ox, Y, TEXT("Going back to the menu discards them."), PanelDim);
	Y += 26.f;

	// SAVE IS THE FIRST OPTION AND DISCARD IS NOT DEFAULTED. The destructive answer
	// should never be the one nearest the pointer or the one a reflex picks.
	MenuRowRects.Reset();
	MenuRowAction.Reset();
	MenuRowRects.Add(FVector4(Ox, Y, Ox + 150.f, Y + 20.f));
	MenuRowAction.Add(-3);
	PanelLabel(Canvas, Ox, Y, TEXT("[ Save and leave ]"), PanelCyan);
	MenuRowRects.Add(FVector4(Ox + 170.f, Y, Ox + 300.f, Y + 20.f));
	MenuRowAction.Add(-4);
	PanelLabel(Canvas, Ox + 170.f, Y, TEXT("[ Discard ]"), PanelAmber);
	MenuRowRects.Add(FVector4(Ox + 320.f, Y, Ox + 440.f, Y + 20.f));
	MenuRowAction.Add(-5);
	PanelLabel(Canvas, Ox + 320.f, Y, TEXT("[ Cancel ]"), PanelText);
}

bool ATUCoasterRide::PresetForTemplate(ETemplatePreset T, ETUPresetLayout& Out)
{
	switch (T)
	{
	case ETemplatePreset::FlatRig:         Out = ETUPresetLayout::FlatRig; return true;
	case ETemplatePreset::OutAndBack:      Out = ETUPresetLayout::OutAndBack; return true;
	case ETemplatePreset::TwoTrainCircuit: Out = ETUPresetLayout::TwoTrainCircuit; return true;
	case ETemplatePreset::SmallBatch:      Out = ETUPresetLayout::SmallBatch; return true;
	case ETemplatePreset::Showcase:        Out = ETUPresetLayout::Showcase; return true;
	case ETemplatePreset::Blank:           return false;
	default:                               Out = ETUPresetLayout::Reference; return true;
	}
}

void ATUCoasterRide::StartFromTemplate(int32 Index)
{
	if (Index < 0 || static_cast<std::size_t>(Index) >= NumTemplates()) { return; }
	const FTemplate T = TemplateAt(static_cast<std::size_t>(Index));

	// A TEMPLATE NAMES A PRESET rather than carrying its own geometry. Five
	// measured worked examples already ship, and a parallel set of starter
	// layouts would be a second set of tracks to keep working — drifting from the
	// ones every number in the docs is quoted from.
	if (!PresetForTemplate(T.Preset, Preset))
	{
		Segments.Reset();
	}
	else
	{
		Segments = PresetLayout(Preset);
		ApplyPresetTrainSetup(Preset);
		ApplyPresetWalkways();
	}

	Session.Enter(EAppMode::Build);
	ApplyAppMode(EAppMode::Build);   // the editor on, the orbit, settings closed
	RebuildFromSegments();

	// ===================== A NEW DOCUMENT IS NOT A MODIFIED ONE =====================
	//
	// REBUILT FIRST, AND THE BASELINE TAKEN AFTER. `DidCreateNew` used to run
	// before the rebuild with an EMPTY string, and the rebuild then `Observe`d the
	// real serialised text -- so saved was "" and current was a document, and every
	// template landed in Build already reporting unsaved changes.
	//
	// Nobody had touched it. The frame showed the asterisk, and going back to the
	// menu asked whether to discard work that did not exist -- which is precisely
	// how people learn to click through the one prompt that matters.
	//
	// Same order as an open, and for the same reason: dirty is a COMPARISON, so
	// the baseline has to be what the document actually is.
	Session.DidCreateNew(TCHAR_TO_UTF8(*SerialiseDocument()));
	ResetHistory();
	FrameWholeTrack();

	// WHAT TO TRY FIRST, next to the thing it describes. Not a tutorial sequence
	// — that is a thing to dismiss, and this project's argument is that the tool
	// tells you the truth plainly.
	UE_LOG(LogTUEvents, Log, TEXT("%s — %s"),
		UTF8_TO_TCHAR(T.Name), UTF8_TO_TCHAR(T.WhatToTryFirst));
}

void ATUCoasterRide::ClickPrimary()
{
	// WHERE THE PRESS STARTED, so a RELEASE can tell a click from a drag. The
	// difference is the whole trigger for the drag answer below: somebody asking
	// "why can't I drag the track" asks it by trying, and that is the moment to
	// answer rather than a paragraph in a menu they will never open.
	if (APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
	{
		float Mx = 0.f, My = 0.f;
		PC->GetMousePosition(Mx, My);
		LeftPressPos = FVector2D(Mx, My);
	}
	bLeftPressHitPanel = true;   // assume a hit; the fall-through below says otherwise

	// THE CONFIRM OWNS THE CLICK ABOVE EVERYTHING, because a question about losing
	// work must not be dismissable by clicking past it into the thing underneath.
	// The recovery offer is above even that: it is asked before anything is open,
	// and answering it is the only way out of Boot.
	if (Session.HasRecovery()) { ClickLeaveConfirm(); return; }
	if (bConfirmingMenu) { ClickLeaveConfirm(); return; }
	// The menu is a widget and takes its own clicks; nothing under it is live.
	if (Session.Mode() == EAppMode::MainMenu) { return; }

	if (APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
	{
		float Bx = 0.f, ByPx = 0.f;
		if (!bHideOverlays && PC->GetMousePosition(Bx, ByPx)
			&& Bx >= MenuButtonRect.X && Bx <= MenuButtonRect.Z
			&& ByPx >= MenuButtonRect.Y && ByPx <= MenuButtonRect.W)
		{
			OpenMainMenu();
			return;
		}
	}

	// THE CONSOLE BEFORE THE EDITOR, because in Operate it is the only thing on
	// screen anybody is working and the editor is refusing edits anyway. The graph
	// after it and before the editor, because it sits along the bottom where the
	// editor's rows do not reach.
	if (APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
	{
		float Cx = 0.f, Cy = 0.f;
		if (PanelMouse(Cx, Cy) && (PressConsole(Cx, Cy) || PressGraph(Cx, Cy)))
		{
			return;
		}
	}
	// The editor gets first refusal because it is the panel somebody is working
	// in; a click that misses every one of its rows falls through to the
	// diagnostics list, which is the other clickable thing on screen.
	const int32 Before = SelectedSegment;
	const int32 DiagBefore = SelectedSegment;
	ClickSegmentEditor();
	if (SelectedSegment == Before && FocusedField == EEditField::Count)
	{
		ClickDiagnostics();
		// NOTHING ON SCREEN WANTED IT, so this press was in the viewport. Recorded
		// rather than acted on: a click there is legitimately nothing, and only a
		// DRAG is the question worth answering.
		bLeftPressHitPanel = SelectedSegment != DiagBefore;
	}
}

bool ATUCoasterRide::PressConsole(float Mx, float My)
{
	for (int32 i = 0; i < ConsoleRects.Num() && i < ConsoleAction.Num(); ++i)
	{
		const FVector4& R = ConsoleRects[i];
		if (Mx < R.X || Mx > R.Z || My < R.Y || My > R.W) { continue; }

		HeldConsoleButton = ConsoleAction[i];
		switch (HeldConsoleButton)
		{
		case 0:
			// HELD, not fired. The permissive reads the button every scan and the
			// anti-tie-down rule needs it to go low between trains — so this is
			// exactly what the key does, and letting go is what completes it.
			PressDispatch();
			break;
		case 1:
			PressEmergencyStop();
			break;
		case 2:
			// MONITORED RESET: the press only arms it. The reset happens on the
			// release, which is what stops a taped button from holding a ride
			// permanently resettable.
			PressResetButton();
			break;
		case 10:
		case 11:
		{
			// Clamped against the same fit rule the button draws with, because a
			// control that is merely drawn disabled is one keyboard away from being
			// pressed anyway.
			const int32 MaxCars = (CarLengthM > 0.f && ShortestHoldM > 0.0)
				? FMath::Max(1, FMath::FloorToInt(
					(ShortestHoldM - HoldNoseClearanceM) / CarLengthM))
				: 12;
			const int32 WasCars = CarCount;
			CarCount = FMath::Clamp(CarCount + (HeldConsoleButton == 11 ? 1 : -1),
				1, MaxCars);
			HeldConsoleButton = -1;
			if (CarCount != WasCars)
			{
				RebuildFromSegments();
				UE_LOG(LogTUEvents, Log, TEXT("maintenance: %d car(s), %.1f m train"),
					CarCount, TrainLengthM);
			}
			break;
		}
		case 8:
		case 9:
		{
			// A REBUILD, because trains are PLACED at their holding devices rather
			// than spawned wherever there is room -- which is also why shedding one
			// re-seats the others rather than leaving a gap. Taking a train off is
			// physically that: the ride stops, it comes off, the rest are re-spaced.
			//
			// NOT AN EDIT, so no history step. The document is the TRACK, and how
			// many trains are on it this morning is not a change to the geometry --
			// undoing a shed train would be undoing an operational decision.
			const int32 Cap = HoldingPlaces == 0 ? 0 : FMath::Max(1, HoldingPlaces - 1);
			const int32 Was = FMath::Clamp(TrainCount, 0, Cap);
			TrainCount = FMath::Clamp(Was + (HeldConsoleButton == 9 ? 1 : -1), 0, Cap);
			HeldConsoleButton = -1;
			if (TrainCount != Was)
			{
				RebuildFromSegments();
				UE_LOG(LogTUEvents, Log, TEXT("maintenance: %d train(s) in service"),
					TrainCount);
			}
			break;
		}
		case 3:
			// A MODE, TAKEN ON THE PRESS. Nothing about a selector needs the
			// release, and holding it should not do anything at all.
			bManualDispatch = !bManualDispatch;
			HeldConsoleButton = -1;
			UE_LOG(LogTUEvents, Log, TEXT("console: dispatch is now %s"),
				bManualDispatch ? TEXT("MANUAL") : TEXT("AUTO"));
			break;
		case 4:
		case 5:
		{
			// COMMANDED, NOT SET. The switch says what the operator wants; the
			// bank takes its travel time and the sensors say when it got there,
			// which is the whole reason "commanded closed but car 3 is not locked"
			// is expressible at all.
			//
			// EVERY PLATFORM, because there is one operator and one console here.
			// A multi-position platform with a console each is the first thing this
			// will have to split, and it is the same limitation the dispatch
			// button already carries.
			if (FTUPlatform* P = ConsolePlatformPtr())
			{
				FCommandedBank& Bank = HeldConsoleButton == 4 ? P->Crew.Gates
															  : P->Crew.Restraints;
				Bank.Command(!Bank.IsCommandedClosed());
			}
			HeldConsoleButton = -1;
			break;
		}
		case 6:
			// THE WALK-ROUND, and it only ever goes one way. Un-declaring an
			// all-clear is not a thing an operator does — if something is wrong
			// after they have given it, the control for that is the E-stop.
			if (FTUPlatform* P = ConsolePlatformPtr())
			{
				P->bOperatorAllClear = true;
				UE_LOG(LogTUEvents, Log, TEXT("console: all clear given at Z%d"), P->Zone);
			}
			HeldConsoleButton = -1;
			break;
		case 7:
			// THE POSITION SELECTOR. -1 follows the train, then each platform in
			// turn, then back — so the useful default is one press away from
			// wherever somebody has got to.
			ConsolePlatform = ConsolePlatform + 1 >= Platforms.Num() ? -1 : ConsolePlatform + 1;
			HeldConsoleButton = -1;
			break;
		default:
			HeldConsoleButton = -1;
			break;
		}
		return true;
	}
	return false;
}

void ATUCoasterRide::ReleaseConsole()
{
	switch (HeldConsoleButton)
	{
	case 0: ReleaseDispatch(); break;
	case 2: ReleaseResetButton(); break;
	default: break;
	}
	HeldConsoleButton = -1;
}

void ATUCoasterRide::ReleasePrimary()
{
	// THE CONSOLE FIRST, ALWAYS, and unconditionally — a dispatch button left
	// held because the pointer wandered off the control before the mouse came up
	// is a wedged button, which is the exact failure anti-tie-down exists for.
	if (HeldConsoleButton >= 0)
	{
		ReleaseConsole();
		return;
	}
	if (bScrubbing)
	{
		// ENDED WHEREVER THE POINTER IS. A scrub left running because the mouse
		// came up outside the graph is a camera that keeps following it.
		bScrubbing = false;
		return;
	}

	// ===================== THE QUESTION IS ASKED BY THE GESTURE =====================
	//
	// `WhyCannotIDragTheTrack()` has been written, reviewed and shown to nobody
	// since the day it landed. Putting it in a help menu would file it where only
	// somebody who already accepted the answer would look; showing it when
	// somebody TRIES TO DRAG THE TRACK puts it exactly where the question is.
	//
	// A drag, not a click. A click in the viewport is legitimately nothing — it is
	// how you deselect — and a wall of text every time somebody clicked empty
	// space would be the most annoying feature in the application.
	if (bLeftPressHitPanel || Session.Mode() != EAppMode::Build || !GetWorld())
	{
		return;
	}
	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (!PC)
	{
		return;
	}
	float Mx = 0.f, My = 0.f;
	if (!PC->GetMousePosition(Mx, My))
	{
		return;
	}
	// Twelve pixels: past a hand tremor on a click, well short of a deliberate
	// drag. The same threshold every drag-versus-click test in every toolkit uses,
	// and it does not need to be tunable to be right.
	if (FVector2D::Distance(FVector2D(Mx, My), LeftPressPos) < 12.0)
	{
		return;
	}
	// TIMED OUT RATHER THAN DISMISSED. It is an answer, not a dialog: nothing is
	// blocked, nothing needs acknowledging, and somebody who has read it should not
	// have to do anything to carry on.
	DragAnswerSeconds = 14.0;
	UE_LOG(LogTUEvents, Log, TEXT("viewport: answered the drag question"));
}

void ATUCoasterRide::ClickLeaveConfirm()
{
	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC) { return; }

	float Mx = 0.f, My = 0.f;
	if (!PC->GetMousePosition(Mx, My)) { return; }

	for (int32 i = 0; i < MenuRowRects.Num() && i < MenuRowAction.Num(); ++i)
	{
		const FVector4& R = MenuRowRects[i];
		if (Mx < R.X || Mx > R.Z || My < R.Y || My > R.W) { continue; }

		switch (MenuRowAction[i])
		{
		case -3:
			// SAVED FIRST, AND ONLY THEN LEFT. A save that failed — a full disc, a
			// read-only folder — must not be followed by discarding the work it
			// failed to preserve, so the prompt stays up and says nothing changed.
			if (SaveDocument())
			{
				ConfirmLeaveToMenu(true);
			}
			break;
		case -4: ConfirmLeaveToMenu(true); break;
		case -6: AnswerRecovery(true); break;
		case -7: AnswerRecovery(false); break;
		default: ConfirmLeaveToMenu(false); break;
		}
		return;
	}
}

void ATUCoasterRide::MenuAction(int32 Action)
{
	if (Session.Mode() != EAppMode::MainMenu) { return; }
	// NOT WHILE A QUESTION IS UP: the confirm and the recovery offer are still
	// canvas, drawn over the widget, and a button under them must not answer.
	if (bConfirmingMenu || Session.HasRecovery()) { return; }
	{
		if (Action >= 0)
		{
			StartFromTemplate(Action);
		}
			else if (Action == -8)
			{
				// THE SAME SCREEN [O] OPENS, hosted by the frame. One settings
				// screen reached two ways, rather than a menu-flavoured copy.
				ToggleSettings();
			}
			else if (Action == -9)
			{
				// ASKED, because quitting from a dirty session is the one exit that
				// loses work. The session already returns the question; this puts it
				// on screen rather than inventing an answer.
				if (Session.IsDirty())
				{
					bConfirmingMenu = true;
					bQuitAfterConfirm = true;
				}
				else if (APlayerController* Q = GetWorld()->GetFirstPlayerController())
				{
					Q->ConsoleCommand(TEXT("quit"));
				}
			}
			else if (Action == -1)
			{
			// THE OS FILE DIALOG IS EDITOR-ONLY, and that is a real limit rather
			// than an oversight: IDesktopPlatform does not ship in a packaged
			// game. The browser list above is the primary path precisely because
			// it works everywhere, and this is the convenience on top of it.
			//
			// A packaged build needs a different picker, and pretending otherwise
			// by calling this unguarded would compile in the editor and fail to
			// link in the thing anybody actually downloads.
#if WITH_EDITOR
				// THE EDITOR HAS ONE, so it gets one. Guarded rather than avoided:
				// calling this unguarded compiles here and fails to LINK in the thing
				// anybody downloads.
				if (IDesktopPlatform* Desktop = FDesktopPlatformModule::Get())
				{
					TArray<FString> Picked;
					const void* ParentWindow = FSlateApplication::IsInitialized()
						? FSlateApplication::Get().FindBestParentWindowHandleForDialogs(nullptr)
						: nullptr;
					if (Desktop->OpenFileDialog(ParentWindow, TEXT("Open a track"), TracksDir(),
						FString(), TEXT("TrackUnlimited track|*.track"),
						EFileDialogFlags::None, Picked) && Picked.Num() > 0)
					{
						if (OpenDocumentFrom(Picked[0]))
						{
							// OPEN LANDS IN RIDE, the same as a recent row: the ACTION
							// picks the mode, and somebody opening a downloaded track
							// wants a ride rather than an editor they did not ask for.
							Session.Enter(EAppMode::Ride);
							ApplyAppMode(EAppMode::Ride);
						}
					}
				}
#else
				// AND A PACKAGED BUILD GETS THE ANSWER IT CAN ACTUALLY GIVE.
				//
				// IDesktopPlatform does not ship, and an in-application file browser
				// is a real piece of work for a job the operating system already does
				// well. So this OPENS THE TRACKS FOLDER: put a file in it and it is a
				// row in the list, which is the primary path anyway.
				//
				// A dead control that logged a warning nobody sees was the alternative,
				// and "the button does nothing" is the single most common complaint
				// about any shell.
				FPlatformFileManager::Get().GetPlatformFile()
					.CreateDirectoryTree(*TracksDir());
				FPlatformProcess::ExploreFolder(*TracksDir());
				RefreshTrackList();
#endif
		}
		else
		{
			// Indexed against the LIST THAT WAS DRAWN, which is the merge of
			// recents and the tracks folder — not against the recent list, which
			// it used to be and which is now a subset. Same index, different
			// array, and the failure would have been opening the wrong track.
			const int32 Which = -1000 - Action;
			if (Which >= 0 && Which < static_cast<int32>(TrackPaths.size()))
			{
				const std::string& Path = TrackPaths[static_cast<std::size_t>(Which)];
				// A ROW THAT OPENS THE TRACK, rather than one that logs its name.
				// A missing file is still listed and still clickable — the
				// commonest cause is an unplugged drive — so this can fail, and it
				// fails by SAYING SO and leaving the menu up rather than by
				// dropping somebody into an empty editor.
				const FString Wanted(UTF8_TO_TCHAR(Path.c_str()));
				if (OpenDocumentFrom(Wanted))
				{
					// OPEN LANDS IN RIDE, per the program-flow decision: the ACTION
					// picks the mode, and somebody opening a downloaded track wants
					// a ride rather than an editor they did not ask for.
					Session.Enter(EAppMode::Ride);
					ApplyAppMode(EAppMode::Ride);
				}
			}
		}
	}
}

bool ATUCoasterRide::PanelMouse(float& Mx, float& My) const
{
	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC || !PC->GetMousePosition(Mx, My)) { return false; }
	Mx /= PaintedScale;
	My /= PaintedScale;
	return true;
}

void ATUCoasterRide::RecordPaintedPanels(TArray<FTUPanelCmd>& Out, float SizeY, float Scale)
{
	GPanelRecord = &Out;
	GPanelSizeY = SizeY;
	PaintedSizeY = SizeY;
	PaintedScale = Scale > 0.f ? Scale : 1.f;
	// The console first, because the graph places itself above wherever the
	// console's top edge landed -- the same order DrawPanels kept.
	ConsolePanelTopY = 1.0e9f;
	DrawConsole(nullptr);
	DrawProfileGraph(nullptr);
	GPanelRecord = nullptr;
}

void ATUCoasterRide::ShowPaintedWidget(bool bShow)
{
	if (!bShow)
	{
		if (PaintedWidget) { PaintedWidget->RemoveFromParent(); }
		// A CONSOLE THAT IS NOT DRAWN HAS NO BUTTONS: the rect lists are filled
		// by the paint, and would otherwise outlive it.
		ConsoleRects.Reset();
		ConsoleAction.Reset();
		return;
	}
	if (!PaintedWidget)
	{
		APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
		if (!PC) { return; }
		PaintedWidget = CreateWidget<UTUPaintedPanelWidget>(PC, UTUPaintedPanelWidget::StaticClass());
		if (!PaintedWidget) { return; }
		PaintedWidget->Ride = this;
		// ON THE USER WIDGET, not only the leaf inside it: the wrapper Slate
		// makes for a UUserWidget is what the hit test meets first.
		PaintedWidget->SetVisibility(ESlateVisibility::HitTestInvisible);
	}
	// Under the menu and the editor, above the frame; full-screen and hit-test
	// invisible, so the viewport click still reaches PressConsole and PressGraph.
	if (!PaintedWidget->IsInViewport()) { PaintedWidget->AddToViewport(5); }
}

void ATUCoasterRide::ShowEditorWidget(bool bShow)
{
	if (!bShow)
	{
		if (EditorWidget) { EditorWidget->RemoveFromParent(); }
		return;
	}
	if (!EditorWidget)
	{
		APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
		if (!PC) { return; }
		EditorWidget = CreateWidget<UTUSegmentEditorWidget>(PC, UTUSegmentEditorWidget::StaticClass());
		if (!EditorWidget) { return; }
		EditorWidget->Ride = this;
	}
	if (!EditorWidget->IsInViewport()) { EditorWidget->AddToViewport(10); }
}

void ATUCoasterRide::ShowMenuWidget(bool bShow)
{
	if (!bShow)
	{
		if (MenuWidget) { MenuWidget->RemoveFromParent(); }
		return;
	}
	if (!MenuWidget)
	{
		APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
		if (!PC) { return; }
		// A NATIVE CLASS, NO ASSET: every row is a live read, so there is nothing
		// for an asset to hold. Created once and re-added, because rebuilding a
		// widget tree on every [M] is work for no change.
		MenuWidget = CreateWidget<UTUMenuWidget>(PC, UTUMenuWidget::StaticClass());
		if (!MenuWidget) { return; }
		MenuWidget->Ride = this;
	}
	// ABOVE THE FRAME, which is full-screen and hit-testable (it has to be, or the
	// mouse is lost at the menu) and is created after this on the boot path.
	if (!MenuWidget->IsInViewport()) { MenuWidget->AddToViewport(10); }
	MenuWidget->Rebuild();
}

void ATUCoasterRide::OpenMainMenu()
{
	if (Session.Mode() == EAppMode::MainMenu)
	{
		return;
	}

	// THE ONLY TRANSITION THAT ASKS, because it is the only one that discards the
	// document. Build to Operate to Ride never asks and never should — nothing is
	// lost — and a shell that asked there would train people to click straight
	// through the dialog that matters.
	//
	// This became reachable at all on 2026-08-08, when the session first got told
	// what the document is: before that `IsDirty` was false for ever, so
	// `MayEnter` could never return NeedsConfirmation and the branch below was
	// dead code that looked alive.
	if (Session.MayEnter(EAppMode::MainMenu) == ELeaveRequest::NeedsConfirmation)
	{
		bConfirmingMenu = true;
		return;
	}
	RefreshTrackList();
	Session.Enter(EAppMode::MainMenu);
	ApplyAppMode(EAppMode::MainMenu);
}

void ATUCoasterRide::ConfirmLeaveToMenu(bool bDiscard)
{
	bConfirmingMenu = false;
	const bool bQuitting = bQuitAfterConfirm;
	bQuitAfterConfirm = false;
	if (!bDiscard)
	{
		return;
	}
	if (bQuitting)
	{
		// THE SAME QUESTION, A DIFFERENT EXIT. Quitting and going back to the
		// menu both discard the document, so they share one prompt rather than
		// having two that read the same and behave differently.
		if (APlayerController* Q = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
		{
			Q->ConsoleCommand(TEXT("quit"));
		}
		return;
	}
	// CONFIRMED HERE AND ONLY HERE, because the question was actually put on
	// screen and answered. Passing bConfirmed without having asked is lying to a
	// class that cannot tell, which is why MayEnter and Enter are two calls.
	//
	// Refreshed on the way through, because "save and leave" has just written a
	// file that ought to be the first row somebody sees.
	RefreshTrackList();
	Session.Enter(EAppMode::MainMenu, true);
	ApplyAppMode(EAppMode::MainMenu);
}

ATUCoasterRide::FTUPlatform* ATUCoasterRide::ConsolePlatformPtr()
{
	// ONE ANSWER, used by the draw AND by every command. The panel picking one
	// platform to display while the buttons commanded all of them is exactly the
	// bug this replaces, and two functions would let it come back.
	if (Platforms.IsValidIndex(ConsolePlatform))
	{
		return &Platforms[ConsolePlatform];
	}
	// FOLLOWING THE TRAIN: whichever position has one and is furthest along, which
	// is the one an operator standing at a console is working.
	FTUPlatform* Best = nullptr;
	for (FTUPlatform& P : Platforms)
	{
		if (P.Inputs.bTrainPresent && (Best == nullptr || P.Zone > Best->Zone))
		{
			Best = &P;
		}
	}
	return Best;
}

double ATUCoasterRide::ConsoleStandS() const
{
	// THE FIRST PLATFORM, which the block walk already derived — nothing here is
	// authored or searched for, so the console stands in the right place on a
	// layout nobody has built yet.
	double End = 0.0;
	for (int32 i = 0; i < Segments.Num(); ++i)
	{
		const double L = static_cast<double>(Segments[i].Length);
		if (Segments[i].Zone == ETUSegmentZone::Station
			|| Segments[i].Zone == ETUSegmentZone::StationLoad
			|| Segments[i].Zone == ETUSegmentZone::StationUnload)
		{
			// A THIRD OF THE WAY IN, not the start: a console sits beside the
			// train it is dispatching, and the platform's leading edge is where
			// the train is leaving from rather than where it stands.
			return End + L * 0.34;
		}
		End += L;
	}
	// NO PLATFORM IS A TEST RIG, and the start of the track is the honest answer
	// rather than an arbitrary metre in the middle of a launch.
	return 0.0;
}

void ATUCoasterRide::FrameStation()
{
	// WHERE AN OPERATOR STANDS. Chase follows a train around the ride, which is
	// the wrong view for a console: the question in Operate is "may this train
	// go", and that is asked and answered at the platform.
	//
	// The station is the first holding zone, which the block walk already
	// derived — nothing here is authored or searched for.
	double Start = 0.0, End = 0.0;
	bool bFound = false;
	for (int32 i = 0; i < Segments.Num(); ++i)
	{
		const double L = static_cast<double>(Segments[i].Length);
		if (!bFound && (Segments[i].Zone == ETUSegmentZone::Station
			|| Segments[i].Zone == ETUSegmentZone::StationLoad
			|| Segments[i].Zone == ETUSegmentZone::StationUnload))
		{
			Start = End;
			bFound = true;
		}
		End += L;
		if (bFound && End - Start > 30.0) { break; }
	}
	if (!bFound)
	{
		// A ride with no platform is a test rig, and framing the whole thing is
		// the honest answer rather than picking an arbitrary metre.
		FrameWholeTrack();
		return;
	}

	// AN OPERATOR'S EYE LINE, NOT A HELICOPTER. The bounding-box fit stood here
	// and framed the platform from the sky, which is the Build view with a
	// different subject. An operator stands beside the platform: focus on the
	// middle of it at head height, a little above it, looking along it from
	// the rider's left -- the side the console camera on [C] already stands on
	// -- far enough back that the whole platform and the train on it are in
	// shot. The block ahead is read off the schematic, as a real operator does.
	const FTrackFrame Mid = Track.EvaluateAt(Start + (End - Start) * 0.5);
	Orbit.Focus = {Mid.Position.X, Mid.Position.Y, Mid.Position.Z + 1.6};
	Orbit.Distance = std::max(20.0, (End - Start) * 1.1);
	Orbit.PitchDeg = -10.0;
	Orbit.YawDeg = FMath::RadiansToDegrees(std::atan2(Mid.Tangent.Y, Mid.Tangent.X)) + 60.0;
	bOrbitIsBackdrop = false;
	bOrbitFramed = true;
}

void ATUCoasterRide::CycleAppMode()
{
	// BUILD to OPERATE to RIDE and back. Never asks, because nothing is
	// discarded — the document is still open and still in memory, and a shell
	// that asked here would train people to click straight through the dialog
	// that matters. The session's own rules say so and are tested.
	const EAppMode Now = Session.Mode();
	EAppMode Want = EAppMode::Build;
	if (Now == EAppMode::Build)        { Want = EAppMode::Operate; }
	else if (Now == EAppMode::Operate) { Want = EAppMode::Ride; }

	if (!Session.Enter(Want))
	{
		const char* Why = Session.WhyNotEnter(Want);
		UE_LOG(LogTUEvents, Warning, TEXT("mode refused: %s"),
			Why ? UTF8_TO_TCHAR(Why) : TEXT("unknown"));
		return;
	}
	ApplyAppMode(Want);
}

void ATUCoasterRide::EnterAppMode(EAppMode Wanted, bool bConfirmed)
{
	// THE SAME DOOR THE KEY GOES THROUGH. [Tab] and a clicked tab must not be
	// two ways of changing mode with two ideas of what a mode does — that is how
	// a shell ends up where the keyboard and the mouse disagree about the state
	// of the application.
	if (!Session.Enter(Wanted, bConfirmed))
	{
		const char* Why = Session.WhyNotEnter(Wanted);
		UE_LOG(LogTUEvents, Warning, TEXT("mode refused: %s"),
			Why ? UTF8_TO_TCHAR(Why) : TEXT("unknown"));
		return;
	}
	ApplyAppMode(Wanted);
}

void ATUCoasterRide::ApplyAppMode(EAppMode Want)
{
	// KEYS GO TO THE GAME AGAIN. Clicking a frame tab or a settings control
	// leaves Slate holding keyboard focus, and from then on [Tab], [M] and the
	// rest went to a button rather than to the ride -- the keyboard "stopped
	// working" after the first click on the chrome. Every mode change and
	// every settings close hands focus back.
	if (FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().SetAllUserFocusToGameViewport();
	}
	// THE MODE DECIDES THE VIEW, which is what makes it a mode rather than a
	// label. Operate is the operator's console and the ride seen from outside;
	// Ride is the seat; Build is the thing you are editing, framed.
	switch (Want)
	{
	case EAppMode::Operate:
		ShowMenuWidget(false);
		PanelView = ETUPanelView::Operator;
		// THE STATION, not a chase. Chase follows a train around the ride; the
		// question in Operate is "may this train go", and that is asked and
		// answered at the platform.
		CameraMode = ETUCameraMode::Orbit;
		SyncCameraRig();
		FrameStation();
		break;
	case EAppMode::Ride:
		ShowMenuWidget(false);
		PanelView = ETUPanelView::Off;
		CameraMode = ETUCameraMode::Rider;
		bShowSegmentEditor = false;   // a list of segments is not a view from a seat; [B] brings it back
		break;
	case EAppMode::Boot:
	case EAppMode::MainMenu:
	{
		// ===================== THE MENU OPENS OVER NOTHING =====================
		//
		// Whatever was open is CLOSED here, on every road into the menu -- boot,
		// [M], the confirm's two "leave" answers. Before this only boot closed it,
		// so [M] from a running ride drew the menu over five moving trains, a
		// block strip, a telemetry readout and the orbit camera still tracking.
		// The session already asked about unsaved work on the way in, so nothing
		// is lost that somebody was not told about.
		PanelView = ETUPanelView::Off;
		SelectedSegment = -1;

		// ===================== AND THE BACKDROP IS A RUNNING RIDE =====================
		//
		// The Showcase, trains dispatching, seen from a slow orbit. It is what the
		// application IS, and a menu over an empty desert said nothing about it.
		// It is NOT the person's document: it is created clean, so the frame shows
		// no asterisk, Quit does not ask, and [Tab] into Build simply makes it the
		// track being edited -- which is the same thing that template row does.
		Preset = ETUPresetLayout::Showcase;
		Segments = PresetLayout(Preset);
		ApplyPresetTrainSetup(Preset);
		ApplyPresetWalkways();
		RebuildFromSegments();
		Session.DidCreateNew(TCHAR_TO_UTF8(*SerialiseDocument()));
		ResetHistory();
		if (UWorld* World = GetWorld())
		{
			FlushPersistentDebugLines(World);
		}
		if (FrameWidget)
		{
			FrameWidget->CloseSettings();
		}
		ShowMenuWidget(true);

		// Framed from a little above the horizon and turning slowly (see the
		// orbit branch of Tick). Framed here rather than on the first tick so the
		// first frame is already the picture.
		CameraMode = ETUCameraMode::Orbit;
		SyncCameraRig();
		FrameWholeTrack();
		Orbit.PitchDeg = -18.0;
		Orbit.YawDeg = -30.0;
		// APPLIED NOW, not on the next tick: Tick returns before the camera while
		// the window is unfocused or the scan has not started, and the first
		// frame of the application was the level's serialised camera meanwhile.
		ApplyOrbitToCamera();
		bOrbitIsBackdrop = true;   // whatever opens next gets the default angle
		break;
	}
	default:
		PanelView = ETUPanelView::Off;
		CameraMode = ETUCameraMode::Orbit;
		SyncCameraRig();
		// BUILD SHOWS THE EDITOR. The boot path quiets every overlay so the menu
		// is not a wall of numbers, and that quiet reached Build too: the hint
		// said "edit the numbers" over a viewport with no numbers on it. [B] still
		// hides it again.
		if (Want == EAppMode::Build) { bShowSegmentEditor = true; }
		ShowMenuWidget(false);
		if (FrameWidget)
		{
			// A SETTINGS PAGE LEFT OPEN ACROSS A MODE CHANGE IS A STUCK MENU. It
			// sits in the frame's content slot over the whole viewport, and the
			// only thing that closed it was the key that opened it.
			FrameWidget->CloseSettings();
		}
		break;
	}
}


void ATUCoasterRide::DrawModeBanner(UCanvas* Canvas)
{
	if (!Canvas || !GEngine) { return; }

	// SMALL AND ALWAYS THERE. Not a title screen — the mode is a fact about what
	// your keys do, and a person who cannot see it will press something and be
	// surprised. `UI_CONVENTIONS`: hue is never the only channel, so the mode is
	// spelled out rather than being a coloured border.
	const TCHAR* Name = UTF8_TO_TCHAR(AppModeName(Session.Mode()));
	const FLinearColor Ink = Session.EditsAllowed() ? PanelCyan : PanelAmber;

	FString Line = FString::Printf(TEXT("  %s   [Tab] mode"), Name);
	if (!Session.EditsAllowed())
	{
		// AND IT SAYS WHY THE EDITOR IS QUIET, because a panel that has simply
		// stopped accepting numbers with no explanation is indistinguishable from
		// a broken one.
		Line += TEXT("   editing is off while the ride is running");
	}
	if (Session.IsDirty()) { Line += TEXT("   *unsaved"); }

	// WHICH TRAIN, WHENEVER THERE IS MORE THAN ONE. Four trains on a circuit are
	// identical objects on identical track, so cycling between them is invisible
	// unless something says which one you landed on — and "the camera did not
	// move" and "it moved to a train in the same place" look the same.
	if (Trains.Num() > 1)
	{
		Line += FString::Printf(TEXT("   train %d/%d  [T]"),
			ActiveTrainIndex() + 1, Trains.Num());
	}
	if (CameraMode == ETUCameraMode::Rider && CarCount > 0)
	{
		const FSeat Seat = SeatByIndex(RiderSeat, CarCount, 1.0);
		Line += FString::Printf(TEXT("   car %d %s  [N]"),
			Seat.Car + 1, Seat.LateralM > 0.0 ? TEXT("left") : TEXT("right"));
	}

	// THE CLOCK IS ALWAYS STATED WHEN IT IS NOT 1x. A ride running at quarter
	// speed with nothing saying so is a ride somebody will report as too slow —
	// and a paused one is a ride they will report as broken.
	if (bSimPaused)
	{
		Line += TEXT("   PAUSED  [.] step  [Pause] run");
	}
	else if (!FMath::IsNearlyEqual(TimeScale, 1.f))
	{
		Line += FString::Printf(TEXT("   %.2fx  [,] slower  [/] faster"), TimeScale);
	}

	// BELOW THE FRAME'S HEADER, not under it. The UMG frame owns the top 40 px
	// (mode, document, tabs) and this sat at y = 6, so two copies of the word
	// BUILD were drawn through each other.
	const float By = 46.f;
	const float Bw = 8.f + PanelTextWidth(Canvas, Line);
	PanelTile(Canvas, 10.f, By, Bw, 18.f, PanelGround);
	PanelLabel(Canvas, 12.f, By + 2.f, Line, Ink);

	// THE WAY OUT, ON SCREEN. [M] existed and nothing said so outside the menu
	// itself, which is the one place somebody who needs it is not.
	const FString MenuText = TEXT("[ MENU ]  M");
	const float Mw = 8.f + PanelTextWidth(Canvas, MenuText);
	MenuButtonRect = FVector4(10.f + Bw + 6.f, By, 10.f + Bw + 6.f + Mw, By + 18.f);
	PanelTile(Canvas, MenuButtonRect.X, MenuButtonRect.Y, Mw, 18.f, PanelRule);
	PanelLabel(Canvas, MenuButtonRect.X + 4.f, By + 2.f, MenuText, PanelText);

	// ===================== THE ONE-LINE VERSION, IN BUILD ONLY =====================
	//
	// `ViewportHint()` says what the view IS before anybody has to discover it by
	// trying to drag. In Build only, because it is an answer about EDITING — in
	// Operate and Ride the viewport is not pretending to be anything else, and a
	// permanent line explaining a constraint that is not currently biting is the
	// kind of chrome people stop seeing and then never see when it matters.
	//
	// It goes away once there is a selection: somebody who has clicked a segment
	// has found the panel, and the hint has done its job.
	if (Session.Mode() == EAppMode::Build && SelectedSegment < 0 && !bHideOverlays)
	{
		const FString Hint(UTF8_TO_TCHAR(ViewportHint()));
		PanelTile(Canvas, 10.f, 66.f, 8.f + PanelTextWidth(Canvas, Hint), 16.f, PanelGround);
		PanelLabel(Canvas, 12.f, 67.f, Hint, PanelDim);
	}
}

void ATUCoasterRide::ClickDiagnostics()
{
	// A FINDING WITHOUT A PLACE IS TRIVIA — the diagnostics model's rule, and this
	// is the half that makes it true. "Curvature implying a radius under 2 m" is
	// only useful if it takes you there.
	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC || !bShowDiagnostics) { return; }

	float Mx = 0.f, My = 0.f;
	if (!PC->GetMousePosition(Mx, My)) { return; }

	for (int32 i = 0; i < DiagRowRects.Num(); ++i)
	{
		const FVector4& R = DiagRowRects[i];
		if (Mx < R.X || Mx > R.Z || My < R.Y || My > R.W) { continue; }
		if (i >= static_cast<int32>(Diagnostics.Num())) { break; }

		const FDiagRow& Row = Diagnostics.At(static_cast<std::size_t>(i));
		if (Row.Target.Segment >= 0)
		{
			SelectedSegment = Row.Target.Segment;
			FrameSelectedSegment();
		}
		else if (Row.Target.S >= 0.0)
		{
			// A finding located by ARC LENGTH rather than by segment — anything
			// derived rather than authored. Frame the point itself; there is no
			// segment to select, and pretending there is would select the wrong
			// one whenever the walk and the authored list disagree.
			const FTrackFrame F = Track.EvaluateAt(Row.Target.S);
			FCamBounds B;
			B.Add({F.Position.X - 8.0, F.Position.Y - 8.0, F.Position.Z - 8.0});
			B.Add({F.Position.X + 8.0, F.Position.Y + 8.0, F.Position.Z + 8.0});
			CameraMode = ETUCameraMode::Orbit;
			SyncCameraRig();
			Orbit.Frame(B, Camera ? static_cast<double>(Camera->FieldOfView) : 90.0, 16.0 / 9.0);
			bOrbitFramed = true;
		}
		return;
	}
}

void ATUCoasterRide::PollMovementKeys()
{
	MoveForward = MoveRight = MoveUp = 0.f;

	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC)
	{
		return;
	}

	// TYPING A NUMBER IS NOT FLYING. The segment editor takes digits, and A, S, D,
	// E and W are all one keystroke away from a field somebody is working in —
	// but more to the point, a camera that drifted while somebody typed a radius
	// would move the thing they are looking at out from under them.
	if (IsTypingInField())
	{
		return;
	}

	auto Down = [PC](const FKey& K) { return PC->IsInputKeyDown(K) ? 1.f : 0.f; };
	MoveForward = Down(EKeys::W) - Down(EKeys::S);
	MoveRight = Down(EKeys::D) - Down(EKeys::A);
	MoveUp = Down(EKeys::E) - Down(EKeys::Q);
}

void ATUCoasterRide::ApplyCursorMode()
{
	APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
	if (!PC) { return; }

	// RIDE IS THE ONLY MODE WITH NOTHING TO CLICK. Everywhere else has rows, and
	// a captured mouse made every one of them keyboard-only.
	const bool bWant = Session.Mode() != EAppMode::Ride;

	// Applied only on a CHANGE. SetInputMode every frame resets Slate's capture
	// state, which shows up as a cursor that stutters and a drag that drops.
	if (bWant == bCursorShown && PC->bShowMouseCursor == bWant) { return; }
	bCursorShown = bWant;
	PC->bShowMouseCursor = bWant;

	if (bWant)
	{
		// GameAndUI rather than UIOnly: the key bindings have to keep working
		// while the cursor is free, because every one of them is still the
		// fastest way to do the thing it does.
		FInputModeGameAndUI Mode;
		Mode.SetHideCursorDuringCapture(false);
		PC->SetInputMode(Mode);
	}
	else
	{
		PC->SetInputMode(FInputModeGameOnly());
	}
}

void ATUCoasterRide::StepSelection(int32 Delta)
{
	if (Segments.Num() == 0) { return; }

	// NOT WHILE TYPING. A bracket is not a digit, so nothing would be corrupted —
	// but moving the selection out from under a half-entered number and then
	// applying it on Enter would write it to the wrong segment, which is the one
	// outcome this panel must never produce.
	if (IsTypingInField()) { return; }

	// WRAPS, because the layouts are rings as often as not and a selection that
	// stopped dead at segment 0 would make walking backwards past the station
	// impossible on exactly the presets where it is most wanted.
	const int32 N = Segments.Num();
	SelectedSegment = SelectedSegment < 0
		? (Delta > 0 ? 0 : N - 1)
		: ((SelectedSegment + Delta) % N + N) % N;

	if (CameraMode == ETUCameraMode::Orbit)
	{
		FrameSelectedSegment();
	}
}

void ATUCoasterRide::FrameSelectedSegment()
{
	// NOTHING SELECTED FALLS BACK TO THE WHOLE TRACK rather than doing nothing. A
	// key that silently did nothing reads as broken, and "show me everything" is
	// the answer somebody pressing it almost certainly wanted anyway.
	if (SelectedSegment < 0 || SelectedSegment >= Segments.Num())
	{
		FrameWholeTrack();
		return;
	}

	// Walk to the selected segment's own span and frame only that. The walk is
	// the same one the mesher uses, so nothing extra is integrated.
	double Start = 0.0;
	for (int32 i = 0; i < SelectedSegment; ++i)
	{
		Start += static_cast<double>(Segments[i].Length);
	}
	const double End = Start + static_cast<double>(Segments[SelectedSegment].Length);

	// Sampled directly across the span rather than walked: a handful of EvaluateAt
	// calls on a BOUNDED range, where walking the whole track to find one segment
	// would be the O(track length) mistake this project keeps having to unlearn.
	FCamBounds B;
	const int Steps = 12;
	for (int i = 0; i <= Steps; ++i)
	{
		const double S = Start + (End - Start) * (static_cast<double>(i) / Steps);
		const FTrackFrame F = Track.EvaluateAt(S);
		B.Add({F.Position.X, F.Position.Y, F.Position.Z});
	}
	// A dead-straight segment is a line with no thickness, and framing a line
	// puts the camera on top of it. Pad by the track's own swept width so there
	// is always something to look at.
	const double Pad = TrackWidth(Profile) * 2.0;
	B.Add({B.Min.X - Pad, B.Min.Y - Pad, B.Min.Z - Pad});
	B.Add({B.Max.X + Pad, B.Max.Y + Pad, B.Max.Z + Pad});

	// MODE FIRST, THEN THE FIT. Switching the rig after framing would hand the
	// fit to the old mode's rig and bring back the stored one.
	CameraMode = ETUCameraMode::Orbit;
	SyncCameraRig();
	Orbit.Frame(B, Camera ? static_cast<double>(Camera->FieldOfView) : 90.0, 16.0 / 9.0);
	bOrbitFramed = true;

	UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: framed segment %d (%.1f-%.1f m)"),
		SelectedSegment, Start, End);
}

void ATUCoasterRide::CycleProfileChannel()
{
	switch (ProfileChannel)
	{
	case ETUProfileChannel::VerticalG: ProfileChannel = ETUProfileChannel::LateralG; break;
	case ETUProfileChannel::LateralG:  ProfileChannel = ETUProfileChannel::Speed; break;
	case ETUProfileChannel::Speed:     ProfileChannel = ETUProfileChannel::RollRate; break;
	default:                           ProfileChannel = ETUProfileChannel::VerticalG; break;
	}
	// Showing the graph is implied by asking to change its channel. A key that
	// silently changed something invisible would be a key nobody trusts.
	bShowProfileGraph = true;
}

void ATUCoasterRide::FrameWholeTrack()
{
	CameraMode = ETUCameraMode::Orbit;
	SyncCameraRig();
	// Walked rather than guessed: the frames the mesher already produces, swept
	// into a bounding box. Nothing extra is integrated for this.
	FCamBounds B;
	for (const FTrackFrame& F : WalkTrack(Track, 2.0))
	{
		B.Add({F.Position.X, F.Position.Y, F.Position.Z});
	}
	// ===================== NOTHING TO FRAME IS STILL A VIEW =====================
	//
	// This used to return, leaving the camera wherever the level had serialised
	// it -- which for a menu with no document open is the origin, INSIDE whatever
	// geometry happens to be there. The first screen was a close-up of the inside
	// of a default material.
	//
	// So an empty track frames a deliberate box instead: 300 m across and lifted
	// off the ground, which pulls the camera back far enough to be looking at the
	// horizon rather than at anything in particular. A backdrop, not a subject.
	if (!B.IsValid())
	{
		B.Add({-150.0, -150.0, 0.0});
		B.Add({ 150.0,  150.0, 60.0});
		// Shallower than the default -25 deg, because there is no layout below to
		// look down at and a downward tilt on an empty scene reads as pointing at
		// the floor.
		Orbit.PitchDeg = -8.0;
		Orbit.YawDeg = -35.0;
		bOrbitIsBackdrop = true;
	}
	else if (bOrbitIsBackdrop)
	{
		// COMING BACK FROM THE BACKDROP, the shallow angle stays unless it is put
		// back: the first track opened from the menu was framed at -8 deg, level
		// with its own lift hill. A person's own orbit angle is kept; only the
		// backdrop's is replaced.
		Orbit.PitchDeg = -25.0;
		Orbit.YawDeg = -45.0;
		bOrbitIsBackdrop = false;
	}

	// FOV from the camera itself, aspect from the viewport, because framing that
	// checked one axis puts a long low layout off both sides of an ultrawide —
	// and this project's layouts are all long and low.
	double Aspect = 16.0 / 9.0;
	if (GEngine && GEngine->GameViewport)
	{
		const FVector2D Size = GEngine->GameViewport->Viewport->GetSizeXY();
		if (Size.Y > 0.f) { Aspect = static_cast<double>(Size.X / Size.Y); }
	}
	Orbit.Frame(B, Camera ? static_cast<double>(Camera->FieldOfView) : 90.0, Aspect);
	// THE SPHERE FIT IS A CEILING, NOT A PICTURE. It guarantees nothing is lost
	// while orbiting, and on a layout 400 m long and 50 m tall that puts the
	// camera 600 m back looking at a thread. Asked for a look, the fit is the
	// guarantee and this is the picture; a mouse wheel gets the rest back.
	// ponytail: one factor; a box-against-frustum fit if it ever matters.
	Orbit.Distance *= 0.75;
	bOrbitFramed = true;

	UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: framed %.0f m of track from %.0f m"),
		Track.TotalLength(), Orbit.Distance);
}

void ATUCoasterRide::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// This is the live preview. OnConstruction runs when the actor is placed,
	// when the level loads, when the actor is moved, and after any property
	// change — which is exactly the set of moments the drawn track is stale.
	RebuildFromSegments();

#if WITH_EDITOR
	// Editor worlds only. In PIE, BeginPlay does the drawing, and doing it here
	// as well would just draw the same lines twice.
	//
	// ponytail: flushes ALL persistent debug lines in the world, so two of these
	// actors in one level would erase each other's preview. Fine while the
	// vertical slice is the only thing in the level; the real fix is a line
	// batcher component per actor, or the Phase 4 track mesh making this whole
	// function unnecessary.
	if (UWorld* World = GetWorld())
	{
		if (!World->IsGameWorld())
		{
			FlushPersistentDebugLines(World);
			if (bDrawTrack)
			{
				DrawTrack();
			}
			DrawRideProfile();
		}
	}
#endif
}

FString ATUCoasterRide::SerialiseDocument() const
{
	FTrackDocument Doc;
	Doc.HeartlineHeight = 1.1;
	Doc.Segments.reserve(static_cast<std::size_t>(Segments.Num()));
	for (const FTUTrackSegment& S : Segments)
	{
		Doc.Segments.push_back(ToAuthored(S));
	}

	std::string Text, Error;
	if (!WriteTrackJson(Doc, Text, Error))
	{
		// A NON-FINITE FIELD IS THE ONLY WAY HERE, and the writer refuses rather
		// than putting "nan" on disc. Returning empty reads as dirty, which is the
		// safe direction: a document that cannot be written must not be able to
		// report that it has nothing worth saving.
		UE_LOG(LogTUEvents, Warning, TEXT("document: cannot serialise — %s"),
			UTF8_TO_TCHAR(Error.c_str()));
		return FString();
	}
	return FString(UTF8_TO_TCHAR(Text.c_str()));
}

FString ATUCoasterRide::TracksDir() const
{
	// ABSOLUTE, because these paths are STORED. `ProjectSavedDir` comes back
	// relative to whatever the working directory happened to be at launch, and a
	// recent list full of `../../../../` entries is one launched-from-elsewhere
	// away from every row in the menu being a file that cannot be found — the
	// browser's "missing" state, arriving for a reason that is our fault.
	return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Tracks"));
}

FString ATUCoasterRide::NextUntitledPath() const
{
	// NEVER THE SAME NAME TWICE. An unnamed save that overwrote the last unnamed
	// save would destroy work by doing exactly what it was asked to do, which is
	// the worst kind of data loss to explain afterwards.
	IPlatformFile& Files = FPlatformFileManager::Get().GetPlatformFile();
	for (int32 N = 1; N < 10000; ++N)
	{
		const FString Name = N == 1 ? FString(TEXT("Untitled"))
									: FString::Printf(TEXT("Untitled-%d"), N);
		const FString Path = TracksDir() / (Name + TEXT(".track"));
		if (!Files.FileExists(*Path))
		{
			return Path;
		}
	}
	// Ten thousand untitled tracks is not a case worth branching for, but silently
	// returning the last one would clobber it.
	return TracksDir() / TEXT("Untitled-overflow.track");
}

bool ATUCoasterRide::SaveDocumentTo(const FString& InPath)
{
	// Normalised once, here, so the recent list and the session agree about what
	// this document is called however the caller spelled it.
	const FString Path = FPaths::ConvertRelativePathToFull(InPath);
	const FString Text = SerialiseDocument();
	if (Text.IsEmpty())
	{
		return false;   // the serialiser has already said why
	}

	IPlatformFile& Files = FPlatformFileManager::Get().GetPlatformFile();
	Files.CreateDirectoryTree(*FPaths::GetPath(Path));

	// FORCED UTF-8, NEVER AUTO-DETECT. SaveStringToFile's default writes ANSI when
	// the text happens to be representable and UTF-16 when it is not — so a track
	// with a non-ASCII character in it silently changes encoding, and the loader,
	// which reads the bytes as UTF-8, gets something else. The file format is
	// text people are meant to diff and hand-edit; it has one encoding.
	if (!FFileHelper::SaveStringToFile(Text, *Path,
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		UE_LOG(LogTUEvents, Warning, TEXT("save: could not write %s"), *Path);
		return false;
	}

	// MARKED CLEAN BY WHAT WAS WRITTEN, not by being told a save happened. A
	// caller that serialised something other than what it displayed cannot
	// accidentally mark the session clean.
	Session.DidSave(TCHAR_TO_UTF8(*Path), TCHAR_TO_UTF8(*Text));
	// Session's comparison is still the one thing that decides dirty; this only
	// stops FTrackHistory's own IsDirty being a second answer that is wrong.
	if (History) { History->MarkSaved(); }
	// THE SIDECAR HAS DONE ITS JOB. A real save supersedes it, and leaving it
	// behind means the next launch offers to recover work that is already on
	// disc — which teaches people to click past the one dialog that matters.
	FPlatformFileManager::Get().GetPlatformFile().DeleteFile(*SidecarPath());
	Browser.Touch(TCHAR_TO_UTF8(*Path));
	WriteRecentList();

	UE_LOG(LogTUEvents, Log, TEXT("saved %d segments to %s"), Segments.Num(), *Path);
	return true;
}

bool ATUCoasterRide::SaveDocument()
{
	return SaveDocumentTo(Session.HasPath()
		? FString(UTF8_TO_TCHAR(Session.Path().c_str()))
		: NextUntitledPath());
}

void ATUCoasterRide::SaveDocumentFromKey()
{
	// SHIFT ALWAYS ASKS; a document with no path asks anyway. See the header for
	// why the first save is where a name belongs and why the leave-confirm's save
	// deliberately does not ask.
	if (bBoost || !Session.HasPath())
	{
		BeginNameSave(bBoost);
		return;
	}
	SaveDocument();
}

void ATUCoasterRide::BeginNameSave(bool bFromShift)
{
	bNamingSave = true;
	NameError.Empty();

	// PRE-FILLED WITH WHAT IT IS CALLED NOW, so save-as on a named track is a
	// small edit rather than retyping. An unnamed one starts EMPTY rather than at
	// "Untitled": a prompt already holding a plausible answer is one people press
	// Enter on, which is the folder-full-of-Untitled outcome this exists to end.
	NameBuffer.Empty();
	if (bFromShift && Session.HasPath())
	{
		NameBuffer = FPaths::GetBaseFilename(
			FString(UTF8_TO_TCHAR(Session.Path().c_str())));
	}
}

void ATUCoasterRide::CancelNameSave()
{
	bNamingSave = false;
	NameBuffer.Empty();
	NameError.Empty();
}

void ATUCoasterRide::CommitNameSave()
{
	FString Name = NameBuffer.TrimStartAndEnd();

	// AN EMPTY NAME IS A CANCEL, not a file called nothing — the same reading the
	// numeric fields give an empty buffer, and for the same reason: somebody who
	// opened this and pressed Enter meant "never mind".
	if (Name.IsEmpty())
	{
		CancelNameSave();
		return;
	}

	const FString Path = TracksDir() / (Name + TEXT(".track"));

	// ===================== IT WILL NOT WRITE OVER SOMETHING ELSE =====================
	//
	// The unnamed save has never been allowed to clobber, and a typed name must
	// not be the hole in that: "Reference" is a name somebody will reach for
	// twice, and the second time the first track is gone with no undo anywhere in
	// the application that could bring it back.
	//
	// Saving over the document you already have open is not that case and is
	// allowed — it is an ordinary save that happens to have been typed.
	IPlatformFile& Files = FPlatformFileManager::Get().GetPlatformFile();
	const bool bIsThisDocument = Session.HasPath()
		&& FPaths::IsSamePath(FString(UTF8_TO_TCHAR(Session.Path().c_str())),
			FPaths::ConvertRelativePathToFull(Path));
	if (Files.FileExists(*Path) && !bIsThisDocument)
	{
		// ponytail: refused rather than offering to overwrite. A confirm is a
		// second dialog above a dialog, and the answer to "that name is taken" is
		// to type another one. Add the overwrite path when somebody asks for it.
		NameError = FString::Printf(
			TEXT("\"%s\" already exists. Type a different name."), *Name);
		return;
	}

	if (!SaveDocumentTo(Path))
	{
		// THE PROMPT STAYS UP ON A FAILURE, because dismissing it would report a
		// save that did not happen by simply going away.
		NameError = TEXT("Could not write that file. Try another name.");
		return;
	}
	CancelNameSave();
}

void ATUCoasterRide::OnAnyKeyTyped(FKey Key)
{
	if (!bNamingSave) { return; }

	// Enter, Escape and Backspace have their own bindings and reach this too.
	// Ignored here so they are not also typed as characters — the specific
	// binding is the one that means something.
	const FName N = Key.GetFName();
	const FString S = N.ToString();

	TCHAR C = 0;
	if (S.Len() == 1)
	{
		const TCHAR Raw = S[0];
		if (Raw >= 'A' && Raw <= 'Z')
		{
			// SHIFT IS ALSO THE CAMERA BOOST, and holding it here is unambiguous
			// because the camera does not move while a field has focus.
			C = bBoost ? Raw : static_cast<TCHAR>(Raw - 'A' + 'a');
		}
	}
	else if (Key == EKeys::SpaceBar)  { C = ' '; }
	else if (Key == EKeys::Hyphen || Key == EKeys::Subtract) { C = '-'; }
	else if (Key == EKeys::Underscore) { C = '_'; }
	else
	{
		// Digits are named "Zero".."Nine" and the numpad "NumPadZero"..; both are
		// wanted, and neither is a single character to test for.
		static const TCHAR* Names[10] = {TEXT("Zero"), TEXT("One"), TEXT("Two"),
			TEXT("Three"), TEXT("Four"), TEXT("Five"), TEXT("Six"), TEXT("Seven"),
			TEXT("Eight"), TEXT("Nine")};
		for (int32 i = 0; i < 10; ++i)
		{
			if (S == Names[i] || S == FString(TEXT("NumPad")) + Names[i])
			{
				C = static_cast<TCHAR>('0' + i);
				break;
			}
		}
	}

	// A CAP, because a file name has one whatever this thinks, and a buffer that
	// grows past the box it is drawn in is a field somebody cannot read.
	if (C != 0 && NameBuffer.Len() < 48)
	{
		NameBuffer.AppendChar(C);
		NameError.Empty();
	}
}

bool ATUCoasterRide::OpenDocumentFrom(const FString& InPath)
{
	const FString Path = FPaths::ConvertRelativePathToFull(InPath);
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *Path))
	{
		// THE LOADER'S OWN REASON, which the browser is built to carry: "plug the
		// drive back in" and "line 12 is wrong" are different problems.
		UE_LOG(LogTUEvents, Warning, TEXT("open: cannot read %s"), *Path);
		return false;
	}

	FTrackDocument Doc;
	std::string Error;
	if (!ParseTrackJson(TCHAR_TO_UTF8(*Text), Doc, Error))
	{
		UE_LOG(LogTUEvents, Warning, TEXT("open: %s — %s"), *Path,
			UTF8_TO_TCHAR(Error.c_str()));
		return false;
	}

	// PARSED FULLY BEFORE ANYTHING IS REPLACED. Clearing the list first and then
	// failing leaves an empty editor and a document that never existed — and the
	// parser refuses on an unknown kind or an unknown device precisely so that a
	// partial read cannot happen quietly.
	Segments.Reset(static_cast<int32>(Doc.Segments.size()));
	for (const FAuthoredSegment& A : Doc.Segments)
	{
		Segments.Add(FromAuthored(A));
	}

	RebuildFromSegments();

	// ===================== CLEAN AGAINST THE CANONICAL TEXT =====================
	//
	// `DidOpen` is given what this build would WRITE, not the bytes that were
	// read. That looks like the wrong way round and is not: `IsDirty` compares the
	// saved text against a re-serialisation, so storing the file's literal bytes
	// would report a hand-edited-but-equivalent file as modified the moment it
	// opened — different spacing, a field written in a different order, a number
	// spelled 30.0 instead of 30.
	//
	// The consequence is stated rather than hidden: saving a non-canonical file
	// normalises it. That is a diff somebody can read, once, instead of an unsaved
	// marker they can never clear.
	Session.DidOpen(TCHAR_TO_UTF8(*Path), TCHAR_TO_UTF8(*SerialiseDocument()));
	// A NEW DOCUMENT IS A NEW HISTORY. Undoing across an open would restore the
	// previous track's geometry into this one, which is not an edit anybody made
	// and not a state any file was ever in.
	ResetHistory();
	Browser.Touch(TCHAR_TO_UTF8(*Path));
	WriteRecentList();
	// A NEW DOCUMENT IS FRAMED WHOLE. Open lands in Ride, so the orbit is not
	// looked through until [Tab]; without this it showed wherever the previous
	// track had left it.
	bOrbitFramed = false;

	UE_LOG(LogTUEvents, Log, TEXT("opened %d segments from %s"), Segments.Num(), *Path);
	return true;
}

namespace
{
	// The segment list as the authored document, which is what history stores and
	// what the file stores. One conversion, used by both, so a step that is
	// restored is exactly a file that would have been written.
	FTrackDocument DocumentFrom(const TArray<FTUTrackSegment>& Segments)
	{
		FTrackDocument Doc;
		Doc.HeartlineHeight = 1.1;
		Doc.Segments.reserve(static_cast<std::size_t>(Segments.Num()));
		for (const FTUTrackSegment& S : Segments)
		{
			Doc.Segments.push_back(ToAuthored(S));
		}
		return Doc;
	}
}

void ATUCoasterRide::ResetHistory()
{
	History = MakeUnique<FTrackHistory>(DocumentFrom(Segments));
}

void ATUCoasterRide::PushHistory(const FString& Label, const FString& MergeKey)
{
	if (!History || bApplyingHistory)
	{
		return;
	}
	// `Commit` returns false and does nothing when the document is unchanged, so
	// a field re-entered with the same number costs no step — which is the whole
	// reason it compares canonical text rather than trusting the caller.
	History->Commit(DocumentFrom(Segments), TCHAR_TO_UTF8(*Label),
		TCHAR_TO_UTF8(*MergeKey));
}

void ATUCoasterRide::ApplyHistoryDocument(const FTrackDocument& Doc)
{
	// GUARDED, because this rebuilds and a rebuild is where an edit would normally
	// be recorded. Without the flag, undo would push the undone state as a new
	// step and the second press would take you forward again.
	bApplyingHistory = true;
	Segments.Reset(static_cast<int32>(Doc.Segments.size()));
	for (const FAuthoredSegment& A : Doc.Segments)
	{
		Segments.Add(FromAuthored(A));
	}
	// A restored document may not contain the segment that was selected, and a
	// selection past the end is an editor drawing a row that is not there.
	SelectedSegment = FMath::Clamp(SelectedSegment, -1, Segments.Num() - 1);
	// A RESTORED DOCUMENT MAY BE SHORTER. A selection holding indices past the end
	// is a panel drawing rows that are not there, and an undo is exactly when that
	// happens — the segment somebody had selected may be the one that came back.
	Selection.RemoveAll([this](int32 I) { return !Segments.IsValidIndex(I); });
	CancelField();
	RebuildFromSegments();
	bApplyingHistory = false;
}

void ATUCoasterRide::UndoEdit()
{
	if (!History || !History->CanUndo())
	{
		// SAID, not swallowed. A key that silently does nothing at the end of the
		// stack is indistinguishable from one that is broken — the same rule the
		// mode tabs and [Z] already follow.
		UE_LOG(LogTUEvents, Log, TEXT("undo: nothing further back"));
		return;
	}
	// The label describes the edit that PRODUCED the current state, which is the
	// one about to be reversed — so it is read before the index moves.
	const FString What(UTF8_TO_TCHAR(History->UndoLabel().c_str()));
	ApplyHistoryDocument(History->Undo());
	UE_LOG(LogTUEvents, Log, TEXT("undo: %s"), *What);
}

void ATUCoasterRide::RedoEdit()
{
	if (!History || !History->CanRedo())
	{
		UE_LOG(LogTUEvents, Log, TEXT("redo: nothing ahead"));
		return;
	}
	const FString What(UTF8_TO_TCHAR(History->RedoLabel().c_str()));
	ApplyHistoryDocument(History->Redo());
	UE_LOG(LogTUEvents, Log, TEXT("redo: %s"), *What);
}

void ATUCoasterRide::RefreshTrackList()
{
	// ===================== A TEMPLATE HAS A PICTURE TOO =====================
	//
	// The same walk the saved-track rows get, over the preset's own segments. A
	// template row was a name and a sentence; five layouts somebody is choosing
	// between are five shapes, and the shape is what they are choosing.
	TemplatePlans.Reset();
	for (std::size_t i = 0; i < NumTemplates(); ++i)
	{
		std::vector<float> Thumb;
		ETUPresetLayout P;
		if (PresetForTemplate(TemplateAt(i).Preset, P))
		{
			FTrackDocument Doc;
			for (const FTUTrackSegment& S : PresetLayout(P))
			{
				Doc.Segments.push_back(ToAuthored(S));
			}
			std::vector<float> Plan;
			for (const FTrackFrame& F : WalkTrack(::BuildTrack(Doc), 2.0))
			{
				Plan.push_back(static_cast<float>(F.Position.X));
				Plan.push_back(static_cast<float>(F.Position.Y));
			}
			Thumb = PlanThumb(Plan, 96);
		}
		TemplatePlans.Add(MoveTemp(Thumb));
	}

	// ONCE, ON THE WAY INTO THE MENU — never per frame. The menu is drawn
	// immediate-mode every frame, and reading and parsing every track file at
	// sixty hertz to draw a list of names is the kind of cost that is invisible
	// until somebody has thirty tracks.
	KnownTracks.clear();
	TrackPaths.clear();

	// RECENTS FIRST, because most-recent-first is the order somebody is looking
	// for. The folder then contributes anything not already listed, which is what
	// makes a track saved on another machine — or by an earlier install whose
	// recent list is gone — findable at all.
	for (std::size_t i = 0; i < Browser.NumRecent(); ++i)
	{
		TrackPaths.push_back(Browser.RecentAt(i));
	}

	TArray<FString> Found;
	IFileManager::Get().FindFiles(Found, *(TracksDir() / TEXT("*.track")), true, false);
	for (const FString& Leaf : Found)
	{
		const std::string Full = TCHAR_TO_UTF8(*(TracksDir() / Leaf));
		bool bAlready = false;
		for (const std::string& P : TrackPaths)
		{
			// The browser normalises three spellings of one path; this is the same
			// question one level up, and asking it here keeps a track that is both
			// recent and in the folder from appearing twice.
			bAlready = bAlready || FPaths::IsSamePath(
				FString(UTF8_TO_TCHAR(P.c_str())), FString(UTF8_TO_TCHAR(Full.c_str())));
		}
		if (!bAlready)
		{
			TrackPaths.push_back(Full);
		}
	}

	// ===================== WHAT EACH ONE IS, INCLUDING THE FAILURES =====================
	//
	// `Rows` marks anything it has no entry for as MISSING, which is right for a
	// file that cannot be read and WRONG for one that simply was not looked at —
	// and until now nothing looked at any of them, so the menu told you every
	// track you had ever opened was on a disconnected drive.
	for (const std::string& P : TrackPaths)
	{
		const FString Path(UTF8_TO_TCHAR(P.c_str()));
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *Path))
		{
			// Left out deliberately: no entry IS the missing row, and the browser
			// already writes that sentence better than a duplicate here would.
			continue;
		}

		FTrackEntry E;
		E.Path = P;
		E.Name = FTrackBrowser::FileNameOf(P);

		FTrackDocument Doc;
		std::string Error;
		if (!ParseTrackJson(TCHAR_TO_UTF8(*Text), Doc, Error))
		{
			// THE LOADER'S OWN REASON, carried to the row. "line 12 is wrong" and
			// "plug the drive back in" are different problems and the list is the
			// only place somebody will see either.
			E.Error = Error;
			KnownTracks.push_back(E);
			continue;
		}

		// ONE FORWARD WALK, not a sample loop. `EvaluateAt` is O(track length) per
		// call, so measuring height by evaluating every couple of metres is the
		// O(n^2) trap this project keeps having to unlearn — `WalkTrack` advances
		// a frame instead, which is linear.
		const FTrack Built = ::BuildTrack(Doc);
		E.LengthM = Built.TotalLength();
		double Lowest = 0.0, Highest = 0.0;
		bool bFirst = true;
		// AND THE THUMBNAIL COMES OUT OF THE SAME WALK. A plan view is X and Y
		// where the height is Z, so the picture is free at the point the numbers
		// are already being read — which is what makes it cheaper than a rendered
		// one by more than a constant factor. See PlanThumb.
		std::vector<float> Plan;
		for (const FTrackFrame& F : WalkTrack(Built, 2.0))
		{
			Lowest = bFirst ? F.Position.Z : std::fmin(Lowest, F.Position.Z);
			Highest = bFirst ? F.Position.Z : std::fmax(Highest, F.Position.Z);
			bFirst = false;
			Plan.push_back(static_cast<float>(F.Position.X));
			Plan.push_back(static_cast<float>(F.Position.Y));
		}
		E.HeightM = Highest - Lowest;
		E.Plan = PlanThumb(Plan);
		KnownTracks.push_back(E);
	}
	if (MenuWidget && MenuWidget->IsInViewport()) { MenuWidget->Rebuild(); }
}

FString ATUCoasterRide::RecentListPath() const
{
	return FPaths::ProjectSavedDir() / TEXT("RecentTracks.txt");
}

void ATUCoasterRide::LoadRecentList()
{
	FString Text;
	if (FFileHelper::LoadFileToString(Text, *RecentListPath()))
	{
		Browser.LoadRecent(TCHAR_TO_UTF8(*Text));
	}
	// Absent is a first run, not an error — the menu's empty state already says
	// what to do about it.
}

void ATUCoasterRide::WriteRecentList() const
{
	const std::string Text = Browser.SaveRecent();
	FFileHelper::SaveStringToFile(FString(UTF8_TO_TCHAR(Text.c_str())), *RecentListPath(),
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

void ATUCoasterRide::CheckDocumentRoundTrip() const
{
	// SAVE, OPEN, SAVE AGAIN — AND THE TWO FILES MUST BE THE SAME TEXT.
	//
	// The bridge between the editor struct and the authored model is the one part
	// of the save path with no engine-free test: `FromAuthored` cannot be asserted
	// in `Prototypes/`, because the type it produces is a UPROPERTY struct the
	// reflection system owns. So the check runs here, on the real preset, at the
	// only moment it costs nothing anybody notices.
	//
	// It is the same shape as the sensor layer's second means of detection: two
	// routes to one answer, and a disagreement is reported rather than resolved.
	//
	// ponytail: this sees the two directions that are new — a field the parser
	// drops and a field FromAuthored drops. It CANNOT see a field ToAuthored never
	// carried, because then the file never had it and both texts agree. The guard
	// for that one is the size static_assert in test_trackspline.cpp, which fires
	// when FAuthoredSegment changes; a new UPROPERTY that nobody adds to the
	// authored model at all is still only caught by review.
	const FString First = SerialiseDocument();
	if (First.IsEmpty())
	{
		return;   // already reported by the serialiser
	}

	FTrackDocument Back;
	std::string Error;
	if (!ParseTrackJson(TCHAR_TO_UTF8(*First), Back, Error))
	{
		UE_LOG(LogTUEvents, Warning,
			TEXT("document: this build wrote a file it cannot read back — %s"),
			UTF8_TO_TCHAR(Error.c_str()));
		return;
	}

	FTrackDocument Again;
	Again.HeartlineHeight = Back.HeartlineHeight;
	Again.Segments.reserve(Back.Segments.size());
	for (const FAuthoredSegment& A : Back.Segments)
	{
		// Through the editor struct and back, which is what an open actually does.
		Again.Segments.push_back(ToAuthored(FromAuthored(A)));
	}

	std::string SecondText;
	if (!WriteTrackJson(Again, SecondText, Error))
	{
		UE_LOG(LogTUEvents, Warning, TEXT("document: re-save refused — %s"),
			UTF8_TO_TCHAR(Error.c_str()));
		return;
	}

	if (FString(UTF8_TO_TCHAR(SecondText.c_str())) != First)
	{
		// LOUD, because the symptom otherwise is somebody's ride quietly opening
		// with a device missing — a file that loads, looks right, and cannot stop
		// a train.
		UE_LOG(LogTUEvents, Error,
			TEXT("document: OPENING AND RE-SAVING THIS TRACK WOULD CHANGE IT. "
				 "A field is being lost between the editor struct and the file."));
		return;
	}

	UE_LOG(LogTUEvents, Log, TEXT("document: %d segments survive a save and open unchanged"),
		Segments.Num());
}

bool ATUCoasterRide::RunDocumentSmokeTest()
{
	// EVERY CHECK RUNS, and the verdict is the AND of all of them. Returning at
	// the first failure would report one broken thing and hide the other eight,
	// which on a build somebody is waiting on is the difference between one
	// round trip and nine. The four below that DO return early are the ones
	// where continuing is meaningless -- if the document will not save, the
	// checks that open it are testing nothing.
	// NAMED, NOT COUNTED. A verdict of FAILED with nothing beside it sends
	// somebody back to read the whole log to find out which of nine checks it
	// was, which is the same defect as a validator that says a track is invalid.
	TArray<FString> Failures;
	// THE SMOKE TEST THE PACKAGING CARD ASKS FOR, or its first half: boot, write a
	// real file, read it back, and prove the ride on the other side is the same
	// one. `-TUSmokeTest` on the command line, so it never runs for a player.
	//
	// It goes through `SaveDocumentTo` and `OpenDocumentFrom` rather than round
	// -tripping the text in memory, because the parts it exists to cover are the
	// ones that only exist on disc: the encoding, the folder, and whether opening
	// really does replace the segment list with what was written.
	// ===================== IT TESTS THE DOCUMENT, SO IT SETS THE MODE =====================
	//
	// EditsAllowed() is true in BUILD and nowhere else, and this runs at BeginPlay
	// -- which since the boot fix means it runs in MAIN MENU, because a non-PIE
	// launch now boots the way a packaged build does. So insert, remove and the
	// multi-select commit were all correctly REFUSED, and three checks failed on a
	// ride that is fine.
	//
	// Not a workaround: a test of the document layer that inherits whatever mode
	// the shell happens to be in is a test that reports different things on
	// different launches, which is what it did -- it passed before the boot change
	// and failed after, with nothing about the document altered. The mode is an
	// input, so it gets stated rather than inherited.
	// ===================== AND IT PUTS EVERYTHING BACK =====================
	//
	// This writes real files into the real folders, which is the point -- the
	// encoding and the browser's four row states only exist on disc. But those
	// folders are the PLAYER'S: SmokeTest, SmokeInFolder and SmokeBroken were
	// still sitting in the track list afterwards, and two of them had been added
	// to the persistent recent list, so a test run left three tracks in somebody's
	// menu that they never made and cannot explain.
	//
	// The recent list is snapshotted rather than edited back out, because the test
	// touches it through several paths and remembering which is how one gets
	// missed.
	FString RecentBefore;
	const bool bHadRecent = FFileHelper::LoadFileToString(RecentBefore, *RecentListPath());

	Session.Enter(EAppMode::Build, /*bConfirmed*/ true);

	// ===================== AND THE DOCUMENT IS AN INPUT TOO =====================
	//
	// The paragraph above says the mode is an input, so it gets stated rather than
	// inherited. THE DOCUMENT IS THE SAME KIND OF THING and was inherited until
	// the first packaged run measured it: in PIE the level IS the document, so
	// this found 33 segments; a packaged build boots to the MENU with nothing
	// open, so it found ZERO -- and every check below is sized to what is loaded.
	//
	// What that cost is the whole point. The round trip compared an empty document
	// with an empty document and reported "saved, opened and unchanged"; the undo
	// check is guarded on `Num() > 0` and the multi-select on `>= 3`, so both were
	// SKIPPED IN SILENCE. The packaged build printed PASSED having exercised
	// neither, which is this project's oldest failure wearing new clothes: a check
	// that cannot tell success from never-having-run gets believed.
	//
	// Stating it makes the packaged run and the PIE run measure the same thing,
	// which is the only way the packaged one is evidence about anything.
	StartFromTemplate(1);   // the launched circuit: launch, brakes, a station
	if (Segments.Num() < 3)
	{
		// The guards below stay, but they must never again be the reason a check
		// does not run. A template that cannot fill them is a broken input, and a
		// broken input is a failure rather than a quiet reduction in coverage.
		UE_LOG(LogTUEvents, Error,
			TEXT("smoke: the stated document has %d segments — too few to test undo or multi-select"),
			Segments.Num());
		Failures.Add(TEXT("stated document"));
	}

	const FString Path = FPaths::ProjectSavedDir() / TEXT("SmokeTest.track");
	const int32 Before = Segments.Num();
	const FString TextBefore = SerialiseDocument();

	if (!SaveDocumentTo(Path))
	{
		UE_LOG(LogTUEvents, Error, TEXT("smoke: save failed"));
		return false;
	}
	if (!OpenDocumentFrom(Path))
	{
		UE_LOG(LogTUEvents, Error, TEXT("smoke: opening what we just wrote failed"));
		return false;
	}

	const FString TextAfter = SerialiseDocument();
	if (Segments.Num() != Before || TextAfter != TextBefore)
	{
		UE_LOG(LogTUEvents, Error,
			TEXT("smoke: the ride changed across a save and open — %d segments became %d"),
			Before, Segments.Num());
		return false;
	}
	// AND THE SESSION AGREES IT IS SAVED. A file on disc that the shell still
	// reports as unsaved is the dirty-marker bug arriving from the other side.
	if (Session.IsDirty() || !Session.HasPath())
	{
		UE_LOG(LogTUEvents, Error,
			TEXT("smoke: saved and opened, but the session does not think so"));
		return false;
	}
	UE_LOG(LogTUEvents, Log,
		TEXT("smoke: %d segments saved, opened and unchanged; document is %s, clean"),
		Segments.Num(), *FPaths::GetCleanFilename(UTF8_TO_TCHAR(Session.Path().c_str())));

	// ===================== UNDO, THERE AND BACK =====================
	//
	// An edit, a step back, a step forward. Asserted on the TEXT rather than on a
	// segment count, because the failure worth catching is a field that does not
	// survive the round trip through the document — which a count cannot see, and
	// which is exactly how the device layer went missing from the save format.
	if (Segments.Num() > 0)
	{
		const FString Original = SerialiseDocument();
		Segments[0].Length += 7.5f;
		RebuildFromSegments();
		PushHistory(TEXT("smoke edit"));
		const FString Edited = SerialiseDocument();

		UndoEdit();
		const bool bBack = SerialiseDocument() == Original;
		RedoEdit();
		const bool bForward = SerialiseDocument() == Edited;
		UndoEdit();   // leave it as it was found

		if (Edited == Original || !bBack || !bForward)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: undo/redo did not round-trip (changed %d, back %d, forward %d)"),
				Edited != Original, bBack, bForward);
			Failures.Add(TEXT("undo/redo"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: an edit undoes and redoes exactly; %d steps deep"),
				static_cast<int32>(History ? History->Depth() : 0));
		}
	}

	// ===================== INSERT AND REMOVE, WHICH UNDO HAS TO COVER =====================
	//
	// Remove is unconfirmed, and that is only defensible if undo genuinely brings
	// the segment back — so it is asserted rather than assumed. Insert then remove
	// is the pair that also proves the selection does not end up past the end.
	{
		const int32 CountBefore = Segments.Num();
		SelectedSegment = 0;
		InsertSegment();
		const bool bGrew = Segments.Num() == CountBefore + 1;
		RemoveSegment();
		const bool bShrank = Segments.Num() == CountBefore;
		UndoEdit();   // the remove
		const bool bCameBack = Segments.Num() == CountBefore + 1;
		UndoEdit();   // the insert
		const bool bGone = Segments.Num() == CountBefore;

		if (!bGrew || !bShrank || !bCameBack || !bGone)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: insert/remove/undo is wrong (grew %d, shrank %d, back %d, gone %d)"),
				bGrew, bShrank, bCameBack, bGone);
			Failures.Add(TEXT("insert/remove"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: a segment inserts, removes, and undo brings it back"));
		}
	}

	// ===================== ONE NUMBER INTO SEVERAL SEGMENTS =====================
	//
	// The point of multi-select, asserted rather than assumed: a field typed once
	// lands on every selected segment that shares it, and one undo takes them all
	// back. A per-segment undo step would be eight presses to reverse one edit,
	// which is the failure the merge key exists to prevent one level up.
	if (Segments.Num() >= 3)
	{
		const float L0 = Segments[0].Length;
		const float L1 = Segments[1].Length;

		SelectSegment(0, false);
		SelectSegment(1, true);
		const bool bBoth = Selection.Num() == 2;

		FocusedField = EEditField::Length;
		FieldBuffer = TEXT("13.5");
		CommitField();

		const bool bWrote = FMath::IsNearlyEqual(Segments[0].Length, 13.5f)
			&& FMath::IsNearlyEqual(Segments[1].Length, 13.5f);
		UndoEdit();
		const bool bBack = FMath::IsNearlyEqual(Segments[0].Length, L0)
			&& FMath::IsNearlyEqual(Segments[1].Length, L1);

		SelectSegment(0, false);
		if (!bBoth || !bWrote || !bBack)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: multi-select is wrong (two %d, wrote %d, undone %d)"),
				bBoth, bWrote, bBack);
			Failures.Add(TEXT("multi-select"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: one number typed into two segments, and one undo takes both back"));
		}
	}

	// ===================== A PRESET BRINGS ITS EVACUATION ROUTE =====================
	//
	// Every shipped preset authored NO walkways, so the evacuation model — whose
	// whole job is deciding whether a stopped train can be reached — had no route
	// to reason about on any ride here. Asserted on a real template rather than
	// reasoned about, because "the spans are empty" is exactly the kind of nothing
	// that goes unnoticed.
	{
		StartFromTemplate(1);   // the launched circuit: launch, brakes, a station
		const bool bSpans = Walkways.Num() > 0;
		const bool bDerived = WalkwaySpans.size() == static_cast<std::size_t>(Walkways.Num());
		double Catwalked = 0.0;
		for (const FTUWalkway& W : Walkways) { Catwalked += W.EndS - W.StartS; }

		if (!bSpans || !bDerived)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: preset walkways are wrong (authored %d, derived %d)"),
				Walkways.Num(), static_cast<int32>(WalkwaySpans.size()));
			Failures.Add(TEXT("preset walkways"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: the preset catwalks %d powered runs on the %s, %.0f m of route"),
				Walkways.Num(), WalkwaySideName(PresetWalkwaySide), Catwalked);
		}

		// AND EVERY DEVICE IS DRESSED. A launch, a station and brakes are on this
		// template; hardware on none of them is the gap this closes, and a
		// rebuild that produced no triangles would be silent from the cockpit.
		if (DeviceTriangles <= 0 || CatchSpanList.Num() == 0)
		{
			UE_LOG(LogTUEvents, Error, TEXT("smoke: device hardware is missing (%d triangles, %d catch spans)"),
				DeviceTriangles, CatchSpanList.Num());
			Failures.Add(TEXT("device hardware"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log, TEXT("smoke: %d zones and %d anti-rollback spans wear their hardware, %d triangles"),
				ZoneSpans.Num(), CatchSpanList.Num(), DeviceTriangles);
		}
		if (StationTriangles <= 0)
		{
			UE_LOG(LogTUEvents, Error, TEXT("smoke: the station has no platform"));
			Failures.Add(TEXT("station platform"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log, TEXT("smoke: the station has its platform, stripe, gates and cabinet, %d triangles"),
				StationTriangles);
		}

		// ===================== AND NONE AUTHORS NOTHING =====================
		//
		// The branch worth checking, because the other settings differ only in
		// which vertices come out: this one decides whether a route EXISTS. A
		// version that authored spans with a side of None instead of returning
		// would pass every geometric check here and still leave the evacuation
		// model believing in walkways nobody can see.
		const ETUWalkway Was = PresetWalkwaySide;
		PresetWalkwaySide = ETUWalkway::None;
		ApplyPresetWalkways();
		const bool bNoRoute = Walkways.Num() == 0;
		PresetWalkwaySide = Was;
		ApplyPresetWalkways();   // put the preset back as it was found

		if (!bNoRoute || Walkways.Num() == 0)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: walkway side None is wrong (authored %d with None, %d after)"),
				bNoRoute ? 0 : 1, Walkways.Num());
			Failures.Add(TEXT("walkway side"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: side None authors no route at all, and it comes back"));
		}
	}

	// ===================== AND THE TRAIN IS A TRAIN =====================
	//
	// The payoff for the physics, the signalling, the envelope and the meshing was
	// nine grey cubes, and this is the check that it is not any more. Asserted
	// rather than eyeballed for the reason every other smoke assertion exists: a
	// train that came out inside out, or with the wrong number of cars, is silent
	// from the cockpit and obvious in a number.
	{
		const FTrainSettings TS = TrainMeshSettings();
		const FTrainMesh CarMesh = BuildCarMesh(TS, Track.GetHeartlineHeight(), Profile);

		// A CAR IS NOT A SAMPLE POINT. Nine is what the gravity integration needs;
		// CarCount is what somebody authored, and the two are allowed to differ.
		const int32 Points = Trains.Num() > 0 ? Trains[0]->NumSamplePoints() : 0;
		const std::vector<FCarPlacement> Placed = TrainPath.empty()
			? std::vector<FCarPlacement>()
			: PlaceCars(TrainPath, TrainPathSpacing, TrainPathTotal,
				TrainPathTotal * 0.5, TS, bTrackIsCircuit);

		// Watertight and outward-wound, which is what a solid body would have shown
		// about the port rule years earlier than the support pier did.
		double V6 = 0.0;
		for (std::size_t t = 0; t + 2 < CarMesh.Body.Index.size(); t += 3)
		{
			V6 += Dot(CarMesh.Body.Position[CarMesh.Body.Index[t]],
				Cross(CarMesh.Body.Position[CarMesh.Body.Index[t + 1]],
					CarMesh.Body.Position[CarMesh.Body.Index[t + 2]]));
		}
		const double BodyVolume = V6 / 6.0;

		const std::vector<FMeshFinding> TrainFindings =
			AuditTrain(TS, Track.GetHeartlineHeight(), Profile, Track.TotalLength());

		const bool bOk = CarMesh.NumTriangles() > 0
			&& BodyVolume > 0.0
			&& static_cast<int32>(Placed.size()) == TS.CarCount
			&& TrainFindings.empty();
		if (!bOk)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: the train mesh is wrong (%d tris, body volume %.6f, %d cars ")
				TEXT("placed for %d authored, %d findings)"),
				static_cast<int32>(CarMesh.NumTriangles()), BodyVolume,
				static_cast<int32>(Placed.size()), TS.CarCount,
				static_cast<int32>(TrainFindings.size()));
			for (const FMeshFinding& F : TrainFindings)
			{
				UE_LOG(LogTUEvents, Error, TEXT("smoke:   %s"), UTF8_TO_TCHAR(F.What.c_str()));
			}
			Failures.Add(TEXT("train mesh"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: the train is %d cars of %.1f m, not %d sample points -- ")
				TEXT("%d triangles a car, body encloses %.3f m3, silent"),
				TS.CarCount, TS.CarLengthM, Points,
				static_cast<int32>(CarMesh.NumTriangles()), BodyVolume);
		}
	}

	// ===================== MENU -> NEW -> AN EMPTY LAYOUT =====================
	//
	// The whole first-run path in one check: a blank template leaves NO segments,
	// lands in Build, and is CLEAN -- nobody has edited anything yet. That last
	// one is the part that was wrong: the baseline was taken before the rebuild,
	// so every new document reported unsaved changes from its first frame.
	//
	// And an empty track has to be somewhere you can start: [I] on nothing is the
	// only way to get a first segment, so it is asserted rather than assumed.
	{
		const std::size_t Blank = NumTemplates() - 1;   // the blank one is last
		StartFromTemplate(static_cast<int32>(Blank));

		const bool bEmpty = Segments.Num() == 0;
		const bool bBuild = Session.Mode() == EAppMode::Build;
		const bool bClean = !Session.IsDirty();

		SelectedSegment = -1;
		InsertSegment();
		const bool bFirst = Segments.Num() == 1;
		UndoEdit();
		const bool bGone = Segments.Num() == 0;

		if (!bEmpty || !bBuild || !bClean || !bFirst || !bGone)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: new-blank is wrong (empty %d, build %d, clean %d, first %d, undone %d)"),
				bEmpty, bBuild, bClean, bFirst, bGone);
			Failures.Add(TEXT("new-blank"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: new blank opens empty, in BUILD, clean, and [I] gives it a first segment"));
		}
	}

	// ===================== AND THE SHIPPING PATH CAN AUTHOR A CURVE =====================
	//
	// PROJECT_PLAN gives Phase 1 the gate "build an arbitrary coaster from scratch
	// in-editor". That was met by the DETAILS PANEL -- the developer path -- and the
	// shipping one had never met it with the whole Phase 1 list ticked: `EEditField`
	// had no Kind entry, so blank plus [I] gave straights for ever, and every curve
	// on every shipped track came from a preset or from the Details panel.
	//
	// So this asserts the END of that path rather than the plumbing: start with
	// nothing, add a piece, make it a turn, and the track has to actually TURN.
	{
		StartFromTemplate(static_cast<int32>(NumTemplates() - 1));
		SelectedSegment = -1;
		InsertSegment();

		// RAW IS NEVER REACHED, however long somebody sits on the row. Eight cycles
		// is twice round the four authorable kinds.
		bool bNeverRaw = true;
		ETUSegmentKind Walk = ETUSegmentKind::Straight;
		for (int32 i = 0; i < 8; ++i)
		{
			Walk = NextAuthorableKind(Walk);
			if (Walk == ETUSegmentKind::Raw) { bNeverRaw = false; }
		}
		// ... and a raw segment cycles OUT rather than being stuck in a kind the
		// panel has no fields for.
		const bool bRawEscapes =
			NextAuthorableKind(ETUSegmentKind::Raw) == ETUSegmentKind::Straight;

		// Cycle to Arc exactly as a click would, then author it.
		FTUTrackSegment& S0 = Segments[0];
		int32 Guard = 0;
		while (S0.Kind != ETUSegmentKind::Arc && Guard++ < 8)
		{
			S0.Kind = NextAuthorableKind(S0.Kind);
		}
		WriteField(S0, EEditField::Length, 40.0);
		WriteField(S0, EEditField::Radius, 25.0);
		WriteField(S0, EEditField::RollStart, 5.0);
		WriteField(S0, EEditField::Roll, 25.0);
		RebuildFromSegments();

		// THE TRACK HAS TO TURN. A kind field that set a value nothing downstream
		// read would pass every check above this line, so the assertion is on the
		// geometry rather than on the field.
		double TurnedDeg = 0.0;
		if (Track.TotalLength() > 1.0)
		{
			const FTrackFrame A = Track.EvaluateAt(0.0);
			const FTrackFrame B = Track.EvaluateAt(Track.TotalLength());
			const double Dot = FMath::Clamp(
				A.Tangent.X * B.Tangent.X + A.Tangent.Y * B.Tangent.Y
				+ A.Tangent.Z * B.Tangent.Z, -1.0, 1.0);
			TurnedDeg = FMath::RadiansToDegrees(FMath::Acos(Dot));
		}

		// ROLL IS A PAIR, and only the END of it used to be writable -- which made a
		// hand-authored bank start at 0 and STEP at every joint, exactly what
		// TrackValidate then complained about.
		const bool bRollPair =
			FMath::IsNearlyEqual(ReadField(S0, EEditField::RollStart), 5.0, 1e-4)
			&& FMath::IsNearlyEqual(ReadField(S0, EEditField::Roll), 25.0, 1e-4);

		if (!bNeverRaw || !bRawEscapes || S0.Kind != ETUSegmentKind::Arc
			|| TurnedDeg < 45.0 || !bRollPair)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: the shipping editor cannot author (never-raw %d, raw-escapes %d, ")
				TEXT("arc %d, turned %.1f deg, roll pair %d)"),
				bNeverRaw, bRawEscapes, S0.Kind == ETUSegmentKind::Arc, TurnedDeg, bRollPair);
			Failures.Add(TEXT("authoring"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: from a blank track, [I] then the kind row gives a real arc -- ")
				TEXT("%.1f m turning %.1f deg, banked %.0f to %.0f, and raw is never cycled into"),
				Track.TotalLength(), TurnedDeg,
				ReadField(S0, EEditField::RollStart), ReadField(S0, EEditField::Roll));
		}
	}

	// ===================== THE SHOWCASE ACTUALLY RUNS =====================
	//
	// It is derived from a proven closed circuit and every change was chosen to be
	// closure-safe, which is an argument rather than a measurement. This is the
	// measurement: it closes, it completes, and the two devices it exists to
	// demonstrate are really there.
	//
	// A showcase that stalled its train, or whose circuit missed the seam by a
	// metre, would demonstrate the opposite of what it is for -- and both are
	// silent from the cockpit.
	{
		Preset = ETUPresetLayout::Showcase;
		Segments = PresetLayout(Preset);
		ApplyPresetTrainSetup(Preset);
		ApplyPresetWalkways();
		RebuildFromSegments();

		// ---- AND EVERY TRAIN CAN COME OFF ----
		//
		// SHED in the maintenance panel can now empty a track that still has places
		// to park on, which the old floor of 1 made impossible -- and the placement
		// code indexed Trains[0] on exactly that combination. Asserted here because
		// it is a crash rather than a wrong number, and because it is a state a
		// button reaches in one press.
		// A CAR COUNT THAT DOES NOT REPRODUCE THE MEASURED TRAIN is the one way
		// this refactor could be silently wrong: every G figure in the docs was
		// taken on a 15 m or a 6 m train, and cars are only a better spelling of
		// those if they come to the same number.
		const bool bTrainExact = FMath::IsNearlyEqual(TrainLengthM, 6.f, 0.001f)
			&& CarCount == 2;

		const int32 FullService = TrainCount;
		TrainCount = 0;
		RebuildFromSegments();
		const bool bEmptied = Trains.Num() == 0;
		TrainCount = FullService;
		RebuildFromSegments();
		const bool bReturned = Trains.Num() == FullService;

		int32 Pads = 0, Trims = 0;
		for (const FTUTrackSegment& S : Segments)
		{
			if (S.ZoneBrakeDecel > 0.f) { ++Pads; }
			if (S.Zone == ETUSegmentZone::Brake) { ++Trims; }
		}

		// AND IT IS WITHIN THE ENVELOPE, at the centre and at the outer seat.
		// Judged for real (both verdicts run on a completed ride and their
		// findings land in the panel), so a showcase that snapped an outboard
		// rider through its helix would fail here rather than demonstrate it.
		int32 EnvelopeRows = 0;
		for (std::size_t i = 0; i < Diagnostics.Num(); ++i)
		{
			if (Diagnostics.At(i).Group == "Envelope") { ++EnvelopeRows; }
		}
		const FGVerdict Outer = JudgeRideProfile(OffsetProfile(Profile_,
			TrainMeshSettings().BodyWidthM * 0.25));
		const bool bJudged = Outer.SamplesJudged > 0;

		if (!bTrackIsCircuit || !Profile_.bCompleted || Pads < 2 || Trims != 1
			|| !bEmptied || !bReturned || !bTrainExact || EnvelopeRows != 0 || !bJudged)
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: the showcase is wrong (circuit %d, completed %d, pads %d, ")
				TEXT("trims %d, shed to zero %d, returned %d, train exact %d, envelope rows %d, judged %d)"),
				bTrackIsCircuit, Profile_.bCompleted, Pads, Trims, bEmptied, bReturned, bTrainExact,
				EnvelopeRows, bJudged);
			Failures.Add(TEXT("showcase"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log,
				TEXT("smoke: the showcase closes and runs -- %d segments, %.1f m, ")
				TEXT("top %.1f km/h, %.0f s, %.2f..%+.2f g vertical, %.2f lateral, ")
				TEXT("%d friction pad(s), %d trim, %d x %.1f m cars, sheds to zero and back"),
				Segments.Num(), Track.TotalLength(), Profile_.TopSpeed * 3.6,
				Profile_.Duration, Profile_.MinVerticalG, Profile_.MaxVerticalG,
				Profile_.MaxAbsLateralG, Pads, Trims, CarCount, CarLengthM);
		}
	}

	// ===================== AND WHAT THE MENU WOULD LIST =====================
	//
	// The three states a row can be in, made to happen rather than reasoned about:
	// a track that loads, one that will not parse, and one that is not there. The
	// failure rows are the ones nobody exercises by hand, and they are the whole
	// reason the browser keeps a file in the list instead of pruning it.
	SaveDocumentTo(TracksDir() / TEXT("SmokeInFolder.track"));
	FFileHelper::SaveStringToFile(FString(TEXT("{\"segments\": [{\"kind\": \"banana\"}]}")),
		*(TracksDir() / TEXT("SmokeBroken.track")),
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
	Browser.Touch("Z:/gone/SmokeMissing.track");

	RefreshTrackList();
	for (const FTrackEntry& E : FTrackBrowser::Rows(KnownTracks, TrackPaths))
	{
		UE_LOG(LogTUEvents, Log, TEXT("smoke: row \"%s\" — %s"),
			UTF8_TO_TCHAR(E.Name.c_str()),
			UTF8_TO_TCHAR(FTrackBrowser::Subtitle(E).c_str()));
	}

	// Swept AFTER the rows above, which are the whole reason these exist.
	IFileManager& Files = IFileManager::Get();
	Files.Delete(*(FPaths::ProjectSavedDir() / TEXT("SmokeTest.track")), false, true, true);
	Files.Delete(*(TracksDir() / TEXT("SmokeInFolder.track")), false, true, true);
	Files.Delete(*(TracksDir() / TEXT("SmokeBroken.track")), false, true, true);
	if (bHadRecent)
	{
		FFileHelper::SaveStringToFile(RecentBefore, *RecentListPath(),
			FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
	}
	else
	{
		// It did not exist before, so leaving an empty one behind is still litter.
		Files.Delete(*RecentListPath(), false, true, true);
	}
	RefreshTrackList();

	// ===================== REBINDING, END TO END =====================
	//
	// The settings page's key capture cannot be driven from here (it is UMG
	// and needs a real key event), so what is proved is everything UNDER it:
	// a rebind moves the LIVE chord, is written to the keys file, a conflict
	// with another action is reported, an F key is refused, and the default
	// restores. Done on [N], the newest binding, and put back after.
	{
		FString SavedKeys;
		const bool bHadKeys = FFileHelper::LoadFileToString(SavedKeys, *KeyBindingsPath());
		auto LiveKeyFor = [this](const FString& Action) -> FString
		{
			const TArray<int32>* Idx = ActionBindingIndex.Find(Action);
			if (!Idx || Idx->Num() == 0 || !InputComponent) { return TEXT("?"); }
			return InputComponent->KeyBindings[(*Idx)[0]].Chord.Key.GetFName().ToString();
		};
		const bool bMoved = RebindKey(TEXT("key.seat"), TEXT("Y")) && LiveKeyFor(TEXT("key.seat")) == TEXT("Y");
		FString Written;
		FFileHelper::LoadFileToString(Written, *KeyBindingsPath());
		const bool bWritten = Written.Contains(TEXT("key.seat=Y"));
		// Onto [T], which is "Next train": both rows must report the other.
		RebindKey(TEXT("key.seat"), TEXT("T"));
		int32 Reported = 0;
		for (const FBindingConflict& C : Bindings.Conflicts())
		{
			if ((C.ActionA == "key.seat" && C.ActionB == "key.train")
				|| (C.ActionA == "key.train" && C.ActionB == "key.seat")) { ++Reported; }
		}
		const bool bRefusedF = !RebindKey(TEXT("key.seat"), TEXT("F5")) && LiveKeyFor(TEXT("key.seat")) == TEXT("T");
		const bool bRestored = RebindKey(TEXT("key.seat"), TEXT("N")) && LiveKeyFor(TEXT("key.seat")) == TEXT("N")
			&& Bindings.Conflicts().empty();
		if (!(bMoved && bWritten && Reported == 1 && bRefusedF && bRestored))
		{
			UE_LOG(LogTUEvents, Error,
				TEXT("smoke: rebinding is wrong (moved %d, written %d, conflicts %d, refused F %d, restored %d)"),
				bMoved, bWritten, Reported, bRefusedF, bRestored);
			Failures.Add(TEXT("rebind"));
		}
		else
		{
			UE_LOG(LogTUEvents, Log, TEXT("smoke: [N] rebinds live, persists, reports a clash with [T], refuses F5, restores"));
		}
		if (bHadKeys) { FFileHelper::SaveStringToFile(SavedKeys, *KeyBindingsPath()); }
		else { Files.Delete(*KeyBindingsPath(), false, true, true); }
	}

	if (Failures.Num() > 0)
	{
		UE_LOG(LogTUEvents, Error, TEXT("smoke: %d of 11 checks failed: %s"),
			Failures.Num(), *FString::Join(Failures, TEXT(", ")));
	}
	return Failures.Num() == 0;
}

FString ATUCoasterRide::ShellSettingsPath() const
{
	// Saved/ rather than Config/: the engine owns Config, and a file of ours in it
	// is one `.ini` away from looking like something the engine will parse.
	return FPaths::ProjectSavedDir() / TEXT("TrackUnlimited.cfg");
}

void ATUCoasterRide::LoadShellSettings()
{
	// DECLARED FIRST, ALWAYS. The store tells a declared key from an unknown one by
	// whether it has a default, and that distinction is the whole of "unknown keys
	// survive" — load before declaring and every setting we own is filed as
	// somebody else's and written back to the bottom of the file.
	DeclareSchemaDefaults(ShellSettings);

	// ===================== AND THE LEVEL'S VALUE IS THE DEFAULT =====================
	//
	// Two of these live on the actor as authored UPROPERTYs, so the table's figure
	// is not the truth about this ride — somebody who set the scan rate to 480 in
	// the Details panel has said what they want, and applying the table's 240 over
	// the top at BeginPlay would silently undo it. That is exactly the failure "a
	// default is not a value" exists to prevent, arriving from the level instead of
	// from an old config file.
	//
	// Re-declaring rather than special-casing the apply: the default becomes the
	// authored value, so a settings screen that has never been touched displays
	// what the ride is actually doing, applying it is a no-op, and an explicit
	// choice still wins. One rule, no exception list.
	ShellSettings.Declare("sim.scanHz", std::to_string(SimHz));
	ShellSettings.Declare("input.sensitivity", std::to_string(LookSensitivity));
	ShellSettings.Declare("input.orbitInvertX", bOrbitInvertX ? "true" : "false");
	ShellSettings.Declare("input.orbitInvertY", bOrbitInvertY ? "true" : "false");
	ShellSettings.Declare("input.flyInvertX", bFlyInvertX ? "true" : "false");
	ShellSettings.Declare("input.flyInvertY", bFlyInvertY ? "true" : "false");

	FString Text;
	if (FFileHelper::LoadFileToString(Text, *ShellSettingsPath()))
	{
		ShellSettings.Load(TCHAR_TO_UTF8(*Text));
		UE_LOG(LogTUEvents, Log, TEXT("settings: %d explicit, %d kept from a newer build"),
			static_cast<int32>(ShellSettings.NumExplicit()),
			static_cast<int32>(ShellSettings.NumUnknownKept()));
	}
	// NO FILE IS NOT AN ERROR. It is a first run, and it is also what the store is
	// designed for: everything resolves to a default until somebody changes
	// something, so there is nothing to write and nothing to warn about.

	// The bindings, from the same table but into the OTHER store — the settings
	// file must never contain a `key.` line, which is what keeps a binding from
	// having two homes.
	for (const FSettingEntry& E : SettingsSchema())
	{
		// Only where nothing is bound yet: the keys file may already have been
		// applied, and Bind REPLACES.
		if (E.Kind == ESettingKind::Key && Bindings.KeyFor(E.Key).empty())
		{
			Bindings.Bind(E.Key, E.Default);
		}
	}

	ApplyShellSettings();
}

void ATUCoasterRide::WriteShellSettings() const
{
	const std::string Text = ShellSettings.Save();
	if (!FFileHelper::SaveStringToFile(FString(UTF8_TO_TCHAR(Text.c_str())),
		*ShellSettingsPath()))
	{
		// SAID, because the alternative is a settings screen that appears to work
		// and loses everything at exit — the failure this whole file exists to
		// prevent, arriving from the one direction the store cannot see.
		UE_LOG(LogTUEvents, Warning, TEXT("settings: could not write %s"),
			*ShellSettingsPath());
	}
}

void ATUCoasterRide::ApplyShellSettings()
{
	// ONLY THE SETTINGS WITH A LIVE CONSUMER. The audio buses have nowhere to go
	// yet and say so on their own rows; `sim.units` is a display concern nothing
	// reads. Both are stored and neither is pretended about.

	// THE SCAN RATE IS RESTART-FLAGGED AND THIS IS THE RESTART. Changing it while
	// the ride is running would move every rate in the control system mid-lap —
	// edge detection, restraint travel, drive ramps — and the run either side of
	// the change would not be one run. So it is taken before the first scan and
	// ignored after, and the row says restart.
	if (!bScanStarted)
	{
		SimHz = FMath::Clamp(
			static_cast<int32>(ShellSettings.GetNumber("sim.scanHz")), 30, 1000);
	}

	// The session owns WHEN an autosave happens; this is the interval it uses.
	Session.SetAutosaveSeconds(ShellSettings.GetNumber("sim.autosaveSeconds"));

	bPauseWhenUnfocused = ShellSettings.GetBool("sim.pauseUnfocused");
	LookSensitivity = FMath::Clamp(
		static_cast<float>(ShellSettings.GetNumber("input.sensitivity")), 0.25f, 4.f);
	bOrbitInvertX = ShellSettings.GetBool("input.orbitInvertX");
	bOrbitInvertY = ShellSettings.GetBool("input.orbitInvertY");
	bFlyInvertX = ShellSettings.GetBool("input.flyInvertX");
	bFlyInvertY = ShellSettings.GetBool("input.flyInvertY");
}

void ATUCoasterRide::CheckBindingsAgainstInput(const UInputComponent* Input) const
{
	if (!Input)
	{
		return;
	}

	// What the input component genuinely answers to, gathered once. The axis
	// bindings are deliberately not in here: the schema lists actions, and the two
	// mouse axes are not actions anybody rebinds.
	TSet<FName> Live;
	for (const FInputKeyBinding& B : Input->KeyBindings)
	{
		Live.Add(B.Chord.Key.GetFName());
	}

	int32 Missing = 0;
	for (const FSettingEntry& E : SettingsSchema())
	{
		if (E.Kind != ESettingKind::Key) { continue; }
		const FName Named(UTF8_TO_TCHAR(E.Default.c_str()));
		if (!Live.Contains(Named))
		{
			// THE PAGE IS THE THING THAT IS WRONG, whichever half moved. A row
			// promising a key nothing answers to is a control somebody will press
			// and conclude the application is broken — which is exactly what F2 did.
			UE_LOG(LogTUEvents, Warning,
				TEXT("controls: the settings page offers [%s] for \"%s\", and nothing is bound to it"),
				*Named.ToString(), UTF8_TO_TCHAR(E.Label.c_str()));
			++Missing;
		}
	}

	// AND THE SILENCE IS THE RESULT. A checker that says nothing when all is well
	// is one people leave switched on; this logs the agreement once, at Log level,
	// because a cross-check never seen to pass is indistinguishable from one that
	// is not running.
	if (Missing == 0)
	{
		UE_LOG(LogTUEvents, Log,
			TEXT("controls: every key the settings page offers is bound"));
	}
}

void ATUCoasterRide::BeginPlay()
{
	Super::BeginPlay();

	// BEFORE ANYTHING ELSE READS ONE. The scan rate is restart-flagged, and this
	// is the only moment it can be applied — RebuildFromSegments and the first
	// Tick are both downstream of it.
	// ===================== THE LEVEL'S TRAIN IS STILL THE LEVEL'S =====================
	//
	// TrainLengthM became derived from cars, and an actor already placed in a
	// level carries a serialised length and no car count -- so without this, a
	// level authored with a 6 m train silently gets the 15 m default the moment
	// the new fields appear. Same trap as the preset enum, arriving through a
	// different door.
	//
	// The same rule the settings loader already applies to SimHz: THE LEVEL'S
	// AUTHORED VALUE IS THE DEFAULT. Cars are back-filled from it rather than the
	// length being overwritten, so the ride is unchanged and the new fields
	// describe what it was already doing.
	if (CarLengthM > 0.f && TrainLengthM > 0.f
		&& !FMath::IsNearlyEqual(TrainLengthM, CarCount * CarLengthM, 0.01f))
	{
		CarCount = FMath::Max(1, FMath::RoundToInt(TrainLengthM / CarLengthM));
		UE_LOG(LogTUEvents, Log,
			TEXT("train: %.1f m authored in the level reads as %d car(s) of %.1f m"),
			TrainLengthM, CarCount, CarLengthM);
	}

	LoadShellSettings();
	LoadRecentList();
	LoadBrakeSounds();

	RebuildFromSegments();

	// THE BASELINE, so an untouched session is CLEAN rather than dirty from the
	// first frame. `RebuildFromSegments` has just told the session what the
	// document is; this says that is also what is saved.
	//
	// `DidCreateNew` rather than `DidOpen` because there is no path: the actor is
	// placed in a level and THE LEVEL IS THE DOCUMENT, which is the same reason
	// the session skips Boot here. Somebody who edits a segment now gets the
	// unsaved marker in the frame, which is true — nothing on disc matches it.
	Session.DidCreateNew(TCHAR_TO_UTF8(*SerialiseDocument()));
	ResetHistory();
	CheckDocumentRoundTrip();

	// ===================== WHERE THE APPLICATION STARTS =====================
	//
	// AN ACTOR PLACED IN A LEVEL HAS ALREADY SKIPPED BOOT, because the LEVEL is
	// the document — that is why the constructor puts the session straight into
	// Build, and why the Details panel can accept a number at all.
	//
	// A GAME SESSION STARTS SOMEWHERE ELSE: boot, then the menu, then whatever you
	// choose.
	//
	// THE TEST IS THE WORLD, NOT THE BUILD, and it was WITH_EDITOR until a
	// standalone launch walked straight past the menu into Build. That macro does
	// not mean "an editor is running" - it means editor support was COMPILED IN,
	// and Play > Standalone Game runs the very same editor binary in another
	// process. So it was true there, and the comment that used to sit here claimed
	// a property of the BUILD when the question has always been a property of the
	// LAUNCH.
	//
	// PIE is the one case that skips boot, and skipping is right there for the
	// reason above: you pressed play on a level you are editing, so the level IS
	// the document and there is nothing for a menu to open. Standalone is not that
	// - it is the packaged flow being rehearsed, which is the whole point of the
	// mode, so it should boot exactly as the shipped build does.
	//
	// `-TUBootMenu` stays and now covers only PIE.
	const UWorld* const BootWorld = GetWorld();
	const bool bBoot = (BootWorld && BootWorld->WorldType != EWorldType::PIE)
		|| FParse::Param(FCommandLine::Get(), TEXT("TUBootMenu"));
	if (bBoot)
	{
		// ===================== A MENU OPENS OVER NOTHING =====================
		//
		// The preset is built before this point, so booting to the menu was drawing
		// it over a RUNNING RIDE: trains moving, panels full of numbers, and the
		// camera wherever the level had it -- which on the vertical slice is under
		// the track at the station. A first impression of the application was a
		// screen of data seen from inside a rail.
		//
		// Nothing is open yet, so the document is EMPTY. Not a special case: it is
		// the same empty document the Blank template produces, and every panel
		// already handles it -- that was the blank-template crash, fixed, and this
		// is the fix being reused rather than a second path.
		// ===================== AND IT STARTS QUIET =====================
		//
		// Every overlay in this project was built for somebody developing it, and
		// they all default ON because that is who has been running it. All at once
		// on a first launch it is a screen of numbers over a wireframe, which reads
		// as complexity rather than as depth -- the exact failure the control-layer
		// argument is about: findable by somebody who wants it, invisible to
		// somebody who does not.
		//
		// SET HERE RATHER THAN AS DEFAULTS, because a UPROPERTY default does not
		// reach an actor already placed in a level -- the level carries its own
		// serialised values, which is the trap this project has hit before. This is
		// the PLAYER'S entry point specifically: PIE keeps whatever the level says,
		// so the developer view is untouched.
		//
		// Nothing is disabled, only unshown. Every one of these is a keypress away
		// and the settings page lists which.
		bDrawTrack = false;        // the mesh is the picture now
		bShowTelemetry = false;
		bShowDiagnostics = false;
		bShowRestraints = false;
		bShowProfileGraph = false;
		bShowSegmentEditor = false;
		PanelView = ETUPanelView::Off;
		// The in-world G traces too: four coloured dotted lines along every rail
		// are a developer's instrument, and on the menu's backdrop they read as
		// something wrong with the track.
		bGraphVerticalG = false;
		bGraphLateralG = false;
		bGraphSpeed = false;
		bGraphRollRate = false;

		Segments.Reset();
		RebuildFromSegments();
		// AND IT IS CLEAN. The rebuild above told the session what the document is;
		// this says nobody has changed it, or the menu would offer to save nothing.
		Session.DidCreateNew(TCHAR_TO_UTF8(*SerialiseDocument()));
		ResetHistory();
		BootSession();
		// The menu's camera. BootSession only changes the session's mode; this is
		// what stops the first screen being the level's serialised camera, which
		// on the vertical slice is under the station.
		ApplyAppMode(EAppMode::MainMenu);
	}
	else
	{
		// ===================== THE MODE DECIDES THE VIEW, INCLUDING AT THE START =====================
		//
		// That rule is what makes a mode a mode rather than a label, and it was
		// applied on every CHANGE and never once at startup -- so a level opened
		// straight into Build kept whatever camera was serialised, which is Rider.
		// With no train moving that is a view from inside the track at the station,
		// at z = 0, looking at the inside of a rail.
		//
		// Build gets orbit, and the orbit branch frames the whole track on its first
		// tick because nothing has framed it yet -- so this is up, back, and looking
		// at what you have. [F] and [Z] adjust from there.
		ApplyAppMode(Session.Mode());
	}

	if (FParse::Param(FCommandLine::Get(), TEXT("TUSmokeTest")))
	{
		// ===================== IT HAS TO FAIL THE BUILD, NOT JUST SAY SO =====================
		//
		// Until now this logged its findings and carried on, which is fine when a
		// person is reading the output and useless to anything automated: a run
		// with nine failures in it exited 0, and the packaged build it was meant to
		// gate would have shipped.
		//
		// AND IT QUITS. A packaged game with no window to close sits there for ever
		// waiting for input nobody is going to give it, which reads as a hang rather
		// than as a test.
		//
		// THE `PASSED` LINE IS LOAD-BEARING, and it is not decoration beside the
		// exit code. A packaged build whose default map has no ATUCoasterRide in it
		// never reaches this function at all -- BeginPlay does not run, nothing is
		// tested, and the process exits 0 looking exactly like a pass. So the script
		// requires this line to be PRESENT rather than trusting the status. A test
		// that cannot tell success from never-having-run is worse than no test,
		// because it is believed.
		const bool bPassed = RunDocumentSmokeTest();
		UE_LOG(LogTUEvents, Display, TEXT("smoke: %s"),
			bPassed ? TEXT("PASSED") : TEXT("FAILED"));
		FPlatformMisc::RequestExitWithStatus(/*Force*/ false, bPassed ? 0 : 1);
	}

	// THE CABINET GETS POWER, and the ride opens the way a real one does: an
	// operator walks the course, declares it clear, and turns the key.
	//
	// Done here rather than left to the player because every measured figure in
	// this project was taken with the ride simply running, and a default that
	// needed a keypress before anything moved would silently invalidate all of
	// them. The sequence is real; performing it automatically at open is the
	// stand-in, exactly as FAutoStationCrew stands in for platform staff.
	Plc.PowerOn();
	Plc.DeclareCourseClear();
	SetPlcMode(EPlcMode::Run);

	// ===================== THE SHELL =====================
	//
	// Created here rather than by a HUD or a game mode, because the frame is a
	// view of THIS ride and there is exactly one of them. Routing it through a
	// game mode would add a class whose only job is to know about this actor.
	//
	// GUARDED, AND MISSING IS FINE. The debug-canvas panels are still the whole
	// UI in a build without the asset, and a ride that refused to run without
	// its chrome would have made a layout asset a hard dependency of the
	// simulation — which is exactly backwards.
	if (FrameWidgetClass && GetWorld())
	{
		if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
		{
			FrameWidget = CreateWidget<UTUFrameWidget>(PC, FrameWidgetClass);
			if (FrameWidget)
			{
				FrameWidget->AttachTo(this);
				FrameWidget->AddToViewport();
				// [TAB] IS THE MODE KEY, NOT SLATE'S. Once a frame button has been
				// clicked, Slate's default navigation takes Tab as "focus the next
				// button" and the game binding never sees it -- which reads as the
				// mode key being broken after the first mouse click on a tab.
				if (FSlateApplication::IsInitialized())
				{
					TSharedRef<FNavigationConfig> Nav = MakeShared<FNavigationConfig>();
					Nav->bTabNavigation = false;
					FSlateApplication::Get().SetNavigationConfig(Nav);
				}
			}
		}
	}

	if (bDrawTrack)
	{
		DrawTrack();
	}
	DrawRideProfile();

	// The debug canvas rather than a UMG widget, deliberately: a generated panel
	// has no fixed set of elements to lay out in an asset, and this needs no asset
	// at all. It draws in play and not in the editor viewport, which is right — a
	// control room is something you look at while the ride is running.
	PanelDrawHandle = UDebugDrawService::Register(TEXT("Game"),
		FDebugDrawDelegate::CreateUObject(this, &ATUCoasterRide::DrawControlPanel));
}

void ATUCoasterRide::EndPlay(const EEndPlayReason::Type Reason)
{
	// Unregistered explicitly: the service holds the delegate, and a stale one
	// fires into a destroyed actor.
	if (PanelDrawHandle.IsValid())
	{
		UDebugDrawService::Unregister(PanelDrawHandle);
		PanelDrawHandle.Reset();
	}
	Super::EndPlay(Reason);
}


// ===================== THE RIDE PROFILE, AS A GRAPH =====================
//
// The view that makes a spike DIAGNOSABLE. The in-world traces show WHERE
// something happens; this shows WHAT, against a labelled axis you can read a
// number off — and they are different questions, which is why both exist.
//
// Drawn on the debug canvas, exactly as the control panel is, and for the same
// reason: it needs no asset, so it works the moment somebody presses the key
// rather than after somebody authors a widget. `UI_CONVENTIONS.md` picked UMG
// for composed panels and custom Slate for DRAWN ones, and this is a drawn one —
// the canvas is the same OnPaint in a cheaper coat.
//
// Every number comes from `GraphAxis.h`, which is tested: nice-number ticks, a
// range grown to a multiple of the step, zero kept on screen for signed
// channels, and a scrubber that round-trips.
// ===================== THE DIAGNOSTICS PANEL =====================
//
// `TrackValidate.h` has produced exactly the right data since Phase 0 and none of
// it has ever been visible outside a log. This is the log becoming a panel.
//
// The rules it draws by are in `Prototypes/Shell/DiagnosticsModel.h` and are
// tested there: severity then LOCATION, whole-ride statements last, height called
// out on its own row saying LOW or HIGH, and a stalled ride hiding every derived
// number.
//
// REPORT, NEVER REPAIR — IN THE UI TOO. There is no fix button and nowhere to put
// one: a row has a place to GO and no action to TAKE. Measured rather than
// principled — clamping a degenerate arc to a straight yields a plausible 1.00 g
// and a clean continuity pass, which is worse than visible breakage.
void ATUCoasterRide::BuildDiagnostics()
{
	Diagnostics.Clear();

	// WHAT THE DEVICES WILL DO, which the validator structurally cannot say.
	// It reads the authored values; these depend on the train's length and on
	// the speed at that point, which depends on every metre of track before it.
	// Computed during the rebuild where the zones and the profile both exist.
	for (const FDeviceFinding& D : LastDeviceFindings)
	{
		FDiagTarget T;
		T.S = D.S;
		Diagnostics.Add(D.bIsError ? EDiagSeverity::Error : EDiagSeverity::Warning,
			"Devices", D.What, T);
	}

	// The validator's own findings, verbatim. Its messages already name fields
	// and quantities, so rewording them here would be a second voice saying
	// nearly the same thing.
	for (const FTrackDiagnostic& D : LastDiagnostics)
	{
		FDiagTarget T;
		T.Segment = static_cast<int>(D.SegmentIndex);
		Diagnostics.Add(D.bIsError ? EDiagSeverity::Error : EDiagSeverity::Warning,
			"Geometry", D.Message, T);
	}

	// Closure, with HEIGHT on its own row. The vertical slice shipped 8.5 m low
	// because plan view looked closed, and one combined number hides exactly that.
	if (!bTrackIsCircuit)
	{
		const FTrackFrame A = Track.EvaluateAt(0.0);
		const FTrackFrame B = Track.EvaluateAt(Track.TotalLength());
		Diagnostics.AddClosure(B.Position.X - A.Position.X,
			B.Position.Y - A.Position.Y,
			B.Position.Z - A.Position.Z, 0.05);
	}

	// The ride itself, and a stall hides every number derived from a run that did
	// not happen.
	double Top = 0.0, PeakG = 0.0, PeakAt = 0.0, Lap = 0.0;
	for (const FRideSample& S : Profile_.Samples)
	{
		Top = FMath::Max(Top, S.Speed);
		if (FMath::Abs(S.VerticalG) > FMath::Abs(PeakG)) { PeakG = S.VerticalG; PeakAt = S.S; }
		Lap = S.Time;
	}
	Diagnostics.AddRideProfile(Profile_.bCompleted, Profile_.StalledAtS,
		Top, PeakG, PeakAt, Lap);

	// ===================== THE ENVELOPE, AT LAST IN THE PANEL =====================
	//
	// JudgeRideProfile had only ever been called from its own suite. It runs
	// here on a COMPLETED ride only (the suite's own old failure: a verdict on a
	// ride that did not happen), for the centre seat and for the outer seat --
	// OffsetProfile adds what a rider BodyWidth/4 off the heartline feels
	// through every roll, which the heartline reports as nothing. Two runs,
	// one per seat, which is the rule the facing sign already follows.
	// Limits are unverified research (the header says so); a finding is a row
	// to go and look at, never a repair.
	if (Profile_.bCompleted)
	{
		const double OuterM = TrainMeshSettings().BodyWidthM * 0.25;
		struct { const char* Seat; double LateralM; } Seats[] = {{"centre", 0.0}, {"outer seat", OuterM}};
		for (const auto& Seat : Seats)
		{
			const FGVerdict V = JudgeRideProfile(OffsetProfile(Profile_, Seat.LateralM));
			for (const FGFinding& F : V.Findings)
			{
				static const char* KindName[] = {"sustained", "impact", "jerk", "combined", "reversal"};
				FDiagTarget T;
				T.S = F.AtS;
				Diagnostics.Add(EDiagSeverity::Warning, "Envelope",
					TCHAR_TO_UTF8(*FString::Printf(TEXT("%s %s %.2f g for %.2f s, limit %.2f (%s, %s)"),
						UTF8_TO_TCHAR(F.Axis), UTF8_TO_TCHAR(KindName[static_cast<int32>(F.Kind)]),
						F.Value, F.Duration, F.Limit, UTF8_TO_TCHAR(Seat.Seat),
						V.Standard == EGStandard::ASTM_F2291 ? TEXT("ASTM") : TEXT("EN"))), T);
			}
		}
	}

	// Support placement, which refuses more than it places on a layout with a
	// helix — and the hole its refusals leave is the number an engineer asks for.
	if (bShowSupportFindings)
	{
		std::vector<FMeshFinding> Ignored;
		const FSupportPlan Plan = PlanSupports(WalkTrack(Track, 1.0),
			Track.GetHeartlineHeight(), Profile, FSupportSettings(),
			FlatGround(-GroundOffsetM));
		for (const FSupportFinding& F : Plan.Finding)
		{
			FDiagTarget T;
			T.S = F.S;
			Diagnostics.Add(EDiagSeverity::Warning, "Structure",
				TCHAR_TO_UTF8(*FString::Printf(TEXT("%s (%.0f m of track)"),
					UTF8_TO_TCHAR(F.What.c_str()), F.LengthM())), T);
		}
		if (Plan.LongestGapM > 0.0)
		{
			Diagnostics.Add(EDiagSeverity::Info, "Structure",
				TCHAR_TO_UTF8(*FString::Printf(TEXT("%d supports, longest unsupported run %.1f m"),
					static_cast<int32>(Plan.Leg.size()), Plan.LongestGapM)));
		}
	}

	Diagnostics.Sort();
}

void ATUCoasterRide::DrawDiagnosticsPanel(UCanvas* Canvas)
{
	if (!bShowDiagnostics || !Canvas || !GEngine) { return; }

	const float W = 620.f;
	const float Row = 16.f;
	const int32 Shown = FMath::Min(static_cast<int32>(Diagnostics.Num()), 14);
	const float H = 26.f + Row * FMath::Max(Shown, 1);
	const float Ox = Canvas->SizeX - W - 20.f;
	const float Oy = 64.f;   // under the frame's mode tabs, which own the top-right

	PanelTile(Canvas, Ox - 8.f, Oy - 8.f, W + 16.f, H + 16.f, PanelGround);
	PanelLabel(Canvas, Ox, Oy,
		FString::Printf(TEXT("DIAGNOSTICS   %s   [V] hide"),
			UTF8_TO_TCHAR(Diagnostics.Summary().c_str())),
		Diagnostics.HasErrors() ? PanelRed : PanelDim);

	if (Diagnostics.Num() == 0)
	{
		// Empty states say what to DO, not what is absent — FirstRun.h owns the
		// words so every panel's is written once and in the same voice.
		PanelLabel(Canvas, Ox, Oy + 22.f,
			UTF8_TO_TCHAR(EmptyStateFor(EPanelKind::Diagnostics)), PanelDim);
		return;
	}

	// IMMEDIATE MODE: the rectangles are recorded as they are drawn and hit-tested
	// next click. There is no widget tree to keep in sync with the findings, which
	// is the whole reason this is cheap enough to rebuild every rebuild.
	DiagRowRects.Reset();

	float Y = Oy + 20.f;
	for (int32 i = 0; i < Shown; ++i)
	{
		const FDiagRow& R = Diagnostics.At(static_cast<std::size_t>(i));
		DiagRowRects.Add(FVector4(Ox, Y, Ox + W, Y + Row));

		// The selected row is banded, so clicking one and then looking at the
		// viewport does not lose which finding you were on.
		if (R.Target.Segment >= 0 && R.Target.Segment == SelectedSegment)
		{
			PanelTile(Canvas, Ox - 4.f, Y - 1.f, W + 8.f, Row, PanelRule);
		}
		const FLinearColor Ink = R.Severity == EDiagSeverity::Error ? PanelRed
			: (R.Severity == EDiagSeverity::Warning ? PanelAmber : PanelDim);

		// HUE IS NEVER THE ONLY CHANNEL — UI_CONVENTIONS. A severity glyph carries
		// the same fact as the colour, so the panel still reads in grey-scale and
		// to somebody who does not separate red from amber.
		const TCHAR* Mark = R.Severity == EDiagSeverity::Error ? TEXT("!!")
			: (R.Severity == EDiagSeverity::Warning ? TEXT(" !") : TEXT("  "));

		// AND WHERE IT IS, because a finding without a place is trivia. Once
		// there is a selection to drive, this column becomes the click target.
		FString Where;
		if (R.Target.S >= 0.0)            { Where = FString::Printf(TEXT("%6.0fm"), R.Target.S); }
		else if (R.Target.Segment >= 0)   { Where = FString::Printf(TEXT("  seg%d"), R.Target.Segment); }
		else                              { Where = TEXT("      "); }

		PanelLabel(Canvas, Ox, Y, FString::Printf(TEXT("%s %s  %s"),
			Mark, *Where, UTF8_TO_TCHAR(R.Text.c_str())), Ink);
		Y += Row;
	}

	if (Diagnostics.Num() > static_cast<std::size_t>(Shown))
	{
		// NO SILENT TRUNCATION. A list that stopped at fourteen without saying so
		// reads as "that is all of them", which is the one thing a diagnostics
		// panel must never imply.
		PanelLabel(Canvas, Ox, Y,
			FString::Printf(TEXT("   ... and %d more"),
				static_cast<int32>(Diagnostics.Num()) - Shown), PanelDim);
	}
}


void ATUCoasterRide::GraphRect(float ViewportHeight, float& OutX, float& OutY,
	float& OutW, float& OutH) const
{
	// ONE ANSWER for where the graph is, shared by the draw and by the hit test.
	// The console had exactly this bug — a panel deciding one thing to display
	// while the clicks were resolved against another — and two copies of four
	// numbers is how it starts.
	OutW = 560.f;
	OutH = 150.f;
	OutX = 20.f;
	// THE HEIGHT COMES IN rather than a canvas, so the hit test does not need one
	// — there is no canvas outside a draw call, and a hit test that had to invent
	// one would be the second source of truth this exists to avoid.
	// ABOVE THE CONTROL PANEL when there is one. Both sat bottom-left and the
	// graph was drawn straight through the block strip.
	// The tile runs 34 px below the plot (axis labels), so that is the clearance.
	OutY = FMath::Min(ViewportHeight - 96.f, ConsolePanelTopY - 46.f) - OutH;
}

bool ATUCoasterRide::PressGraph(float Mx, float My)
{
	if (!bShowProfileGraph || bHideOverlays || !Profile_.bCompleted)
	{
		// A STALLED RIDE HAS NOTHING TO SCRUB. The graph already refuses to draw a
		// trace from a run that did not happen, and a scrubber over a blank panel
		// would be a control that moves the camera to a metre that means nothing.
		return false;
	}
	float Gx = 0.f, Gy = 0.f, Gw = 0.f, Gh = 0.f;
	GraphRect(PaintedSizeY, Gx, Gy, Gw, Gh);
	if (Mx < Gx || Mx > Gx + Gw || My < Gy || My > Gy + Gh)
	{
		return false;
	}

	bScrubbing = true;
	TickScrub();
	return true;
}

void ATUCoasterRide::TickScrub()
{
	if (!bScrubbing)
	{
		return;
	}
	float Mx = 0.f, My = 0.f;
	if (!PanelMouse(Mx, My))
	{
		return;
	}
	float Gx = 0.f, Gy = 0.f, Gw = 0.f, Gh = 0.f;
	GraphRect(PaintedSizeY, Gx, Gy, Gw, Gh);

	// NOT CLAMPED TO THE RECTANGLE, deliberately. Dragging off the end of the
	// graph should pin to the end and keep tracking, the way every scrubber in
	// every editor does — a drag that stops responding because the pointer left a
	// box reads as the control having broken.
	Graph.SetDomain(Track.TotalLength());
	Graph.ScrubTo(Graph.ScrubToS(Gw > 0.f ? (Mx - Gx) / Gw : 0.0));

	// AND THE OTHER PICTURE MOVES. The orbit focus goes to the metre under the
	// cursor, so finding a number on the trace leaves you looking at the track
	// that produces it. Only in orbit: the free camera is somewhere somebody flew
	// it deliberately, and yanking it would undo that.
	if (CameraMode == ETUCameraMode::Orbit && Track.NumSegments() > 0)
	{
		const FTrackFrame F = Track.EvaluateAt(Graph.ScrubbedS());
		Orbit.Focus = {F.Position.X, F.Position.Y, F.Position.Z};
		bOrbitFramed = true;
	}
}

void ATUCoasterRide::DrawProfileGraph(UCanvas* Canvas)
{
	if (!bShowProfileGraph || !GEngine) { return; }

	const std::vector<FRideSample>& Sample = Profile_.Samples;
	float Ox = 0.f, Oy = 0.f, Wx = 0.f, Hy = 0.f;
	GraphRect(PanelSizeY(Canvas), Ox, Oy, Wx, Hy);

	PanelTile(Canvas, Ox - 8.f, Oy - 24.f, Wx + 56.f, Hy + 58.f, PanelGround);   // +40 for the axis labels on the right

	// A STALLED RIDE SHOWS THE STALL AND NOTHING ELSE. The envelope suite already
	// had this failure — "within envelope, zero findings" over a train that
	// stalled at 46 m — and a trace drawn from a run that did not happen reads as
	// a result. Same rule the diagnostics model runs on.
	if (!Profile_.bCompleted)
	{
		PanelLabel(Canvas, Ox, Oy - 20.f, TEXT("RIDE PROFILE"), PanelDim);
		PanelLabel(Canvas, Ox, Oy + 8.f,
			FString::Printf(TEXT("the train does not complete: it stalls at %.1f m, %.1f m up"),
				Profile_.StalledAtS, Profile_.StalledHeight), PanelRed);
		PanelLabel(Canvas, Ox, Oy + 26.f,
			TEXT("no speed or G is shown, because there was no ride to measure"), PanelDim);
		return;
	}
	if (Sample.size() < 2)
	{
		PanelLabel(Canvas, Ox, Oy - 20.f, TEXT("RIDE PROFILE"), PanelDim);
		PanelLabel(Canvas, Ox, Oy + 8.f,
			TEXT("no ride to measure yet"), PanelDim);
		return;
	}

	// The channel on show. One at a time rather than four overlaid, because four
	// traces on one axis is a picture rather than a reading — and the per-channel
	// scale is exactly what the Phase 1 legibility card asked for.
	std::vector<double> Values;
	Values.reserve(Sample.size());
	const TCHAR* ChannelName = TEXT("");
	const TCHAR* ChannelUnit = TEXT("");
	bool bSigned = true;
	FLinearColor Ink = PanelCyan;

	switch (ProfileChannel)
	{
	case ETUProfileChannel::Speed:
		for (const FRideSample& S : Sample) { Values.push_back(S.Speed * 3.6); }
		ChannelName = TEXT("SPEED"); ChannelUnit = TEXT("km/h");
		bSigned = false; Ink = PanelCyan;
		break;
	case ETUProfileChannel::LateralG:
		for (const FRideSample& S : Sample) { Values.push_back(S.LateralG); }
		ChannelName = TEXT("LATERAL G"); ChannelUnit = TEXT("g");
		Ink = PanelAmber;
		break;
	case ETUProfileChannel::RollRate:
		// FIRST-CLASS, WITH ITS OWN AXIS. The one thing no G trace can ever show:
		// felt G models the rider as a point at the heartline, so spinning that
		// point costs exactly nothing.
		for (const FRideSample& S : Sample) { Values.push_back(S.RollRateDegPerSec); }
		ChannelName = TEXT("ROLL RATE"); ChannelUnit = TEXT("deg/s");
		Ink = PanelGreen;
		break;
	default:
		for (const FRideSample& S : Sample) { Values.push_back(S.VerticalG); }
		ChannelName = TEXT("VERTICAL G"); ChannelUnit = TEXT("g");
		Ink = PanelCyan;
		break;
	}

	const double Total = Sample.back().S;
	double Lo = Values[0], Hi = Values[0];
	for (double V : Values) { Lo = FMath::Min(Lo, V); Hi = FMath::Max(Hi, V); }
	const FAxis Axis = MakeAxis(Lo, Hi, 5, bSigned);

	// ---- Gridlines, at the nice numbers, LABELLED. An unlabelled gridline is
	// decoration; the whole reason for snapping the step is that each one has a
	// number you can read.
	for (std::size_t t = 0; t < Axis.TickCount(); ++t)
	{
		const double V = Axis.TickAt(t);
		const float Y = Oy + Hy - static_cast<float>(Axis.Fraction(V)) * Hy;
		// ZERO IS DRAWN BRIGHTER, because on a signed channel it is the line the
		// trace means anything against.
		const bool bZero = FMath::Abs(V) < Axis.Step * 0.001;
		PanelTile(Canvas, Ox, Y, Wx, 1.f, bZero ? PanelRule : FLinearColor(
			PanelRule.R * 0.6f, PanelRule.G * 0.6f, PanelRule.B * 0.6f, 1.f));
		PanelLabel(Canvas, Ox + Wx + 4.f, Y - 7.f,
			FString::Printf(TEXT("%.2f"), V), PanelDim);
	}

	// ---- The trace. One line segment per sample pair, clamped into the box.
	for (std::size_t i = 0; i + 1 < Values.size(); ++i)
	{
		const float X0 = Ox + static_cast<float>(Sample[i].S / Total) * Wx;
		const float X1 = Ox + static_cast<float>(Sample[i + 1].S / Total) * Wx;
		const float Y0 = Oy + Hy - static_cast<float>(Axis.Fraction(Values[i])) * Hy;
		const float Y1 = Oy + Hy - static_cast<float>(Axis.Fraction(Values[i + 1])) * Hy;
		PanelLine(Canvas, X0, Y0, X1, Y1, Ink, 1.6f);
	}

	// ---- The scrubber, and the train's own position on it.
	//
	// THE TRAIN IS ALWAYS DRAWN, because the most useful thing this panel does is
	// let you watch the trace and the ride at the same time and see which bit of
	// the graph is the bit you are on.
	if (!Trains.IsEmpty() && Trains[ActiveTrainIndex()])
	{
		const double S = Trains[ActiveTrainIndex()]->GetDistance();
		const float X = Ox + static_cast<float>(FMath::Clamp(S / Total, 0.0, 1.0)) * Wx;
		PanelTile(Canvas, X, Oy, 1.f, Hy, PanelAmber);
		PanelLabel(Canvas, X + 3.f, Oy + Hy - 12.f,
			FString::Printf(TEXT("%.0f m"), S), PanelAmber);
	}

	// ---- THE SCRUBBER, drawn from the value it just produced.
	//
	// That is why `ScrubToS` and `SToScrub` are asserted to round-trip: this line
	// is placed by converting the arc length BACK to a fraction, so a mismatch
	// makes the handle drift away from the cursor as you drag — visible
	// immediately, and impossible to reason about from the code alone.
	if (bScrubbing || Graph.ScrubbedS() > 0.0)
	{
		const double Sc = Graph.ScrubbedS();
		const float Xs = Ox + static_cast<float>(Graph.SToScrub(Sc)) * Wx;
		PanelTile(Canvas, Xs, Oy, 1.f, Hy, PanelCyan);
		// THE VALUE OF THE CHANNEL BEING LOOKED AT, not of all of them. One
		// channel at a time is the rule the graph already follows, and a readout
		// listing four numbers would be a second answer to a question the trace
		// has already given.
		PanelLabel(Canvas, Xs + 3.f, Oy + 2.f,
			FString::Printf(TEXT("%.0f m   %.2f"), Sc,
				FProfileGraph::SampleAt(Values, Total, Sc)), PanelCyan);
	}

	// ---- The heading, the extremes, and WHERE they happened.
	//
	// "4.25 g at 310 m" is somewhere to go and look; "4.25 g" is trivia. Same rule
	// the diagnostics model runs on, and the reason both say it.
	const FChannelExtremes E = ExtremesOf(Values, Total);
	PanelLabel(Canvas, Ox, Oy - 20.f,
		FString::Printf(TEXT("%s  (%s)   [G] channel   [H] hide"), ChannelName, ChannelUnit),
		PanelDim);
	PanelLabel(Canvas, Ox + 240.f, Oy - 20.f,
		FString::Printf(TEXT("peak %.2f at %.0f m      min %.2f at %.0f m"),
			E.Max, E.MaxAtS, E.Min, E.MinAtS), PanelText);
	PanelLabel(Canvas, Ox, Oy + Hy + 6.f, TEXT("0 m"), PanelDim);
	PanelLabel(Canvas, Ox + Wx - 46.f, Oy + Hy + 6.f,
		FString::Printf(TEXT("%.0f m"), Total), PanelDim);
}


void ATUCoasterRide::DrawControlPanel(UCanvas* Canvas, APlayerController* /*PC*/)
{
	// ===================== Z-ORDER IS DRAW ORDER, AND IT WAS BACKWARDS =====================
	//
	// EVERYTHING RIDES ON ONE CALLBACK rather than registering a second delegate.
	// Two registrations can be added, removed and ordered independently, and the
	// failure — one panel drawing over the other depending on which was registered
	// first — is the kind that only appears on somebody else's machine.
	//
	// The cost of that is that draw ORDER is the only z-order there is, and the
	// three things that must be ON TOP were being drawn FIRST: the drag answer
	// appeared under the segment editor and the diagnostics list, and the unsaved
	// -changes confirm carried a comment claiming it was last when it was third.
	//
	// They cannot simply move to the bottom of DrawPanels, because that returns
	// early in three places — hidden overlays, no panel view, no signals — so
	// anything appended there is drawn only when the ride is in one particular
	// state. Hence the split: the panels, and then the things above them.
	// THE RIDE'S OVERLAYS BELONG TO A RIDE. In Boot and MainMenu there is no
	// document open, so the mode banner, the diagnostics list, the graph and the
	// control panel are all reporting on nothing -- and drawing them under the
	// menu is what made the first screen unreadable.
	//
	// The menu and the recovery offer are drawn either way, below, because those
	// ARE the screen in those two modes.
	// THE SETTINGS PAGE OWNS THE SCREEN. The debug canvas draws ABOVE Slate, so
	// every panel here was painted over the settings page while it was up.
	if (FrameWidget && FrameWidget->IsSettingsOpen())
	{
		MenuRowRects.Reset();
		MenuRowAction.Reset();
		ConsoleRects.Reset();
		ConsoleAction.Reset();
		return;
	}

	const EAppMode Now = Session.Mode();
	if (Now != EAppMode::Boot && Now != EAppMode::MainMenu)
	{
		DrawPanels(Canvas);
	}

	// THE MENU IS UMG NOW (UTUMenuWidget), below this canvas; the confirm and
	// the recovery offer still draw here, above it, and it is made hit-test
	// invisible while either is up (see Tick).
	DrawDragAnswer(Canvas);
	// ABOVE THE MENU, because nothing may be opened while it is up: opening a
	// document is what would overwrite the evidence, and the session refuses to
	// leave Boot until this is answered.
	DrawRecoveryOffer(Canvas);
	// LAST, and now genuinely: it is on top of whatever it is asking about, and it
	// rebuilds the row list, so a click while it is up can only hit its own three
	// answers.
	DrawLeaveConfirm(Canvas);
	// ABOVE EVEN THAT, because it is the only thing on screen that is taking
	// keystrokes: a prompt drawn under the panel somebody is typing into reads as
	// the keyboard being broken. Z-order here IS draw order — see the note above.
	DrawSaveNamePrompt(Canvas);
}

void ATUCoasterRide::DrawSaveNamePrompt(UCanvas* Canvas)
{
	if (!Canvas || !GEngine || !bNamingSave) { return; }

	const float Ox = 70.f;
	float Y = 150.f;
	PanelTile(Canvas, Ox - 18.f, Y - 16.f, 560.f, NameError.IsEmpty() ? 108.f : 128.f,
		PanelGround);
	PanelLabel(Canvas, Ox, Y, TEXT("SAVE AS"), PanelCyan);
	Y += 26.f;

	// THE CARET IS DRAWN, not implied. A field with an empty buffer and no caret
	// is indistinguishable from a label, and somebody types into the void.
	PanelLabel(Canvas, Ox, Y, NameBuffer + TEXT("_"), PanelText);
	Y += 8.f;
	PanelLine(Canvas, Ox, Y + 10.f, Ox + 420.f, Y + 10.f, PanelRule);
	Y += 22.f;

	// WHERE IT IS GOING, spelled out. A save prompt that does not say the folder
	// is one people go looking for the file after.
	PanelLabel(Canvas, Ox, Y, FString::Printf(TEXT("%s/%s.track"),
		*FPaths::GetCleanFilename(TracksDir()),
		NameBuffer.IsEmpty() ? TEXT("...") : *NameBuffer), PanelDim);
	Y += 20.f;

	if (!NameError.IsEmpty())
	{
		PanelLabel(Canvas, Ox, Y, NameError, PanelAmber);
		Y += 20.f;
	}
	PanelLabel(Canvas, Ox, Y,
		TEXT("letters, digits, space, - and _   ·   [Enter] save   ·   [Esc] cancel"),
		PanelDim);
}

void ATUCoasterRide::DrawPanels(UCanvas* Canvas)
{
	if (bHideOverlays) { return; }

	// The console first, because the graph places itself above wherever the
	// console's top edge landed this frame.
	// The console and the graph are PAINTED by UTUPaintedPanelWidget now, below
	// this canvas; see RecordPaintedPanels.
	DrawModeBanner(Canvas);
	DrawTelemetry(Canvas);
	DrawDiagnosticsPanel(Canvas);
}

void ATUCoasterRide::DrawTelemetry(UCanvas* Canvas)
{
	if (!Canvas || TelemetryLines.IsEmpty()) { return; }

	// ITS OWN SLOT, under the banner and the hint. As on-screen debug messages
	// these were drawn by the engine wherever it chose, which was over the frame
	// header and the banner; the text itself is unchanged.
	TArray<int32> Keys;
	TelemetryLines.GetKeys(Keys);
	Keys.Sort();
	float Y = 90.f;
	for (int32 K : Keys)
	{
		const TPair<FColor, FString>& L = TelemetryLines[K];
		TArray<FString> Parts;
		L.Value.ParseIntoArray(Parts, TEXT("\n"), false);
		for (const FString& Part : Parts)
		{
			PanelTile(Canvas, 10.f, Y, 8.f + PanelTextWidth(Canvas, Part), 16.f, PanelGround);
			PanelLabel(Canvas, 12.f, Y + 1.f, Part, FLinearColor(L.Key));
			Y += 16.f;
		}
	}
}

void ATUCoasterRide::DrawConsole(UCanvas* Canvas)
{
	if (PanelView == ETUPanelView::Off || !Signals || !Drives || !GEngine)
	{
		return;
	}

	// TWO VIEWS OF ONE PANEL, and the split is the one every real installation
	// already makes rather than a detail level invented here. The operator's
	// question is "may this train go, and if not, what is holding it"; the
	// maintainer's is "what is this machine actually doing". Motor current answers
	// only the second, which is why an operator console does not carry it and why
	// hiding it from one view is more faithful than showing it to both.
	//
	// Same walk, same live reads, one branch per section. Nothing is computed for
	// a view it is not shown in, and nothing is cached for either.
	const bool bMaint = PanelView == ETUPanelView::Maintenance;

	// GENERATED FROM THE SAME LISTS AS THE RIDE. Nothing below is authored per
	// layout: the rows come from walking the blocks, the zones and the platforms
	// that RebuildFromSegments already derived, so a track with more blocks gets
	// more indicators without anything here being told about it.
	const int32 NumBlocks = static_cast<int32>(Signals->NumBlocks());
	const int32 NumDrives = static_cast<int32>(Drives->Num());
	const float Row = 15.f;
	const float Pad = 8.f;
	// WIDER FOR MAINTENANCE: the drives table carries three readings, a torque
	// bar and the CiA 402 statusword per row, and at 470 the statusword ran
	// past the panel's edge.
	const float W = bMaint ? 560.f : 470.f;
	const float StripH = bMaint ? 58.f + Row : 58.f;   // schematic, + the counts row
	const int32 Rows = 2                           // title, status
		+ 1 + NumDrives                            // DRIVES heading + VFD modules
		+ (bMaint ? 3 : 0)                         // scan line, CONTROLLER, DETECTION
		+ (Platforms.Num() > 0 ? 1 + Platforms.Num() + 3 : 0)    // + CONSOLE heading, lamps, tall buttons
		+ (EventLog.Num() > 0 ? 1 + FMath::Min(EventLog.Num(), 4) : 0);
	// The console row sat on the bottom edge, half off it: the section gaps are
	// worth about a row and a half between them and were not being counted.
	// MEASURED, NOT ONLY COUNTED. The count above is an estimate that has been
	// wrong twice (section gaps, then the event log), each time with the bottom
	// rows drawn outside the panel. What was actually drawn last frame is the
	// height; the estimate only covers the first frame.
	const float H = FMath::Max(Pad * 2.f + Rows * Row + StripH + 34.f, ConsoleContentH);

	const float X = 16.f;
	// 40 from the bottom, not 16: the frame's status line lives there.
	const float Y = FMath::Max(16.f, PanelSizeY(Canvas) - H - 40.f);
	ConsolePanelTopY = Y;

	PanelTile(Canvas, X, Y, W, H, PanelGround);
	PanelTile(Canvas, X, Y, W, 1.f, PanelRule);
	PanelTile(Canvas, X, Y + H - 1.f, W, 1.f, PanelRule);
	PanelTile(Canvas, X, Y, 1.f, H, PanelRule);
	PanelTile(Canvas, X + W - 1.f, Y, 1.f, H, PanelRule);

	// THE COLOUR BANDS, which are the one convention every real console shares:
	// the operating controls live on a GREEN field and the stop authority on a RED
	// one, physically separated, so a hand reaching for one is nowhere near the
	// other. On a screen there is no hand to misplace, but the grouping still does
	// the work of saying which half of the panel you are reading — and it costs two
	// tiles.
	//
	// The third console adds YELLOW between them for lockout, which is not modelled
	// here and so is not drawn. A band for a control that does not exist would be
	// the panel telling its first lie.
	PanelTile(Canvas, X + 1.f, Y + 1.f, 3.f, H - 2.f, FLinearColor(0.16f, 0.55f, 0.24f, 1.f));
	PanelTile(Canvas, X + W - 4.f, Y + 1.f, 3.f, 34.f, FLinearColor(0.62f, 0.14f, 0.12f, 1.f));

	float Ty = Y + Pad;
	const float Lx = X + Pad + 4.f;   // clear of the green field band

	PanelLabel(Canvas, Lx, Ty, bMaint
		? TEXT("TRACKUNLIMITED  ·  MAINTENANCE")
		: TEXT("TRACKUNLIMITED  ·  RIDE CONTROL"), bMaint ? PanelAmber : PanelCyan);
	Ty += Row;

	// Top-level state, which is the one line an operator glances at.
	{
		const bool bStopped = Drives->IsEmergencyStopped();
		FString Status = FString::Printf(TEXT("%s   %d TRAIN%s"),
			bManualDispatch ? TEXT("MANUAL") : TEXT("AUTO"),
			Trains.Num(), Trains.Num() == 1 ? TEXT("") : TEXT("S"));
		if (Signals->Violations() > 0)
		{
			Status += FString::Printf(TEXT("   %d VIOLATION(S)"),
				static_cast<int32>(Signals->Violations()));
		}
		PanelLabel(Canvas, Lx, Ty, Status, Signals->Violations() > 0 ? PanelRed : PanelDim);
		if (bMaint)
		{
			// THE SCAN RATE IS AN ENGINEERING FACT, and overruns are the one thing
			// that makes it stop being a constant. A dropped backlog means the ride
			// ran slower than real time for a moment, which is safe and is still
			// something a maintainer wants to know happened.
			// ON ITS OWN LINE, under the status: on the same line it ran into the
			// stop box and off the panel.
			FString Scan = FString::Printf(TEXT("%d Hz SCAN"), SimHz);
			if (ScanOverruns > 0)
			{
				// THE SECONDS, NOT JUST THE COUNT. 545 overruns reads as "a bit
				// stuttery"; 54 s dropped says the ride on screen is not the ride
				// the model computed, and nothing watched across it can be judged.
				Scan += FString::Printf(TEXT("   %d OVERRUN%s, %.1f s DROPPED"),
					ScanOverruns, ScanOverruns == 1 ? TEXT("") : TEXT("S"),
					ScanTimeDroppedS);
			}
			// SCAN NUMBER AND FINGERPRINT TOGETHER, never the digest alone. It is a
			// running hash, so two rides only agree AT THE SAME POINT — a digest
			// without its scan number is not a comparable quantity, it is a number
			// that happens to be printed.
			Scan += FString::Printf(TEXT("   #%lld %08x"),
				static_cast<long long>(ScanNumber),
				static_cast<uint32>(SimFingerprint.Value() & 0xFFFFFFFFull));
			PanelLabel(Canvas, Lx, Ty + Row, Scan, PanelDim);
		}

		// THE STOP AUTHORITY IN ITS OWN BOX, outlined red, away from everything
		// else. Straight off the console photo: the operating controls are grouped
		// inside a green outline and the stop controls inside a red one, so a hand
		// reaching for one is nowhere near the other. On a screen the outline is
		// the whole of that idea, and it is worth keeping.
		const float StopX = Lx + 292.f;
		const float StopW = W - Pad * 2.f - 292.f;
		PanelTile(Canvas, StopX, Ty - 2.f, StopW, 1.f, bStopped ? PanelRed : PanelDim);
		PanelTile(Canvas, StopX, Ty + Row, StopW, 1.f, bStopped ? PanelRed : PanelDim);
		PanelTile(Canvas, StopX, Ty - 2.f, 1.f, Row + 2.f, bStopped ? PanelRed : PanelDim);
		PanelTile(Canvas, StopX + StopW - 1.f, Ty - 2.f, 1.f, Row + 2.f,
			bStopped ? PanelRed : PanelDim);
		// THE CATEGORY IS PART OF THE STOP, so the panel says which one it is.
		// STOPPING and STOPPED are also different facts under a Cat 1 — power is
		// retained while the drives wind down — and an operator watching a train
		// still moving after they hit the button needs to know that is the stop
		// working rather than the stop failing.
		// WHY THE RIDE IS NOT RUNNING BELONGS ON THE OPERATOR'S VIEW, not only on
		// the engineering page. The controller module was maintenance-only, so a
		// faulted PLC showed as a ride that simply sat there — every drive
		// stopped, every platform "train not in position", and nothing anywhere
		// saying the controller had gone. That is precisely the complaint this
		// project wrote into PlcUnit.h and then shipped.
		FString StopText(TEXT("RUNNING"));
		const char* PlcWhy = Plc.WhyNotRun();
		if (!bStopped && PlcWhy != nullptr)
		{
			StopText = FString::Printf(TEXT("PLC — %s"), UTF8_TO_TCHAR(PlcWhy));
		}
		if (bStopped)
		{
			const bool bCat1 =
				Drives->EmergencyStopCategory() == FTrackDrives::EStopCategory::One;
			StopText = FString::Printf(TEXT("E-STOP %s · %s"),
				bCat1 ? (Drives->IsPowerRemoved() ? TEXT("CAT1") : TEXT("CAT1 STOPPING"))
					  : TEXT("CAT0"),
				UTF8_TO_TCHAR(Drives->EmergencyStopReason()));
		}
		PanelLabel(Canvas, StopX + 5.f, Ty, StopText,
			bStopped ? PanelRed : (PlcWhy != nullptr ? PanelAmber : PanelGreen));
		Ty += Row + 6.f + (bMaint ? Row : 0.f);   // the scan line, in maintenance
	}

	// ---- THE CIRCUIT, AS A SCHEMATIC ----------------------------------------
	//
	// A real ride's HMI draws the TRACK and puts the lamps on it, rather than
	// listing blocks in a table. It is the better view for the same reason the
	// coloured rails are: you are looking for where something is, and a diagram
	// answers that in one glance where a table makes you read.
	//
	// Block widths are PROPORTIONAL TO BLOCK LENGTH, so the strip is a plan of the
	// circuit rather than a row of equal boxes — the 130 m mid-course brake looks
	// like the long block it is, and the four 10 m platform positions look like the
	// tight cluster they are. All of it comes off Signals->Boundaries().
	PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
	PanelLabel(Canvas, Lx, Ty, TEXT("CIRCUIT"), PanelDim);
	Ty += Row + 2.f;

	{
		const std::vector<double>& Bounds = Signals->Boundaries();
		const double Total = FMath::Max(1.0, Track.TotalLength());
		const float StripX = Lx;
		const float StripW = W - Pad * 2.f - 4.f;
		const float BoxY = Ty + 10.f;
		// TALL ENOUGH TO CARRY ITS NUMBER AND ITS TRAIN. The strip is the
		// instrument -- an operator at the console cannot see the mid-course
		// brake and reads it off this -- so it is the one thing here that gets
		// bigger rather than smaller.
		const float BoxH = 28.f;

		// EVERY BLOCK GETS A FLOOR. Strictly proportional widths made the four 10 m
		// platform positions three pixels each on a 1288 m circuit — the single most
		// interesting part of the ride, invisible, while the 696 m free run got
		// two-fifths of the panel to say nothing in.
		//
		// So each block takes a minimum, and the REMAINDER is shared out
		// proportionally. Long blocks still read as long and short ones still read
		// as short; what goes is only the claim that the strip is to scale, which is
		// a claim no control room screen makes either.
		const float MinW = 9.f;
		const float Floor = MinW * NumBlocks;
		const float Share = FMath::Max(0.f, StripW - Floor);

		auto BlockX = [&](std::size_t Upto) -> float
		{
			float Out = StripX;
			for (std::size_t i = 0; i < Upto; ++i)
			{
				const double A = Bounds[i];
				const double Bnd = (i + 1 < Bounds.size()) ? Bounds[i + 1] : Total;
				Out += MinW + Share * static_cast<float>((Bnd - A) / Total);
			}
			return Out;
		};

		for (int32 b = 0; b < NumBlocks; ++b)
		{
			const std::size_t B = static_cast<std::size_t>(b);
			const double S0 = Bounds[B];
			const double S1 = (B + 1 < Bounds.size()) ? Bounds[B + 1] : Total;
			const float X0 = BlockX(B);
			const float X1 = BlockX(B + 1);
			const float BW = FMath::Max(2.f, X1 - X0 - 1.f);

			const EBlockState State = Signals->GetState(B);
			const bool bOcc = State == EBlockState::Occupied;
			const bool bBuf = State == EBlockState::Buffer;
			FLinearColor Lamp = bOcc ? PanelAmber : (bBuf ? PanelCyan : PanelGreen);

			// A disagreement between the two detection methods paints the block RED
			// on the diagram, which is where somebody is already looking.
			if (Counter && B < Counter->NumBlocks()
				&& (Counter->IsOccupied(B) != Signals->Occupies(B) || Counter->IsOverOccupied(B)))
			{
				Lamp = PanelRed;
			}

			// Filled when it holds something, outlined when clear: an occupied
			// block should read from across a room. The fill is dimmed so the
			// train drawn over it still reads as the train.
			if (bOcc || bBuf)
			{
				PanelTile(Canvas, X0, BoxY, BW, BoxH, Lamp * FLinearColor(0.45f, 0.45f, 0.45f, 1.f));
			}
			PanelTile(Canvas, X0, BoxY, BW, 1.f, Lamp);
			PanelTile(Canvas, X0, BoxY + BoxH - 1.f, BW, 1.f, Lamp);
			PanelTile(Canvas, X0, BoxY, 1.f, BoxH, Lamp);
			PanelTile(Canvas, X0 + BW - 1.f, BoxY, 1.f, BoxH, Lamp);

			// THE BLOCK NUMBER ON THE BLOCK, where a block schematic puts it, so
			// the event log's "block 12" is a place on the strip and not a count
			// from the left. Only where it fits; a 9 px platform position
			// cannot carry two digits and is read from its neighbours.
			if (BW >= 16.f)
			{
				PanelLabel(Canvas, X0 + 3.f, BoxY + 1.f, FString::Printf(TEXT("%d"), b), PanelDim);
			}
			// THE BUFFER TIMER, which is the one number on the strip that is a
			// countdown: the block is empty and will report clear in this long.
			if (bBuf && BW >= 30.f)
			{
				PanelLabel(Canvas, X0 + 3.f, BoxY + BoxH - 14.f,
					FString::Printf(TEXT("%.1f s"), Signals->GetBufferRemaining(B)), PanelCyan);
			}

			// The device under the block, in the SAME colours the rails use, so the
			// panel and the track view are the same object seen twice. Blocks and
			// zones come from one walk, so a device that starts in this block fills
			// it — there is no partial case to draw.
			for (const FTUZoneSpan& Z : ZoneSpans)
			{
				if (Z.StartS >= S0 - 0.01 && Z.StartS < S1)
				{
					PanelTile(Canvas, X0, BoxY + BoxH + 2.f, BW, 3.f,
						RailColourAt(Z.StartS + 0.01));
				}
			}
		}

		// EVERY TRAIN ON THE DIAGRAM, at its real position. The single most useful
		// thing a control room screen shows, and it costs one tile each. Placed
		// through the same widened mapping the blocks use, or a train would sit
		// somewhere other than the block the panel says it is in.
		auto SToX = [&](double S) -> float
		{
			const std::size_t B = Signals->BlockAt(S);
			const double A = Bounds[B];
			const double Bn = (B + 1 < Bounds.size()) ? Bounds[B + 1] : Total;
			const float X0 = BlockX(B);
			const float X1 = BlockX(B + 1);
			const double F = (Bn > A) ? (S - A) / (Bn - A) : 0.0;
			return X0 + static_cast<float>(F) * (X1 - X0);
		};

		// A TRAIN IS A SHAPE WITH A LENGTH, not a tick: its rear to its nose
		// through the same mapping, so a train straddling a boundary is drawn
		// straddling it -- which is exactly the case the counter's falling-edge
		// rule exists for, and a tick at the centre could never show. Floored at
		// a few pixels so a 6 m train in a 700 m block does not vanish.
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			float Xr = SToX(Trains[t]->GetRearS());
			float Xf = SToX(Trains[t]->GetFrontS());
			if (Xf < Xr) { Xf = Xr; }   // a train across the circuit seam: draw from its rear
			const float TW = FMath::Max(6.f, Xf - Xr);
			PanelTile(Canvas, Xr, BoxY + 11.f, TW, BoxH - 16.f, PanelText);
			PanelLabel(Canvas, Xr + TW * 0.5f - 3.f, BoxY - 16.f, FString::Printf(TEXT("%d"), t), PanelText);
			PanelTile(Canvas, Xr + TW * 0.5f - 0.5f, BoxY - 4.f, 1.f, 14.f, PanelText);
		}

		// Scale, so the strip is a drawing rather than a picture.
		PanelLabel(Canvas, StripX, BoxY + BoxH + 6.f, TEXT("0"), PanelDim);
		PanelLabel(Canvas, StripX + StripW - 44.f, BoxY + BoxH + 6.f,
			FString::Printf(TEXT("%.0f m"), Total), PanelDim);

		// MAINTENANCE: what the SECOND detection method counted, under the block it
		// counted it for. The strip above paints a disagreeing block red, which says
		// where; this says what the counter actually believes, which is the
		// difference between "something is wrong here" and a diagnosis. A block
		// reading 2 is a collision derived from switches alone; one reading below
		// zero is a missed trip or a bad seed, and is a LIE rather than a collision.
		//
		// Aligned to the blocks through the same widened mapping, or a number would
		// sit under a block it does not describe.
		if (bMaint && Counter)
		{
			for (int32 b = 0; b < NumBlocks; ++b)
			{
				const std::size_t B = static_cast<std::size_t>(b);
				if (B >= Counter->NumBlocks())
				{
					break;
				}
				const int N = Counter->TrainsIn(B);
				const bool bAgrees = Counter->IsOccupied(B) == Signals->Occupies(B);
				PanelLabel(Canvas, BlockX(B) + 2.f, BoxY + BoxH + 19.f,
					FString::Printf(TEXT("%d"), N),
					(!bAgrees || N > 1 || N < 0) ? PanelRed : PanelDim);
			}
		}
		Ty += StripH;
	}

	// ---- THE CONTROLLER ITSELF (maintenance only) ----------------------------
	//
	// The cabinet has a PLC in it, so the panel shows one — the same reasoning
	// that gave every drive a VFD module. Mode, scan health, and the identity of
	// the program it is running, which is the field a real installation cares
	// most about and the one that catches a program built for another layout.
	if (bMaint)
	{
		Ty += 4.f;
		PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
		PanelLabel(Canvas, Lx, Ty, TEXT("CONTROLLER"), PanelDim);

		const EPlcMode M = Plc.GetMode();
		const bool bRunning = Plc.OutputsEnabled();
		const TCHAR* ModeName = M == EPlcMode::Run ? TEXT("RUN")
			: (M == EPlcMode::Program ? TEXT("PROGRAM") : TEXT("STOPPED"));

		PanelTile(Canvas, Lx + 130.f, Ty + 2.f, 7.f, 7.f,
			bRunning ? PanelGreen : (Plc.IsFaulted() ? PanelRed : PanelAmber));
		PanelLabel(Canvas, Lx + 142.f, Ty, ModeName,
			bRunning ? PanelGreen : (Plc.IsFaulted() ? PanelRed : PanelAmber));

		// The program's identity, truncated. It is a digest of the derived blocks
		// and zones, so two rides showing the same one are running the same
		// control structure — which is the whole point of printing it.
		PanelLabel(Canvas, Lx + 226.f, Ty,
			FString::Printf(TEXT("PGM %08x"),
				static_cast<uint32>(Plc.ProgramIdentity() & 0xFFFFFFFFull)),
			Plc.ProgramMatchesLayout() ? PanelDim : PanelRed);

		// Why it is not running, when it is not. "The ride will not start" with
		// no reason attached is the commonest complaint about real ride control,
		// and the machine always knows.
		const char* Why = Plc.WhyNotRun();
		if (Why != nullptr)
		{
			PanelLabel(Canvas, Lx + 320.f, Ty, UTF8_TO_TCHAR(Why), PanelAmber);
		}
		Ty += Row;
	}

	// ---- DETECTION: the two methods, stated side by side (maintenance only) ----
	//
	// Agreement is the normal case and is still worth showing, because a second
	// means of detection that is never SEEN to agree is indistinguishable from one
	// that is not running. This is the line that says the cross-check is alive.
	//
	// Operator view leaves it out deliberately: a disagreement already trips the
	// E-stop and names itself in the events, and there is nothing an operator can
	// do with the per-block breakdown that the stop has not already done for them.
	if (bMaint)
	{
		Ty += 4.f;
		PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
		PanelLabel(Canvas, Lx, Ty, TEXT("DETECTION"), PanelDim);

		if (Counter)
		{
			const int32 N = FMath::Min(static_cast<int32>(Counter->NumBlocks()), NumBlocks);
			int32 Agree = 0;
			int32 Over = 0;
			for (int32 b = 0; b < N; ++b)
			{
				const std::size_t B = static_cast<std::size_t>(b);
				if (Counter->IsOccupied(B) == Signals->Occupies(B)) { ++Agree; }
				if (Counter->IsOverOccupied(B)) { ++Over; }
			}
			FString S = FString::Printf(TEXT("COUNTER v INTERLOCKING   %d/%d AGREE"), Agree, N);
			if (Over > 0)
			{
				S += FString::Printf(TEXT("   %d OVER"), Over);
			}
			PanelLabel(Canvas, Lx + 130.f, Ty, S,
				(Agree < N || Over > 0) ? PanelRed : PanelGreen);
		}
		else
		{
			// No counter on an open layout, and that is a property of the layout
			// rather than a fault: FBlockCounter counts over a RING, so sensor 0
			// going low is a train being PLACED at the start, not one leaving the
			// end. Saying so beats an empty row that looks like a dead instrument.
			PanelLabel(Canvas, Lx + 130.f, Ty,
				TEXT("SPAN ONLY — no counter on an open layout"), PanelDim);
		}
		Ty += Row;
	}

	// ---- DRIVES: a module per powered run --------------------------------------
	//
	// THE ONE SECTION THE TWO VIEWS GENUINELY DISAGREE ABOUT. Commanded against
	// output against motor feedback, and torque, are how you tell a slipping tyre
	// from a ramping one — engineering questions, on a page an engineer opens. An
	// operator dispatching trains needs one word per drive and cannot act on the
	// numbers, and a real console reflects that: motor current is not on it.
	//
	// So the operator gets the STATE and the maintainer gets the instrument, both
	// off the same live reading. Neither is a summary of the other — they are two
	// different questions about one drive.
	Ty += 4.f;
	PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
	// Headings at the SAME x as the values they head. Spelled out in one padded
	// string they drifted, because the small font is not monospace.
	PanelLabel(Canvas, Lx, Ty, TEXT("DRIVES"), PanelDim);
	if (bMaint)
	{
		PanelLabel(Canvas, Lx + 130.f, Ty, TEXT("CMD"), PanelDim);
		PanelLabel(Canvas, Lx + 178.f, Ty, TEXT("OUT"), PanelDim);
		PanelLabel(Canvas, Lx + 226.f, Ty, TEXT("MOTOR"), PanelDim);
		PanelLabel(Canvas, Lx + 288.f, Ty, TEXT("TORQUE"), PanelDim);
	}
	Ty += Row;

	for (int32 z = 0; z < NumDrives; ++z)
	{
		const std::size_t Z = static_cast<std::size_t>(z);
		const FDriveReading& R = Drives->Read(Z);
		const bool bFault = Drives->IsFaulted(Z);
		const ETUSegmentZone Kind = ZoneSpans.IsValidIndex(z)
			? ZoneSpans[z].Kind : ETUSegmentZone::None;

		PanelLabel(Canvas, Lx, Ty, FString::Printf(TEXT("Z%d %s"), z, ZoneKindName(Kind)),
			bFault ? PanelRed : PanelText);

		if (!bMaint)
		{
			// FOUR STATES, and every one is a fact the drive holds about ITSELF —
			// no inference about trains, which a drive has no way to make. RAMPING
			// is the state that only became sayable when a command stopped taking
			// effect instantly, and it is the one that explains a dispatch being
			// refused by the pre-launch term.
			const TCHAR* State = TEXT("STOPPED");
			FLinearColor Col = PanelDim;
			if (bFault)                          { State = TEXT("FAULT");   Col = PanelRed; }
			else if (!Drives->IsReady(Z))        { State = TEXT("RAMPING"); Col = PanelAmber; }
			else if (FMath::Abs(R.Output) > 0.01) { State = TEXT("RUNNING"); Col = PanelGreen; }

			PanelTile(Canvas, Lx + 130.f, Ty + 2.f, 7.f, 7.f, Col);
			PanelLabel(Canvas, Lx + 142.f, Ty, State, Col);
			Ty += Row;
			continue;
		}

		PanelLabel(Canvas, Lx + 130.f, Ty, FString::Printf(TEXT("%5.1f"), R.Commanded), PanelDim);
		PanelLabel(Canvas, Lx + 178.f, Ty, FString::Printf(TEXT("%5.1f"), R.Output), PanelText);
		PanelLabel(Canvas, Lx + 226.f, Ty, FString::Printf(TEXT("%5.1f"), R.Actual),
			R.bLoaded ? PanelCyan : PanelDim);

		// Torque as a bar, because a number that spends its life at 0 or 1 says
		// less at a glance than a bar that fills.
		const float BarX = Lx + 288.f;
		const float BarW = 96.f;
		PanelTile(Canvas, BarX, Ty + 3.f, BarW, 6.f, FLinearColor(0.12f, 0.14f, 0.16f, 1.f));
		PanelTile(Canvas, BarX, Ty + 3.f, BarW * static_cast<float>(R.Load), 6.f,
			bFault ? PanelRed : (R.Load > 0.99 ? PanelAmber : PanelCyan));
		if (bFault)
		{
			// ACKNOWLEDGED is worth a word of its own here and nowhere else: it is
			// the maintainer's own audit trail, and "faulted, nobody has looked" is
			// a different state from "faulted, seen, not yet cleared".
			PanelLabel(Canvas, BarX + BarW + 6.f, Ty,
				Drives->IsAcknowledged(Z) ? TEXT("FAULT ACK") : TEXT("FAULT"), PanelRed);
		}
		else
		{
			// THE STATUSWORD, which is what an engineering page on a real
			// installation shows. Four states this project invented are a fair
			// summary; 6041h is what the drive actually says, and a maintainer
			// who knows CiA 402 reads it without being taught anything.
			static const TCHAR* CiaNames[] = {
				TEXT("NOT RDY"), TEXT("SW ON DIS"), TEXT("RDY SW ON"), TEXT("SWITCHED ON"),
				TEXT("OP ENABLED"), TEXT("QUICK STOP"), TEXT("FLT REACT"), TEXT("FAULT")};
			const FCia402Drive& C = Drives->Cia402(Z);
			PanelLabel(Canvas, BarX + BarW + 6.f, Ty,
				FString::Printf(TEXT("%s %04x"),
					CiaNames[static_cast<int32>(C.State())], C.Statusword()),
				C.ProducesTorque() ? PanelDim : PanelAmber);
		}
		Ty += Row;
	}

	// ---- PLATFORMS: where each one is in its sequence, and what is holding it ----
	if (Platforms.Num() > 0)
	{
		Ty += 4.f;
		PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
		PanelLabel(Canvas, Lx, Ty, TEXT("PLATFORMS"), PanelDim);
		Ty += Row;

		for (const FTUPlatform& P : Platforms)
		{
			const bool bReady = P.Process.IsReadyToDispatch();
			const ETUSegmentZone Kind = ZoneSpans.IsValidIndex(P.Zone)
				? ZoneSpans[P.Zone].Kind : ETUSegmentZone::None;

			PanelTile(Canvas, Lx, Ty + 3.f, 6.f, 6.f, bReady ? PanelGreen : PanelDim);
			PanelLabel(Canvas, Lx + 12.f, Ty,
				FString::Printf(TEXT("Z%d %s"), P.Zone, ZoneKindName(Kind)), PanelText);
			PanelLabel(Canvas, Lx + 130.f, Ty, PhaseName(P.Process.GetPhase()),
				bReady ? PanelGreen : PanelAmber);

			// WHAT IS HOLDING IT, which is the reason the station's gates are
			// modelled one at a time. "Not ready" is useless to somebody standing
			// on the platform; "restraints not locked" is somewhere to go and look.
			const FString Holding = UTF8_TO_TCHAR(P.Process.WhatIsHolding());
			if (!Holding.IsEmpty())
			{
				PanelLabel(Canvas, Lx + 226.f, Ty, Holding, PanelDim);
			}
			else if (P.Process.GetPhase() == EStationPhase::Departing)
			{
				// A DEPARTING TRAIN IS NOT A BLOCKED ONE, and the permissive text
				// below would call it one. The release is latched, so `bReady`
				// stays true while the train rolls off its mark — and the block it
				// is being asked about is the block it has just entered ITSELF, so
				// CanRelease says no and the row read "DEPARTING · DISPATCH — block
				// ahead". Both halves true, the pair nonsense.
				//
				// The reason column answers "why has it not gone". A train that has
				// gone has no answer to give, and the phase word already says so.
			}
			else if (bReady)
			{
				// Ready and still standing means the OTHER half of the AND is
				// saying no, which is the distinction an operator most wants and
				// can least otherwise see.
				const double AtS = ZoneSpans.IsValidIndex(P.Zone) ? ZoneSpans[P.Zone].StartS : 0.0;
				const bool bBlocked = !Signals->CanRelease(0, AtS);
				// PRE-LAUNCH gets its own words, because "waiting" for a block that
				// is occupied and "waiting" for a launch that has not armed are two
				// different waits and an operator needs to know which.
				const bool bNotArmed = !DeviceAheadIsReady(AtS);

				// ADVANCE OR DISPATCH, and a real console has them as separate
				// labelled buttons. They are not the same move: an advance shuffles
				// a train up to the next position on the same platform, still with
				// riders boarding behind it; a dispatch sends it onto the course
				// and is the last point anybody can change their mind. The model
				// can already tell them apart — it is whether the next block along
				// is another platform position or open track.
				const TCHAR* Verb = TEXT("DISPATCH");
				if (Signals->NumBlocks() > 0)
				{
					const std::size_t Here = Signals->BlockAt(AtS + 0.01);
					const std::size_t Next = (Here + 1) % Signals->NumBlocks();
					const double NextS = Signals->Boundaries()[Next];
					for (const FTUPlatform& Q : Platforms)
					{
						if (ZoneSpans.IsValidIndex(Q.Zone)
							&& FMath::Abs(ZoneSpans[Q.Zone].StartS - NextS) < 0.01)
						{
							Verb = TEXT("ADVANCE");
						}
					}
				}
				FString Why = Verb;
				if (bBlocked)      { Why = FString::Printf(TEXT("%s — block ahead"), Verb); }
				else if (bNotArmed) { Why = FString::Printf(TEXT("%s — PRE-LAUNCH"), Verb); }
				PanelLabel(Canvas, Lx + 226.f, Ty, Why,
					(bBlocked || bNotArmed) ? PanelAmber : PanelGreen);
			}
			Ty += Row;
		}

		// ---- THE CONSOLE, as lamps ------------------------------------------
		//
		// Straight off an operator's panel: CONTROL POWER, RESTRAINTS, GATES,
		// DISPATCH, E-STOP RESET, EMERGENCY STOP — each an illuminated control that
		// shows its own state, and every one live.
		//
		// THEY ARE PRESSABLE NOW. This said "a screen cannot be pressed, so these
		// are indicators rather than buttons", which the menu, the segment editor
		// and the diagnostics list had already disproved three times over.
		//
		// AND A MOUSE HAS BOTH EDGES, WHICH IS THE PART THAT MATTERS. Two of these
		// controls are only correct if the release is real: the dispatch button is
		// anti-tie-down, so holding it does not dispatch train after train, and the
		// E-stop reset is MONITORED and fires on the release rather than the press.
		// A pointer models both exactly — press to hold, let go to release — so
		// clicking is not a weaker stand-in for the keys here, it is the same
		// signal arriving another way.
		//
		// WHAT IS STILL AN INDICATOR IS DELIBERATE. Harness, gates and dispatch
		// ready report; they are not buttons, because in this model the CREW owns
		// the banks and the interlocking grants the permission. Making them
		// pressable would move an authority, which is a design change rather than
		// a wiring one.
		//
		// It shows ONE platform: whichever has a train and is furthest along, which
		// is the one an operator standing at a station console is working. Nothing
		// here is invented — a control this cannot honestly light is left off it,
		// which is why there is no FLOOR RAISE/LOWER lamp.
		const FTUPlatform* Console = const_cast<ATUCoasterRide*>(this)->ConsolePlatformPtr();

		Ty += 4.f;
		PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
		// THE HEADER IS THE POSITION SELECTOR. It says where the operator is
		// standing, and says whether that is following the train or pinned — which
		// matters, because "the gates I just shut" is a different sentence at each
		// position of a multi-position platform.
		ConsoleRects.Reset();
		ConsoleAction.Reset();
		ConsoleRects.Add(FVector4(Lx, Ty, Lx + 220.f, Ty + Row - 2.f));
		ConsoleAction.Add(7);
		PanelLabel(Canvas, Lx, Ty, Console != nullptr
			? FString::Printf(TEXT("CONSOLE · Z%d%s"), Console->Zone,
				ConsolePlatform >= 0 ? TEXT("  PINNED") : TEXT("  follows train"))
			: FString(TEXT("CONSOLE · no platform")), PanelDim);
		Ty += Row;

		auto Lamp = [&](float Lx2, const TCHAR* Label, bool bLit, const FLinearColor& Col)
		{
			PanelTile(Canvas, Lx2, Ty + 2.f, 7.f, 7.f, bLit ? Col : FLinearColor(0.16f, 0.18f, 0.20f, 1.f));
			PanelLabel(Canvas, Lx2 + 12.f, Ty, Label, bLit ? Col : PanelDim);
		};

		const bool bStop = Drives->IsEmergencyStopped();
		// LOCK HARNESS / GATES / DISPATCH READY, in the order and the words a real
		// console uses. "DISPATCH READY" is its own lamp on all three panels rather
		// than a property of the dispatch button, because the machine granting
		// permission and a person taking it are two different events.
		// HARNESS shows the bank's own sensors, not the switch position. Commanded
		// closed with bars still travelling reads amber with a count, because "told
		// to close" and "closed" are different facts and the gap between them is
		// exactly what a walk-round is looking for.
		if (Console != nullptr && Console->Crew.Restraints.IsCommandedClosed()
			&& !Console->Crew.Restraints.IsClosedAndLocked())
		{
			PanelTile(Canvas, Lx, Ty + 2.f, 7.f, 7.f, PanelAmber);
			PanelLabel(Canvas, Lx + 12.f, Ty,
				FString::Printf(TEXT("HARNESS %d/%d"),
					Console->Crew.Restraints.GroupsConfirmed(),
					Console->Crew.Restraints.Groups), PanelAmber);
		}
		else
		{
			Lamp(Lx, TEXT("HARNESS LOCKED"),
				Console != nullptr && Console->Inputs.bRestraintsLocked, PanelGreen);
		}
		// GATES read their own sensors too, so a jammed section shows as a count
		// rather than as a dark lamp indistinguishable from "not commanded yet".
		if (Console != nullptr && Console->Crew.Gates.IsCommandedClosed()
			&& !Console->Crew.Gates.IsClosedAndLocked())
		{
			PanelTile(Canvas, Lx + 116.f, Ty + 2.f, 7.f, 7.f, PanelAmber);
			PanelLabel(Canvas, Lx + 128.f, Ty,
				FString::Printf(TEXT("GATES %d/%d"),
					Console->Crew.Gates.GroupsConfirmed(),
					Console->Crew.Gates.Groups), PanelAmber);
		}
		else
		{
			Lamp(Lx + 116.f, TEXT("GATES"),
				Console != nullptr && Console->Crew.Gates.IsClosedAndLocked(), PanelGreen);
		}
		Lamp(Lx + 186.f, TEXT("DISPATCH READY"),
			Console != nullptr && Console->Process.IsReadyToDispatch(), PanelGreen);
		Lamp(Lx + 300.f, TEXT("E-STOP"), bStop, PanelRed);
		// RESET is only a live control once everything has been acknowledged, so it
		// lights differently from the thing it cannot yet do.
		Lamp(Lx + 372.f, bStop && Drives->AnyUnacknowledged() ? TEXT("ACK") : TEXT("RESET"),
			bStop, bStop && Drives->AnyUnacknowledged() ? PanelAmber : PanelCyan);
		Ty += Row;

		// ---- THE CONTROLS, as things you press ------------------------------
		//
		// A SECOND ROW, not lamps made clickable. On a real console the indicators
		// and the controls are physically different objects in different places,
		// and a lamp you can press is a lamp somebody presses by accident while
		// pointing at what it says.
		Ty += 2.f;

		// TWO ROWS TALL, and DISPATCH and E-STOP in the large face: they are the
		// two controls an operator's hand goes to, and on a real console they are
		// the two that are physically bigger than everything else. AUTO/MANUAL
		// and RESET keep the body size inside the same height.
		float Bh = Row * 2.f - 2.f;   // the rows below put it back to one row
		auto Button = [&](float Bx, float Bw, const TCHAR* Label, int32 Action,
			const FLinearColor& Col, bool bEnabled, bool bBig = false)
		{
			ConsoleRects.Add(FVector4(Lx + Bx, Ty, Lx + Bx + Bw, Ty + Bh));
			ConsoleAction.Add(Action);
			// HELD SHOWS AS HELD. A button that looks identical pressed and
			// released is one nobody can tell they are still holding, which for an
			// anti-tie-down control is the whole point of it.
			const bool bHeld = HeldConsoleButton == Action;
			PanelTile(Canvas, Lx + Bx, Ty, Bw, Bh,
				bHeld ? Col : FLinearColor(0.10f, 0.12f, 0.14f, 1.f));
			const FLinearColor Ink = bHeld ? PanelGround : (bEnabled ? Col : PanelDim);
			if (bBig) { PanelLabelBig(Canvas, Lx + Bx + 8.f, Ty + 4.f, Label, Ink); }
			else      { PanelLabel(Canvas, Lx + Bx + 8.f, Ty + (Bh - 13.f) * 0.5f, Label, Ink); }
		};

		Button(0.f, 104.f, TEXT("DISPATCH"), 0, PanelGreen,
			Console != nullptr && Console->Process.IsReadyToDispatch(), true);
		// AUTO/MANUAL is the one that changes what the ride DOES rather than
		// commanding a single action, so it reads as a mode and says which it is
		// in — never as a button labelled with the mode you would be switching to,
		// which is the ambiguity every toggle-labelled control has.
		Button(116.f, 104.f, bManualDispatch ? TEXT("MANUAL") : TEXT("AUTO"), 3,
			PanelCyan, true, false);
		Button(232.f, 104.f, TEXT("E-STOP"), 1, PanelRed, !bStop, true);
		Button(348.f, 104.f,
			bStop && Drives->AnyUnacknowledged() ? TEXT("ACKNOWLEDGE") : TEXT("RESET"), 2,
			bStop && Drives->AnyUnacknowledged() ? PanelAmber : PanelCyan, bStop, false);
		Ty += Bh + 2.f;
		Bh = Row - 2.f;

		// ---- THE OPERATOR'S OWN ROW, when there is an operator -------------
		//
		// NOTHING ON A REAL RIDE HAPPENS ON ITS OWN. A person shuts the gates when
		// the platform is clear and a person confirms by hand that every harness is
		// locked; there is no safety device that does either without somebody.
		// "Automatic" never meant otherwise — the machine PERMITS what is safe and
		// refuses the rest, which is why these three sit beside DISPATCH rather
		// than replacing it.
		//
		// THE CONTROLS ARE HIDDEN WHEN THE SIMULATED CREW HAS THEM, because a
		// control pressed and then immediately overridden by a dwell timer is worse
		// than one that is not offered — but the ROW STILL SAYS SO. A panel that
		// simply lacked the gate switches would read as a ride with no gates, when
		// what is true is that somebody else is working them.
		Ty += 2.f;
		if (!bPlayerIsCrew)
		{
			PanelLabel(Canvas, Lx, Ty,
				TEXT("PLATFORM CREW: SIMULATED  ·  gates, harness and the walk-round are theirs"),
				PanelDim);
			Ty += Row;
		}
		else
		{
			const bool bGatesShut = Console != nullptr && Console->Crew.Gates.IsCommandedClosed();
			const bool bBarsDown = Console != nullptr
				&& Console->Crew.Restraints.IsCommandedClosed();

			// SELECTORS SAY WHERE THEY ARE, never what they would do next. A switch
			// labelled with its opposite is the oldest ambiguity on any panel.
			Button(0.f, 104.f, bGatesShut ? TEXT("GATES SHUT") : TEXT("GATES OPEN"), 4,
				PanelGreen, Console != nullptr);
			Button(116.f, 104.f, bBarsDown ? TEXT("BARS DOWN") : TEXT("BARS UP"), 5,
				PanelGreen, Console != nullptr);
			// THE WALK-ROUND. Not derived from anything and it must not be: it is a
			// person having walked the train and looked at every car. Latched for
			// this train, and the next one needs its own.
			Button(232.f, 220.f,
				Console != nullptr && Console->bOperatorAllClear
					? TEXT("ALL CLEAR GIVEN") : TEXT("ALL CLEAR"), 6,
				PanelCyan, Console != nullptr && !Console->bOperatorAllClear);
			Ty += Row;
		}

		// ---- TRAINS IN SERVICE, which is a MAINTENANCE decision ------------
		//
		// Adding and removing trains is not something an operator does mid-shift --
		// it is a morning or an overnight job, and it happens because a train is due
		// an inspection or the queue does not justify running them all. So it lives
		// in the maintenance view rather than beside DISPATCH.
		//
		// THE CEILING IS SAFETY AND IS NOT NEGOTIABLE FROM HERE. N holding places
		// run N-1 trains; the button simply stops, because a control that let you
		// ask for an unsafe number and then quietly gave you a safe one would be
		// lying about what the ride is doing. Downward there is no limit at all --
		// zero trains is a ride with everything in the shed, which is an ordinary
		// state and was refused until today.
		if (bMaint)
		{
			Ty += 4.f;
			PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
			const int32 Cap = HoldingPlaces == 0 ? 0 : FMath::Max(1, HoldingPlaces - 1);
			const int32 Running = FMath::Clamp(TrainCount, 0, Cap);
			PanelLabel(Canvas, Lx, Ty, FString::Printf(
				TEXT("TRAINS IN SERVICE   %d of %d   (%d holding place(s), one stays free)"),
				Running, Cap, HoldingPlaces), PanelDim);
			Ty += Row;
			Button(0.f, 104.f, TEXT("- SHED"), 8, PanelAmber, Running > 0);
			Button(116.f, 104.f, TEXT("+ RETURN"), 9, PanelGreen, Running < Cap);
			Ty += Row;

			// ---- CARS, which is also a shed decision ----
			//
			// Parks really do run shorter trains on quiet days, so this belongs
			// beside the train count rather than in the geometry editor.
			//
			// THE CEILING IS THE SHORTEST DEVICE A TRAIN CAN PARK ON. Longer than
			// that and the stop mark lands past the far end, nothing trips it, and
			// the train crawls out of its block into the next one -- measured at
			// seven signalling violations in four seconds. So the button stops, and
			// says which device it is stopping against.
			const int32 MaxCars = (CarLengthM > 0.f && ShortestHoldM > 0.0)
				? FMath::Max(1, FMath::FloorToInt(
					(ShortestHoldM - HoldNoseClearanceM) / CarLengthM))
				: 12;
			PanelLabel(Canvas, Lx, Ty, FString::Printf(
				TEXT("CARS PER TRAIN      %d x %.1f m = %.1f m   (shortest device %.1f m)"),
				CarCount, CarLengthM, TrainLengthM, ShortestHoldM), PanelDim);
			Ty += Row;
			Button(0.f, 104.f, TEXT("- CAR"), 10, PanelAmber, CarCount > 1);
			Button(116.f, 104.f, TEXT("+ CAR"), 11, PanelGreen, CarCount < MaxCars);
			if (CarCount >= MaxCars)
			{
				PanelLabel(Canvas, Lx + 232.f, Ty + 1.f,
					TEXT("any longer and it will not fit its shortest block"), PanelDim);
			}
			// SAID WHEN IT BITES, not as a permanent caption. A ceiling nobody has
			// reached is noise; one somebody is pressing against is the answer.
			if (Running >= Cap && Cap > 0)
			{
				PanelLabel(Canvas, Lx + 232.f, Ty + 1.f,
					TEXT("at capacity - add a block brake to park another"), PanelDim);
			}
			Ty += Row;
		}

		// ---- THE EVENT LOG ------------------------------------------------
		//
		// The difference between a status display and a RECORD. Everything above
		// says what is true now; this says what happened, which is the first thing
		// anybody asks after a ride stops. Newest first, and fed by the same places
		// that log, so there is one story rather than two that can disagree.
		if (EventLog.Num() > 0)
		{
			Ty += 4.f;
			PanelTile(Canvas, Lx, Ty + 5.f, W - Pad * 2.f, 1.f, PanelRule);
			PanelLabel(Canvas, Lx, Ty, TEXT("EVENTS"), PanelDim);
			Ty += Row;

			const int32 Show = FMath::Min(EventLog.Num(), 4);
			for (int32 e = 0; e < Show; ++e)
			{
				const FTURideEvent& Ev = EventLog[e];
				PanelLabel(Canvas, Lx, Ty,
					FString::Printf(TEXT("%6.1f s"), Ev.AtSeconds), PanelDim);
				PanelLabel(Canvas, Lx + 56.f, Ty, Ev.Text, Ev.bBad ? PanelRed : PanelDim);
				Ty += Row;
			}
		}
		ConsoleContentH = (Ty - Y) + Pad;
	}
}

// THE OPERATOR'S BUTTON IS THE ONE CATEGORY 1 STOP ON THE RIDE, and every
// automatic trip is Category 0. That split is a risk judgement rather than a
// preference, and IEC 60204-1 leaves it to exactly that:
//
//   A person pressing the button has decided the ride should stop. Nothing is
//   known to be broken, so a drive can be trusted to wind its own output down
//   before power goes — which is what stops a train being dropped mid-push.
//
//   A protective trip — signalling violation, detection disagreement, a counter
//   that has gone inconsistent, a faulted drive — means something IS broken, and
//   often that the thing being asked to perform a controlled stop is the thing
//   that failed. Power goes now.
//
// Both are safe because THE BRAKES ARE FAIL-SAFE: a zone commanded to zero
// bites, so removing power applies the brakes rather than merely ceasing to
// push. Cat 0 stops trains, it does not just stop driving them.
void ATUCoasterRide::PressEmergencyStop()
{
	bEmergencyStop = true;
	if (Drives && Drives->PressEmergencyStopButton("operator"))
	{
		UE_LOG(LogTemp, Error, TEXT("TrackUnlimited: EMERGENCY STOP — operator (Cat 1)."));
		LogEvent(TEXT("EMERGENCY STOP — operator (Cat 1)"));
	}
}

// Channel ranges. Allocated by hand and deliberately far apart: a collision here
// is silent and reads as a phantom transition on an unrelated signal.
namespace TUWatch
{
	constexpr std::size_t Blocks    = 0;
	constexpr std::size_t Platforms = 1000;
	constexpr std::size_t Drives    = 2000;
	constexpr std::size_t Console   = 3000;
	constexpr std::size_t Motion    = 4000;
}

// Which platform, if any, this train is standing at. A platform's bank is the
// only place its state exists, so a train has to be matched to one to sample it.
// Matched by SPAN rather than by a stored index, because the platform genuinely
// does not know which train is over it — the same honesty the stop marks keep.
void ATUCoasterRide::SampleRestraints()
{
	const int32 N = Trains.Num();
	if (TrainGroupClosed.Num() != N)
	{
		TrainGroupClosed.SetNum(N);
	}

	for (int32 t = 0; t < N; ++t)
	{
		const double S = Trains[t]->GetDistance();
		const FTUPlatform* At = nullptr;
		for (const FTUPlatform& P : Platforms)
		{
			if (!ZoneSpans.IsValidIndex(P.Zone)) { continue; }
			if (S >= ZoneSpans[P.Zone].StartS - 0.01 && S < ZoneSpans[P.Zone].EndS)
			{
				At = &P;
				break;
			}
		}
		// Away from a platform the last sample STANDS. The bars did not open
		// because the train left the room that was commanding them.
		if (At == nullptr) { continue; }

		const FCommandedBank& B = At->Crew.Restraints;
		const int32 G = FMath::Max(1, B.Groups);
		const int32 Base = t * G;
		if (TrainGroupState.Num() < Base + G)
		{
			TrainGroupState.SetNum(Base + G);
		}
		for (int32 g = 0; g < G; ++g)
		{
			TrainGroupState[Base + g] = static_cast<uint8>(B.GroupState(g));
		}
		TrainGroupClosed[t] = B.IsCommandedClosed();
	}
}

void ATUCoasterRide::BuildBlockMarks()
{
	BlockMarks.Reset();
	if (!Signals || Track.TotalLength() <= 0.0)
	{
		return;
	}

	// ONE WALK for every boundary. Boundaries are ascending, so AdvanceFrom can
	// carry the frame forward from one to the next — eleven posts cost one
	// integration of the track rather than eleven.
	const std::vector<double>& B = Signals->Boundaries();
	FTrackFrame Walk = Track.EvaluateAt(0.0);
	double At = 0.0;

	for (std::size_t i = 0; i < B.size(); ++i)
	{
		const double To = FMath::Clamp(B[i], 0.0, Track.TotalLength());
		if (To > At)
		{
			Walk = Track.AdvanceFrom(Walk, At, To);
			At = To;
		}
		FTUBlockMark M;
		M.World = ToWorld(Walk.Position);
		// Kept in WORLD space, converted once here rather than per frame, and
		// through the same mapping the rest of the drawing uses — a post that
		// leans the wrong way on a banked boundary would be its own small lie.
		//
		// NORMALISED, and it was not. Differencing two ToWorld calls a metre
		// apart gives a vector of length 100, because ToWorld converts metres to
		// centimetres — so this was a UNIT-PER-METRE vector wearing the name of a
		// direction, and every multiplier applied to it downstream came out 100x.
		// The gate meant to stand 4.2 m over the rails stood 420 m over them.
		M.Up = (ToWorld(Walk.Position + Walk.Up) - M.World).GetSafeNormal();
		M.Lateral = (ToWorld(Walk.Position + Walk.Lateral) - M.World).GetSafeNormal();
		BlockMarks.Add(M);
	}
}

void ATUCoasterRide::DrawBlockMarkers() const
{
	if (!bShowBlockMarkers || !Signals || BlockMarks.Num() == 0)
	{
		return;
	}

	for (int32 i = 0; i < BlockMarks.Num(); ++i)
	{
		const FTUBlockMark& M = BlockMarks[i];
		const std::size_t B = static_cast<std::size_t>(i);

		// A boundary is the START of block i, so it carries block i's colour —
		// THE SAME COLOURS THE PANEL'S BLOCK STRIP USES, because the strip and
		// the track are meant to be one object seen twice. Amber occupied, cyan
		// in its overlap, green clear.
		const EBlockState State = Signals->GetState(B);
		FColor Col = State == EBlockState::Occupied ? FColor(250, 158, 41)
			: (State == EBlockState::Buffer ? FColor(89, 189, 255) : FColor(89, 209, 115));

		// A disagreement between the two detection methods paints it RED, exactly
		// as it does on the panel — so a cross-check trip can be found on the
		// track rather than only read about.
		if (Counter && B < Counter->NumBlocks()
			&& (Counter->IsOccupied(B) != Signals->Occupies(B) || Counter->IsOverOccupied(B)))
		{
			Col = FColor(215, 60, 50);
		}

		// A gate across the track rather than a post beside it: the boundary is a
		// line the train crosses, and a marker off to one side reads as scenery.
		// CENTIMETRES, because M.Up and M.Lateral are unit vectors in UE space.
		// A gate 4.2 m over the rails and 1.6 m under them, 2.6 m to each side —
		// which is about a train and a half wide and reads as a gantry rather
		// than a wall.
		const FVector Top = M.World + M.Up * 420.f;
		const FVector Foot = M.World - M.Up * 160.f;
		const FVector Half = M.Lateral * 260.f;

		DrawDebugLine(GetWorld(), Foot, Top, Col, false, -1.f, 0, 4.f);
		DrawDebugLine(GetWorld(), Top - Half, Top + Half, Col, false, -1.f, 0, 4.f);
		DrawDebugLine(GetWorld(), Foot - Half, Foot + Half, Col, false, -1.f, 0, 2.f);
	}
}

void ATUCoasterRide::DrawRestraints() const
{
	if (!bShowRestraints || Trains.Num() == 0 || Platforms.Num() == 0)
	{
		return;
	}
	// Group count comes from the banks, which all carry the same authored number.
	const int32 G = FMath::Max(1, Platforms[0].Crew.Restraints.Groups);
	const double Len = FMath::Max(1.0, static_cast<double>(TrainLengthM));

	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		const int32 Base = t * G;
		if (!TrainGroupState.IsValidIndex(Base + G - 1)) { continue; }
		const bool bClosed = TrainGroupClosed.IsValidIndex(t) && TrainGroupClosed[t];

		for (int32 g = 0; g < G; ++g)
		{
			// Group g spans its share of the train, measured from the rear. The
			// nine sample points are a physics discretisation and are NOT cars;
			// the groups are the things with a sensor on them.
			const double Offset = -Len * 0.5 + (static_cast<double>(g) + 0.5) * (Len / G);
			const FTrackFrame& F = Trains[t]->GetFrameAt(Offset);

			const auto State =
				static_cast<FCommandedBank::EGroupState>(TrainGroupState[Base + g]);

			// STUCK IS RED WHATEVER IT WAS DOING. A bar that will not close is a
			// train that cannot go; a bar that will not OPEN is a rider who cannot
			// get out, which is a different emergency and just as much a fault.
			FColor Col = FColor(90, 95, 100);                       // open, at rest
			if (State == FCommandedBank::EGroupState::Stuck)        { Col = FColor(215, 60, 50); }
			else if (State == FCommandedBank::EGroupState::Travelling) { Col = FColor(235, 160, 30); }
			else if (bClosed)                                       { Col = FColor(70, 200, 110); }

			// Drawn ABOVE the rails at the heartline, where a bar actually is, and
			// as a box rather than a tint so it reads from outside the train.
			const FVec3 P = F.Position + F.Up * 0.35;
			const float Half = static_cast<float>(Len / G) * 0.5f * 0.85f;
			DrawDebugBox(GetWorld(), ToWorld(P),
				FVector(Half * 100.f, 55.f, 30.f), ToWorldRotation(F), Col,
				false, -1.f, 0, State == FCommandedBank::EGroupState::Stuck ? 4.f : 2.f);
		}
	}
}

void ATUCoasterRide::PublishShowEvents()
{
	// TIER 3'S ONE INTERFACE. Events out, fixture commands out the other side, and
	// no path by which either can reach the ride — not because something checks,
	// but because there is nowhere to name a train.
	//
	// The same three sources the panel draws from, because a cue that fired on
	// something the operator could not see would be a fourth idea of the ride's
	// state. Blocks, sensors, platforms: sensor -> pyro, sensor -> camera,
	// station phase -> audio, block occupied -> scenery are all this one
	// subscription with a different fixture on the end.
	if (!Signals)
	{
		return;
	}

	ShowPublisher.BeginScan(ScanNumber);
	for (std::size_t b = 0; b < Signals->NumBlocks(); ++b)
	{
		ShowPublisher.Observe(ERideEventKind::BlockState, static_cast<int>(b),
			static_cast<int>(Signals->GetState(b)));
	}
	if (StopMarks)
	{
		for (std::size_t i = 0; i < StopMarks->Num(); ++i)
		{
			ShowPublisher.Observe(ERideEventKind::SensorTrip, static_cast<int>(i),
				StopMarks->IsBlocked(i) ? 1 : 0);
		}
	}
	for (int32 p = 0; p < Platforms.Num(); ++p)
	{
		ShowPublisher.Observe(ERideEventKind::StationPhase, p,
			static_cast<int>(Platforms[p].Process.GetPhase()));
	}

	// Nothing authors triggers yet, so this delivers into an empty list and the
	// return is dropped. That is the honest state of Tier 3 and it is deliberately
	// visible here rather than hidden behind an `if (bShowEnabled)` — the wiring
	// is what was missing, and a fixture layer plugs into this line.
	ShowBus.Deliver(ShowPublisher.Scanned());
}

void ATUCoasterRide::LogTransitions()
{
	if (!bLogStateTransitions || !Signals || !Drives)
	{
		return;
	}

	// ---- Blocks. Clear / occupied / buffer, per block.
	for (std::size_t b = 0; b < Signals->NumBlocks(); ++b)
	{
		const EBlockState S = Signals->GetState(b);
		const int Code = S == EBlockState::Occupied ? 1 : (S == EBlockState::Buffer ? 2 : 0);
		if (StateWatch.ChangedFrom(TUWatch::Blocks + b, Code))
		{
			static const TCHAR* Names[] = {TEXT("CLEAR"), TEXT("OCCUPIED"), TEXT("BUFFER")};
			LogEvent(FString::Printf(TEXT("block %d  %s -> %s"), static_cast<int32>(b),
				Names[StateWatch.Previous()], Names[Code]), false);
		}
	}

	// ---- Platforms. The sequence, which is what a dwell argument is settled by.
	for (int32 i = 0; i < Platforms.Num(); ++i)
	{
		const int Code = static_cast<int>(Platforms[i].Process.GetPhase());
		if (StateWatch.ChangedFrom(TUWatch::Platforms + static_cast<std::size_t>(i), Code))
		{
			LogEvent(FString::Printf(TEXT("Z%d  %s -> %s"), Platforms[i].Zone,
				PhaseName(static_cast<EStationPhase>(StateWatch.Previous())),
				PhaseName(static_cast<EStationPhase>(Code))), false);
		}
	}

	// ---- Drives. The four states the operator view shows, so the log and the
	// panel cannot tell different stories.
	for (std::size_t z = 0; z < Drives->Num(); ++z)
	{
		const FDriveReading& R = Drives->Read(z);
		int Code = 0;                                            // STOPPED
		if (Drives->IsFaulted(z))                { Code = 3; }   // FAULT
		else if (!Drives->IsReady(z))            { Code = 2; }   // RAMPING
		else if (FMath::Abs(R.Output) > 0.01)    { Code = 1; }   // RUNNING

		if (StateWatch.ChangedFrom(TUWatch::Drives + z, Code))
		{
			static const TCHAR* Names[] = {TEXT("STOPPED"), TEXT("RUNNING"),
										   TEXT("RAMPING"), TEXT("FAULT")};
			LogEvent(FString::Printf(TEXT("drive Z%d  %s -> %s  (cmd %.1f out %.1f)"),
				static_cast<int32>(z), Names[StateWatch.Previous()], Names[Code],
				R.Commanded, R.Output), Code == 3);
		}
	}

	// ---- TRAIN MOTION, which is what settles a throughput argument.
	//
	// A block releases when the REAR clears it, not when the train stops — that is
	// what OCCUPIED -> BUFFER already means. But without this you cannot tell from
	// a log whether the train ahead was still trucking to its mark while the next
	// one was released, or had already parked, and those are a ride that overlaps
	// its moves and a ride that queues them.
	//
	// CRAWLING is its own state rather than a slow RUNNING because it is a
	// different activity: the pad has stopped the train and the drive tyres are
	// conveying it to its stop mark, which on the mid-course brake is 67 m of
	// travel at walking pace. That stretch is exactly the window a busy operation
	// wants to overlap into.
	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		const double V = FMath::Abs(Trains[t]->GetSpeed());
		const int Code = V < 0.05 ? 0 : (V < 2.5 ? 1 : 2);
		if (StateWatch.ChangedFrom(TUWatch::Motion + static_cast<std::size_t>(t), Code))
		{
			static const TCHAR* Names[] = {TEXT("STOPPED"), TEXT("CRAWLING"), TEXT("RUNNING")};
			LogEvent(FString::Printf(TEXT("train %d  %s -> %s  at %.0f m"), t,
				Names[StateWatch.Previous()], Names[Code],
				Trains[t]->GetDistance()), false);
		}
	}

	// ---- THE CONSOLE CONTACTS, which are the ones a still frame cannot answer.
	//
	// "Is the harness lamp dark because the crew has already released, or because
	// it never lit?" is not a question a screenshot can settle, and it is exactly
	// the question worth asking about a securing sequence. Logged per platform
	// rather than for the one console the panel picks, or the answer depends on
	// which platform happened to be selected when you looked.
	for (int32 i = 0; i < Platforms.Num(); ++i)
	{
		const FTUPlatform& P = Platforms[i];
		const std::size_t Base = TUWatch::Console + static_cast<std::size_t>(i) * 8;

		// A BANK HAS THREE STATES, NOT A COUNT. `GroupsConfirmed` counts groups
		// that have reached their COMMANDED position, whichever direction that
		// is — so "4/4" means fully locked when closing and fully OPEN when
		// releasing. The first version of this logged it as "harness 4/4 locked"
		// either way, which read as authoritative and said the opposite of the
		// truth half the time. A lying instrument is worse than no instrument.
		//
		// So log what it MEANS, and keep the count only where it is the
		// interesting part: mid-travel, where "commanded closed but 3 of 4" is
		// the failure a walk-round exists to find.
		auto BankState = [](const FCommandedBank& B) -> int
		{
			if (B.IsClosedAndLocked()) { return 2; }
			if (B.IsFullyOpen())       { return 0; }
			return 1;                  // travelling, or stuck
		};
		auto BankWords = [](const FCommandedBank& B, int S) -> FString
		{
			if (S == 2) { return FString(TEXT("CLOSED AND LOCKED")); }
			if (S == 0) { return FString(TEXT("open")); }
			return FString::Printf(TEXT("%s — %d/%d"),
				B.IsCommandedClosed() ? TEXT("closing") : TEXT("releasing"),
				B.GroupsConfirmed(), B.Groups);
		};

		const int RState = BankState(P.Crew.Restraints);
		if (StateWatch.ChangedFrom(Base + 0, RState))
		{
			LogEvent(FString::Printf(TEXT("Z%d  restraints %s"), P.Zone,
				*BankWords(P.Crew.Restraints, RState)), false);
		}
		const int GState = BankState(P.Crew.Gates);
		if (StateWatch.ChangedFrom(Base + 1, GState))
		{
			LogEvent(FString::Printf(TEXT("Z%d  gates %s"), P.Zone,
				*BankWords(P.Crew.Gates, GState)), false);
		}
		if (StateWatch.ChangedFrom(Base + 3, P.Process.IsReadyToDispatch() ? 1 : 0))
		{
			LogEvent(FString::Printf(TEXT("Z%d  dispatch permission %s"), P.Zone,
				P.Process.IsReadyToDispatch() ? TEXT("GRANTED") : TEXT("withdrawn")), false);
		}
	}
}

void ATUCoasterRide::LogEvent(const FString& Text, bool bBad)
{
	// Newest first, capped. Oldest falls off the end rather than the ring wrapping
	// in place, because the panel wants "the last few" and a wrapped array has to
	// be unwrapped to give it.
	EventLog.Insert(FTURideEvent{RideClock, Text, bBad}, 0);
	// MIRRORED TO THE ENGINE LOG, WHICH IS WHERE IT PERSISTS. The in-memory ring
	// is what the panel reads while you are standing there; the file is what you
	// read afterwards, and "afterwards" is when every interesting question gets
	// asked. UE stamps it with a wall clock and a frame number for free, so the
	// two timebases sit side by side: RideClock says where in the ride, the file
	// says when in the session.
	//
	// Its own category rather than LogTemp, so `LogTUEvents` can be filtered to on
	// its own — with transitions on, this is the loudest thing in the file.
	UE_LOG(LogTUEvents, Log, TEXT("[%7.2f] %s"), RideClock, *Text);

	// Deeper when transitions are being recorded: eight is the right size for a
	// panel that shows four, and useless for a securing sequence that spends a
	// dozen lines getting a train out of the station.
	const int32 Keep = bLogStateTransitions ? 256 : 8;
	if (EventLog.Num() > Keep)
	{
		EventLog.SetNum(Keep);
	}
}

void ATUCoasterRide::AcknowledgeFaults()
{
	if (!Drives || !Drives->AnyUnacknowledged())
	{
		return;
	}
	for (std::size_t z = 0; z < Drives->Num(); ++z)
	{
		if (Drives->IsFaulted(z) && !Drives->IsAcknowledged(z))
		{
			Drives->AcknowledgeFault(z);
			LogEvent(FString::Printf(TEXT("zone %d fault acknowledged"),
				static_cast<int32>(z)), false);
			UE_LOG(LogTemp, Warning,
				TEXT("TrackUnlimited: drive %d fault ACKNOWLEDGED — seen, not fixed. "
					"[End] to reset once it is."),
				static_cast<int32>(z));
		}
	}
}

// The reset button as a CONTACT rather than an event. The drives layer decides
// when the 0-1-0 sequence is complete; this just reports the switch, which is all
// a real panel wire does.
//
// The acknowledgement check stays HERE, on the press, so an operator is told why
// nothing happened at the moment they press rather than at the moment they let
// go — and so the ordering rule keeps living in the layer that owns it.
// Turn the key. Refused with a readable reason rather than silently ignored:
// "the ride will not start" with no explanation is the commonest complaint about
// real ride control, and the machine always knows why.
void ATUCoasterRide::SetPlcMode(EPlcMode Wanted)
{
	if (Plc.RequestMode(Wanted))
	{
		static const TCHAR* Names[] = {TEXT("STOPPED"), TEXT("PROGRAM"), TEXT("RUN")};
		LogEvent(FString::Printf(TEXT("PLC mode -> %s"),
			Names[static_cast<int32>(Wanted)]), false);
		return;
	}
	const char* Why = Plc.WhyNotRun();
	UE_LOG(LogTemp, Warning, TEXT("TrackUnlimited: PLC refused RUN — %s."),
		Why ? UTF8_TO_TCHAR(Why) : TEXT("not powered"));
	LogEvent(FString::Printf(TEXT("PLC refused RUN — %s"),
		Why ? UTF8_TO_TCHAR(Why) : TEXT("not powered")));
}

// The operator's walkdown. A controller that came up knowing where trains are
// would be inventing occupancy it cannot possibly have watched.
void ATUCoasterRide::DeclareCourseClear()
{
	if (Plc.DeclareCourseClear())
	{
		LogEvent(TEXT("course declared clear"), false);
	}
}

void ATUCoasterRide::PressResetButton()
{
	if (Drives && Drives->AnyUnacknowledged())
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrackUnlimited: reset REFUSED — a drive fault has not been acknowledged. "
				"[Home] to acknowledge, then [End]."));
		return;
	}
	if (Drives) { Drives->ScanResetInput(true); }
}

void ATUCoasterRide::ReleaseResetButton()
{
	if (!Drives)
	{
		return;
	}
	// A fault raised BETWEEN the press and the release. Abandon the sequence
	// rather than complete it — otherwise the acknowledge-before-reset rule leaks
	// out through a gap in time, and a stop nobody has read clears itself.
	if (Drives->AnyUnacknowledged())
	{
		Drives->AbortReset();
		return;
	}
	if (Drives->ScanResetInput(false))
	{
		ResetEmergencyStop();
	}
}

void ATUCoasterRide::ResetEmergencyStop()
{
	// Cleared only here, never because the condition passed. Same reasoning as a
	// drive fault needing a reset: a stop nobody has looked at has not been dealt
	// with. Drive faults are cleared with it, because an operator resetting the
	// ride has been to look at what tripped it.
	// REFUSED WHILE ANYTHING IS UNACKNOWLEDGED. Every real console has ACKNOWLEDGE
	// and RESET as two separate controls, and the order between them is the point:
	// acknowledging says "I have seen this", resetting says "I have dealt with it".
	// A reset nobody had to read first clears faults nobody knows about.
	if (Drives && Drives->AnyUnacknowledged())
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrackUnlimited: reset REFUSED — a drive fault has not been acknowledged. "
				"[A] to acknowledge, then [End]."));
		return;
	}

	bEmergencyStop = false;
	if (Drives)
	{
		Drives->ResetEmergencyStop();
		for (std::size_t z = 0; z < Drives->Num(); ++z)
		{
			Drives->ResetFault(z);
		}
	}
	ReportedDriveFault.Reset();
	UE_LOG(LogTemp, Warning, TEXT("TrackUnlimited: emergency stop reset."));
	LogEvent(TEXT("emergency stop RESET"), false);
}

void ATUCoasterRide::ServeStations(float DeltaSeconds)
{
	// The station's inputs, from instruments, once per scan. Only two of the six
	// are real here and both are readings a control system genuinely has: a train
	// is in the zone, and it is stopped ON ITS MARK. The other four are the crew's,
	// which is the part that goes away when there are riders.
	//
	// "In position" is the stop mark AND a motor reading nothing, because a train
	// running through the platform covers the same switch. Two instruments, and the
	// pair means something neither of them does alone.
	TrainLoaded.SetNum(Trains.Num());

	for (FTUPlatform& P : Platforms)
	{
		const std::size_t Z = static_cast<std::size_t>(P.Zone);
		bool bPresent = false;
		int32 Who = 0;
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			if (Trains[t]->IsInZone(Z, Trains[t]->GetDistance()))
			{
				bPresent = true;
				Who = t;
			}
		}
		P.Inputs.bTrainPresent = bPresent;
		P.Inputs.bTrainInPosition = bPresent && StopMarks && Z < StopMarks->Num()
			&& StopMarks->IsBlocked(Z)
			&& Drives && FMath::Abs(Drives->Read(Z).Actual) < 1e-6;

		// One button, every platform. A real ride has one per position and an
		// operator standing at it; there is one keyboard here, so it presses them
		// all — which is right for the single-platform layouts this project has and
		// is the first thing a multi-position platform will have to split.
		// THE BUTTON IS AT ONE POSITION, and so is the operator holding it. It used
		// to be pressed at every platform at once, which on a three-position
		// platform is one press dispatching three trains.
		//
		// Only bites in MANUAL — the permissive ignores the request in AUTO — so
		// every measured figure, all of which were taken in AUTO, is unmoved.
		P.Inputs.bDispatchRequest = bDispatchHeld && &P == ConsolePlatformPtr();
		P.Process.SetMode(bManualDispatch ? EDispatchMode::Manual
										  : EDispatchMode::Automatic);

		// Already aboard, so there is nobody to board again. Only a LOAD position
		// passes through — a combined station's arriving train is full and genuinely
		// does need unloading, and an unload position's certainly does.
		const bool bAlready = bPresent && TrainLoaded.IsValidIndex(Who)
			&& P.Process.GetRole() == EStationRole::Load && TrainLoaded[Who];

		P.Process.Update(P.Inputs);

		if (bPlayerIsCrew)
		{
			// ===================== YOU ARE THE CREW =====================
			//
			// The banks still tick, because the HARDWARE is real whether or not
			// anybody is standing there — a bar takes the same two seconds to
			// travel on a quiet Tuesday. What is gone is the clock deciding when
			// the switches get thrown.
			P.Crew.Restraints.Tick(DeltaSeconds, P.Crew.StuckGroup);
			P.Crew.Gates.Tick(DeltaSeconds, P.Crew.StuckGate);

			// AND THE CONTACTS COME FROM THE SWITCH POSITIONS, not from a timer
			// and not from a second set of buttons. Each is what its own field
			// comment already says it is:
			//
			//   bLoadComplete   — "everyone seated, airgates shut". The operator
			//                     shuts the gates when the platform is clear, so
			//                     the gates BEING shut is that statement.
			//   bUnloadComplete — bars up is what lets riders out, so commanding
			//                     the restraints open is the operator saying the
			//                     train may empty.
			//   bPlatformClear  — the walk-round. Nothing derives this: it is a
			//                     person having looked, and it has its own button.
			P.Inputs.bRestraintsLocked = P.Crew.Restraints.IsClosedAndLocked();
			P.Inputs.bUnloadComplete = !P.Crew.Restraints.IsCommandedClosed();
			P.Inputs.bLoadComplete = P.Crew.Gates.IsClosedAndLocked()
				&& P.Crew.Restraints.IsCommandedClosed();
			P.Inputs.bPlatformClear = P.bOperatorAllClear;
		}
		else
		{
			P.Crew.Serve(P.Process, P.Inputs, DeltaSeconds, bAlready);
		}

		// THE ALL-CLEAR IS ABOUT ONE TRAIN. Cleared the moment this one is on its
		// way, so the next arrival needs its own walk-round rather than inheriting
		// a statement somebody made about a different train.
		if (P.Process.GetPhase() == EStationPhase::Departing
			|| P.Process.GetPhase() == EStationPhase::Empty)
		{
			P.bOperatorAllClear = false;
		}

		// Boarded here, and it stays boarded. Set on readiness rather than on the
		// load contact so it survives being re-checked at the next position, which
		// is the whole reason the flag exists. Cleared when the train unloads.
		if (bPresent && TrainLoaded.IsValidIndex(Who))
		{
			if (P.Process.NeedsUnload() && P.Process.GetPhase() == EStationPhase::Unloading)
			{
				TrainLoaded[Who] = false;
			}
			if (P.Process.NeedsLoad() && P.Process.IsReadyToDispatch())
			{
				TrainLoaded[Who] = true;
			}
		}
	}
}

void ATUCoasterRide::ScanBlockSensors()
{
	if (!BlockSensors)
	{
		return;
	}
	BlockSensors->BeginScan();
	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		// Always circuit-wrapped: FBlockCounter is a counter over a RING, and this
		// only exists on a layout that is one.
		BlockSensors->Cover(Trains[t]->GetRearS(), Trains[t]->GetFrontS(),
			true, Track.TotalLength());
	}
	BlockSensors->EndScan();
}

void ATUCoasterRide::CrossCheckOccupancy()
{
	// TWO INDEPENDENT MEANS OF KNOWING WHERE THE TRAINS ARE, AND THEY MUST AGREE.
	// FRideSignals is handed each train's exact span every frame; the counter has
	// nothing but rising and falling edges at the block boundaries. They are
	// derived from different information, so a disagreement means one of them is
	// wrong and neither can say which — which is precisely why a real installation
	// pays for a second detection method rather than a better single one.
	//
	// Verified in test_twotrains.cpp: on both circuits, at every train count, the
	// two agree on every block on every frame. Breaking the counter's falling-edge
	// rule makes that assertion fail, so the check bites.
	if (!Counter || !BlockSensors || !Signals)
	{
		return;
	}
	ScanBlockSensors();
	Counter->Scan();

	for (std::size_t b = 0; b < Counter->NumBlocks(); ++b)
	{
		const TCHAR* Why = nullptr;
		if (Counter->IsOverOccupied(b))
		{
			// Two trains counted into a block nothing has counted out of. Derived
			// from switches alone, so it is a collision detected without anything
			// ever having known a position.
			Why = TEXT("two trains counted into one block");
		}
		else if (Counter->IsOccupied(b) != Signals->Occupies(b))
		{
			Why = TEXT("occupancy disagrees with the train counter");
		}
		if (Why != nullptr && Drives && Drives->TripEmergencyStop("detection disagreement"))
		{
			bEmergencyStop = true;
			LogEvent(FString::Printf(TEXT("block %d: %s"), static_cast<int32>(b), Why));
			UE_LOG(LogTemp, Error,
				TEXT("TrackUnlimited: EMERGENCY STOP — block %d: %s. The interlocking says "
					"%s and the counter says %d train(s)."),
				static_cast<int32>(b), Why,
				Signals->Occupies(b) ? TEXT("occupied") : TEXT("clear"),
				Counter->TrainsIn(b));
		}
	}

	// Below zero is not a collision, it is a LIE: the counter has been told a train
	// left somewhere it was never told one arrived. Seeding is wrong or a trip was
	// missed, and either way it can no longer be trusted to detect anything.
	if (Counter->IsInconsistent() && Drives
		&& Drives->TripEmergencyStop("train counter inconsistent"))
	{
		bEmergencyStop = true;
		UE_LOG(LogTemp, Error,
			TEXT("TrackUnlimited: EMERGENCY STOP — a block counted below zero. A trip was "
				"missed or the counters were seeded wrong; sweep the ride and reset."));
	}
}

bool ATUCoasterRide::DeviceAheadIsReady(double AtS) const
{
	if (!Signals || !Drives || Signals->NumBlocks() == 0)
	{
		return true;
	}
	const std::size_t Next = (Signals->BlockAt(AtS) + 1) % Signals->NumBlocks();
	const double NextS = Signals->Boundaries()[Next];

	// A zone's start IS a block boundary by construction - blocks and zones fall
	// out of the same walk - so the device in the next block is the zone that
	// begins there. No zone there means plain track, which takes a train perfectly
	// well and is always ready.
	for (int32 z = 0; z < ZoneSpans.Num(); ++z)
	{
		if (FMath::Abs(ZoneSpans[z].StartS - NextS) < 0.01)
		{
			return Drives->IsReady(static_cast<std::size_t>(z));
		}
	}
	return true;
}

bool ATUCoasterRide::StationSaysGo(std::size_t Zone) const
{
	for (const FTUPlatform& P : Platforms)
	{
		if (static_cast<std::size_t>(P.Zone) == Zone)
		{
			return P.Process.IsReadyToDispatch();
		}
	}
	return true;   // not a platform; nothing to ask
}

FColor ATUCoasterRide::RailColourAt(double S) const
{
	// A debug view, and the reason it earns its keep is that the DEVICES are the
	// part of a layout you cannot see. Geometry is visible — a hill is a hill —
	// but which stretch of track can hold a train, which can only slow one, and
	// which can only push one all look identical in wireframe, and confusing them
	// is the single most expensive authoring mistake this model allows.
	//
	// Linear scan over a handful of zones, once per half metre, once at BeginPlay.
	for (const FTUZoneSpan& Z : ZoneSpans)
	{
		if (S < Z.StartS || S > Z.EndS)
		{
			continue;
		}
		switch (Z.Kind)
		{
		case ETUSegmentZone::Station:
		case ETUSegmentZone::StationLoad: return FColor(35, 70, 165);   // dark blue
		// Distinguishable from the load platform without ceasing to read as a
		// station, because on a split operation they are different rooms and
		// telling them apart at a glance is the point of the view.
		case ETUSegmentZone::StationUnload: return FColor(70, 110, 200);
		case ETUSegmentZone::Lift:
		case ETUSegmentZone::Launch:     return FColor(70, 210, 95);    // green
		case ETUSegmentZone::Brake:
		case ETUSegmentZone::BlockBrake: return FColor(230, 60, 50);    // red
		default:                         break;
		}
	}
	return FColor(235, 235, 235);   // plain track
}

// One buffer, ported. Everything geometric already happened in TrackMesh.h; this
// converts units and handedness and nothing else, which is the whole job of this
// file.
FTrainSettings ATUCoasterRide::TrainMeshSettings() const
{
	// ONE ANSWER. The car geometry and the placement both come through here, so
	// they cannot be built from two different ideas of the same train - which is
	// the GraphRect and ConsolePlatformPtr() rule, and the exact shape of the bug
	// this whole card exists to fix.
	FTrainSettings S;
	S.CarCount = FMath::Max(1, CarCount);
	S.CarLengthM = FMath::Max(0.5f, CarLengthM);
	S.WheelSides = FMath::Clamp(TrainWheelSides, 5, 24);
	return S;
}

void ATUCoasterRide::PushOrUpdateMeshSection(UProceduralMeshComponent* Target,
	const FMeshBuffer& M) const
{
	if (Target == nullptr)
	{
		return;
	}
	if (M.NumTriangles() == 0)
	{
		Target->ClearAllMeshSections();
		return;
	}

	// SAME VERTEX COUNT MEANS SAME TOPOLOGY HERE, and that is a fact about this
	// caller rather than a general truth: a train's triangles are fixed by its
	// car count, and only their POSITIONS change as it runs. So the common case is
	// an in-place vertex update, and a section is recreated only when the train
	// itself changes shape - a car added, a train dispatched, a preset loaded.
	// Recreating one rebuilds a render proxy, which at frame rate is the entire
	// cost of the feature.
	const FProcMeshSection* Existing = Target->GetProcMeshSection(0);
	const bool bSameTopology = Existing != nullptr
		&& Existing->ProcVertexBuffer.Num() == static_cast<int32>(M.NumVertices());

	TArray<FVector> Pos;
	TArray<FVector> Nrm;
	TArray<FVector2D> UV;
	Pos.Reserve(static_cast<int32>(M.NumVertices()));
	Nrm.Reserve(static_cast<int32>(M.NumVertices()));
	UV.Reserve(static_cast<int32>(M.NumVertices()));
	for (std::size_t v = 0; v < M.NumVertices(); ++v)
	{
		Pos.Add(ToLocal(M.Position[v]));
		Nrm.Add(ToLocalDirection(M.Normal[v]));
		UV.Add(FVector2D(M.UV[v].U, M.UV[v].V));
	}

	if (bSameTopology)
	{
		Target->UpdateMeshSection(0, Pos, Nrm, UV, TArray<FColor>(),
			TArray<FProcMeshTangent>());
		return;
	}

	// NO INDEX SWAP, exactly as PushMeshSection does not swap: M(x,y,z) = (x,-y,z)
	// reverses orientation and UE's front-face rule reverses it again. Two
	// flips. Doing one explicitly left every surface on this ride inside out for
	// weeks, and a car body is SOLID - so it is the first thing that would show it
	// again.
	TArray<int32> Tri;
	Tri.Reserve(static_cast<int32>(M.Index.size()));
	for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
	{
		Tri.Add(static_cast<int32>(M.Index[t]));
		Tri.Add(static_cast<int32>(M.Index[t + 1]));
		Tri.Add(static_cast<int32>(M.Index[t + 2]));
	}
	Target->CreateMeshSection_LinearColor(0, Pos, Tri, Nrm, UV,
		TArray<FLinearColor>(), TArray<FProcMeshTangent>(), /*bCreateCollision*/ false);
}

// ===================== THE TRAIN, EVERY FRAME =====================
//
// Nine engine cubes until today, one per PHYSICS SAMPLE POINT - which is a
// physics resolution and not a vehicle count. See TrainMesh.h; the geometry is
// engine-free and assert-tested and this is only the port.
void ATUCoasterRide::RebuildTrainMesh()
{
	if (!bBuildTrainMesh || TrainPath.empty() || Trains.Num() == 0)
	{
		if (TrainBodyMesh) { TrainBodyMesh->ClearAllMeshSections(); }
		if (TrainChassisMesh) { TrainChassisMesh->ClearAllMeshSections(); }
		if (TrainWheelMesh) { TrainWheelMesh->ClearAllMeshSections(); }
		if (TrainCouplerMesh) { TrainCouplerMesh->ClearAllMeshSections(); }
		return;
	}

	const FTrainSettings S = TrainMeshSettings();
	const double Heartline = Track.GetHeartlineHeight();

	// ONE CAR, built once and stamped wherever a vehicle is. It is the same
	// geometry for every car on the ride; what differs is where each one is, and
	// that is a transform rather than a mesh.
	const FTrainMesh Car = BuildCarMesh(S, Heartline, Profile);

	FTrainMesh All;
	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		// THE NOSE IS THE REFERENCE, and it is the one the signalling and the stop
		// marks already use - so there is nothing to convert, and nothing for the
		// picture and the interlocking to disagree about.
		//
		// GetFrontS() rather than GetDistance() + half a length, which is what it
		// is: the accessor already exists and re-deriving it here would be a
		// second answer to a question this class has answered. Its own header
		// warns that front-versus-rear is a TRAP for anything reasoning about
		// direction, and that warning is about the signalling rather than about
		// this - a reversing train wants the two signs in DIRECTION_AND_ROUTES.md,
		// which is designed and unbuilt. Drawing a forward train, this is the nose.
		const double NoseS = Trains[t]->GetFrontS();
		// NOT `Cars`, which is the instanced-cube COMPONENT one scope up. The
		// compiler caught it; the two would have been readable and wrong.
		const std::vector<FCarPlacement> Placed = PlaceCars(TrainPath, TrainPathSpacing,
			TrainPathTotal, NoseS, S, bTrackIsCircuit);
		const FTrainMesh One = BuildTrainMesh(Placed, Car, S, Heartline);
		AppendBuffer(All.Body, One.Body);
		AppendBuffer(All.Chassis, One.Chassis);
		AppendBuffer(All.Wheels, One.Wheels);
		AppendBuffer(All.Couplers, One.Couplers);
	}

	PushOrUpdateMeshSection(TrainBodyMesh, All.Body);
	PushOrUpdateMeshSection(TrainChassisMesh, All.Chassis);
	PushOrUpdateMeshSection(TrainWheelMesh, All.Wheels);
	PushOrUpdateMeshSection(TrainCouplerMesh, All.Couplers);
	// The train is built after the track, so its sections did not exist when
	// the style was applied. Cheap: the instances are cached.
	ApplyTrackStyle();
}

void ATUCoasterRide::PushMeshSection(UProceduralMeshComponent* Target, const FMeshBuffer& M) const
{
	if (Target == nullptr)
	{
		return;
	}
	Target->ClearAllMeshSections();
	if (M.NumTriangles() == 0)
	{
		return;
	}

	TArray<FVector> Pos;
	TArray<FVector> Nrm;
	TArray<FVector2D> UV;
	TArray<int32> Tri;
	Pos.Reserve(static_cast<int32>(M.NumVertices()));
	Nrm.Reserve(static_cast<int32>(M.NumVertices()));
	UV.Reserve(static_cast<int32>(M.NumVertices()));
	Tri.Reserve(static_cast<int32>(M.Index.size()));

	for (std::size_t v = 0; v < M.NumVertices(); ++v)
	{
		Pos.Add(ToLocal(M.Position[v]));
		Nrm.Add(ToLocalDirection(M.Normal[v]));
		UV.Add(FVector2D(M.UV[v].U, M.UV[v].V));
	}

	// ===================== TWO FLIPS, NOT ONE. MEASURED, NOT ARGUED =====================
	//
	// This swapped two indices of every triangle, on the argument that M(x,y,z) =
	// (x,-y,z) is a reflection with determinant -1 and so reverses triangle
	// orientation. That argument is CORRECT and the conclusion drawn from it was
	// not: it is a statement about geometry, and says nothing about UE's
	// front-face rule, which is the opposite handedness and flips it straight
	// back. Two flips. Doing one explicitly left every surface on this ride
	// inside out.
	//
	// HOW IT SURVIVED: every assertion this project has stops at the port. Signed
	// volume, watertightness and normals-agree-with-winding were all checked and
	// all passed, because the engine-free mesh was never the thing that was wrong.
	// Nothing on that side can see UE's rasteriser.
	//
	// HOW IT WAS FOUND: a camera placed INSIDE a support pier saw solid walls,
	// which correctly outward-wound geometry cannot do -- from inside, backfaces
	// cull and you see the world beyond. Settled by looking, both ways: with the
	// swap the pier was hollow from outside and solid from within, and without it
	// exactly the reverse.
	//
	// It read as a support problem because a column and a pier are the only solid
	// shapes here. A rail is a thin tube: inside out, its silhouette is unchanged
	// and you are simply looking at the far inner wall, which is why 1288 m of it
	// looked plausible for weeks.
	for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
	{
		Tri.Add(static_cast<int32>(M.Index[t]));
		Tri.Add(static_cast<int32>(M.Index[t + 1]));
		Tri.Add(static_cast<int32>(M.Index[t + 2]));
	}

	Target->CreateMeshSection_LinearColor(0, Pos, Tri, Nrm, UV,
		TArray<FLinearColor>(), TArray<FProcMeshTangent>(), /*bCreateCollision*/ false);
}

FTUTrackStyle ATUCoasterRide::StylePreset(ETUTrackStyleName Which)
{
	FTUTrackStyle S;
	switch (Which)
	{
	case ETUTrackStyleName::SteelClassic:
		S.Name = TEXT("Steel - classic tubular");
		S.RailColour = FLinearColor(0.55f, 0.56f, 0.58f);
		S.SpineColour = FLinearColor(0.16f, 0.35f, 0.55f);
		S.TieColour = FLinearColor(0.22f, 0.24f, 0.26f);
		S.TrainColour = FLinearColor(0.08f, 0.22f, 0.50f);   // blue cars on a blue spine
		S.GaugeM = 1.22f;              // the wide end of the real range
		S.RailDiameterM = 0.127f;      // and the fat end of the rail range
		S.SpineDropM = 0.38f;
		S.SpineDiameterM = 0.36f;
		S.TieSpacingM = 0.75f;
		S.TieDiameterM = 0.06f;
		break;
	case ETUTrackStyleName::SteelCompact:
		S.Name = TEXT("Steel - compact / family");
		S.RailColour = FLinearColor(0.70f, 0.72f, 0.75f);
		S.SpineColour = FLinearColor(0.20f, 0.60f, 0.42f);
		S.TieColour = FLinearColor(0.34f, 0.36f, 0.38f);
		S.TrainColour = FLinearColor(0.90f, 0.55f, 0.05f);   // a family ride is bright
		// NARROW GAUGE FOR THE SMALL VEHICLES the small-batch preset already
		// ships. 0.76 m is the bottom of the real range and it is where a
		// six-metre car belongs.
		S.GaugeM = 0.78f;
		S.RailDiameterM = 0.09f;
		S.SpineDropM = 0.30f;
		S.SpineDiameterM = 0.22f;
		S.TieSpacingM = 0.60f;
		S.TieDiameterM = 0.04f;
		break;
	default:
		break;                          // the struct's own defaults are Modern
	}
	return S;
}

FTUTrackStyle ATUCoasterRide::ActiveStyle() const
{
	return bUseCustomStyle ? CustomStyle : StylePreset(TrackStyle);
}

void ATUCoasterRide::ApplyTrackStyle()
{
	// NO AUTHORED ASSET. Dynamic instances over an engine material referenced by
	// path, which is the same trick the cars already use for their cube. A style
	// is therefore data in this file rather than a .uasset somebody has to make
	// before anything looks like anything.
	if (!BaseMaterial)
	{
		BaseMaterial = LoadObject<UMaterialInterface>(nullptr,
			TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	}
	if (!BaseMaterial)
	{
		// REPORTED, NOT FAKED. Without a base the sections keep the engine's
		// default grey, which is honest — a track that silently rendered
		// untextured with no explanation would be read as the mesher failing.
		UE_LOG(LogTemp, Warning,
			TEXT("TrackUnlimited: no base material; the track will draw untextured."));
		return;
	}

	const FTUTrackStyle S = ActiveStyle();
	auto Paint = [this](TObjectPtr<UMaterialInstanceDynamic>& Mid,
		UProceduralMeshComponent* Mesh, const FLinearColor& C)
	{
		if (!Mesh) { return; }
		if (!Mid) { Mid = UMaterialInstanceDynamic::Create(BaseMaterial, this); }
		if (!Mid) { return; }
		// The engine base exposes a "Color" parameter. Setting one that does not
		// exist is a no-op rather than an error, so this degrades to plain grey
		// rather than failing if the asset ever changes.
		Mid->SetVectorParameterValue(TEXT("Color"), C);
		Mesh->SetMaterial(0, Mid);
	};

	// THREE MATERIALS FOR THREE BUFFERS, which is why the mesher emits three:
	// running rail is polished where the wheels touch it, spine is painted
	// structure, ties are usually neither.
	Paint(RailMaterial, RailMesh, S.RailColour);
	Paint(SpineMaterial, SpineMesh, S.SpineColour);
	Paint(TieMaterial, TieMesh, S.TieColour);

	// AND THE OTHER SEVEN, which were all the engine's grey. The sections exist
	// BECAUSE they are different materials, and every one drew as the same one.
	// Supports and the body are the style's; the rest are what they are on any
	// ride -- a deck is dark grating, a handrail is safety yellow, a chassis is
	// dark steel, a wheel is polyurethane over a hub, a coupler is steel.
	// ponytail: colour only on one engine material; roughness/metal per section
	// wants a material asset, which is an editor job.
	Paint(SupportMaterial, SupportMesh, S.SupportColour);
	Paint(CatwalkDeckMaterial, CatwalkDeckMesh, FLinearColor(0.16f, 0.17f, 0.18f));
	Paint(CatwalkRailMaterial, CatwalkRailMesh, FLinearColor(0.85f, 0.62f, 0.08f));
	Paint(DeviceSteelMaterial, DeviceSteelMesh, S.SupportColour);
	Paint(DeviceRubberMaterial, DeviceRubberMesh, FLinearColor(0.06f, 0.06f, 0.06f));
	Paint(StationConcreteMaterial, StationConcreteMesh, FLinearColor(0.52f, 0.51f, 0.49f));
	Paint(StationSteelMaterial, StationSteelMesh, FLinearColor(0.62f, 0.64f, 0.68f));
	Paint(StationStripeMaterial, StationStripeMesh, FLinearColor(0.95f, 0.80f, 0.05f));
	Paint(TrainBodyMaterial, TrainBodyMesh, S.TrainColour);
	Paint(TrainChassisMaterial, TrainChassisMesh, FLinearColor(0.10f, 0.11f, 0.12f));
	Paint(TrainWheelMaterial, TrainWheelMesh, FLinearColor(0.05f, 0.05f, 0.05f));
	Paint(TrainCouplerMaterial, TrainCouplerMesh, FLinearColor(0.28f, 0.29f, 0.30f));
}

void ATUCoasterRide::LoadBrakeSounds()
{
	// BY PATH, AND MISSING IS FINE. The .wav files sit in Content/Audio/Brakes and
	// become USoundWave assets when the editor imports them; until somebody has
	// done that these resolve to null and the ride is simply quiet. A ride that
	// refused to run without its sound effects would have made an audio asset a
	// hard dependency of the simulation, which is exactly backwards — the same
	// rule the frame widget already follows.
	static const TCHAR* Names[] = {
		TEXT("/Game/Audio/Brakes/416080__davidlay1__air-release.416080__davidlay1__air-release"),
		TEXT("/Game/Audio/Brakes/131931__mcpable__slips-air-release-v4.131931__mcpable__slips-air-release-v4"),
		TEXT("/Game/Audio/Brakes/131932__mcpable__slips-air-release-v3.131932__mcpable__slips-air-release-v3"),
		TEXT("/Game/Audio/Brakes/131933__mcpable__slips-air-release-v2.131933__mcpable__slips-air-release-v2"),
		TEXT("/Game/Audio/Brakes/131934__mcpable__slips-air-release-v1.131934__mcpable__slips-air-release-v1"),
	};

	BrakeReleaseSounds.Reset();
	for (const TCHAR* Path : Names)
	{
		if (USoundBase* S = LoadObject<USoundBase>(nullptr, Path))
		{
			BrakeReleaseSounds.Add(S);
		}
	}
	// ===================== WHAT MAKES IT AN EMITTER =====================
	//
	// A sound with no attenuation settings is played FLAT: full volume, no
	// direction, no distance, wherever the listener is. That is not a quiet bug —
	// it is every brake on the ride hissing in your ear as you ride past them.
	//
	// NaturalSound is the inverse-square-ish curve real sound follows, rather than
	// Linear, which fades at a constant rate per metre and reads as a sound being
	// turned down by hand as you walk away from it.
	BrakeAttenuation = NewObject<USoundAttenuation>(this);
	FSoundAttenuationSettings& A = BrakeAttenuation->Attenuation;
	A.bAttenuate = true;
	A.bSpatialize = true;
	A.DistanceAlgorithm = EAttenuationDistanceModel::NaturalSound;
	A.AttenuationShape = EAttenuationShape::Sphere;
	// Radius of the full-volume core, not the audible range. Small: standing at the
	// valve and standing a metre from it are the same thing, and a large core is how
	// a sound ends up flat again over the whole station.
	A.AttenuationShapeExtents = FVector(200.0, 0.0, 0.0);
	A.FalloffDistance = static_cast<float>(BrakeReleaseRangeM) * 100.f;

	if (BrakeReleaseSounds.Num() == 0)
	{
		// SAID ONCE, at Log rather than Warning. Not having imported them yet is
		// an ordinary state, not a fault — but silence with no explanation is the
		// kind of thing somebody spends an hour on.
		UE_LOG(LogTUEvents, Log,
			TEXT("brakes: no release sounds imported. Drop Content/Audio/Brakes/*.wav "
				 "into the Content Browser and they are picked up next play."));
	}
	else
	{
		UE_LOG(LogTUEvents, Log, TEXT("brakes: %d release sounds"),
			BrakeReleaseSounds.Num());
	}
}

void ATUCoasterRide::ServeBrakeReleaseSound()
{
	if (!bBrakeReleaseSound || !Drives || BrakeReleaseSounds.Num() == 0)
	{
		return;
	}

	// ===================== THE EDGE, NOT THE STATE =====================
	//
	// A release is a TRANSITION: the pad was holding and now it is not. Watching
	// the level instead would hiss continuously for as long as a brake was open,
	// which is every brake on the ride most of the time.
	//
	// Output rather than Commanded, because Output is what the train actually
	// feels — and on a drive with a ramp the two differ by exactly the interval
	// during which the valve is opening, which is the sound.
	const std::size_t N = Drives->Num();
	if (static_cast<std::size_t>(LastDriveOutput.Num()) != N)
	{
		// Sized here rather than at rebuild, and seeded from the CURRENT output so
		// the first frame after a rebuild is not one enormous chord of every brake
		// on the ride releasing at once.
		LastDriveOutput.Init(0.f, static_cast<int32>(N));
		for (std::size_t d = 0; d < N; ++d)
		{
			LastDriveOutput[static_cast<int32>(d)] = static_cast<float>(Drives->Output(d));
		}
		return;
	}

	// Below this a pad is holding; above it, air has moved. Well clear of the
	// crawl speeds a drive trucks a train at, so trucking is not a release.
	const float Threshold = 0.05f;

	for (std::size_t d = 0; d < N; ++d)
	{
		const float Now = static_cast<float>(Drives->Output(d));
		const float Was = LastDriveOutput[static_cast<int32>(d)];
		LastDriveOutput[static_cast<int32>(d)] = Now;

		if (!(Was <= Threshold && Now > Threshold))
		{
			continue;
		}
		if (!ZoneSpans.IsValidIndex(static_cast<int32>(d)))
		{
			continue;
		}
		const FTUZoneSpan& Z = ZoneSpans[static_cast<int32>(d)];

		// ONLY THE THINGS WITH PADS. A lift chain and a launch have no friction
		// brake to vent, so giving them a hiss would be a sound effect explaining
		// a mechanism that is not there — and this project's whole argument is
		// that what you see and hear is what the machine is doing.
		const bool bHasPad = Z.Kind == ETUSegmentZone::Brake
			|| Z.Kind == ETUSegmentZone::BlockBrake
			|| Z.Kind == ETUSegmentZone::Station
			|| Z.Kind == ETUSegmentZone::StationLoad
			|| Z.Kind == ETUSegmentZone::StationUnload;
		if (!bHasPad)
		{
			continue;
		}

		// ===================== WHERE THE VALVE IS =====================
		//
		// At the STOP MARK, because that is where a held train stands and
		// therefore where the calipers gripping it are — not the middle of the
		// zone, which on a 67 m brake run is nowhere near the train.
		//
		// And BELOW THE RAILS, along the frame's own Up rather than the world's:
		// the exhaust port hangs under the track, so through an inversion it
		// should be above you in world terms. Following the frame gets that for
		// free; a world-space offset would put it in the wrong place exactly where
		// somebody would notice.
		double AtS = 0.5 * (Z.StartS + Z.EndS);
		if (StopMarks.IsValid() && d < StopMarks->Num())
		{
			AtS = StopMarks->PositionOf(d);
		}
		const FTrackFrame F = Track.EvaluateAt(AtS);
		const FVector Where = ToWorld(F.Position - F.Up * static_cast<double>(BrakeValveDropM));

		// RANDOM, so five brakes releasing in a lap do not sound like one brake
		// releasing five times. Nothing here needs to be reproducible: it is the
		// one part of the ride that does not feed a measurement, and the sim clock
		// is untouched by it.
		const int32 Pick = FMath::RandRange(0, BrakeReleaseSounds.Num() - 1);
		UGameplayStatics::PlaySoundAtLocation(this, BrakeReleaseSounds[Pick], Where,
			FRotator::ZeroRotator, BrakeReleaseVolume, 1.f, 0.f, BrakeAttenuation);
	}
}

void ATUCoasterRide::RebuildTrackMesh()
{
	FMeshSettings Settings;
	Settings.SampleSpacing = MeshSampleSpacingM;
	Settings.Sides = MeshSides;

	// ===================== THE WALK COMES FIRST, AND IT IS NOT THE TRACK MESH'S =====================
	//
	// The train rides these frames, and it is a SEPARATE feature behind a separate
	// switch. Caching the walk below the guard made `bBuildTrainMesh` quietly
	// depend on `bBuildTrackMesh`: turning the rails off would have left the cars
	// on a stale path, or on none at all, and the symptom would have been a train
	// that vanished when somebody hid the track to look at something else.
	//
	// One walk still serves both, which is the point of doing it here rather than
	// in two places: the cars ride exactly the samples the rails were swept along,
	// so the two cannot disagree about where the track is.
	TrainPath = Track.TotalLength() > 0.0
		? WalkTrack(Track, Settings.SampleSpacing)
		: std::vector<FTrackFrame>();
	TrainPathSpacing = Settings.SampleSpacing;
	TrainPathTotal = Track.TotalLength();

	if (!bBuildTrackMesh || Track.TotalLength() <= 0.0)
	{
		if (RailMesh) { RailMesh->ClearAllMeshSections(); }
		if (SpineMesh) { SpineMesh->ClearAllMeshSections(); }
		if (TieMesh) { TieMesh->ClearAllMeshSections(); }
		if (SupportMesh) { SupportMesh->ClearAllMeshSections(); }
		if (CatwalkDeckMesh) { CatwalkDeckMesh->ClearAllMeshSections(); }
		if (CatwalkRailMesh) { CatwalkRailMesh->ClearAllMeshSections(); }
		if (DeviceSteelMesh) { DeviceSteelMesh->ClearAllMeshSections(); }
		if (DeviceRubberMesh) { DeviceRubberMesh->ClearAllMeshSections(); }
		if (StationConcreteMesh) { StationConcreteMesh->ClearAllMeshSections(); }
		if (StationSteelMesh) { StationSteelMesh->ClearAllMeshSections(); }
		if (StationStripeMesh) { StationStripeMesh->ClearAllMeshSections(); }
		return;
	}

	// CAP THE OPEN ENDS, AND ONLY IF THERE ARE ANY. On a circuit the first ring
	// and the last are the same ring, so a cap is a disc buried in the seam —
	// invisible, and coplanar with the geometry either side of it, which is what
	// z-fighting is made of.
	//
	// bTrackIsCircuit is MEASURED at rebuild rather than authored, so this
	// follows the geometry rather than somebody's intent. Ties are capped either
	// way: a strut has two free ends whatever the layout does.
	Settings.bCapEnds = !bTrackIsCircuit;

	// The walk is the ONLY thing that touches the track, and it walks with
	// AdvanceFrom. The sweep below has no FTrack at all, which is what makes the
	// O(n^2) trap unreachable rather than merely discouraged — see TrackMesh.h.
	// THE SAME WALK THE TRAIN IS PLACED ALONG, cached above the guard so the two
	// features do not depend on each other.
	std::vector<FMeshFinding> Findings;
	const FTrackMesh Mesh = BuildTrackMesh(TrainPath, Track.GetHeartlineHeight(),
		Profile, Settings, &Findings);

	PushMeshSection(RailMesh, Mesh.Rails);
	PushMeshSection(SpineMesh, Mesh.Spine);
	PushMeshSection(TieMesh, Mesh.Ties);

	// ===================== AND WHAT HOLDS IT UP =====================
	//
	// PLACEMENT WAS NEVER THE MISSING PART. `PlanSupports` refuses track under
	// grade, refuses through an inversion where the spine is above the rails,
	// refuses a column that would pass through the track it is not carrying, and
	// reports the longest unsupported run — all of it asserted engine-free since
	// the day it was written. The actor read `Plan.Finding` and dropped `Plan.Leg`
	// on the floor every rebuild.
	//
	// ITS OWN WALK, at a coarser spacing than the mesh. A support span is 9 m and
	// the mesher samples every half metre; asking the placer to consider eighteen
	// candidates per span is eighteen times the work for the same answer.
	if (bBuildSupports)
	{
		std::vector<FMeshFinding> Unused;
		const FSupportPlan Plan = PlanSupports(WalkTrack(Track, 1.0),
			// The TRACK profile (the cross-section), not the RIDE profile. Two
			// different things one letter apart, and the compiler caught it.
			Track.GetHeartlineHeight(), Profile, FSupportSettings(),
			FlatGround(-GroundOffsetM));
		PushMeshSection(SupportMesh, BuildSupportMesh(Plan));
		// THE FOOTINGS ARE COUNTED SEPARATELY because they do not correspond one to
		// one: at-grade track gets a pad and no column, which is the station.
		UE_LOG(LogTemp, Log,
			TEXT("TrackUnlimited: %d support legs on %d footings, longest unsupported run %.1f m"),
			static_cast<int32>(Plan.Leg.size()), static_cast<int32>(Plan.Footing.size()),
			Plan.LongestGapM);
	}
	else if (SupportMesh)
	{
		SupportMesh->ClearAllMeshSections();
	}

	// ===================== AND THE WALKWAY ALONG IT =====================
	//
	// The spans were derived a few hundred lines up and handed to the EVACUATION
	// model, which has decided whether a stopped train can be reached ever since
	// — over a walkway nobody could see. Drawing it makes "every train is
	// reachable" something you can check at a glance, and a gap in the route a
	// hole rather than a log line.
	//
	// THE SAME WALK THE TRACK USED, at the mesher's spacing rather than the
	// placer's: a deck follows the track closely and a catwalk sampled every metre
	// would visibly cut the corners the rails do not.
	if (bBuildCatwalks && !WalkwaySpans.empty())
	{
		// A PLATFORM IS THE WALKWAY ON A STATION SPAN. The authored route keeps
		// the station -- the evacuation model is right that a stopped train
		// there can be reached -- but the DRAWN deck and handrail stop at the
		// platform's ends, or the rail posts stand between the gates and the
		// train. Each span is cut around every station zone.
		std::vector<FWalkwaySpan> Drawn;
		for (const FWalkwaySpan& W : WalkwaySpans)
		{
			std::vector<FWalkwaySpan> Pieces{W};
			for (const FTUZoneSpan& Z : ZoneSpans)
			{
				if (Z.Kind != ETUSegmentZone::Station && Z.Kind != ETUSegmentZone::StationLoad
					&& Z.Kind != ETUSegmentZone::StationUnload) { continue; }
				std::vector<FWalkwaySpan> Next;
				for (const FWalkwaySpan& P : Pieces)
				{
					if (Z.EndS <= P.StartS || Z.StartS >= P.EndS) { Next.push_back(P); continue; }
					if (Z.StartS > P.StartS) { FWalkwaySpan L = P; L.EndS = Z.StartS; Next.push_back(L); }
					if (Z.EndS < P.EndS) { FWalkwaySpan R = P; R.StartS = Z.EndS; Next.push_back(R); }
				}
				Pieces.swap(Next);
			}
			for (const FWalkwaySpan& P : Pieces) { if (P.EndS - P.StartS > 0.5) { Drawn.push_back(P); } }
		}
		const FCatwalkMesh Walk = BuildCatwalks(WalkTrack(Track, Settings.SampleSpacing),
			bBuildStations ? Drawn : WalkwaySpans, Profile);
		PushMeshSection(CatwalkDeckMesh, Walk.Deck);
		PushMeshSection(CatwalkRailMesh, Walk.Rail);

		// REPORTED, NEVER REPAIRED. A catwalk too steeply banked to walk on is
		// still drawn, because the author asked for it and dropping the geometry
		// would hide the thing being complained about.
		for (const FMeshFinding& F : Walk.Finding)
		{
			UE_LOG(LogTUEvents, Warning, TEXT("catwalk at %.1f m: %s"),
				F.S, UTF8_TO_TCHAR(F.What.c_str()));
		}
		UE_LOG(LogTemp, Log,
			TEXT("TrackUnlimited: catwalk %d triangles over %d span(s)"),
			static_cast<int32>(Walk.NumTriangles()), static_cast<int32>(WalkwaySpans.size()));
	}
	else
	{
		if (CatwalkDeckMesh) { CatwalkDeckMesh->ClearAllMeshSections(); }
		if (CatwalkRailMesh) { CatwalkRailMesh->ClearAllMeshSections(); }
	}

	// ===================== AND THE HARDWARE THAT MAKES A ZONE A DEVICE =====================
	//
	// From the same zone walk the physics and the signalling use, so a brake's
	// fins are exactly where the brake is. The catch spans are the anti-rollback
	// list -- the first time that flag has been visible at all.
	DeviceTriangles = 0;
	if (bBuildDeviceHardware && (ZoneSpans.Num() > 0 || CatchSpanList.Num() > 0))
	{
		std::vector<FTrackDeviceSpan> DevSpans;
		for (const FTUZoneSpan& Z : ZoneSpans)
		{
			FTrackDeviceSpan D;
			D.StartS = Z.StartS;
			D.EndS = Z.EndS;
			D.Hardware = HardwareForZoneName(TCHAR_TO_UTF8(ZoneKindName(Z.Kind)));
			if (D.Hardware != DeviceNone) { DevSpans.push_back(D); }
		}
		for (const TPair<double, double>& C : CatchSpanList)
		{
			FTrackDeviceSpan D;
			D.StartS = C.Key;
			D.EndS = C.Value;
			D.Hardware = DeviceCatch;
			DevSpans.push_back(D);
		}
		const FTrackDeviceMesh Dev = BuildDeviceHardware(WalkTrack(Track, Settings.SampleSpacing),
			DevSpans, Track.GetHeartlineHeight(), Profile);
		PushMeshSection(DeviceSteelMesh, Dev.Hardware);
		PushMeshSection(DeviceRubberMesh, Dev.Rubber);
		DeviceTriangles = static_cast<int32>(Dev.NumTriangles());
		UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: device hardware %d triangles over %d span(s)"),
			DeviceTriangles, static_cast<int32>(DevSpans.size()));
	}
	else
	{
		if (DeviceSteelMesh) { DeviceSteelMesh->ClearAllMeshSections(); }
		if (DeviceRubberMesh) { DeviceRubberMesh->ClearAllMeshSections(); }
	}

	// ===================== AND THE PLATFORM BESIDE THE STATION =====================
	//
	// On the side the preset puts its walkways, because that is where the
	// people are; Both means the rider's left. One gate per CAR, read off the
	// same CarLengthM the train mesh uses, so a six-car train gets six gates.
	StationTriangles = 0;
	if (bBuildStations)
	{
		std::vector<FStationSpan> StSpans;
		for (const FTUZoneSpan& Z : ZoneSpans)
		{
			if (Z.Kind != ETUSegmentZone::Station && Z.Kind != ETUSegmentZone::StationLoad
				&& Z.Kind != ETUSegmentZone::StationUnload) { continue; }
			FStationSpan P;
			P.StartS = Z.StartS;
			P.EndS = Z.EndS;
			P.bLeft = PresetWalkwaySide != ETUWalkway::Right;
			// THE CABINET GOES ON THE LAST POSITION OF A CONTIGUOUS PLATFORM:
			// a span whose end is the next station span's start is not the end
			// of the platform, and a console per position is four consoles.
			if (!StSpans.empty() && std::fabs(StSpans.back().EndS - P.StartS) < 1e-6)
			{
				StSpans.back().bCabinet = false;
			}
			StSpans.push_back(P);
		}
		if (!StSpans.empty())
		{
			FStationSettings St;
			if (CarLengthM > 0.5f) { St.GatePitchM = CarLengthM; }
			const FStationMesh Stn = BuildStations(WalkTrack(Track, Settings.SampleSpacing), StSpans, St);
			PushMeshSection(StationConcreteMesh, Stn.Concrete);
			PushMeshSection(StationSteelMesh, Stn.Steel);
			PushMeshSection(StationStripeMesh, Stn.Stripe);
			StationTriangles = static_cast<int32>(Stn.NumTriangles());
			UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: station %d triangles over %d platform(s)"),
				StationTriangles, static_cast<int32>(StSpans.size()));
		}
	}
	if (StationTriangles == 0)
	{
		if (StationConcreteMesh) { StationConcreteMesh->ClearAllMeshSections(); }
		if (StationSteelMesh) { StationSteelMesh->ClearAllMeshSections(); }
		if (StationStripeMesh) { StationStripeMesh->ClearAllMeshSections(); }
	}

	// AFTER the sections exist, because SetMaterial on a component with no
	// section is a colour assigned to nothing.
	ApplyTrackStyle();

	// REPORTED, NEVER REPAIRED. A curve tighter than half the gauge folds the
	// inner rail through its own axis, and the fix is a wider curve — geometry
	// quietly straightened to fit would read as a layout that works.
	for (const FMeshFinding& F : Findings)
	{
		UE_LOG(LogTUEvents, Warning,
			TEXT("track mesh at %.1f m: %s (curvature %.4f 1/m, inner rail radius %.3f m)"),
			F.S, UTF8_TO_TCHAR(F.What.c_str()), F.Curvature, F.MinRadius);
	}
	UE_LOG(LogTemp, Log, TEXT("TrackUnlimited: track mesh %d vertices, %d triangles at %.2f m / %d sides"),
		static_cast<int32>(Mesh.NumVertices()), static_cast<int32>(Mesh.NumTriangles()),
		MeshSampleSpacingM, MeshSides);
}

void ATUCoasterRide::DrawTrack() const
{
	// THE WIREFRAME SURVIVED PHASE 4, and is not redundant beside the mesh.
	// It draws two things the swept geometry structurally cannot: the HEARTLINE,
	// which is not a physical part of the track at all, and the DEVICE COLOURS,
	// which say which block and which brake each stretch belongs to. A solid
	// track hides both — so this stays on by default and the mesh grows over it.
	//
	// The rails and spine drawn here are inside their own tubes and will be
	// hidden. Left in rather than trimmed, because turning the mesh off has to
	// give back the view every screenshot before Phase 4 was taken with.
	//
	// Everything comes off the frame the walk already has. The previous version
	// called Track.RailCentreAt(S) in here, which re-runs EvaluateAt — O(track
	// length) per call, so the loop was quadratic: about 118 million integrator
	// steps on this 543 m layout, all of it at BeginPlay.
	const double Total = Track.TotalLength();
	const double Step = 0.5;
	const double Heartline = Track.GetHeartlineHeight();

	FTrackFrame Walk = Track.EvaluateAt(0.0);
	FTrackCrossSection Section = CrossSectionAt(Walk, Heartline, Profile);
	double SinceTie = 0.0;

	for (double S = 0.0; S < Total; S += Step)
	{
		const double Next = FMath::Min(S + Step, Total);
		const FTrackFrame NextFrame = Track.AdvanceFrom(Walk, S, Next);
		const FTrackCrossSection NextSection = CrossSectionAt(NextFrame, Heartline, Profile);

		// The RAILS carry the device colour and nothing else does. Heartline, spine
		// and ties keep their own so the geometry stays readable underneath it —
		// a view where everything changes colour at once says less, not more.
		const FColor Rail = RailColourAt(S);

		DrawDebugLine(GetWorld(), ToWorld(Walk.Position), ToWorld(NextFrame.Position),
			FColor(90, 190, 255), true, -1.f, 0, 2.f);                 // heartline
		DrawDebugLine(GetWorld(), ToWorld(Section.LeftRail), ToWorld(NextSection.LeftRail),
			Rail, true, -1.f, 0, 3.f);                                 // running rails
		DrawDebugLine(GetWorld(), ToWorld(Section.RightRail), ToWorld(NextSection.RightRail),
			Rail, true, -1.f, 0, 3.f);
		DrawDebugLine(GetWorld(), ToWorld(Section.SpineCentre), ToWorld(NextSection.SpineCentre),
			FColor(150, 150, 160), true, -1.f, 0, 4.f);                // spine

		SinceTie += Next - S;
		if (SinceTie >= Profile.TieSpacing)
		{
			SinceTie = 0.0;
			DrawDebugLine(GetWorld(), ToWorld(Section.LeftRail), ToWorld(Section.RightRail),
				FColor(200, 160, 90), true, -1.f, 0, 2.f);             // tie across the rails
			DrawDebugLine(GetWorld(), ToWorld(Section.RailCentre), ToWorld(Section.SpineCentre),
				FColor(200, 160, 90), true, -1.f, 0, 2.f);             // down to the spine
		}

		Walk = NextFrame;
		Section = NextSection;
	}
}

void ATUCoasterRide::DrawRideProfile() const
{
	// Each channel is a curve offset from the heartline along the rider's UP,
	// so the track itself is the zero line and the offset is in the rider's own
	// frame rather than the world's. Through an inversion the trace goes with
	// the rider, which is the honest reading — "how hard, which way, for the
	// person in the seat".
	//
	// Per-channel units chosen so the four are comparable at one scale: 1 metre
	// per G, per 10 km/h, per 30 deg/s.
	struct FChannel
	{
		bool bEnabled;
		FColor Colour;
		double PerUnit;
		double FRideSample::*Field;
	};
	// Okabe-Ito, the colourblind-safe qualitative palette — see Docs/UI_CONVENTIONS.md.
	// The previous green/orange pair was the common deuteranopia collision. Blue #0072B2 is
	// deliberately unused: ~3:1 against the background, too weak for a 2 px line.
	const FChannel Channels[] = {
		{bGraphVerticalG, FColor(86, 180, 233), 1.0, &FRideSample::VerticalG},          // sky blue
		{bGraphLateralG, FColor(230, 159, 0), 1.0, &FRideSample::LateralG},             // orange
		{bGraphSpeed, FColor(0, 158, 115), 1.0 / (10.0 / 3.6), &FRideSample::Speed},    // bluish green
		{bGraphRollRate, FColor(204, 121, 167), 1.0 / 30.0, &FRideSample::RollRateDegPerSec}, // reddish purple
	};

	if (Profile_.Samples.size() < 2)
	{
		return;
	}
	NonFiniteTraceSamples = 0;

	// One walk for every channel rather than one per channel: the frames are
	// the expensive part and they do not depend on which trace is being drawn.
	//
	// SEEDED AT THE FIRST SAMPLE, NOT AT ZERO. It used to start from
	// EvaluateAt(0.0) while telling AdvanceFrom the walk was already at
	// Samples[0].S — and a train's profile starts wherever it was parked, which
	// on every preset here is its holding position rather than the seam. Every
	// frame after that was offset along the track by exactly that distance, so
	// the whole trace was drawn on the wrong piece of layout while looking
	// perfectly plausible.
	double PrevS = Profile_.Samples[0].S;
	FTrackFrame Walk = Track.EvaluateAt(PrevS);
	for (std::size_t i = 1; i < Profile_.Samples.size(); ++i)
	{
		const FRideSample& A = Profile_.Samples[i - 1];
		const FRideSample& B = Profile_.Samples[i];
		const FTrackFrame FrameA = Walk;
		const FTrackFrame FrameB = Track.AdvanceFrom(Walk, PrevS, B.S);

		for (const FChannel& C : Channels)
		{
			if (!C.bEnabled)
			{
				continue;
			}
			const double VA = A.*(C.Field) * C.PerUnit * GraphScale;
			const double VB = B.*(C.Field) * C.PerUnit * GraphScale;

			// A NON-FINITE SAMPLE DRAWS A WEDGE ACROSS THE SCREEN, and it is
			// invisible in the reported maxima — std::max(a, NaN) returns a, so
			// the profile's own "peak lateral 0.30 g" is silent about it while
			// ToWorld(Position + Up * NaN) sends one endpoint to nowhere. A thick
			// debug line with one end at infinity renders as a solid triangle
			// with its apex on the track, which is exactly what was on screen.
			//
			// SKIPPED AND COUNTED, not clamped: clamping would draw a plausible
			// trace over data that is broken, which is the failure this project
			// refuses everywhere else. The count is reported once per rebuild.
			if (!FMath::IsFinite(VA) || !FMath::IsFinite(VB))
			{
				++NonFiniteTraceSamples;
				continue;
			}
			DrawDebugLine(GetWorld(), ToWorld(FrameA.Position + FrameA.Up * VA),
				ToWorld(FrameB.Position + FrameB.Up * VB), C.Colour, true, -1.f, 0, 2.f);
		}

		Walk = FrameB;
		PrevS = B.S;
	}

	// SAID OUT LOUD, because a hole in a trace looks like a trace. If a channel
	// went non-finite the ride profile has a real problem somewhere upstream of
	// the drawing, and the drawing is the only thing that noticed.
	if (NonFiniteTraceSamples > 0)
	{
		UE_LOG(LogTUEvents, Warning,
			TEXT("ride profile: %d trace segment(s) skipped, a channel was not finite. "
				 "The reported maxima do NOT show this -- std::max ignores NaN."),
			NonFiniteTraceSamples);
	}

	// Where the train gave up, if it did. A marker beats a log line nobody reads.
	if (!Profile_.bCompleted)
	{
		const FTrackFrame Stall = Track.EvaluateAt(Profile_.StalledAtS);
		DrawDebugSphere(GetWorld(), ToWorld(Stall.Position), 200.f, 12, FColor::Red, true, -1.f, 0,
			4.f);
	}
}

void ATUCoasterRide::ServeHolds(std::size_t TrainIndex)
{
	// THE DISPATCHER, entire. Everything else in this file is geometry, physics or
	// drawing; this is the ride control system, and it is four lines because the
	// two layers underneath it already know what they are doing.
	//
	// It asks about the train's CENTRE, because that is what FTrain::Step tests a
	// zone against, and because zones and blocks fall out of the same walk — so a
	// zone never straddles a block boundary and "the block my centre is in" is
	// unambiguous.
	//
	// A device with nobody standing at it is left where it was, which is closed:
	// brakes-on is the resting state, and a device that opens because nobody is
	// asking is a device that fails open.
	// It writes to a DRIVE, not to the track. A command is a request; how fast the
	// drive gets there, and whether it manages to, is the drive's business and the
	// panel's story. This is the whole of the PLC's authority over the ride.
	if (!Signals || !Drives || TrainIndex >= static_cast<std::size_t>(Trains.Num()))
	{
		return;
	}
	FTrain& T = *Trains[TrainIndex];
	const int Z = T.FindHoldZoneAt(T.GetDistance());
	if (Z < 0)
	{
		return;   // not standing at a holding device; nothing to command
	}
	const std::size_t Zi = static_cast<std::size_t>(Z);

	// THE PERMISSIVE IS AN AND, and the interlocking is only one term of it. A real
	// dispatch needs the blocks clear AND the riders aboard AND the restraints
	// locked AND the platform confirmed AND the device about to take the train
	// ready — and on a working ride the block is usually the term that went green
	// first, while an operator was still walking the train. Before the station
	// process existed a train left the instant the track ahead was free, which is a
	// ride with nobody in it.
	//
	// The last term is PRE-LAUNCH, and it is a genuinely different question from
	// the first: CLEAR IS NOT READY. A block with a launch in it can be perfectly
	// empty and still refuse a train, because the launch has not armed.
	if (Signals->CanRelease(TrainIndex, T.GetDistance()) && StationSaysGo(Zi)
		&& DeviceAheadIsReady(T.GetDistance()))
	{
		Drives->Command(Zi, ZoneReleaseSpeed[Z]);
		return;
	}

	// HELD, and the hardware does this in TWO STAGES rather than one glide.
	//
	// A real block brake is two devices sharing a stretch of track. A sensor sits
	// just before the pad; the brake trips as the train ENTERS and clamps a fin
	// under the car, stopping it as hard as it is allowed to — and the limit is
	// RIDER COMFORT, not distance, because the alternative is whiplash. So the
	// train stops wherever that lands. Only then do rubber tyres engage and convey
	// it forward into an acceptable holding position.
	//
	// Commanding a crawl speed says both stages in one number, because a zone
	// closes the gap to its target using its full authority: from 26 m/s the brake
	// bites at everything it has, and from rest the tyres push. The sequence falls
	// out — hard stop, creep to the mark, held.
	//
	// The conveying stage is also what keeps the train INSIDE ITS OWN BLOCK. Brake
	// alone and it stops ~0.3 m past the zone start; the station's start is the
	// circuit's seam, so that leaves its back half in the LAST block, a dwelling
	// train holds two, and three trains deadlock — each denied by the tail of the
	// one in front. Real rides reposition for exactly the same reason.
	// WHERE it parks: the nose HoldNoseClearanceM short of the far end of the
	// block. Measured from the end and applied to the nose, because the thing
	// being prevented is a train protruding into the next zone through a defect —
	// the margin is the whole point, so it is expressed as the margin.
	//
	// Not the middle. Mid-device was the minimum fix for the seam straddle and is
	// arbitrary everywhere else: on the 130 m mid-course brake it parked a train
	// 65 m in with 65 m of empty brake ahead of it, where a real one holds near
	// the exit.
	//
	// AND A SWITCH IS WHAT SAYS SO, not a sum. The rule is "truck forward until the
	// stop mark trips" — no train length, no arc length, no zone extent. A PLC has
	// none of those three and does not need them: the margin was surveyed into the
	// track when the switch was bolted down, and from then on the ride enforces it
	// by geometry rather than by arithmetic.
	//
	// The mark trips on the NOSE, because a span covers a point the moment its
	// front reaches it — the same asymmetry the block counter runs on, and the
	// reason a stop mark measures the thing it is named after.
	//
	// A mark can be tripped by ANY train, since a switch has no idea which. It is
	// unambiguous only because the interlocking guarantees the train standing at
	// this device is the only one that can be in this block. That mutual support is
	// the whole sensor layer's premise; read a mark for a block that is not yours
	// and you are reading somebody else's train.
	//
	// MEASURED on the two-train circuit: the station mark sits at 25.00 m, the nose
	// parks at 25.18 m, and 0.82 m of the metre survives the crawl overshoot.
	// ponytail: 1.5 m/s of crawl, a maintenance-pace guess.
	const double Convey = FMath::Min(ZoneReleaseSpeed[Z], 1.5);

	// A MISSING SWITCH IS NOT PERMISSION TO MOVE. No sensor list, or one that
	// disagrees with the zone list about how many devices there are, means the mark
	// this train is trucking towards cannot be read — and a device that cannot be
	// told when to stop must not be told to go. Same direction as every other rule
	// in this system: fail closed.
	const bool bAtMark = !StopMarks || Zi >= StopMarks->Num() || StopMarks->IsBlocked(Zi);
	Drives->Command(Zi, bAtMark ? 0.0 : Convey);
}

// ONE SCAN, AT A FIXED PERIOD. Called from Tick as many times as the elapsed
// frame is worth, never once per rendered frame — see Tick below for why.
void ATUCoasterRide::SimStep(double DeltaSeconds)
{
	// THE SCAN CYCLE, and it is the whole shape of this function. IEC 61131-3 runs
	// a PLC as read inputs -> execute the program -> write outputs, and each step
	// below is one of those:
	//
	//   1. read every switch on the track, once
	//   2. run the dispatcher for every train against that ONE snapshot
	//   3. let the drives ramp toward what they were just commanded
	//   4. write those outputs to the track
	//   5. step the world, and tell the signalling where everything ended up
	//   6. read the motors back
	//
	// Interleaving 2 with 5 — serving train 1 after train 0 has already moved — is
	// what a game does and what a PLC cannot: a program that re-reads its inputs
	// mid-scan acts on two different worlds in one pass.
	//
	// Overlaps live on blocks rather than on trains, so Signals->Tick is ONCE PER
	// FRAME: ticking per train would age a 5 s overlap in 5/N seconds and nothing
	// anywhere would say so. Same contract for Drives->Tick, which ramps at a rate.
	//
	// Holding is done by GATING THE ZONE, not by declining to integrate. The old
	// version held the train at the station by simply not stepping it, which
	// worked for exactly one train in exactly one place; a zone commanded to zero
	// holds a train anywhere there is a device to hold it, and the station stops
	// being a special case.
	if (StopMarks)
	{
		StopMarks->BeginScan();
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			StopMarks->Cover(Trains[t]->GetRearS(), Trains[t]->GetFrontS(),
				bTrackIsCircuit, Track.TotalLength());
		}
		StopMarks->EndScan();
	}
	// Session time, for the event log. Not the wall clock: what matters is how long
	// before the stop something happened, and a relative figure survives a paused
	// PIE session where a wall clock does not.
	RideClock += DeltaSeconds;

	// AFTER the scan and the physics below would be wrong: this reads the state
	// the frame ENDED in, and reading it at the top means every transition is
	// reported one frame after it happened. Placed here, against the same values
	// the panel is about to draw, so the log and the screen cannot disagree.
	SampleRestraints();
	LogTransitions();

	// THE SCAN'S FINGERPRINT. Same fields, same order, as the prototype suite's
	// digest — two sessions on the same preset, left alone, must agree at the same
	// scan number. Physics AND control state, because either can drift alone: a
	// train in the right place with the wrong block state is still a different run.
	++ScanNumber;
	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		SimFingerprint.Add(Trains[t]->GetDistance());
		SimFingerprint.Add(Trains[t]->GetSpeed());
	}
	if (Signals)
	{
		for (std::size_t k = 0; k < Signals->NumBlocks(); ++k)
		{
			SimFingerprint.Add(static_cast<int>(Signals->GetState(k)));
		}
	}
	if (Drives)
	{
		for (std::size_t z = 0; z < Drives->Num(); ++z)
		{
			SimFingerprint.Add(Drives->Output(z));
		}
	}

	// TIER 3, LAST, AFTER THE FINGERPRINT IS TAKEN. That placement is the whole
	// claim: everything hashed above is the ride, and the show layer is a
	// subscriber to it that cannot reach back. Asserted rather than asserted-to —
	// test_twotrains.cpp runs the same circuit with and without this and the two
	// digests are equal to the bit.
	PublishShowEvents();

	// The Details-panel checkbox, so the stop can be tripped without playing. Read
	// here rather than in a PostEditChangeProperty because it has to work in PIE.
	if (bEmergencyStop && Drives && !Drives->IsEmergencyStopped())
	{
		Drives->PressEmergencyStopButton("operator");
	}
	// THE CONTROLLER SCANS, and its watchdog is the overrun the accumulator
	// already detects one layer up. Reporting that as a note rather than a trip
	// was the machine having a symptom with nowhere to put it.
	// THE SCAN OVERRUN IS NOT A WATCHDOG EVENT, and wiring it to one was a
	// mistake that killed the ride.
	//
	// Two different things were conflated. A real PLC watchdog means THE
	// PROGRAM'S OWN LOGIC took longer than its scan period — a controller fault,
	// and rightly latching. The accumulator's overrun means THE HOST CANNOT
	// RENDER FAST ENOUGH to give the simulation its share of wall-clock time.
	// The controller is executing every scan perfectly; there are simply fewer of
	// them per second.
	//
	// Measured, which is how the difference became obvious: the editor dropped to
	// ~3 fps, so every 333 ms frame could only afford 100 ms of simulation and
	// dropped 233 ms. That is correct graceful degradation — the ride runs at 30%
	// speed and remains bit-identical, because the step is fixed. Faulting on it
	// meant a machine that renders slowly is a machine whose ride will not run,
	// which is nonsense.
	//
	// AND THE BAR WAS THIS LOW: the 3 fps was the print-screen overlay taking
	// priority while somebody took a SCREENSHOT of the panel. Observing the ride
	// killed it, permanently, and any hitch would have done — an alt-tab, a
	// shader compile, something else waking up. Not an edge case worth a note; a
	// thing that would have happened to everybody, constantly.
	//
	// So the overrun stays what it was: a reported performance note. The PLC's
	// watchdog stays for a genuine one, which in this model would be the scan's
	// own logic exceeding its period — straight-line code, so never yet.
	const bool bWasFaulted = Plc.IsFaulted();
	Plc.Scan(DeltaSeconds, /*bOverran*/ false);
	bScanOverranThisFrame = false;
	// A CONTROLLER FAULTING IS THE LOUDEST THING THAT CAN HAPPEN TO A RIDE, and
	// it was silent. The watchdog tripped, the ride stopped being commanded, and
	// the only trace was three violations a second and a half later with nothing
	// connecting them to a cause.
	if (Plc.IsFaulted() && !bWasFaulted)
	{
		LogEvent(FString::Printf(TEXT("PLC FAULT — %s"),
			UTF8_TO_TCHAR(Plc.FaultReason())));
	}

	ServeStations(DeltaSeconds);

	// THE PROGRAM RUNS ONLY IF THE MACHINE IS RUNNING IT. Not a stop — a
	// controller with no permission simply commands nothing, and a device with
	// no command falls to its safe state exactly as it would with the cabinet
	// unplugged. The stop does not come from here, and the E-stop below is
	// untouched by it: that separation is constraint 7 and it is asserted in
	// test_plcunit.cpp against the real drive layer.
	if (Plc.OutputsEnabled())
	{
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			ServeHolds(static_cast<std::size_t>(t));
		}
	}
	else if (Drives)
	{
		// A CONTROLLER THAT IS NOT COMMANDING MUST NOT LEAVE EVERYTHING RUNNING,
		// and the first version of this did exactly that.
		//
		// Skipping ServeHolds does not mean "no command" — it means the LAST
		// command stands. On a ride that has just opened that is each zone's
		// PRESET, so every brake sat at its release speed and all three trains
		// left the moment the watchdog tripped. Three signalling violations, 1.47
		// seconds in, from a controller that had faulted and was doing nothing.
		//
		// The header of PlcUnit.h claimed a device with no command falls to its
		// safe state "exactly as it would with the cabinet unplugged". That was
		// aspirational: nothing made it true. This does.
		//
		// In a real cabinet the PLC's output card DE-ENERGISES when it faults —
		// the drives lose their enable and the brakes, being spring-applied,
		// bite. Commanding zero is that, expressed in the one authority this
		// layer has.
		for (std::size_t z = 0; z < Drives->Num(); ++z)
		{
			Drives->Command(z, 0.0);
		}
	}

	// One drive, every train's copy of the zone. Zones live on each FTrain, so
	// before drives existed every train carried its own private idea of what every
	// brake on the ride was doing; a real brake is ONE device acting on whatever is
	// in it, and this is what makes them agree.
	if (Drives)
	{
		Drives->Tick(DeltaSeconds);

		// STRAIGHT AFTER THE TICK, because that is the statement that moves Output
		// and the release is an edge on it. SHOW READS, IT DOES NOT ASK: nothing
		// here can change whether a brake released, only notice that it did.
		ServeBrakeReleaseSound();
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			for (std::size_t z = 0; z < Drives->Num(); ++z)
			{
				Trains[t]->SetZoneTargetSpeed(z, Drives->Output(z));

				// THE PAD TRACKS THE SAME COMMAND, and bites at its own rate.
				//
				// Not a second command from the PLC, and that is deliberate: a
				// block brake is commanded to a SPEED, and the pad and the tyres
				// are two ways of getting there. Above the commanded speed the
				// pad is what removes the energy, at whatever the hardware was
				// specified at; below it the tyres push. The four-state sequence
				// ServeHolds describes — bite, stop, release, convey — then falls
				// out of one number instead of needing a state machine, because
				// "release" is just the train no longer being above the limit.
				//
				// A ZONE WITH NO PAD KEEPS ITS NEGATIVE LIMIT and is untouched,
				// which is every zone on every preset until somebody authors one.
				if (z < static_cast<std::size_t>(ZoneBrakeDecel.Num())
					&& ZoneBrakeDecel[static_cast<int32>(z)] > 0.0)
				{
					Trains[t]->SetZoneBrakeLimit(z, Drives->Output(z));
				}
				// And how much of that the hardware is actually producing, for the
				// same reason and by the same route: one device, one number, every
				// train's copy hearing the same thing about it. 1.0 unless
				// something has injected a failure, so this is free until it is not.
				Trains[t]->SetZoneHealth(z, Drives->DeliveredFraction(z));
			}
		}
	}

	for (int32 t = 0; t < Trains.Num(); ++t)
	{
		Trains[t]->Step(DeltaSeconds);

		// The return is the only record a signalling violation leaves besides the
		// counter, so it is read rather than discarded. With two trains it is also
		// the collision report.
		if (Signals && !Signals->Update(static_cast<std::size_t>(t),
			Trains[t]->GetRearS(), Trains[t]->GetFrontS()))
		{
			UE_LOG(LogTemp, Error,
				TEXT("TrackUnlimited: SIGNALLING VIOLATION — train %d entered a block that "
					"was not clear, at %.1f m."),
				t, Trains[t]->GetDistance());

			// AND IT STOPS THE RIDE. Until the E-stop existed this was a log line
			// and nothing else: the interlocking detected the one thing it is for
			// and then let the ride carry on into it. A violation is the definition
			// of an E-stop condition on real hardware.
			LogEvent(FString::Printf(TEXT("SIGNALLING VIOLATION — train %d at %.0f m"),
				t, Trains[t]->GetDistance()));
			if (Drives && Drives->TripEmergencyStop("signalling violation"))
			{
				bEmergencyStop = true;
			}
		}
	}
	if (Signals)
	{
		Signals->Tick(DeltaSeconds);
	}
	CrossCheckOccupancy();

	// The motors report back: what each is actually turning at, and how much of its
	// authority that is taking. A drive with no train on it goes unreported, which
	// is how EndFeedback learns it is free-running rather than slipping against a
	// stale reading from the last train through.
	if (Drives)
	{
		Drives->BeginFeedback();
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			const double At = Trains[t]->GetDistance();
			for (std::size_t z = 0; z < Drives->Num(); ++z)
			{
				if (Trains[t]->IsInZone(z, At))
				{
					Drives->ReportFeedback(z, Trains[t]->GetSpeed(), Trains[t]->GetZoneLoad(z));
				}
			}
		}
		Drives->EndFeedback();

		// A drive that trips is a ride an operator has to go and look at, so it says
		// so once rather than every frame. Reported, never acted on: what a ride does
		// about a failed drive is an E-stop policy and the PLC's decision, not a
		// property of the motor.
		for (std::size_t z = 0; z < Drives->Num(); ++z)
		{
			if (Drives->IsFaulted(z) && !ReportedDriveFault.Contains(static_cast<int32>(z)))
			{
				ReportedDriveFault.Add(static_cast<int32>(z));
				UE_LOG(LogTemp, Error,
					TEXT("TrackUnlimited: DRIVE FAULT — zone %d commanded %.1f m/s, output "
						"%.1f, motor reading %.1f, at full torque and not gaining."),
					static_cast<int32>(z), Drives->Read(z).Commanded, Drives->Read(z).Output,
					Drives->Read(z).Actual);

				// A motor at full torque going nowhere is a stalled lift, a failed
				// launch or a brake that is not biting, and every one of those is a
				// ride an operator has to walk out to. The DRIVE still only reports
				// — deciding what a ride does about a failed motor is the PLC's job,
				// and this is the PLC doing it.
				LogEvent(FString::Printf(TEXT("DRIVE FAULT — zone %d, full torque, not gaining"),
					static_cast<int32>(z)));
				if (Drives->TripEmergencyStop("drive fault"))
				{
					bEmergencyStop = true;
				}
			}
		}
	}
}

// THE NEAR PLANE IS ONE GLOBAL, and that is UE's answer rather than a limitation
// of this code: UCameraComponent carries an ortho near plane and no perspective
// one, because FMinimalViewInfo::PerspectiveNearClipPlane defaults negative
// meaning "use GNearClippingPlane". SetNearClipPlaneGlobals is what the engine's
// own r.SetNearClipPlane console command calls, and it moves the render thread's
// copy as well — writing GNearClippingPlane directly moves only half of it.
//
// One global is not a compromise here. There is one camera.
//
// GUARDED, because this is called every frame and the setter flushes rendering
// commands. A near plane that has not moved must not cost a flush.
static void SetNearPlaneMetres(double Metres)
{
	// The engine clamps its own console command to 1 cm, so match it rather than
	// discovering the floor by rendering through it.
	const float Cm = FMath::Max(1.f, static_cast<float>(Metres * MetresToUU));

	// A PROPORTIONAL GUARD, not an absolute one. The near plane is
	// distance * 0.002, so it moves every frame the camera does — and an absolute
	// 0.05 cm tolerance meant a render command enqueued on every one of them.
	// It is only an enqueue rather than a flush, so this was not the performance
	// problem it looked like, but a near plane exists to be the right ORDER OF
	// MAGNITUDE and updating it by half a percent buys exactly nothing.
	if (Cm < GNearClippingPlane * 0.8f || Cm > GNearClippingPlane * 1.25f)
	{
		SetNearClipPlaneGlobals(Cm);
	}
}

// [U] — EVERY OVERLAY OFF, AND BACK EXACTLY AS IT WAS.
//
// A MASTER GATE RATHER THAN A SNAPSHOT. Saving the eight toggles, clearing them
// and putting them back is the obvious version and it is wrong: anything that
// touches one while hidden gets overwritten on restore, and the symptom is a
// panel coming back that somebody had deliberately turned off. One bool
// consulted at draw time restores the previous state for free, because nothing
// was ever changed.
//
// The wireframe and the ride-profile trace are PERSISTENT debug lines drawn once
// at BeginPlay rather than every frame, so hiding them means flushing and
// unhiding means redrawing. Everything else is a per-frame gate.
void ATUCoasterRide::NextRiderSeat()
{
	RiderSeat = (RiderSeat + 1) % FMath::Max(1, CarCount * 2);
	UE_LOG(LogTUEvents, Log, TEXT("[N] seat %d"), RiderSeat);
}

void ATUCoasterRide::NextRiderTrain()
{
	if (Trains.Num() <= 1)
	{
		// SAID RATHER THAN IGNORED. A key that does nothing on a one-train
		// layout is indistinguishable from a key that is broken, and this one is
		// pressed precisely when somebody is looking for the other trains.
		UE_LOG(LogTUEvents, Log, TEXT("[T] one train on this layout"));
		return;
	}
	RiderTrain = (ActiveTrainIndex() + 1) % Trains.Num();

	// A CAMERA SNAP IS NOT A TELEPORT, and the chase camera has to be told so.
	// Left alone it smooths from where it was toward the new train, which on a
	// circuit is a long sweep across the whole layout rather than a cut.
	bFreeInitialised = false;
	UE_LOG(LogTUEvents, Log, TEXT("riding train %d of %d"), RiderTrain + 1, Trains.Num());
}

void ATUCoasterRide::ToggleSettings()
{
	if (FrameWidget)
	{
		FrameWidget->ToggleSettings();
	}
}

void ATUCoasterRide::ToggleOverlays()
{
	bHideOverlays = !bHideOverlays;

	if (UWorld* World = GetWorld())
	{
		FlushPersistentDebugLines(World);
	}
	if (!bHideOverlays)
	{
		// bDrawTrack is still consulted, so [F2] does not turn on a wireframe
		// somebody had switched off in the Details panel.
		if (bDrawTrack) { DrawTrack(); }
		DrawRideProfile();
	}

	// SAID ONCE, IN THE LOG, because the banner that would normally carry a hint
	// is one of the things just hidden — and a screenshot mode that printed a
	// caption over the screenshot would defeat itself.
	UE_LOG(LogTUEvents, Log, TEXT("overlays %s  [U]"),
		bHideOverlays ? TEXT("hidden") : TEXT("shown"));
}

void ATUCoasterRide::SyncCameraRig()
{
	// THE SWAP HAPPENS WHEN THE MODE CHANGES, NOT ON THE NEXT TICK. It used to
	// run at the top of Tick, one frame after a mode change -- AFTER the menu,
	// a template open and [M] had each framed the orbit, so every one of them
	// was replaced by whatever the stored rig last held. That is the "framed
	// tight once, far the next" bug: the distance was leaking across modes.
	// Callers that frame call this first; Tick still calls it as a backstop.
	if (CameraMode != LastCameraMode)
	{
		CameraRigs.For(static_cast<int>(LastCameraMode)) = Orbit;
		Orbit = CameraRigs.For(static_cast<int>(CameraMode));
		LastCameraMode = CameraMode;
	}
}

void ATUCoasterRide::ApplyOrbitToCamera(double DeltaSeconds)
{
	if (!Camera) { return; }
	// THE CAMERA GLIDES TO THE ORBIT RATHER THAN SNAPPING. `Orbit` is where
	// the framing and the mouse say to be; `OrbitShown` is where the camera is,
	// and it closes the gap with the frame-rate-independent Smoothed() from
	// CameraRig.h. A frame call with no delta snaps, which is what the menu's
	// first frame wants. A snap loses the sense of where you were, which is the
	// whole reason a validation row that jumps you somewhere is disorienting.
	if (DeltaSeconds <= 0.0)
	{
		OrbitShown = Orbit;
	}
	else
	{
		const double HalfLife = 0.12;
		OrbitShown.Focus = Smoothed(OrbitShown.Focus, Orbit.Focus, HalfLife, DeltaSeconds);
		OrbitShown.Distance = Smoothed(OrbitShown.Distance, Orbit.Distance, HalfLife, DeltaSeconds);
		OrbitShown.PitchDeg = Smoothed(OrbitShown.PitchDeg, Orbit.PitchDeg, HalfLife, DeltaSeconds);
		// Yaw the short way round, or a target across the +-180 seam spins the
		// camera the long way.
		double DYaw = Orbit.YawDeg - OrbitShown.YawDeg;
		while (DYaw > 180.0) { DYaw -= 360.0; }
		while (DYaw < -180.0) { DYaw += 360.0; }
		OrbitShown.YawDeg = Orbit.YawDeg - Smoothed(DYaw, 0.0, HalfLife, DeltaSeconds);
	}
	const FCamVec P = OrbitShown.Position();
	const FVector World = ToLocal(FVec3{P.X, P.Y, P.Z}) + GetActorLocation();
	const FVector Focus = ToLocal(FVec3{OrbitShown.Focus.X, OrbitShown.Focus.Y, OrbitShown.Focus.Z})
		+ GetActorLocation();
	Camera->SetWorldLocationAndRotation(World, (Focus - World).Rotation().Quaternion());

	// THE NEAR PLANE FOLLOWS THE CAMERA. A 90 m lift hill and a 2 cm bolt
	// cannot share a fixed one: set it for the bolt and depth precision at the
	// top of the hill is gone; set it for the hill and the restraint in front
	// of a rider is clipped away.
	const FDepthRange D = DepthRangeFor(OrbitShown.Distance, OrbitShown.Distance);
	SetNearPlaneMetres(D.Near);
}

void ATUCoasterRide::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// THE MENU YIELDS TO A QUESTION. The confirm, the recovery offer and the
	// settings page are above it (canvas, or the frame's slot); a widget that
	// kept taking clicks under them would answer a question nobody asked it.
	// THE EDITOR FOLLOWS ITS FLAG, which [B], [U] and the mode switch all flip
	// from different places; syncing here is one line against four call sites.
	const bool bPanelsUp = !bHideOverlays
		&& Session.Mode() != EAppMode::Boot && Session.Mode() != EAppMode::MainMenu
		&& !(FrameWidget && FrameWidget->IsSettingsOpen());
	ShowEditorWidget(bShowSegmentEditor && bPanelsUp);
	ShowPaintedWidget(bPanelsUp);
	if (MenuWidget && MenuWidget->IsInViewport())
	{
		const bool bAsking = bConfirmingMenu || Session.HasRecovery();
		const bool bSettings = FrameWidget && FrameWidget->IsSettingsOpen();
		MenuWidget->SetVisibility(bSettings ? ESlateVisibility::Collapsed
			: bAsking ? ESlateVisibility::HitTestInvisible
			: ESlateVisibility::SelfHitTestInvisible);
	}

	// ===================== AUTOSAVE, WHICH NOW HAS SOMEBODY TO TELL =====================
	//
	// Deliberately unwired until today: it writes a SIDECAR for crash recovery to
	// OFFER at boot, and until boot existed there was nothing that could ever
	// offer it. Writing files nobody can recover from is half a feature.
	//
	// It NEVER TOUCHES THE DOCUMENT. Writing over somebody's file on a timer is
	// data loss with extra steps — it destroys the last known-good state to
	// preserve one they did not ask for — so this is a separate file, and the
	// session owns WHEN while this owns HOW.
	//
	// On the wall clock and above the early returns: a paused ride is still a
	// session somebody is working in.
	if (Session.TickAutosave(static_cast<double>(DeltaSeconds)))
	{
		const FString Text = SerialiseDocument();
		if (!Text.IsEmpty())
		{
			FFileHelper::SaveStringToFile(Text, *SidecarPath(),
				FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
		}
	}

	// A DRAG IS A THING THAT HAPPENS BETWEEN EVENTS, so it is followed here rather
	// than on a mouse-move that this input setup does not have. Above the early
	// returns for the same reason the timer below is: it is not part of the ride.
	TickScrub();

	// WALL CLOCK, NOT THE SIM CLOCK. A paused ride still counts this down, because
	// the person reading it is not paused -- and it sits above the early returns
	// below for the same reason: it is not part of the ride.
	if (DragAnswerSeconds > 0.0)
	{
		DragAnswerSeconds -= static_cast<double>(DeltaSeconds);
	}

	// ===================== THE CURSOR IS NOT THE TRAIN'S =====================
	//
	// This guard is right -- everything below it needs a train -- and it sat ABOVE
	// ApplyCursorMode, which needs nothing at all. Harmless until the menu became
	// a document with no train in it, and then the one screen that is nothing but
	// things to click was the one screen with no pointer.
	ApplyCursorMode();
	if (Trains.Num() == 0 || !Trains[0].IsValid())
	{
		return;
	}
	// THE TRAIN YOU ARE ON, which used to be Trains[0] and a comment saying so.
	// The camera, the seat readout and the profile scrubber all follow it; the
	// simulation does not care, because every train is stepped either way.
	const int32 Rider = ActiveTrainIndex();
	if (!Trains[Rider].IsValid())
	{
		return;
	}
	FTrain* const Train = Trains[Rider].Get();

	// ===================== A FIXED SCAN PERIOD =====================
	//
	// The scan used to run ONCE PER RENDERED FRAME, which is wrong twice over.
	//
	// IT IS UNFAITHFUL. A PLC scans on a fixed period; it does not scan faster
	// because the graphics card is idle. Running it at frame rate meant the ride's
	// control system executed at 144 Hz on one machine and 40 Hz on another, and
	// every rate in it moved with that — edge detection, restraint travel, overlap
	// ageing, drive ramps. The already-documented "a train can cross a sensor
	// inside one scan" limitation was quietly WORSE on a slow machine, which is
	// exactly backwards from how a limitation should behave.
	//
	// AND IT IS NOT REPRODUCIBLE. Two runs of the same session diverge on the
	// first frame, so nothing can be recorded and replayed — and a fault-injection
	// scenario that reproduces differently every run cannot prove anything to
	// anybody. Everything else about replay is downstream of this one change.
	//
	// It also brings the actor into line with the prototype suites, which have
	// always stepped at a fixed 1/240 s. The canonical figures were never affected
	// because they come from those; it was only the thing you actually played that
	// varied.
	const double Step = 1.0 / static_cast<double>(FMath::Max(1, SimHz));

	// THE FIRST FRAME AFTER A LOAD IS NOT AN OVERRUN. It carries the whole level
	// load — hundreds of milliseconds of it — and reporting that as a missed scan
	// deadline is the panel's first lie of the session: nothing was late, because
	// nothing had started. A controller powers up and begins scanning; it does not
	// log the time before power-on as lost.
	//
	// Swallowed rather than clamped, because there is no correct amount of that
	// frame to simulate. The ride begins now.
	// WAIT FOR A PLAUSIBLE FRAME BEFORE THE RIDE STARTS, rather than swallowing
	// exactly one. Swallowing one was not enough: PIE takes several frames to
	// settle — shaders, streaming, the first draw — and the SECOND frame still
	// carried 300 ms of it, overran, and tripped the PLC watchdog. None of that
	// is ride time, and a controller coming online is not one missing a deadline.
	//
	// Self-calibrating rather than a frame count, because how many frames a load
	// takes is a property of the machine and the threshold is a property of what
	// a frame plausibly is.
	if (!bScanStarted)
	{
		if (DeltaSeconds > 0.1)
		{
			SimAccumulator = 0.0;
			return;
		}
		bScanStarted = true;
		SimAccumulator = 0.0;
		return;
	}

	// ANOTHER WINDOW HAS FOCUS, AND THE RIDE WAITS. `sim.pauseUnfocused`.
	//
	// It is the SAME MECHANISM as every other clock control — no time arrives, and
	// the step never changes — so an alt-tab is a gap in the wall clock rather than
	// a different ride. What it avoids is the alternative: an unfocused window is
	// throttled to a few frames a second, each carrying more time than the
	// accumulator may work off, so the ride drops it and reports overruns. That run
	// cannot be judged afterwards, and the seconds it lost are gone either way.
	//
	// Not folded into bSimPaused, which is the OPERATOR's pause and shows in the
	// banner: coming back from a coffee to a ride that says somebody paused it is
	// a different and worse lie than the one this fixes.
	if (bPauseWhenUnfocused && !FApp::HasFocus())
	{
		SimAccumulator = 0.0;
		return;
	}

	// PAUSED, SCALED, OR STEPPED — and all three change how much TIME arrives,
	// never the step itself. The scan period stays 1/SimHz, so a quarter-speed
	// run is the same sequence of scans delivered more slowly and the digest
	// still matches a real-time run. Scaling the step instead would change every
	// rate inside the controller and make slow motion a different ride.
	if (bSimPaused)
	{
		// A single step is COUNTED rather than timed, so one press is exactly one
		// scan whatever the frame rate — which is the whole reason to want it.
		if (StepsOwed > 0)
		{
			SimStep(Step);
			--StepsOwed;
		}
		SimAccumulator = 0.0;
		return;
	}

	// THE FIRST FRAME'S DELTA IS NOT RIDE TIME, and counting it reported an
	// overrun on every single run before anything had moved.
	//
	// BeginPlay sweeps a quarter-million-triangle mesh, runs the whole ride
	// profile, derives the blocks and lays down the persistent debug lines — and
	// UE's first Tick delta includes all of it plus the level load. Roughly 0.3 s
	// on this layout, which is thirty times the cap, so the accumulator opened
	// owing more than it could ever work off.
	//
	// A CONTROLLER DOES NOT OWE SCANS FOR THE TIME BEFORE IT WAS POWERED ON.
	// That is the actual argument, not that the number is inconvenient: the ride
	// starts when it starts, and there is no backlog to drop because there was
	// no ride yet to fall behind.
	//
	// ONLY THE FIRST FRAME. A hitch mid-run is a real overrun and still reports
	// one — this must not become a general "clamp the delta", which would quietly
	// disable the detector that alt-tabbing already showed is worth having.
	if (!bSimClockStarted)
	{
		bSimClockStarted = true;
		SimAccumulator = 0.0;
		return;
	}
	SimAccumulator += DeltaSeconds * static_cast<double>(TimeScale);

	int32 Ran = 0;
	while (SimAccumulator >= Step && Ran < MaxStepsPerFrame)
	{
		SimStep(Step);
		SimAccumulator -= Step;
		++Ran;
	}

	// SCAN OVERRUN. A frame so long that the backlog exceeds the cap — a hitch, a
	// breakpoint, a level load. THE BACKLOG IS DROPPED rather than worked off,
	// because catching up means running the ride faster than real time, and a ride
	// that fast-forwards through a hitch can skip a train past a block boundary:
	// the one failure this whole layer exists to prevent.
	//
	// Real controllers treat a missed scan deadline as a fault to report rather
	// than time to make up, which is the same call for the same reason.
	if (SimAccumulator >= Step)
	{
		++ScanOverruns;
		bScanOverranThisFrame = true;   // the PLC's watchdog reads this next scan

		// THE TOTAL IS THE NUMBER THAT MATTERS, not the count. One overrun is a
		// hitch; a count says how many times without saying how much ride went
		// missing, and 545 of them reads as "a bit stuttery" when it can be a
		// minute of simulation that never happened.
		//
		// This is what decides whether a run can be JUDGED. Dropped time is not
		// slow motion — the scans never ran, so a train really did move further
		// between two scans than it should have, and any behaviour somebody
		// watched across a drop is behaviour of a ride the model did not compute.
		ScanTimeDroppedS += SimAccumulator;
		WorstOverrunS = FMath::Max(WorstOverrunS, SimAccumulator);

		LogEvent(FString::Printf(
			TEXT("SCAN OVERRUN — dropped %.0f ms (overrun %d, %.2f s dropped in total)"),
			SimAccumulator * 1000.0, ScanOverruns, ScanTimeDroppedS));
		SimAccumulator = 0.0;
	}

	// ---- Everything below is DRAWING, and runs once per rendered frame. It reads
	// the state the last scan left and never advances anything.
	// NO MARKERS ON THE MENU'S BACKDROP: it is a picture of a ride, not a ride
	// being diagnosed.
	const bool bDocumentOpen = Session.Mode() != EAppMode::MainMenu
		&& Session.Mode() != EAppMode::Boot;
	if (!bHideOverlays && bDocumentOpen)
	{
		// RESTRAINT MARKERS DO NOT DRAW FROM THE SEAT. They sit 0.35 m above the
		// heartline and the eye sits 0.25 m above it, so on the train you are
		// riding the camera is INSIDE one — a green wireframe box filling the
		// screen with the ride behind it.
		//
		// Suppressed by CAMERA rather than by app mode, because the obstruction
		// is a property of where the camera is: Ride is not the only way to end
		// up in the seat, and Build with [C] on rider is the same view.
		//
		// Block markers stay. A signal going amber as you approach it is the
		// causal chain this project exists to make visible, and seeing it from
		// the train is the best place to see it from.
		if (CameraMode != ETUCameraMode::Rider)
		{
			DrawRestraints();
		}
		DrawBlockMarkers();
	}

	// WHERE THE RIDER IS SITTING IS A SEAT, not a fraction of a train. The
	// continuous -1..+1 slider described a rider half in one car; a real ride
	// has row 1 and row 2. Seat.h is the one answer for along, across and up,
	// and the camera sits exactly in it -- the eye offset that used to be
	// added here is the seat's VerticalM.
	FSeat Seat = SeatByIndex(RiderSeat, CarCount,
		TrainMeshSettings().BodyWidthM * 0.25);   // a seat each side of centre
	Seat.VerticalM = RiderEyeAboveHeartlineM;     // the knob still owns the eye height
	const double SeatOffset = SeatOffsetAlongM(Seat, CarCount, static_cast<double>(CarLengthM));
	const FTrackFrame Frame = SeatFrame(Train->GetFrameAt(SeatOffset), Seat);
	const FQuat Rotation = ToWorldRotation(Frame);

	// THE BODY OF THE TRAIN, ONE BOX PER SAMPLE POINT — and the boxes ABUT into
	// one continuous body rather than standing apart as vehicles.
	//
	// A SAMPLE POINT IS NOT A CAR, and treating it as one is what the first
	// version did. SampleCount() is 9 because that is what the mean-height
	// gravity integration needs; it has nothing to do with how many vehicles a
	// train has, and FTrainConfig has no car count at all — it models a train as
	// a LENGTH. Dividing that length into nine and leaving a gap between each
	// piece made a 15 m train look accidentally plausible (nine 1.5 m boxes) and
	// the 6 m small-batch vehicle look like nine playing cards standing on edge.
	// Same code, and only one of the two presets exposed it.
	//
	// So the boxes are the sample SPACING long and touch end to end. What is
	// drawn is then exactly what the physics knows: a body of TrainLength,
	// following the track, with no invented vehicle count.
	//
	// Frames come from the train rather than from Track.EvaluateAt, which is
	// O(track length) a call and would be nine of those per train every frame.
	//
	// ===================== AND ALL OF THAT IS NOW THE FALLBACK =====================
	//
	// Everything above is still true and still correct, and it is no longer what
	// gets drawn. It was the honest answer while a train was a LENGTH with nine
	// sample points and no car count; `CarCount` and `CarLengthM` exist now, and
	// `RebuildTrainMesh` builds real cars with bogies, three sets of wheels and
	// couplers off those instead.
	//
	// KEPT RATHER THAN DELETED, and behind the same kind of switch the track mesh,
	// the supports and the catwalks each have: unchecking `bBuildTrainMesh` puts
	// the boxes back. The physics is untouched either way - the sample points go on
	// doing exactly what they always did, which is the whole point of the split.
	RebuildTrainMesh();
	if (Cars && Cars->IsVisible() == bBuildTrainMesh)
	{
		Cars->SetVisibility(!bBuildTrainMesh);
	}
	if (!bBuildTrainMesh)
	{
		const int32 Points = Train->NumSamplePoints();
		const int32 Total = Points * Trains.Num();
		// Nose-to-tail over Points samples is Points-1 gaps.
		const double Spacing = (Points > 1 && TrainLengthM > 0.f)
			? static_cast<double>(TrainLengthM) / (Points - 1) : 2.4;
		if (Cars->GetInstanceCount() != Total)
		{
			Cars->ClearInstances();
			for (int32 i = 0; i < Total; ++i)
			{
				Cars->AddInstance(FTransform::Identity, true);
			}
		}

		// Generic, per constraint 5: a shade under a metre and a half across, and
		// CHEST HEIGHT ON A SEATED RIDER rather than head height.
		//
		// 0.9 m, and the number is load-bearing: the heartline is 1.1 m above the
		// rail centreline, so a body any taller than that swallows the point the
		// ride camera sits at and the rider spends the lap inside a box. A real
		// car body comes up to about the chest for the same reason — you have to
		// be able to see out of it.
		const double BodyWidth = 1.4;
		const double BodyHeight = 0.9;
		const FVector CarScale(Spacing, BodyWidth, BodyHeight);

		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			for (int32 i = 0; i < Points; ++i)
			{
				const int32 Slot = t * Points + i;
				const FTrackFrame& CarFrame = Trains[t]->GetSamplePoint(i);

				// SITS ON THE RAILS, WHICH IS NOT THE SAME AS CENTRED ON THEM.
				// The first version put the box's CENTRE at the rail plane, so
				// half of it was buried in the ties and the spine — which is
				// exactly what the screenshot showed. A cube's origin is its
				// middle, so the body has to be lifted by half its height.
				//
				// The rider still sits at the heartline, which is above this.
				// That distinction is the entire reason the heartline model
				// exists, so the slice shows it rather than putting both in one
				// place.
				const FVec3 OnRails = CarFrame.Position
					- CarFrame.Up * (Track.GetHeartlineHeight() - BodyHeight * 0.5);
				Cars->UpdateInstanceTransform(Slot,
					FTransform(ToWorldRotation(CarFrame), ToWorld(OnRails), CarScale), true,
					Slot == Total - 1, true);
			}
		}
	}

	// EACH MODE KEEPS ITS OWN CAMERA, so Build to Ride to Build returns you where
	// you were. Cheap, obvious, and never added later because it is never the most
	// urgent bug — but it is the difference between a viewport somebody works in
	// and one they fight.
	//
	// BEFORE the chain below, not inside it: the swap has to happen whichever mode
	// is being entered, and putting it between two branches would run Free and
	// then fall into the rest of the chain as well.
	SyncCameraRig();

	// LOOKING IS A DRAG NOW, because the cursor is free.
	//
	// A camera that turned whenever the mouse moved is fine when the mouse has
	// nothing else to do, and impossible once there are rows to click: every
	// click becomes a gamble on where the panel went while you were reaching for
	// it. Holding the right button is what every editor with both a viewport and
	// a panel does, and it costs nothing to somebody who only wants to look.
	//
	// ZEROED RATHER THAN SKIPPED, so a drag starts from where the mouse is
	// instead of applying everything it travelled while the button was up.
	PollMovementKeys();
	if (!bDraggingLook && Session.Mode() != EAppMode::Ride)
	{
		LookYaw = 0.f;
		LookPitch = 0.f;
	}

	if (CameraMode == ETUCameraMode::Free)
	{
		// Flown by hand while the ride carries on without you. Seeded from
		// wherever the camera already was, so switching in does not teleport.
		if (!bFreeInitialised)
		{
			FreeLocation = Camera->GetComponentLocation();
			FreeRotation = Camera->GetComponentRotation();
			FreeRotation.Roll = 0.f;
			bFreeInitialised = true;
		}

		// FREE-FLY IS HEAD-TURNING: mouse down looks down, uninverted. The two
		// signs flip from THAT rather than from a raw axis, which is what makes
		// "invert" mean the same thing to somebody who has never read this file.
		const float Sens = FMath::Clamp(LookSensitivity, 0.25f, 4.f) * 2.2f;
		const float FlyX = bFlyInvertX ? -1.f : 1.f;
		const float FlyY = bFlyInvertY ? -1.f : 1.f;
		FreeRotation.Yaw += LookYaw * Sens * FlyX;
		FreeRotation.Pitch =
			FMath::Clamp(FreeRotation.Pitch + LookPitch * Sens * FlyY, -87.f, 87.f);
		FreeRotation.Roll = 0.f; // a free camera that rolls is a lost camera

		const FVector Forward = FreeRotation.Vector();
		const FVector Right = FRotationMatrix(FreeRotation).GetScaledAxis(EAxis::Y);
		const float Speed = FreeCameraSpeedMs * MetresToUU * (bBoost ? 5.f : 1.f) * DeltaSeconds;
		FreeLocation += (Forward * MoveForward + Right * MoveRight) * Speed
			+ FVector(0.f, 0.f, MoveUp * Speed);

		Camera->SetWorldLocationAndRotation(FreeLocation, FreeRotation.Quaternion());
	}
	else if (CameraMode == ETUCameraMode::Console)
	{
		// ===================== WHERE AN OPERATOR STANDS =====================
		//
		// Beside the platform at eye height, looking along the track the way the
		// train leaves. Not a free camera parked there: the position is DERIVED
		// from the station zone the block walk already found, so it is right on
		// every layout including the ones that do not exist yet.
		//
		// Offset to the rider's LEFT of the heartline, which is +Lateral in the
		// prototypes' frame — a console sits beside the train, not on it.
		const double S = ConsoleStandS();
		const FTrackFrame F = Track.EvaluateAt(S);
		const FVector Base = ToWorld(F.Position);
		// A DIRECTION, NOT A POINT — the actor transform's rotation without its
		// translation, which is what ToLocalDirection exists to keep separate.
		const FVector Side = GetActorTransform().TransformVectorNoScale(
			ToLocalDirection(F.Lateral));
		const FVector Along = GetActorTransform().TransformVectorNoScale(
			ToLocalDirection(F.Tangent));

		// 2.5 m to the side and 1.6 m up: standing at a waist-high desk beside the
		// train, which is what every station photograph this was corrected against
		// shows. Height is from the HEARTLINE, so it is eye level rather than
		// track level.
		// ON THE PLATFORM SIDE, which is the walkway side, and at a standing
		// person's eye height above the PLATFORM: the platform top is 0.75 m
		// under the heartline, so eyes are 0.85 m over it rather than 1.6.
		const double SideSign = PresetWalkwaySide != ETUWalkway::Right ? 1.0 : -1.0;
		const FVector Eye = Base + Side * (SideSign * 2.5 * MetresToUU) + FVector(0.f, 0.f, 0.85f * MetresToUU);
		// LOOKING DOWN THE TRACK, slightly toward it. An operator watches the train
		// and the way out, which is the same direction.
		const FVector Aim = Base + Along * (18.0 * MetresToUU);
		const FRotator Face = (Aim - Eye).Rotation();
		Camera->SetWorldLocationAndRotation(Eye, Face.Quaternion());
	}
	else if (CameraMode == ETUCameraMode::Orbit)
	{
		// Around a point, which is what editing wants. Free-fly is for going
		// somewhere; orbit is for looking at what you are already at.
		//
		// The state and the arithmetic are in Prototypes/Shell/CameraRig.h and
		// are tested there — this converts at the same boundary as everything
		// else, so the framing maths never learns which way round Unreal's Y is.
		if (!bOrbitFramed) { FrameWholeTrack(); }

		const double OrbitSens = FMath::Clamp(LookSensitivity, 0.25f, 4.f) * 2.2;
		if (Session.Mode() == EAppMode::MainMenu || Session.Mode() == EAppMode::Boot)
		{
			// THE MENU'S BACKDROP TURNS. Slowly: a full circle in about two
			// minutes, which reads as alive rather than as a camera somebody is
			// flying. Wall-clock, because the menu is not the ride.
			Orbit.AddYaw(3.0 * DeltaSeconds);
		}
		Orbit.AddYaw(LookYaw * OrbitSens * (bOrbitInvertX ? -1.0 : 1.0));
		// ORBIT IS SUBJECT-DRAGGING, SO PITCH IS NEGATED. You are pulling the
		// thing you are looking at, not turning your head: mouse down swings the
		// camera up and over the top of the subject, which is what every DCC tool
		// does and what the raw axis did backwards.
		//
		// The invert flips from THAT convention, not from the raw axis — so
		// somebody who ticks it gets the opposite of what they were just feeling,
		// which is the only reading of the word that is useful.
		// The sign here used to compensate for Position() having pitch backwards;
		// Position() is right now, so this is the plain axis. Mouse down still
		// swings the camera up and over the subject.
		Orbit.AddPitch(LookPitch * OrbitSens * (bOrbitInvertY ? -1.0 : 1.0));   // clamped, not wrapped
		Orbit.Pan(MoveRight * DeltaSeconds * 120.0, MoveUp * DeltaSeconds * 120.0);

		ApplyOrbitToCamera(DeltaSeconds);
	}
	else if (CameraMode == ETUCameraMode::Rider)
	{
		// THE HEARTLINE IS THE RIDER'S HEART, NOT THEIR EYES, and the camera wants
		// the second one. Sitting exactly on it puts the view at chest height,
		// inside the car body — which is where it was.
		//
		// ADDED HERE AND NOWHERE ELSE. The heartline is what FeltG is computed
		// about and what the banking is built around; moving it to suit a camera
		// would change every G number on the ride to fix a framing problem. This
		// is a cosmetic offset on the view alone and touches no physics.
		Camera->SetWorldLocationAndRotation(ToWorld(Frame.Position), Rotation);
		// Centimetres in a seat, or the restraint in front of the rider is clipped
		// off — the other end of the same trade the orbit camera makes. 2 cm is
		// twice the engine's own 1 cm floor, so it survives the clamp.
		SetNearPlaneMetres(0.02);
	}
	else
	{
		// Behind and above, held LEVEL with the world. The old non-ride view
		// took the rider's rotation and simply stepped back from it, which meant
		// it turned upside down through the loop — disorienting rather than
		// dramatic, and it made the inversion hard to actually look at.
		//
		// Direction of travel flattened into the horizontal plane. Through a
		// vertical section there is no horizontal component to speak of, so the
		// last good one is held rather than letting the camera spin.
		const FVector Target = ToWorld(Train->GetFrame().Position);
		FVector Forward = ToWorldRotation(Train->GetFrame()).GetForwardVector();
		Forward.Z = 0.f;
		if (Forward.SizeSquared() > 0.05f)
		{
			LastChaseForward = Forward.GetSafeNormal();
		}

		const FVector Desired = Target - LastChaseForward * (ChaseDistanceM * MetresToUU)
			+ FVector(0.f, 0.f, ChaseHeightM * MetresToUU);

		// Critically-damped-ish follow. A rigid offset reads as a camera bolted
		// to the train; lagging it is what makes the speed legible.
		//
		// ponytail: one exponential smooth, no spring, no collision. Add a
		// spring arm the day the camera needs to avoid terrain.
		if (!bChaseInitialised)
		{
			ChaseLocation = Desired;
			bChaseInitialised = true;
		}
		const float Alpha = 1.f - FMath::Exp(-6.f * DeltaSeconds);
		ChaseLocation = FMath::Lerp(ChaseLocation, Desired, Alpha);

		Camera->SetWorldLocationAndRotation(ChaseLocation,
			(Target - ChaseLocation).ToOrientationQuat());
	}

	TelemetryLines.Reset();
	if (bShowTelemetry && !bHideOverlays && GEngine)
	{
		// What the RIDER feels, at whichever row they are sitting in — not the
		// train's centre. That is the whole point of choosing a seat.
		const FGForces G = Train->GetForcesAt(SeatOffset);
		const double S = Train->GetDistance();
		Telemetry(1, 0.f, FColor::White,
			FString::Printf(TEXT("%6.1f km/h    %5.1f m along %.0f m    height %5.1f m"),
				Train->GetSpeed() * 3.6, S, Track.TotalLength(), Frame.Position.Z));
		Telemetry(2, 0.f,
			G.Vertical > 4.5 || G.Vertical < -1.0 ? FColor::Red : FColor::Green,
			FString::Printf(TEXT("vertical %+5.2f G    lateral %+5.2f G    fore-aft %+5.2f G"),
				G.Vertical, G.Lateral, Train->GetTangentialG()));

		// Front and back, when there is a train to have ends. Every car shares
		// one speed, so the spread is purely which curvature each one is on —
		// and it is the whole reason people queue for the back row.
		if (TrainLengthM > 0.f)
		{
			const double Half = TrainLengthM * 0.5;
			Telemetry(6, 0.f, FColor(200, 200, 120),
				FString::Printf(TEXT("%.0f m train:  front %+5.2f G    back %+5.2f G"),
					TrainLengthM, Train->GetForcesAt(+Half).Vertical,
					Train->GetForcesAt(-Half).Vertical));
		}
		Telemetry(3, 0.f, FColor::Silver,
			S >= BrakeStartS ? TEXT("BRAKE RUN") : TEXT("on course"));

		// The block row. Making the causal chain VISIBLE is the pillar, not
		// decoration — a block holding its overlap is why the train in the
		// station is not moving, and that should be readable off the screen
		// rather than inferred.
		if (Signals)
		{
			FString Row;
			for (std::size_t b = 0; b < Signals->NumBlocks(); ++b)
			{
				const TCHAR* Tag = TEXT("CLEAR");
				if (Signals->GetState(b) == EBlockState::Occupied) { Tag = TEXT("OCCUPIED"); }
				else if (Signals->GetState(b) == EBlockState::Buffer) { Tag = TEXT("BUFFER"); }

				Row += FString::Printf(TEXT("[%d %s"), static_cast<int32>(b), Tag);
				if (Signals->GetState(b) == EBlockState::Buffer)
				{
					Row += FString::Printf(TEXT(" %.1fs"), Signals->GetBufferRemaining(b));
				}
				Row += TEXT("]  ");

				// WRAPPED, because this line does not. AddOnScreenDebugMessage
				// draws one unbroken run and clips what will not fit — and it
				// clips the START, so an eleven-block circuit showed a row
				// beginning "CUPIED]" with blocks 0 to 5 simply gone. The blocks
				// most likely to be missing were the low-numbered ones, which on
				// every preset here is the station.
				//
				// Six per line, and the newline is what does it: the on-screen
				// message renderer honours embedded ones, so this stays a single
				// keyed message that replaces itself rather than a pile of them
				// fighting over slots.
				if ((b + 1) % 6 == 0 && b + 1 < Signals->NumBlocks())
				{
					Row += TEXT("\n");
				}
			}
			// "Held" is now a property of a train standing at a device, not a flag:
			// a train is held when it is at a holding place and its permissive is
			// refusing. Reported per train, because with two of them "the ride is
			// held" is not a fact about the ride.
			FString HeldRow;
			for (int32 t = 0; t < Trains.Num(); ++t)
			{
				const double At = Trains[t]->GetDistance();
				if (Trains[t]->FindHoldZoneAt(At) >= 0
					&& !Signals->CanRelease(static_cast<std::size_t>(t), At))
				{
					HeldRow += FString::Printf(TEXT("train %d HELD at %.0f m   "), t, At);
				}
			}
			Telemetry(8, 0.f,
				HeldRow.IsEmpty() ? FColor(120, 200, 140) : FColor(255, 176, 32), Row);

			if (!HeldRow.IsEmpty())
			{
				Telemetry(9, 0.f, FColor(255, 176, 32),
					HeldRow + TEXT("— dispatch permissive not satisfied"));
			}
			if (Signals->Violations() > 0)
			{
				Telemetry(10, 0.f, FColor::Red,
					FString::Printf(TEXT("%d SIGNALLING VIOLATION(S)"),
						static_cast<int32>(Signals->Violations())));
			}

			// A caught train is a THIRD outcome, and it needs its own words. The
			// device did its job and the ride still failed: nobody is in danger and
			// the layout is still wrong. Reporting it as "held" would read as the
			// signalling working; reporting nothing would read as the ride working.
			FString CaughtRow;
			for (int32 t = 0; t < Trains.Num(); ++t)
			{
				if (Trains[t]->GetRollbacksCaught() > 0)
				{
					CaughtRow += FString::Printf(TEXT("train %d %s at %.0f m   "), t,
						Trains[t]->IsHeldByCatch() ? TEXT("HELD BY ANTI-ROLLBACK")
												   : TEXT("was caught"),
						Trains[t]->GetDistance());
				}
			}
			if (!CaughtRow.IsEmpty())
			{
				Telemetry(11, 0.f, FColor(255, 90, 60),
					CaughtRow + TEXT("— the catch worked, the layout did not"));
			}

			// WHAT IS HOLDING THE DISPATCH, named. This is the whole reason the
			// station's gates are modelled one at a time instead of as a single
			// "platform ready" flag: "the station is not ready" is useless to
			// somebody standing on it, and "restraints not locked" is somewhere to
			// go and look. It is also the first piece of the control panel that
			// exists, and it is a view over data the PLC layer already had.
			FString StationRow;
			for (const FTUPlatform& P : Platforms)
			{
				static const TCHAR* PhaseName[] = {
					TEXT("empty"), TEXT("arriving"), TEXT("unloading"), TEXT("loading"),
					TEXT("securing"), TEXT("READY"), TEXT("departing")};
				const int32 Ph = static_cast<int32>(P.Process.GetPhase());
				StationRow += FString::Printf(TEXT("zone %d %s"), P.Zone,
					Ph >= 0 && Ph < UE_ARRAY_COUNT(PhaseName) ? PhaseName[Ph] : TEXT("?"));

				const FString Holding = UTF8_TO_TCHAR(P.Process.WhatIsHolding());
				if (!Holding.IsEmpty())
				{
					StationRow += FString::Printf(TEXT(" — %s"), *Holding);
				}
				else if (P.Process.IsReadyToDispatch())
				{
					// Ready but still standing means the OTHER half of the AND is
					// the one saying no, which is exactly the distinction an
					// operator wants and cannot otherwise see.
					StationRow += Signals && !Signals->CanRelease(0, ZoneSpans[P.Zone].StartS)
						? TEXT(" — waiting on the block ahead") : TEXT("");
				}
				StationRow += TEXT("   ");
			}
			if (!StationRow.IsEmpty())
			{
				Telemetry(12, 0.f, FColor(120, 170, 255),
					StationRow + (bManualDispatch
						? TEXT("   [Space] dispatch") : TEXT("   auto")));
			}

			// Loudest thing on screen, and it stays until somebody resets it. A
			// stop nobody has looked at has not been dealt with.
			if (Drives && Drives->IsEmergencyStopped())
			{
				Telemetry(13, 0.f, FColor::Red,
					FString::Printf(
						TEXT("*** EMERGENCY STOP — %s ***   power is cut to every drive; ")
						TEXT("trains run to the next brake and hold.   %s"),
						UTF8_TO_TCHAR(Drives->EmergencyStopReason()),
						Drives->AnyUnacknowledged()
							? TEXT("[Home] acknowledge, then [End] to reset")
							: TEXT("[End] to reset")));
			}
			else
			{
				Telemetry(13, 0.f, FColor(120, 120, 120),
					TEXT("[Backspace] emergency stop"));
			}
		}

		// Discoverable, because a keybinding nobody knows about does not exist.
		//
		// Each mode names its OWN controls rather than sharing one line. A hint
		// listing keys that do nothing in the mode you are actually in is worse
		// than no hint, because it is the same amount of reading and then wrong.
		FString CamLine;
		switch (CameraMode)
		{
		case ETUCameraMode::Rider:
			CamLine = TEXT("[C] camera: rider");
			break;
		case ETUCameraMode::Chase:
			CamLine = TEXT("[C] camera: chase");
			break;
		case ETUCameraMode::Free:
			CamLine = TEXT("[C] camera: free     WASD / Q E / mouse, Shift to hurry");
			break;
		default:
			CamLine = FString::Printf(
				TEXT("[C] camera: orbit    mouse to turn, wheel to zoom, [F] frames all  (%.0f m)"),
				Orbit.Distance);
			break;
		}
		Telemetry(7, 0.f, FColor(120, 170, 200), CamLine);

		// The ride's own worst case, alongside the current reading, so a number
		// on screen means something without having to remember the whole lap.
		// Roll rate is here rather than in the G line because it belongs to a
		// different question: G is what presses on you, roll rate is what spins
		// you, and no amount of looking at the first will show you the second.
		Telemetry(4, 0.f, FColor(150, 150, 150),
			FString::Printf(
				TEXT("this ride: %.0f km/h max, %+.2f..%+.2f vertical, %.2f lateral, ")
				TEXT("%.0f deg/s roll"),
				Profile_.TopSpeed * 3.6, Profile_.MinVerticalG, Profile_.MaxVerticalG,
				Profile_.MaxAbsLateralG, Profile_.MaxAbsRollRate));
		if (!Profile_.bCompleted)
		{
			Telemetry(5, 0.f, FColor::Red,
				FString::Printf(TEXT("TRAIN DOES NOT GET ROUND — stalls at %.0f m"),
					Profile_.StalledAtS));
		}
	}

	// ONLY THE MOUSE IS RESET HERE NOW. A bound axis ACCUMULATES over the frame
	// and has to be cleared after it is read; a polled key is read fresh at the
	// top of the frame, and clearing it here as well would be harmless today and
	// wrong the first time anything reads movement after this line.
	LookYaw = LookPitch = 0.f;

	// Send it round again once it has settled at the END of the track. The return
	// to the station is a teleport, and the range diff handles it with no special
	// case: the last block exits and arms its overlap, the station block enters.
	//
	// ONLY FOR LAYOUTS THAT DO NOT CLOSE. On a measured circuit FTrain wraps, so
	// a train drives into the station under its own power and none of this runs —
	// IsAtEnd is false for ever and the last block is just another block. This is
	// the fallback for the three presets whose two ends are hundreds of metres
	// apart, where a wrap would be inventing continuity that is not there.
	//
	// TWO GUARDS, both of which only matter with more than one train: it must be
	// stopped in the LAST block (a train held at a block brake is also stopped,
	// and teleporting that one would take it out of the queue it is waiting in),
	// and the station must be empty (or two trains land on each other, which the
	// interlocking would then correctly and uselessly report as a violation).
	if (Signals && !bTrackIsCircuit)
	{
		const std::size_t Last = Signals->NumBlocks() - 1;
		for (int32 t = 0; t < Trains.Num(); ++t)
		{
			// "Arrived" is stopped OR at the end of the track, and the second half
			// is not redundant: the last block is now a block brake that HOLDS its
			// target, so a train released from it is still doing its release speed
			// when it runs out of track. Testing speed alone would leave it
			// pressed against the end at 2 m/s for ever and the ride would hang.
			const bool bSettled = Signals->BlockAt(Trains[t]->GetDistance()) == Last
				&& (Trains[t]->GetSpeed() <= 0.0 || Trains[t]->IsAtEnd());
			if (!bSettled)
			{
				StoppedForS[t] = 0.f;
				continue;
			}
			StoppedForS[t] += DeltaSeconds;
			if (StoppedForS[t] >= RestartDelaySeconds && !Signals->Occupies(0))
			{
				Trains[t]->Place(0.0, 0.0);
				Signals->Update(static_cast<std::size_t>(t), Trains[t]->GetRearS(),
					Trains[t]->GetFrontS());
				StoppedForS[t] = 0.f;
			}
		}
	}
}
