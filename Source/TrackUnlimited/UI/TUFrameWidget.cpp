#include "TUFrameWidget.h"

#include "TUStyle.h"
// Up a directory: the module's public include path is Source/TrackUnlimited, so
// the actor is a sibling of the UI folder rather than inside it.
#include "../TUCoasterRide.h"

#include "Components/TextBlock.h"
#include "Components/Button.h"
#include "Components/NamedSlot.h"
#include "TUSettingsWidget.h"
#include "Framework/Application/SlateApplication.h"
#include "UObject/ConstructorHelpers.h"

UTUFrameWidget::UTUFrameWidget(const FObjectInitializer& Init)
	: Super(Init)
{
	// By path, the same way the frame itself is found. Constructor rather than
	// BeginPlay so the Details panel shows what it will use.
	static ConstructorHelpers::FClassFinder<UTUSettingsWidget> SettingsBP(
		TEXT("/Game/UI/WBP_TUSettings"));
	if (SettingsBP.Succeeded())
	{
		SettingsWidgetClass = SettingsBP.Class;
	}
}

void UTUFrameWidget::AttachTo(ATUCoasterRide* InRide)
{
	Ride = InRide;
}

void UTUFrameWidget::NativeConstruct()
{
	Super::NativeConstruct();

	// Bound here rather than in the asset, because a click handler is behaviour
	// and behaviour is this file's half of the split. An asset that wired its own
	// events would be the first step back toward logic nobody can diff.
	if (BuildTab)   { BuildTab->OnClicked.AddDynamic(this, &UTUFrameWidget::OnBuildClicked); }
	if (OperateTab) { OperateTab->OnClicked.AddDynamic(this, &UTUFrameWidget::OnOperateClicked); }
	if (RideTab)    { RideTab->OnClicked.AddDynamic(this, &UTUFrameWidget::OnRideClicked); }

	// The style set is registered by the module, so it is here by now — but the
	// text colours are applied from code rather than left to the asset for the
	// same reason the palette is C++ at all: a colour typed into a widget asset
	// is a colour nobody can review.
	const FLinearColor Primary = FTUStyle::TextPrimary;
	const FLinearColor Secondary = FTUStyle::TextSecondary;
	if (ModeLabel)     { ModeLabel->SetColorAndOpacity(FSlateColor(Primary)); }
	if (DocumentLabel) { DocumentLabel->SetColorAndOpacity(FSlateColor(Secondary)); }
	if (StatusLabel)   { StatusLabel->SetColorAndOpacity(FSlateColor(Secondary)); }
}

void UTUFrameWidget::NativeTick(const FGeometry& Geometry, float DeltaSeconds)
{
	Super::NativeTick(Geometry, DeltaSeconds);

	ATUCoasterRide* R = Ride.Get();
	if (!R)
	{
		return;
	}

	// ===================== READ LIVE, CACHE NOTHING BUT THE STRING =====================
	//
	// Every number the control panel shows is read live so it cannot drift, and
	// the frame follows the same rule. What IS cached is the last string pushed
	// to each label — SetText on an identical string still marks the widget
	// dirty and re-lays it out, and this runs every frame.

	// ===================== THE FRAME BELONGS TO A DOCUMENT =====================
	//
	// In Boot and MainMenu nothing is open, so the mode buttons offer to switch
	// between views of nothing and the document label names no document.
	//
	// ITS CHILDREN HIDE, NOT THE WIDGET ITSELF, and the first version got that
	// wrong with a symptom worth remembering: hiding the root LOST THE MOUSE at
	// the main menu. FInputModeGameAndUI needs something focusable in the UI to
	// hold, and with the only widget in the viewport hidden the viewport took the
	// capture back -- so the cursor vanished on the one screen that is nothing but
	// things to click.
	//
	// Collapsed rather than Hidden on the children, because these sit in a layout:
	// Hidden keeps a button's footprint and would leave three gaps across the top.
	const EAppMode Now = R->GetSession().Mode();
	const bool bDocumentOpen = Now != EAppMode::Boot && Now != EAppMode::MainMenu;
	const ESlateVisibility Parts = bDocumentOpen
		? ESlateVisibility::Visible : ESlateVisibility::Collapsed;
	if (ModeLabel)     { ModeLabel->SetVisibility(Parts); }
	if (DocumentLabel) { DocumentLabel->SetVisibility(Parts); }
	if (StatusLabel)   { StatusLabel->SetVisibility(Parts); }
	if (BuildTab)      { BuildTab->SetVisibility(Parts); }
	if (OperateTab)    { OperateTab->SetVisibility(Parts); }
	if (RideTab)       { RideTab->SetVisibility(Parts); }
	if (!bDocumentOpen) { return; }

	const FString Mode = UTF8_TO_TCHAR(AppModeName(R->GetSession().Mode()));
	if (Mode != LastMode)
	{
		LastMode = Mode;
		if (ModeLabel) { ModeLabel->SetText(FText::FromString(Mode)); }
	}

	// THE DIRTY MARK IS PART OF THE NAME, not a separate lamp. It is the one
	// piece of state somebody needs at a glance without going looking, and
	// FSession::IsDirty is a COMPARISON against the saved text rather than a
	// flag — so undo back to where you started and this clears itself.
	FString Doc = R->GetSession().HasPath()
		? FString(UTF8_TO_TCHAR(R->GetSession().Path().c_str()))
		: FString(TEXT("Untitled"));
	if (R->GetSession().IsDirty())
	{
		Doc += TEXT("  *");
	}
	if (Doc != LastDocument)
	{
		LastDocument = Doc;
		if (DocumentLabel) { DocumentLabel->SetText(FText::FromString(Doc)); }
	}

	// One line. The panels carry detail, and a status bar that tries to is a
	// panel in the wrong place.
	FString Status = FString::Printf(TEXT("%d train%s"),
		R->NumTrains(), R->NumTrains() == 1 ? TEXT("") : TEXT("s"));
	if (R->GetScanOverruns() > 0)
	{
		// THE SECONDS, NOT THE COUNT. 545 overruns reads as "a bit stuttery";
		// the seconds say whether the run can be judged at all.
		Status += FString::Printf(TEXT("   %d OVERRUN%s, %.1f s DROPPED"),
			R->GetScanOverruns(), R->GetScanOverruns() == 1 ? TEXT("") : TEXT("S"),
			R->GetScanTimeDroppedS());
	}
	if (Status != LastStatus)
	{
		LastStatus = Status;
		if (StatusLabel)
		{
			StatusLabel->SetText(FText::FromString(Status));
			// Colour carries it too, but never alone — the text says OVERRUN.
			StatusLabel->SetColorAndOpacity(FSlateColor(
				R->GetScanOverruns() > 0 ? FTUStyle::LampOccupied : FTUStyle::TextSecondary));
		}
	}
}

void UTUFrameWidget::ToggleSettings()
{
	if (!ContentSlot)
	{
		return;
	}
	if (SettingsWidget)
	{
		// REMOVED, NOT HIDDEN. A hidden settings screen keeps ticking and keeps
		// its generated rows alive behind whatever is in front of it.
		CloseSettings();
		return;
	}
	if (!SettingsWidgetClass)
	{
		UE_LOG(LogTUEvents, Warning, TEXT("settings: no widget class"));
		return;
	}
	SettingsWidget = CreateWidget<UTUSettingsWidget>(GetOwningPlayer(), SettingsWidgetClass);
	if (SettingsWidget)
	{
		SettingsWidget->AttachTo(Ride.Get());
		ContentSlot->SetContent(SettingsWidget);
	}
}

void UTUFrameWidget::CloseSettings()
{
	if (SettingsWidget && ContentSlot)
	{
		ContentSlot->ClearChildren();
		SettingsWidget = nullptr;
	}
	if (FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().SetAllUserFocusToGameViewport();
	}
}

void UTUFrameWidget::OnBuildClicked()   { RequestMode(EAppMode::Build); }
void UTUFrameWidget::OnOperateClicked() { RequestMode(EAppMode::Operate); }
void UTUFrameWidget::OnRideClicked()    { RequestMode(EAppMode::Ride); }

void UTUFrameWidget::RequestMode(EAppMode Wanted)
{
	ATUCoasterRide* R = Ride.Get();
	if (!R)
	{
		return;
	}

	const ELeaveRequest Answer = R->GetSession().MayEnter(Wanted);

	if (Answer == ELeaveRequest::Refused)
	{
		// SAID, NOT SWALLOWED. A tab that appears dead is indistinguishable from
		// one that is broken, and the session already knows the reason — riding
		// needs something to ride.
		UE_LOG(LogTUEvents, Log, TEXT("%s: %s"),
			UTF8_TO_TCHAR(AppModeName(Wanted)),
			UTF8_TO_TCHAR(R->GetSession().WhyNotEnter(Wanted)));
		return;
	}

	if (Answer == ELeaveRequest::NeedsConfirmation)
	{
		// NOT CONFIRMED HERE. The session returns the question and the shell is
		// supposed to put it on screen; passing bConfirmed without having asked
		// is lying to a class that cannot tell, which is exactly why MayEnter
		// and Enter are two calls rather than one with a bool parameter.
		//
		// Until the dialog exists this refuses and says why, which loses a
		// keystroke and never loses work. The wrong shortcut here would be to
		// confirm on the author's behalf.
		UE_LOG(LogTUEvents, Warning, TEXT("%s: %s — save first, or discard deliberately"),
			UTF8_TO_TCHAR(AppModeName(Wanted)),
			UTF8_TO_TCHAR(R->GetSession().WhyNotEnter(Wanted)));
		return;
	}

	// The actor owns what a mode DOES — camera, panels, whether edits are
	// accepted. The frame only asks for it, so there is one place that knows.
	R->EnterAppMode(Wanted);
}
