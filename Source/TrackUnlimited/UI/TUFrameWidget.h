// TrackUnlimited: the application frame — the persistent chrome every mode sits
// inside, and the first thing that makes this an application rather than a
// viewport with text drawn over it.
//
// ===================== WHAT THIS IS FOR =====================
//
// Everything on screen today is immediate-mode drawing on the debug canvas. That
// was the right call while nothing else existed — it needs no asset and it works
// the moment somebody runs it — but it cannot do the one thing a shell must:
// look like a designed application with a place for each thing.
//
// The frame owns MODE, IDENTITY and STATUS, and nothing else. What each mode
// shows goes in the content slot, so a panel never has to know what is around it
// and the frame never has to know what a panel does.
//
// ===================== BEHAVIOUR IN C++, LAYOUT IN THE ASSET =====================
//
// `UI_CONVENTIONS.md` decided UMG-primary with the mitigation that behaviour
// lives in `UUserWidget` subclasses and the asset holds layout only. That is what
// `BindWidget` enforces: the asset must provide widgets with these exact names or
// it fails to compile, so the contract is checked rather than hoped for. Nothing
// here reads a colour, a font or a size from the asset — those come from
// `FTUStyle`, which is text.
//
// ===================== AND IT SIZES NOTHING =====================
//
// The layout decision is "fixed, no docking", kept cheap to revisit by one
// restraint: NO PANEL HARDCODES ITS OWN DIMENSIONS. So this file contains no
// widths, no heights and no row counts — a 4K display is meant to show more
// rows, not larger ones, and a frame that pinned a panel size would be the first
// thing to break that.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Shell/SessionState.h"
#include "TUFrameWidget.generated.h"

class ATUCoasterRide;
class UTextBlock;
class UButton;
class UNamedSlot;

UCLASS(Abstract)
class TRACKUNLIMITED_API UTUFrameWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	UTUFrameWidget(const FObjectInitializer& Init);

	/**
	 * The ride this frame is a shell around.
	 *
	 * A WEAK POINTER, because the frame outlives nothing and the actor outlives
	 * it: a raw pointer here is a dangling read the first time somebody
	 * reloads a level with the UI still up.
	 */
	UPROPERTY(BlueprintReadOnly, Category = "TrackUnlimited")
	TWeakObjectPtr<ATUCoasterRide> Ride;

	void AttachTo(ATUCoasterRide* InRide);

	/**
	 * Show or hide the settings screen IN THE CONTENT SLOT.
	 *
	 * Not a separate full-screen widget stacked on top: the frame's whole job is
	 * that each mode's UI is composed into it, and a settings screen that floated
	 * over the chrome would be the first exception to that — after which there
	 * would be others.
	 *
	 * The main menu will call this same method when it exists. Wiring it to a key
	 * first means the screen is testable before the menu is written, rather than
	 * the two having to land together.
	 */
	void ToggleSettings();

	/** Close it if it is open. Every mode change calls this: a settings page left
	 *  in the content slot over a new mode is a menu nobody can get rid of. */
	void CloseSettings();

	bool IsSettingsOpen() const { return SettingsWidget != nullptr; }

	/** Found by path, like the frame itself. Null is a valid state. */
	UPROPERTY(EditAnywhere, Category = "TrackUnlimited|UI")
	TSubclassOf<class UTUSettingsWidget> SettingsWidgetClass;

protected:
	virtual void NativeConstruct() override;
	virtual void NativeTick(const FGeometry& Geometry, float DeltaSeconds) override;

	// ===================== THE CONTRACT WITH THE ASSET =====================
	//
	// BindWidget is REQUIRED rather than Optional, deliberately. An optional
	// binding that silently comes back null gives a frame that runs with half
	// its chrome missing and no complaint; a required one refuses to compile the
	// asset until the widget is there. The asset is the thing most likely to be
	// edited by somebody who has not read this file, so the check belongs where
	// they will meet it.

	/** Which mode: BUILD / OPERATE / RIDE. Reads AppModeName, so the frame and
	 *  the session can never disagree about what to call a mode. */
	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UTextBlock> ModeLabel;

	/** The document, and whether it has unsaved work. */
	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UTextBlock> DocumentLabel;

	/** One line of what the ride is doing. Deliberately short: the panels carry
	 *  detail, and a status bar that tries to is a panel in the wrong place. */
	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UTextBlock> StatusLabel;

	/**
	 * WHERE THE MODE'S OWN UI GOES.
	 *
	 * A named slot rather than a hardcoded child, so a panel is composed into the
	 * frame rather than known by it. That is what keeps "the frame owns mode,
	 * identity and status, and nothing else" true as panels are added.
	 */
	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UNamedSlot> ContentSlot;

	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UButton> BuildTab;

	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UButton> OperateTab;

	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UButton> RideTab;

	UFUNCTION()
	void OnBuildClicked();

	UFUNCTION()
	void OnOperateClicked();

	UFUNCTION()
	void OnRideClicked();

private:
	/**
	 * A MODE CHANGE IS A REQUEST, NOT A COMMAND.
	 *
	 * `FSession::MayEnter` can answer Refused — riding needs something to ride —
	 * or NeedsConfirmation, when leaving would lose unsaved work. So this asks,
	 * and reports the reason rather than silently doing nothing. A tab that
	 * appears dead is indistinguishable from one that is broken, which is the
	 * same rule the [T] key already follows.
	 *
	 * Confirmation is NOT invented here: the session returns the question and
	 * the shell puts it on screen. Passing bConfirmed without having asked is
	 * lying to a class that cannot tell, which is exactly why MayEnter and Enter
	 * are separate calls rather than one with a bool.
	 */
	void RequestMode(EAppMode Wanted);

	/** What was last pushed to each label, so a tick that changes nothing does
	 *  not invalidate layout. SetText on an unchanged string still dirties the
	 *  widget, and this runs every frame. */
	FString LastMode;
	FString LastDocument;
	FString LastStatus;

	UPROPERTY(Transient)
	TObjectPtr<class UTUSettingsWidget> SettingsWidget;
};
