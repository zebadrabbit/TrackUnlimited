// TrackUnlimited: the settings screen, BUILT BY WALKING THE SCHEMA.
//
// ===================== NO ROW ASSETS, AND NO ROW LIST =====================
//
// The rows are constructed at runtime from `SettingsSchema()`, so adding a
// setting is one entry in that table and this file does not change. There is no
// per-row widget asset either — `WidgetTree->ConstructWidget` builds them from
// UMG primitives, which means a new setting cannot be blocked on somebody making
// an asset for it.
//
// That is the same answer the control panel gives: it walks the block and zone
// lists the geometry walks rather than keeping its own. A second list is a second
// thing to keep true, and the failure mode is silent — a setting that exists and
// is not shown looks exactly like a setting that does not exist.
//
// ===================== WHO OWNS THE VALUE IS NOT THIS FILE'S PROBLEM =====================
//
// An entry says where its value lives: ours (`FSettings`), the engine's
// (`UGameUserSettings`), or the input map's. This reads and writes through that,
// so the screen never becomes a third place a setting is stored — which is how
// the value you see and the value in effect start disagreeing.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Shell/SettingsSchema.h"
#include "TUSettingsWidget.generated.h"

class ATUCoasterRide;
class UVerticalBox;
class UHorizontalBox;
class UTextBlock;
class UButton;
class UTUSettingsWidget;

/**
 * ONE OF THESE PER GENERATED CONTROL, and it exists because UMG's delegates are
 * DYNAMIC: they take a UFUNCTION on a UObject and nothing else. No lambda, no
 * capture, and no sender passed to the handler — so a single handler on the
 * screen cannot tell which of thirty-seven rows called it.
 *
 * So the row's identity lives in an object bound alongside it. That is the
 * standard UMG answer to a generated list, and it is the cost of generation
 * rather than a smell: the alternative is thirty-seven hand-written handlers,
 * which is the hardcoded list this design exists to avoid.
 *
 * Kept alive by a UPROPERTY array on the screen — a bare `NewObject` with no
 * reference is garbage two frames later, and the symptom is a control that works
 * until it silently stops.
 */
UCLASS()
class UTUSettingBinding : public UObject
{
	GENERATED_BODY()

public:
	UPROPERTY()
	TWeakObjectPtr<UTUSettingsWidget> Owner;

	/** Deliberately not a UPROPERTY: FSettingEntry is a plain C++17 struct with
	 *  std::string in it, which the reflection system cannot describe and does
	 *  not need to. */
	FSettingEntry Entry;
	ESettingPage Page = ESettingPage::Video;

	UFUNCTION() void OnBool(bool bOn);
	UFUNCTION() void OnScalar(float Value);
	UFUNCTION() void OnChoice(FString Selected, ESelectInfo::Type Info);
	UFUNCTION() void OnPageClicked();

	/** A key row's button: arms the screen to take the next key press. */
	UFUNCTION() void OnKeyClicked();
	/** The text inside that button, so the capture can say "press a key". */
	UPROPERTY() TObjectPtr<UTextBlock> KeyText;

	/**
	 * A SLIDER IS RELEASED, AND ONLY THEN IS THE FILE WRITTEN.
	 *
	 * `OnValueChanged` fires every frame of a drag. Writing the settings file from
	 * it is a whole file rewritten per frame for as long as somebody holds the
	 * mouse down — which on a laptop is a disc kept awake by a volume slider, and
	 * is the same mistake `TickAutosave` refuses to make one layer up.
	 *
	 * The VALUE still updates live, so the setting applies as you drag. Only the
	 * write waits, which is the correct split: applying is cheap and immediate,
	 * persisting is neither.
	 */
	UFUNCTION() void OnScalarDone();
};

UCLASS(Abstract)
class TRACKUNLIMITED_API UTUSettingsWidget : public UUserWidget
{
	GENERATED_BODY()

public:
	void AttachTo(ATUCoasterRide* InRide);

	/** Rebuild the visible page. Called on construct and whenever a tab changes. */
	void ShowPage(ESettingPage Page);

	/** Arm the next key press for this row's action. Escape cancels. */
	void BeginKeyCapture(UTUSettingBinding* Row);

protected:
	virtual void NativeConstruct() override;
	/** The capture itself. Only does anything while a row is armed. */
	virtual FReply NativeOnKeyDown(const FGeometry& InGeometry, const FKeyEvent& InKeyEvent) override;
	virtual void NativeOnFocusLost(const FFocusEvent& InFocusEvent) override;

	/** A BACKSTOP, not the main path. Every control persists its own change as it
	 *  is made; this catches a slider moved with the keyboard, which never sends
	 *  the mouse-capture-end that the drag path relies on. */
	virtual void NativeDestruct() override;

	/** Where the generated rows go. The asset provides an empty box; everything
	 *  inside it is built here, so the asset never needs editing to add a row. */
	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UVerticalBox> RowBox;

	/** Where the generated page tabs go, for the same reason — the page list is
	 *  the enum, and hardcoding five buttons in the asset would be a sixth page
	 *  waiting to be forgotten. */
	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UHorizontalBox> PageTabs;

	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UTextBlock> PageHelp;

	/**
	 * RESET AFFECTS THE VISIBLE PAGE ONLY, and the button says which.
	 *
	 * A single "reset everything" is the kind of control somebody presses once,
	 * loses their bindings to, and never trusts again. Per-page is what they
	 * almost always mean, and the whole-lot version can be a second, plainer
	 * button if anybody asks for it.
	 */
	UPROPERTY(meta = (BindWidget))
	TObjectPtr<UButton> ResetPageButton;

	UFUNCTION()
	void OnResetPage();

	/** The close button, generated beside the page tabs. The page had no way off
	 *  it but the key that opened it, which from the menu nobody knows. */
	UFUNCTION()
	void OnClose();

private:
	UPROPERTY()
	TWeakObjectPtr<ATUCoasterRide> Ride;

	ESettingPage Current = ESettingPage::Video;

	/** The row waiting for a key, or null. */
	UPROPERTY() TObjectPtr<UTUSettingBinding> Capturing;

	void BuildPageTabs();
	void BuildRow(const FSettingEntry& Entry);
	UTUSettingBinding* Bind(const FSettingEntry& Entry);

public:
	/** Read a setting's current value as a string, from whichever store owns it. */
	FString ReadValue(const FSettingEntry& Entry) const;

	/** Write it back to whichever store owns it, and apply it. Public because the
	 *  per-row bindings call it — they are the handler UMG insists on, not a
	 *  second owner of the logic. Does NOT write the file; see PersistSettings. */
	void WriteValue(const FSettingEntry& Entry, const FString& Value);

	/**
	 * BACK TO THE DEFAULT, WHICH IS A DELETION AND NOT A WRITE.
	 *
	 * This existed as `WriteValue(E, E.Default)` and that was wrong in a way that
	 * shows up a year later: writing the default's current TEXT records it in the
	 * file as though the person had chosen it, so a better default shipped
	 * afterwards reaches everybody EXCEPT the ones who pressed reset. They are
	 * pinned to the value they were trying to get away from.
	 *
	 * `FSettings::Reset` erases the entry instead, and the read falls through to
	 * whatever the current default is. The store has always done this properly;
	 * the screen was going round it.
	 *
	 * UE-owned settings have no "unset", so those still write the default — which
	 * is the honest limit rather than an inconsistency, and it is UE's file.
	 */
	void ResetValue(const FSettingEntry& Entry);

	/** Write the settings file. Called when a change is FINISHED rather than as it
	 *  is made, because a drag makes sixty of them a second. */
	void PersistSettings() const;

private:
	/**
	 * TWO ARRAYS, BECAUSE THEY HAVE DIFFERENT LIFETIMES.
	 *
	 * Page tabs are built once and live as long as the screen. Row bindings are
	 * thrown away and rebuilt every time the page changes, or they would be live
	 * handlers on widgets that no longer exist.
	 *
	 * Kept apart rather than distinguished by a test on their contents. The first
	 * version had one array and told them apart by whether the binding had an
	 * entry key — which worked, and encoded "a tab has no entry" as a rule
	 * nothing states and the next person to give a tab an entry would break.
	 */
	UPROPERTY()
	TArray<TObjectPtr<UTUSettingBinding>> TabBindings;

	UPROPERTY()
	TArray<TObjectPtr<UTUSettingBinding>> RowBindings;
};
