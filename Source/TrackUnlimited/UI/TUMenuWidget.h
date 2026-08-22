// TrackUnlimited: the main menu, as UMG rather than debug-canvas drawing.
//
// The first piece of the shell to leave the canvas, per the order on the card:
// menu and browser first. It is BUILT IN CODE from the widget tree rather than
// from an asset, because every string, colour and row it shows is a live read
// off the ride (templates, the track list, whether the session is dirty) and an
// asset would hold nothing but a vertical box. Behaviour routes back through
// `ATUCoasterRide::MenuAction`, which is the same dispatcher the canvas clicks
// used -- the widget knows the row codes and nothing else.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Components/Widget.h"
#include <vector>
#include "TUMenuWidget.generated.h"

class ATUCoasterRide;
class UVerticalBox;
class UTextBlock;

/** A plan-view thumbnail: interleaved x,y in [0,1], painted as lines. */
UCLASS()
class UTUPlanThumb : public UWidget
{
	GENERATED_BODY()
public:
	std::vector<float> Plan;
	FLinearColor Colour = FLinearColor::White;
	float SizePx = 28.f;

protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;
};

/** One clickable row: UButton has no payload, so this carries the code. */
UCLASS()
class UTUMenuRow : public UObject
{
	GENERATED_BODY()
public:
	TWeakObjectPtr<ATUCoasterRide> Ride;
	int32 Action = 0;
	UFUNCTION() void OnClicked();
};

UCLASS()
class TRACKUNLIMITED_API UTUMenuWidget : public UUserWidget
{
	GENERATED_BODY()
public:
	TWeakObjectPtr<ATUCoasterRide> Ride;

	/** Rebuilds every row from the ride's current lists. Call whenever the menu
	 *  is shown or the track list changes -- this is the immediate-mode read,
	 *  done once per change instead of once per frame. */
	void Rebuild();

protected:
	virtual TSharedRef<SWidget> RebuildWidget() override;

private:
	UPROPERTY() TObjectPtr<UVerticalBox> Rows;
	UPROPERTY() TArray<TObjectPtr<UTUMenuRow>> Bindings;

	class UButton* AddRow(int32 Action, float PadY);
	UTextBlock* MakeText(const FString& S, const FLinearColor& C, const char* Font = "Font.Body");
};
