#include "TUMenuWidget.h"

#include "TUStyle.h"
#include "../TUCoasterRide.h"
#include "Shell/FirstRun.h"
#include "Shell/TrackBrowser.h"

#include "Blueprint/WidgetTree.h"
#include "Components/Border.h"
#include "Components/Button.h"
#include "Components/CanvasPanel.h"
#include "Components/CanvasPanelSlot.h"
#include "Components/HorizontalBox.h"
#include "Components/HorizontalBoxSlot.h"
#include "Components/SizeBox.h"
#include "Components/TextBlock.h"
#include "Components/VerticalBox.h"
#include "Components/VerticalBoxSlot.h"
#include "Widgets/SLeafWidget.h"
#include "Styling/CoreStyle.h"
#include "Components/ButtonSlot.h"

// ===================== THE THUMBNAIL =====================

class STUPlanThumb : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(STUPlanThumb) {}
	SLATE_END_ARGS()
	void Construct(const FArguments&) {}

	std::vector<float> Plan;
	FLinearColor Colour = FLinearColor::White;
	float SizePx = 28.f;

	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(SizePx, SizePx); }

	virtual int32 OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&,
		FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const override
	{
		FSlateDrawElement::MakeBox(Out, Layer, G.ToPaintGeometry(),
			FCoreStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, FTUStyle::Border);
		// Y IS FLIPPED: a plan view grows up, a widget grows down. Mirrored it
		// still looks like a coaster, which is why it would ship.
		const float S = static_cast<float>(G.GetLocalSize().X);
		const float Inset = 3.f, Span = S - 2.f * Inset;
		for (std::size_t p = 0; p + 3 < Plan.size(); p += 2)
		{
			TArray<FVector2f> Pts;
			Pts.Add(FVector2f(Inset + Plan[p] * Span, Inset + (1.f - Plan[p + 1]) * Span));
			Pts.Add(FVector2f(Inset + Plan[p + 2] * Span, Inset + (1.f - Plan[p + 3]) * Span));
			FSlateDrawElement::MakeLines(Out, Layer + 1, G.ToPaintGeometry(), Pts,
				ESlateDrawEffect::None, Colour, true, 1.f);
		}
		return Layer + 1;
	}
};

TSharedRef<SWidget> UTUPlanThumb::RebuildWidget()
{
	TSharedRef<STUPlanThumb> W = SNew(STUPlanThumb);
	W->Plan = Plan;
	W->Colour = Colour;
	W->SizePx = SizePx;
	return W;
}

// ===================== THE ROWS =====================

void UTUMenuRow::OnClicked()
{
	if (ATUCoasterRide* R = Ride.Get()) { R->MenuAction(Action); }
}

TSharedRef<SWidget> UTUMenuWidget::RebuildWidget()
{
	if (!WidgetTree->RootWidget)
	{
		// Canvas -> border (the panel tile) -> vertical box of rows. Positioned
		// where the canvas menu was, so the backdrop orbit still shows beside it.
		UCanvasPanel* Root = WidgetTree->ConstructWidget<UCanvasPanel>();
		UBorder* Panel = WidgetTree->ConstructWidget<UBorder>();
		Panel->SetBrushColor(FTUStyle::Panel);
		Panel->SetPadding(FMargin(20.f, 12.f));
		Rows = WidgetTree->ConstructWidget<UVerticalBox>();
		Panel->AddChild(Rows);
		Root->AddChild(Panel);
		if (UCanvasPanelSlot* S = Cast<UCanvasPanelSlot>(Panel->Slot))
		{
			S->SetAutoSize(true);
			S->SetPosition(FVector2D(40.f, 50.f));
		}
		WidgetTree->RootWidget = Root;
	}
	return Super::RebuildWidget();
}

UTextBlock* UTUMenuWidget::MakeText(const FString& S, const FLinearColor& C, const char* Font)
{
	UTextBlock* T = WidgetTree->ConstructWidget<UTextBlock>();
	T->SetText(FText::FromString(S));
	T->SetColorAndOpacity(FSlateColor(C));
	T->SetFont(FTUStyle::Get().GetFontStyle(Font));
	return T;
}

UButton* UTUMenuWidget::AddRow(int32 Action, float PadY)
{
	UButton* B = WidgetTree->ConstructWidget<UButton>();
	// A FLAT BUTTON: the row is the control, and the hover tint is the only
	// thing that says so. Normal is transparent so the list reads as a list.
	FButtonStyle St = B->GetStyle();
	St.Normal.TintColor = FLinearColor::Transparent;
	St.Hovered.TintColor = FTUStyle::Border;
	St.Pressed.TintColor = FTUStyle::Border;
	B->SetStyle(St);
	if (UVerticalBoxSlot* S = Rows->AddChildToVerticalBox(B))
	{
		S->SetPadding(FMargin(0.f, PadY));
		S->SetHorizontalAlignment(HAlign_Fill);
	}
	UTUMenuRow* R = NewObject<UTUMenuRow>(this);
	R->Ride = Ride;
	R->Action = Action;
	Bindings.Add(R);
	B->OnClicked.AddDynamic(R, &UTUMenuRow::OnClicked);
	return B;
}

void UTUMenuWidget::Rebuild()
{
	ATUCoasterRide* R = Ride.Get();
	if (!R || !Rows) { return; }
	Rows->ClearChildren();
	Bindings.Reset();

	auto Heading = [&](const FString& S, float Top)
	{
		if (UVerticalBoxSlot* Sl = Rows->AddChildToVerticalBox(MakeText(S, FTUStyle::TextSecondary, "Font.Small")))
		{
			Sl->SetPadding(FMargin(0.f, Top, 0.f, 4.f));
		}
	};
	// A row with a picture: thumb, then name over subtitle.
	auto PictureRow = [&](int32 Action, const std::vector<float>& Plan, float Thumb,
		const FString& Name, const FString& Sub, const FLinearColor& Tint)
	{
		UButton* B = AddRow(Action, 1.f);
		UHorizontalBox* H = WidgetTree->ConstructWidget<UHorizontalBox>();
		UTUPlanThumb* T = WidgetTree->ConstructWidget<UTUPlanThumb>();
		T->Plan = Plan; T->Colour = Tint; T->SizePx = Thumb;
		if (UHorizontalBoxSlot* S = H->AddChildToHorizontalBox(T)) { S->SetPadding(FMargin(4.f, 2.f, 12.f, 2.f)); S->SetVerticalAlignment(VAlign_Center); }
		UVerticalBox* V = WidgetTree->ConstructWidget<UVerticalBox>();
		V->AddChildToVerticalBox(MakeText(Name, Tint == FTUStyle::LampOccupied ? Tint : FTUStyle::TextPrimary));
		UTextBlock* SubT = MakeText(Sub, FTUStyle::TextSecondary, "Font.Small");
		SubT->SetAutoWrapText(true);
		V->AddChildToVerticalBox(SubT);
		if (UHorizontalBoxSlot* S = H->AddChildToHorizontalBox(V))
		{
			S->SetVerticalAlignment(VAlign_Center);
			S->SetSize(FSlateChildSize(ESlateSizeRule::Fill));
		}
		USizeBox* Wide = WidgetTree->ConstructWidget<USizeBox>();
		Wide->SetWidthOverride(600.f);
		Wide->AddChild(H);
		B->AddChild(Wide);
		if (UButtonSlot* Sl = Cast<UButtonSlot>(Wide->Slot)) { Sl->SetHorizontalAlignment(HAlign_Left); }
	};
	auto TextRow = [&](int32 Action, const FString& S, const FLinearColor& C)
	{
		UButton* B = AddRow(Action, 0.f);
		UTextBlock* T = MakeText(S, C);
		T->SetMargin(FMargin(12.f, 2.f));
		B->AddChild(T);
		if (UButtonSlot* Sl = Cast<UButtonSlot>(T->Slot)) { Sl->SetHorizontalAlignment(HAlign_Left); }
	};

	UHorizontalBox* Title = WidgetTree->ConstructWidget<UHorizontalBox>();
	Title->AddChildToHorizontalBox(MakeText(TEXT("TRACKUNLIMITED"), FTUStyle::LampMeasured, "Font.Heading"));
	if (UHorizontalBoxSlot* S = Title->AddChildToHorizontalBox(MakeText(
		TEXT("free and open source  ·  github.com/zebadrabbit/TrackUnlimited"), FTUStyle::TextSecondary, "Font.Small")))
	{
		S->SetPadding(FMargin(24.f, 0.f, 0.f, 0.f)); S->SetVerticalAlignment(VAlign_Bottom);
	}
	Rows->AddChildToVerticalBox(Title);

	Heading(TEXT("START"), 12.f);
	for (std::size_t i = 0; i < NumTemplates(); ++i)
	{
		const FTemplate T = TemplateAt(i);
		const int32 Ix = static_cast<int32>(i);
		PictureRow(Ix, R->TemplatePlans.IsValidIndex(Ix) ? R->TemplatePlans[Ix] : std::vector<float>(),
			44.f, UTF8_TO_TCHAR(T.Name), UTF8_TO_TCHAR(T.Description), FTUStyle::LampMeasured);
	}

	Heading(TEXT("TRACKS"), 14.f);
	const std::vector<FTrackEntry> Tracks = FTrackBrowser::Rows(R->KnownTracks, R->TrackPaths);
	if (Tracks.empty())
	{
		Rows->AddChildToVerticalBox(MakeText(UTF8_TO_TCHAR(EmptyStateFor(EPanelKind::RecentTracks)), FTUStyle::TextSecondary));
	}
	for (std::size_t i = 0; i < Tracks.size(); ++i)
	{
		const FTrackEntry& E = Tracks[i];
		PictureRow(-1000 - static_cast<int32>(i), E.Plan, 28.f, UTF8_TO_TCHAR(E.Name.c_str()),
			UTF8_TO_TCHAR(FTrackBrowser::Subtitle(E).c_str()),
			E.IsUsable() ? FTUStyle::LampMeasured : FTUStyle::LampOccupied);
	}

	Heading(TEXT(""), 6.f);
#if WITH_EDITOR
	TextRow(-1, TEXT("Open a track file..."), FTUStyle::TextPrimary);
#else
	TextRow(-1, TEXT("Open the tracks folder...   (drop a .track in it and it appears here)"), FTUStyle::TextPrimary);
#endif
	TextRow(-8, TEXT("Settings"), FTUStyle::TextPrimary);
	const bool bDirty = R->Session.IsDirty();
	TextRow(-9, bDirty ? TEXT("Quit  -  there is unsaved work") : TEXT("Quit"),
		bDirty ? FTUStyle::LampOccupied : FTUStyle::TextPrimary);

	if (UVerticalBoxSlot* S = Rows->AddChildToVerticalBox(MakeText(
		TEXT("click to choose   ·   [Tab] once a track is open   ·   [M] back here"), FTUStyle::TextSecondary, "Font.Small")))
	{
		S->SetPadding(FMargin(0.f, 10.f, 0.f, 0.f));
	}
}
