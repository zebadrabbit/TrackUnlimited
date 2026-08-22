#include "TUPaintedPanelWidget.h"

#include "TUStyle.h"
#include "../TUCoasterRide.h"

#include "Styling/CoreStyle.h"
#include "Widgets/SLeafWidget.h"

class STUPaintedPanel : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(STUPaintedPanel) {}
	SLATE_END_ARGS()
	void Construct(const FArguments&) {}

	TWeakObjectPtr<ATUCoasterRide> Ride;

	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(0, 0); }

	virtual int32 OnPaint(const FPaintArgs&, const FGeometry& G, const FSlateRect&,
		FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle&, bool) const override
	{
		ATUCoasterRide* R = Ride.Get();
		if (!R) { return Layer; }

		// RECORDED NOW, every paint: the list is the live read.
		TArray<FTUPanelCmd> Cmds;
		R->RecordPaintedPanels(Cmds, static_cast<float>(G.GetLocalSize().Y),
			G.GetAccumulatedLayoutTransform().GetScale());

		const FSlateFontInfo Body = FTUStyle::Get().GetFontStyle("Font.Small");   // the canvas medium font is this size
		FSlateFontInfo Big = Body;
		Big.Size = FMath::RoundToInt(Body.Size * 1.6f);
		const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");

		// ONE LAYER PER COMMAND, because draw order IS z-order here exactly as
		// it was on the canvas, and Slate batches within a layer by element type.
		for (const FTUPanelCmd& C : Cmds)
		{
			++Layer;
			switch (C.Kind)
			{
			case FTUPanelCmd::Tile:
				FSlateDrawElement::MakeBox(Out, Layer,
					G.ToPaintGeometry(FVector2f(C.B), FSlateLayoutTransform(FVector2f(C.A))),
					White, ESlateDrawEffect::None, C.Col);
				break;
			case FTUPanelCmd::Label:
			case FTUPanelCmd::BigLabel:
				FSlateDrawElement::MakeText(Out, Layer,
					G.ToPaintGeometry(FVector2f(1.f, 1.f), FSlateLayoutTransform(FVector2f(C.A))),
					C.Text, C.Kind == FTUPanelCmd::BigLabel ? Big : Body,
					ESlateDrawEffect::None, C.Col);
				break;
			case FTUPanelCmd::Line:
			{
				TArray<FVector2f> Pts;
				Pts.Add(FVector2f(C.A));
				Pts.Add(FVector2f(C.B));
				FSlateDrawElement::MakeLines(Out, Layer, G.ToPaintGeometry(), Pts,
					ESlateDrawEffect::None, C.Col, true, C.Thickness);
				break;
			}
			}
		}
		return Layer;
	}
};

TSharedRef<SWidget> UTUPaintedPanelWidget::RebuildWidget()
{
	TSharedRef<STUPaintedPanel> W = SNew(STUPaintedPanel);
	W->Ride = Ride;
	W->SetVisibility(EVisibility::HitTestInvisible);
	return W;
}
