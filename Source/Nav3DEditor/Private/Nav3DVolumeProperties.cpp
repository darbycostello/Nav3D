
#include "Nav3DVolumeProperties.h"
#include "Nav3DVolume.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"


#define LOCTEXT_NAMESPACE "Nav3DVolumeProperties"

TSharedRef<IDetailCustomization> FNav3DVolumeProperties::MakeInstance()
{
	return MakeShareable( new FNav3DVolumeProperties);
}

void FNav3DVolumeProperties::CustomizeDetails( IDetailLayoutBuilder& DetailBuilder )
{
	TSharedPtr<IPropertyHandle> PrimaryTickProperty = DetailBuilder.GetProperty(GET_MEMBER_NAME_CHECKED(UActorComponent, PrimaryComponentTick));

	if (PrimaryTickProperty->IsValidHandle() && DetailBuilder.HasClassDefaultObject())
	{
		IDetailCategoryBuilder& TickCategory = DetailBuilder.EditCategory("ComponentTick");
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, bStartWithTickEnabled)));
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, TickInterval)));
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, bTickEvenWhenPaused)), EPropertyLocation::Advanced);
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, bAllowTickOnDedicatedServer)), EPropertyLocation::Advanced);
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, TickGroup)), EPropertyLocation::Advanced);
	}

	PrimaryTickProperty->MarkHiddenByCustomization();
	DetailBuilder.HideCategory("BrushSettings");
	DetailBuilder.HideCategory("Navigation");
	DetailBuilder.HideCategory("Tags");
	DetailBuilder.HideCategory("Collision");
	DetailBuilder.HideCategory("HLOD");
	DetailBuilder.HideCategory("Mobile");
	DetailBuilder.HideCategory("Actor");

	const TArray<TWeakObjectPtr<UObject>> &SelectedObjects = DetailBuilder.GetSelectedObjects();
	for (int32 ObjectIndex = 0; ObjectIndex < SelectedObjects.Num(); ++ObjectIndex)
	{
		const TWeakObjectPtr<UObject>& CurrentObject = SelectedObjects[ObjectIndex];
		if (CurrentObject.IsValid())
		{
			ANav3DVolume* CurrentVolume = Cast<ANav3DVolume>(CurrentObject.Get());
			if (CurrentVolume != nullptr)
			{
				Volume = CurrentVolume;
				break;
			}
		}
	}

	DetailBuilder.EditCategory("Nav3D")
		.AddCustomRow(NSLOCTEXT("Nav3DVolume", "Build Octree", "Build Octree"))
		.ValueContent()
		.MaxDesiredWidth(125.f)
		.MinDesiredWidth(125.f)
		[
			SNew(SButton)
			.ContentPadding(2)
			.VAlign(VAlign_Center)
			.HAlign(HAlign_Center)
			.OnClicked(this, &FNav3DVolumeProperties::OnBuildOctree)
		[
			SNew(STextBlock)
			.Font(IDetailLayoutBuilder::GetDetailFont())
			.Text(NSLOCTEXT("Nav3DVolume", "Build Octree", "Build Octree"))
		]
		];

	DetailBuilder.EditCategory("Nav3D")
		.AddCustomRow(NSLOCTEXT("Nav3DVolume", "Clear Octree", "Clear Octree"))
		.ValueContent()
		.MaxDesiredWidth(125.f)
		.MinDesiredWidth(125.f)
		[
			SNew(SButton)
			.ContentPadding(2)
			.VAlign(VAlign_Center)
			.HAlign(HAlign_Center)
			.OnClicked(this, &FNav3DVolumeProperties::OnClearOctree)
		[
			SNew(STextBlock)
			.Font(IDetailLayoutBuilder::GetDetailFont())
			.Text(NSLOCTEXT("Nav3DVolume", "Clear Octree", "Clear Octree"))
		]
		];
}

FReply FNav3DVolumeProperties::OnBuildOctree() const {
	if (Volume.IsValid())
	{		
		Volume->BuildOctree();
	}
	return FReply::Handled();
}

FReply FNav3DVolumeProperties::OnClearOctree() const {
	if (Volume.IsValid())
	{
		Volume->Initialise();
	}
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
