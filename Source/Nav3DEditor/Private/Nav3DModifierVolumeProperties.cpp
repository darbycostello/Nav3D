
#include "Nav3DModifierVolumeProperties.h"
#include "Nav3DModifierVolume.h"
#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"


#define LOCTEXT_NAMESPACE "Nav3DModifierVolumeProperties"

TSharedRef<IDetailCustomization> FNav3DModifierVolumeProperties::MakeInstance()
{
	return MakeShareable( new FNav3DModifierVolumeProperties);
}

void FNav3DModifierVolumeProperties::CustomizeDetails( IDetailLayoutBuilder& DetailBuilder )
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
			ANav3DModifierVolume* CurrentVolume = Cast<ANav3DModifierVolume>(CurrentObject.Get());
			if (CurrentVolume != nullptr)
			{
				Volume = CurrentVolume;
				break;
			}
		}
	}
}

#undef LOCTEXT_NAMESPACE
