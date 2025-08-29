#include "Nav3DDataDetailCustomization.h"
#include "Nav3DDataGenerator.h"
#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"
#include "DetailWidgetRow.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Nav3D/Public/Nav3DData.h"
#include "PropertyHandle.h"

TSharedRef<IDetailCustomization> FNav3DDataDetailCustomization::MakeInstance()
{
    return MakeShareable(new FNav3DDataDetailCustomization);
}

void FNav3DDataDetailCustomization::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    // Find the Nav3DData being customized
    TArray<TWeakObjectPtr<>> Objects;
    DetailBuilder.GetObjectsBeingCustomized(Objects);

    for (TWeakObjectPtr Object : Objects)
    {
        if (ANav3DData* Nav3DData = Cast<ANav3DData>(Object.Get()))
        {
            Nav3DDataPtr = Nav3DData;
            break;
        }
    }

    if (!Nav3DDataPtr.IsValid())
    {
        return;
    }

    // Get the Nav3D category
    IDetailCategoryBuilder& Nav3DCategory = DetailBuilder.EditCategory("Nav3D");
    
    // Check if a build is in progress
    bool bIsBuildInProgress = false;
    if (FNavDataGenerator* Generator = Nav3DDataPtr->GetGenerator())
    {
        bIsBuildInProgress = Generator->IsBuildInProgressCheckDirty();
    }
    
    if (bIsBuildInProgress)
    {
        // Add a simple notification instead of the full panel
        Nav3DCategory.AddCustomRow(FText::FromString("BuildInProgress"))
            .WholeRowContent()
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(5)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    .Padding(0, 0, 5, 0)
                    [
                        SNew(SImage)
                        .Image(FAppStyle::GetBrush("Icons.Warning"))
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Navigation data is currently building..."))
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                ]
            ];
    }
    else
    {
        // Generate the voxel info panel only when no build is in progress
        GenerateVoxelInfoPanel(DetailBuilder, Nav3DCategory);
    }
}

void FNav3DDataDetailCustomization::GenerateVoxelInfoPanel(IDetailLayoutBuilder& DetailBuilder, IDetailCategoryBuilder& CategoryBuilder) const
{
    if (!Nav3DDataPtr.IsValid())
    {
        return;
    }

    // Add a custom row for voxel information
    CategoryBuilder.AddCustomRow(FText::FromString("VoxelInfo"))
        .WholeRowContent()
        [
            SNew(SVerticalBox)
            // Header
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(0, 5, 0, 5)
            [
                SNew(STextBlock)
                .Text(FText::FromString("Nav3D Volume Data"))
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
            ]
            // Content Box
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(5, 0, 0, 0) // Left padding for indentation
            [
                SNew(SVerticalBox)
                
                // Voxel Extent
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 2)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Voxel Extent: "))
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text_Lambda([this]() 
                        {
                            if (Nav3DDataPtr.IsValid())
                            {
                                return FText::AsNumber(Nav3DDataPtr->GetVoxelExtent());
                            }
                            return FText::FromString("N/A");
                        })
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                ]
                
                // Required Layers
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 2)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Required Layers: "))
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text_Lambda([this]() 
                        {
                            if (Nav3DDataPtr.IsValid())
                            {
                                return FText::AsNumber(Nav3DDataPtr->GetLayerCount());
                            }
                            return FText::FromString("N/A");
                        })
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                ]
                
                // Volume Size
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 2)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Volume Size: "))
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text_Lambda([this]() 
                        {
                            if (Nav3DDataPtr.IsValid())
                            {
                                FBox Bounds = Nav3DDataPtr->GetBoundingBox();
                                if (Bounds.IsValid)
                                {
                                    FVector Extent = Bounds.GetExtent();
                                    return FText::Format(FText::FromString("{0}"),
                                        FText::AsNumber(Extent.X * 2.0f));
                                }
                            }
                            return FText::FromString("N/A");
                        })
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                ]
                
                // Occluded Voxels
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 2)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Occluded Voxels: "))
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(STextBlock)
                        .Text_Lambda([this]() 
                        {
                            if (Nav3DDataPtr.IsValid())
                            {
                                int32 TotalOccluded = 0;
                                const TArray<FNav3DVolumeNavigationData>& VolumeData = Nav3DDataPtr->GetVolumeNavigationData();
                                for (const FNav3DVolumeNavigationData& Volume : VolumeData)
                                {
                                    TotalOccluded += Volume.GetData().GetTotalOccludedLeafNodes();
                                }
                                return FText::AsNumber(TotalOccluded);
                            }
                            return FText::FromString("N/A");
                        })
                        .Font(DetailBuilder.GetDetailFont())
                    ]
                ]

                // Divider at the bottom
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 10, 0, 10)  // Add some padding above and below the divider
                [
                    SNew(SSeparator)
                    .Thickness(1.0f)
                    .ColorAndOpacity(FSlateColor(FColor(128, 128, 128)))
                    .SeparatorImage(FAppStyle::Get().GetBrush("Menu.Separator"))
                ]   
            ]
        ];
} 