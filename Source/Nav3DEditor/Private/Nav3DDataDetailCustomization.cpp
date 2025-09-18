#include "Nav3DDataDetailCustomization.h"
#include "Nav3DDataGenerator.h"
#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"
#include "DetailWidgetRow.h"
#include "IPropertyUtilities.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Nav3D/Public/Nav3DData.h"
#include "Nav3D/Public/Nav3DUtils.h"
#include "PropertyHandle.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DEditor.h"

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

    // Get the Nav3D category. Force refresh when chunk revision changes.
    IDetailCategoryBuilder& Nav3DCategory = DetailBuilder.EditCategory("Nav3D");

    // Listen for property changes that indicate chunk list changed
    DetailBuilder.GetPropertyUtilities()->ForceRefresh();
    
    // Check if a build is in progress
    bool bIsBuildInProgress = false;
    if (const FNavDataGenerator* Generator = Nav3DDataPtr->GetGenerator())
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
        // If chunks changed, refresh the panel
        DetailBuilder.ForceRefreshDetails();
        // Generate the voxel info panel only when no build is in progress
        GenerateVoxelInfoPanel(DetailBuilder, Nav3DCategory);

        // Add Volumes management panel
        Nav3DCategory.AddCustomRow(FText::FromString("VolumesAndChunks"))
        .WholeRowContent()
        [
            SNew(SVerticalBox)
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(0, 10, 0, 5)
            [
                SNew(STextBlock)
                .Text(FText::FromString("Volumes"))
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
            ]
            + SVerticalBox::Slot()
            .AutoHeight()
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(SSeparator)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(SVerticalBox)
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(0, 5)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Discovered Volumes"))
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 10))
                    ]
                ]
            ]
        ];

        // For each volume, add a sub-panel with a Rebuild Volume button and its chunks
        const TArray<FBox> Volumes = Nav3DDataPtr->GetAllDiscoverableVolumes();
        for (int32 VolumeIdx = 0; VolumeIdx < Volumes.Num(); ++VolumeIdx)
        {
            const FBox& VolumeBounds = Volumes[VolumeIdx];
            FText VolumeLabel = FText::FromString(FString::Printf(TEXT("Volume %d  (%s)"), VolumeIdx,
                *Volumes[VolumeIdx].GetCenter().ToString()));

            Nav3DCategory.AddCustomRow(FText::FromString("VolumeRow"))
            .WholeRowContent()
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0,5)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    .Padding(0,0,10,0)
                    [
                        SNew(STextBlock)
                        .Text(VolumeLabel)
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(SButton)
                        .Text(FText::FromString("Rebuild Nav"))
                        .OnClicked_Lambda([Nav3D = Nav3DDataPtr.Get(), VolumeBounds]() -> FReply
                        {
                            if (Nav3D)
                            {
                                Nav3D->BuildSingleVolume(VolumeBounds);
                            }
                            return FReply::Handled();
                        })
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    .Padding(5,0,0,0)
                    [
                        SNew(SButton)
                        .Text(FText::FromString("Rebuild Tactical"))
                        .IsEnabled_Lambda([Nav3D = Nav3DDataPtr.Get()]()
                        {
                            return Nav3D && Nav3D->TacticalSettings.bEnableTacticalReasoning;
                        })
                        .OnClicked_Lambda([Nav3D = Nav3DDataPtr.Get(), VolumeBounds]() -> FReply
                        {
                            if (Nav3D && Nav3D->TacticalSettings.bEnableTacticalReasoning)
                            {
                                // Gather chunks in this volume and clear tactical data
                                const TArray<ANav3DDataChunkActor*> AllChunks = Nav3D->GetAllChunkActors();
                                TArray<ANav3DDataChunkActor*> VolumeChunks;
                                for (ANav3DDataChunkActor* Chunk : AllChunks)
                                {
                                    if (Chunk && VolumeBounds.IsInside(Chunk->DataChunkActorBounds.GetCenter()))
                                    {
                                        Chunk->ClearTacticalData();
                                        VolumeChunks.Add(Chunk);
                                    }
                                }

                                // Kick per-volume tactical rebuild
                                Nav3D->RebuildTacticalDataForVolume(VolumeChunks, VolumeBounds);
                            }
                            return FReply::Handled();
                        })
                    ]
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(SVerticalBox)
                    // List chunks inside this volume
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    [
                        SNew(SVerticalBox)
                        + SVerticalBox::Slot()
                        .AutoHeight()
                        .Padding(10,2)
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString("Chunks"))
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                        ]
                    ]
                ]
            ];

            // Build a list of chunks belonging to this volume
            TArray<ANav3DDataChunkActor*> Chunks = Nav3DDataPtr->GetChunkActors();
            int32 GlobalIndex = 0;
            for (int32 ChunkIdx = 0; ChunkIdx < Chunks.Num(); ++ChunkIdx, ++GlobalIndex)
            {
                ANav3DDataChunkActor* ChunkActor = Chunks[ChunkIdx];
                if (!ChunkActor) { continue; }
                const FVector Center = ChunkActor->DataChunkActorBounds.GetCenter();
                if (!VolumeBounds.IsInside(Center)) { continue; }

                const FLinearColor Color = FNav3DUtils::GetChunkColorByIndex(ChunkIdx);
                
                // === FIXED: Calculate chunk size using compact data ===
                float ChunkSizeMB;
                if (ChunkActor->HasCompactTacticalData())
                {
                    // Use compact tactical data for size estimation
                    const FCompactTacticalData& CompactData = ChunkActor->CompactTacticalData;
                    const int32 RegionCount = CompactData.Regions.Num();
                    const int32 AdjacencyCount = CompactData.RegionAdjacency.Num();
                    const int32 InterfaceCount = ChunkActor->ConnectionInterfaces.Num();
                    
                    // Estimate bytes for compact format
                    const float EstimatedBytes = (RegionCount * sizeof(FCompactRegion)) + 
                                          (AdjacencyCount * sizeof(uint64)) + // Bitmask adjacency
                                          (InterfaceCount * sizeof(FChunkConnectionInterface)) +
                                          CompactData.VisibilityMatrix.SparseReferences.Num() * sizeof(uint64);
                    ChunkSizeMB = EstimatedBytes / (1024.0f * 1024.0f);
                }
                else
                {
                    // Fallback for chunks without tactical data
                    ChunkSizeMB = 0.1f; // Minimal size for nav-only chunks
                }
                
                FText ChunkLabel = FText::FromString(FString::Printf(TEXT("%d  [%.1fMB]"), ChunkIdx, ChunkSizeMB));

                Nav3DCategory.AddCustomRow(FText::FromString("ChunkRow"))
                .WholeRowContent()
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    .Padding(20,0,8,0)
                    [
                        SNew(SBox)
                        .WidthOverride(12)
                        .HeightOverride(12)
                        [
                            SNew(SBorder)
                            .BorderBackgroundColor(Color)
                            .BorderImage(FAppStyle::GetBrush("WhiteBrush"))
                        ]
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    .Padding(0,0,8,0)
                    [
                        SNew(STextBlock)
                        .Text(ChunkLabel)
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        SNew(SButton)
                        .Text(FText::FromString("Rebuild Chunk"))
                        .OnClicked_Lambda([Nav3D = Nav3DDataPtr.Get(), ChunkActor]() -> FReply
                        {
                            if (Nav3D && ChunkActor)
                            {
                                const FBox Bounds = ChunkActor->DataChunkActorBounds;
                                if (UWorld* World = ChunkActor->GetWorld())
                                {
                                    // Destroy the existing chunk actor first (auto-unregisters)
                                    World->DestroyActor(ChunkActor);
                                }
                                // Then rebuild only this chunk
                                Nav3D->RebuildSingleChunk(Bounds);
                            }
                            return FReply::Handled();
                        })
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    .Padding(5,0,0,0)
                    [
                        SNew(SButton)
                        .Text(FText::FromString("Unload Chunk"))
                        .OnClicked_Lambda([Nav3D = Nav3DDataPtr.Get(), ChunkActor]() -> FReply
                        {
                            if (Nav3D && ChunkActor)
                            {
                                Nav3D->OnChunkActorUnloaded(ChunkActor);
                                if (UWorld* World = ChunkActor->GetWorld())
                                {
                                    World->DestroyActor(ChunkActor);
                                }
                            }
                            return FReply::Handled();
                        })
                    ]
                ];
            }
        }
        
        // Add consolidated tactical status panel
        AddConsolidatedTacticalStatusPanel(DetailBuilder, Nav3DCategory);
    }
}

void FNav3DDataDetailCustomization::AddConsolidatedTacticalStatusPanel(IDetailLayoutBuilder& DetailBuilder, IDetailCategoryBuilder& CategoryBuilder) const
{
    if (!Nav3DDataPtr.IsValid())
    {
        return;
    }

    CategoryBuilder.AddCustomRow(FText::FromString("TacticalStatus"))
        .WholeRowContent()
        [
            SNew(SVerticalBox)
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(0, 5, 0, 2)
            [
                SNew(STextBlock)
                .Text(FText::FromString("Tactical Status"))
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
            ]
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(10, 2)
            [
                SNew(SVerticalBox)
                // === COMPACT DATA STATS (PRIMARY) ===
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 2, 0, 2)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString("Compact Tactical Data (Runtime)"))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text_Lambda([this]() {
                        if (Nav3DDataPtr.IsValid())
                        {
                            const int32 CompactRegionCount = Nav3DDataPtr->ConsolidatedCompactTacticalData.GetRegionCount();
                            const int32 ChunkCount = Nav3DDataPtr->ConsolidatedCompactTacticalData.SourceChunks.Num();
                            
                            if (CompactRegionCount > 0)
                            {
                                return FText::FromString(FString::Printf(TEXT("Regions: %d (compact) | Source Chunks: %d (loaded)"), 
                                                                        CompactRegionCount, ChunkCount));
                            }
                            else
                            {
                                return FText::FromString(TEXT("No compact tactical data - rebuild required"));
                            }
                        }
                        return FText::FromString(TEXT("No compact tactical data"));
                    })
                ]
                // Compact adjacency stats
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 2, 0, 0)
                [
                    SNew(STextBlock)
                    .Text_Lambda([this]() {
                        if (Nav3DDataPtr.IsValid())
                        {
                            const int32 CompactAdjacencyCount = Nav3DDataPtr->ConsolidatedCompactTacticalData.GlobalRegionAdjacency.Num();
                            int32 TotalCompactConnections = 0;
                            for (const auto& AdjPair : Nav3DDataPtr->ConsolidatedCompactTacticalData.GlobalRegionAdjacency)
                            {
                                TotalCompactConnections += FVolumeRegionMatrix::CountBits(AdjPair.Value);
                            }
                            return FText::FromString(FString::Printf(TEXT("Adjacency: %d regions, %d connections (bitmask)"), 
                                                                    CompactAdjacencyCount, TotalCompactConnections));
                        }
                        return FText::FromString(TEXT("No compact adjacency data"));
                    })
                ]
                // Compact visibility stats
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 2, 0, 0)
                [
                    SNew(STextBlock)
                    .Text_Lambda([this]() {
                        if (Nav3DDataPtr.IsValid())
                        {
                            const int32 CompactVisibilityVolumeCount = Nav3DDataPtr->ConsolidatedCompactTacticalData.VolumeVisibilityData.Num();
                            int32 TotalCompactVisibilityPairs = 0;
                            for (const auto& VolumePair : Nav3DDataPtr->ConsolidatedCompactTacticalData.VolumeVisibilityData)
                            {
                                // Count sparse matrix entries
                                TotalCompactVisibilityPairs += VolumePair.Value.SparseReferences.Num();
                            }
                            return FText::FromString(FString::Printf(TEXT("Visibility: %d volumes, %d sparse entries"), 
                                                                    CompactVisibilityVolumeCount, TotalCompactVisibilityPairs));
                        }
                        return FText::FromString(TEXT("No compact visibility data"));
                    })
                ]
                
                // === PERFORMANCE METRICS ===
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 8, 0, 2)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString("Performance"))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text_Lambda([this]() {
                        if (Nav3DDataPtr.IsValid())
                        {
                            const auto& PerfStats = Nav3DDataPtr->PerformanceStats;
                            return FText::FromString(FString::Printf(TEXT("Memory: %.2f MB | Update Time: %.2fs"), 
                                                                    PerfStats.EstimatedMemoryUsage / (1024.0f * 1024.0f),
                                                                    PerfStats.LastUpdateTime));
                        }
                        return FText::FromString(TEXT("No performance data"));
                    })
                ]
                
                // === CHUNK-LEVEL STATS ===
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 8, 0, 2)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString("Chunk Statistics"))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text_Lambda([this]() {
                        if (Nav3DDataPtr.IsValid())
                        {
                            const int32 TotalChunks = Nav3DDataPtr->GetChunkCount();
                            int32 ChunksWithCompactData = 0;
                            int32 TotalCompactRegions = 0;
                            
                            for (const ANav3DDataChunkActor* ChunkActor : Nav3DDataPtr->GetChunkActors())
                            {
                                if (ChunkActor && ChunkActor->HasCompactTacticalData())
                                {
                                    ChunksWithCompactData++;
                                    TotalCompactRegions += ChunkActor->CompactTacticalData.Regions.Num();
                                }
                            }
                            
                            return FText::FromString(FString::Printf(TEXT("Chunks: %d total | %d with compact tactical data | %d total compact regions"), 
                                                                    TotalChunks, ChunksWithCompactData, TotalCompactRegions));
                        }
                        return FText::FromString(TEXT("No chunk data"));
                    })
                ]
                
                // === DATA STATUS ===
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0, 8, 0, 2)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString("System Status"))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text_Lambda([this]() {
                        if (Nav3DDataPtr.IsValid())
                        {
                            const bool bTacticalEnabled = Nav3DDataPtr->TacticalSettings.bEnableTacticalReasoning;
                            const bool bHasCompactData = !Nav3DDataPtr->ConsolidatedCompactTacticalData.IsEmpty();
                            
                            FString Status;
                            if (!bTacticalEnabled)
                            {
                                Status = TEXT("Tactical reasoning disabled");
                            }
                            else if (!bHasCompactData)
                            {
                                Status = TEXT("No compact tactical data - rebuild required");
                            }
                            else
                            {
                                Status = TEXT("Compact tactical system ready");
                            }
                            
                            return FText::FromString(Status);
                        }
                        return FText::FromString(TEXT("Unknown status"));
                    })
                ]
            ]
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(10, 5, 0, 0)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot()
                .AutoWidth()
                [
                    SNew(SButton)
                    .Text(FText::FromString("Rebuild Compact Data"))
                    .OnClicked_Lambda([Nav3D = Nav3DDataPtr.Get()]() -> FReply
                    {
                        if (Nav3D && Nav3D->TacticalSettings.bEnableTacticalReasoning)
                        {
                            Nav3D->RebuildConsolidatedCompactTacticalData();
                            Nav3D->RequestDrawingUpdate();
                        }
                        return FReply::Handled();
                    })
                ]
                + SHorizontalBox::Slot()
                .AutoWidth()
                .Padding(5, 0, 0, 0)
                [
                    SNew(SButton)
                    .Text(FText::FromString("Validate Compact Data"))
                    .OnClicked_Lambda([Nav3D = Nav3DDataPtr.Get()]() -> FReply
                    {
                        if (Nav3D && Nav3D->TacticalSettings.bEnableTacticalReasoning)
                        {
                            // Validate compact data integrity
                            const bool bValid = !Nav3D->ConsolidatedCompactTacticalData.IsEmpty() &&
                                               Nav3D->ConsolidatedCompactTacticalData.GetRegionCount() > 0;
                            
                            if (bValid)
                            {
                                UE_LOG(LogNav3DEditor, Display, TEXT("Compact tactical data validation passed"));
                            }
                            else
                            {
                                UE_LOG(LogNav3DEditor, Error, TEXT("Compact tactical data validation failed - no valid data found"));
                            }
                        }
                        return FReply::Handled();
                    })
                ]
                + SHorizontalBox::Slot()
                .AutoWidth()
                .Padding(5, 0, 0, 0)
                [
                    SNew(SButton)
                    .Text(FText::FromString("Update Performance Stats"))
                    .OnClicked_Lambda([Nav3D = Nav3DDataPtr.Get()]() -> FReply
                    {
                        if (Nav3D && Nav3D->TacticalSettings.bEnableTacticalReasoning)
                        {
                            Nav3D->UpdatePerformanceStats();
                        }
                        return FReply::Handled();
                    })
                ]
            ]
        ];
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
                                const FBox Bounds = Nav3DDataPtr->GetBoundingBox();
                                if (Bounds.IsValid)
                                {
                                    const FVector Extent = Bounds.GetExtent();
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
                                for (ANav3DDataChunkActor* ChunkActor : Nav3DDataPtr->GetChunkActors())
                                {
                                    if (ChunkActor)
                                    {
                                        for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
                                        {
                                            if (Chunk)
                                            {
                                                if (const FNav3DVolumeNavigationData* VolumeData = Chunk->GetVolumeNavigationData())
                                                {
                                                    TotalOccluded += VolumeData->GetData().GetTotalOccludedLeafNodes();
                                                }
                                            }
                                        }
                                    }
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
