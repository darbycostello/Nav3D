#include "Nav3DNavDataRenderingComponent.h"
#include "Nav3DData.h"
#include "Nav3DUtils.h"
#include <Debug/DebugDrawService.h>
#include <Engine/CollisionProfile.h>
#include <Materials/Material.h>
#include <Engine/Engine.h>
#include <PrimitiveSceneProxy.h>
#include "Nav3D.h"
#include "Materials/MaterialRenderProxy.h"
#include "Nav3DDataChunkActor.h"
#include "Tactical/Nav3DTacticalDataConverter.h"

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
#endif

#if WITH_EDITOR
#include <Editor.h>
#include <EditorViewportClient.h>
#endif

static constexpr FColor OccludedVoxelColor = FColor(128, 0, 0);
static constexpr FColor FreeVoxelColor = FColor(0, 128, 0);

// Performance threshold for automatic wireframe rendering (8^6 = 262,144 voxels)
static constexpr int32 MaxVoxelsForSolidRendering = 262144;

// Colors for tactical reasoning visualization
static const FColor RegionColors[32] = {
    FColor::Red, FColor::Green, FColor::Blue, FColor::Yellow, 
    FColor::Cyan, FColor::Magenta, FColor::Orange, FColor::Purple,
    FColor(255, 128, 0), FColor(0, 255, 128), FColor(128, 0, 255), FColor(255, 255, 128),
    FColor(255, 128, 255), FColor(128, 255, 255), FColor(192, 0, 64), FColor(0, 192, 64),
    FColor(64, 0, 192), FColor(192, 192, 0), FColor(0, 192, 192), FColor(192, 0, 192),
    FColor(128, 64, 0), FColor(0, 128, 64), FColor(64, 0, 128), FColor(128, 128, 0),
    FColor(0, 128, 128), FColor(128, 0, 128), FColor(255, 128, 128), FColor(128, 255, 128),
    FColor(128, 128, 255), FColor(192, 192, 64), FColor(64, 192, 192), FColor(192, 64, 192)
};

void FNav3DMeshSceneProxy::EnsureConsolidatedDataForDebugDrawing() const
{
	if (!NavigationData.IsValid())
	{
		return;
	}

#if WITH_EDITOR || !UE_BUILD_SHIPPING
	const bool bHasCompactData = !NavigationData->ConsolidatedCompactTacticalData.IsEmpty();

	if (const bool bHasConsolidatedData = !NavigationData->GetConsolidatedTacticalData().IsEmpty();
		NavigationData->TacticalSettings.bEnableTacticalReasoning && bHasCompactData && !bHasConsolidatedData)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Converting compact tactical data to consolidated format for superior debug rendering"));
		NavigationData->RebuildConsolidatedTacticalDataFromCompact();
		const int32 ConvertedRegions = NavigationData->GetConsolidatedTacticalData().AllLoadedRegions.Num();
		const int32 ConvertedAdjacency = NavigationData->GetConsolidatedTacticalData().RegionAdjacency.Num();
		const int32 ConvertedVisibility = NavigationData->GetConsolidatedTacticalData().RegionVisibility.Num();
		if (ConvertedRegions > 0)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("Tactical data conversion successful - %d regions, %d adjacency entries, %d visibility entries available for original debug rendering"), ConvertedRegions, ConvertedAdjacency, ConvertedVisibility);
		}
		else
		{
			UE_LOG(LogNav3D, Warning, TEXT("Tactical data conversion produced no regions - debug rendering may not work"));
		}
	}
#endif
}

FNav3DMeshSceneProxy::FNav3DMeshSceneProxy(
	const UPrimitiveComponent& Component,
	const FNav3DMeshSceneProxyData& ProxyData)
	: FDebugRenderSceneProxy(&Component)
{
	// Determine rendering mode based on voxel count (estimate from volumes + voxel extent)
	int32 TotalVoxelCount = 0;
	if (ProxyData.NavigationData != nullptr)
	{
		const ANav3DData* NavData = ProxyData.NavigationData.Get();
		const TArray<FBox> Volumes = NavData->GetAllDiscoverableVolumes();
		const auto& DebugInfos = NavData->GetDebugData();
		const int32 LayerCount = NavData->GetLayerCount();
		if (LayerCount > 0 && Volumes.Num() > 0)
		{
			const int32 SelectedLayer = DebugInfos.bDebugDrawLayers
				? FMath::Clamp(static_cast<int32>(DebugInfos.LayerIndexToDraw), 0, LayerCount - 1)
				: 0; // default to leaf estimate if no specific layer requested
			const float LeafNodeSize = NavData->GetVoxelExtent() * 4.0f; // matches generation
			const float NodeSize = LeafNodeSize * static_cast<float>(1 << SelectedLayer);
			for (const FBox& B : Volumes)
			{
				if (!B.IsValid) { continue; }
				const FVector Size = B.GetSize();
				const int32 Nx = FMath::Max(1, FMath::CeilToInt(Size.X / NodeSize));
				const int32 Ny = FMath::Max(1, FMath::CeilToInt(Size.Y / NodeSize));
				const int32 Nz = FMath::Max(1, FMath::CeilToInt(Size.Z / NodeSize));
				TotalVoxelCount += Nx * Ny * Nz;
			}
		}
	}
	
	// Default to wireframe until we have a reliable voxel count; wireframe is safer for large scenes
	const bool bUnknownVoxelCount = (TotalVoxelCount <= 0);
	DrawType = (bUnknownVoxelCount || TotalVoxelCount > MaxVoxelsForSolidRendering) ? WireMesh : SolidAndWireMeshes;
	
	// Log when wireframe mode is used (either unknown or too large)
	if (bUnknownVoxelCount)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Nav3D Rendering: Using wireframe by default (voxel count unknown)"));
	}
	else if (TotalVoxelCount > MaxVoxelsForSolidRendering)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("Nav3D Rendering: Using wireframe mode for performance (voxel count: %d > %d)"), 
			TotalVoxelCount, MaxVoxelsForSolidRendering);
	}
	
	TextWithoutShadowDistance = 1500;
	bWantsSelectionOutline = false;
	ViewFlagName = TEXT("Navigation");
	ViewFlagIndex = static_cast<uint32>(FEngineShowFlags::FindIndexByName(*ViewFlagName));

	RenderingComponent = ProxyData.RenderingComponent;
	NavigationData = ProxyData.NavigationData;

	if (NavigationData == nullptr)
	{
		return;
	}

	// Ensure we have consolidated data for debug paths
	EnsureConsolidatedDataForDebugDrawing();

	const auto& DebugInfos = NavigationData->GetDebugData();

	// Reserve for filled voxel surfaces to reduce reallocations
	VoxelSurfaces.Reserve(1000);

	if (DebugInfos.bDebugDrawVolumes)
	{
		AddVolumeTextInfos();
	}

	for (ANav3DDataChunkActor* ChunkActor : NavigationData->GetChunkActors())
	{
		if (!ChunkActor) continue;
		
		// Skip rendering if this chunk is still building to prevent access violations
		if (ChunkActor->bIsBuilding)
		{
			continue;
		}
		
		for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
		{
			if (!Chunk) continue;
			
			const FNav3DVolumeNavigationData* NavigationBoundsData = Chunk->GetVolumeNavigationData();
			if (!NavigationBoundsData) continue;
			
			const auto& OctreeData = NavigationBoundsData->GetData();
			const auto LayerCount = OctreeData.GetLayerCount();

			if (LayerCount == 0)
			{
				continue;
			}

			if (DebugInfos.bDebugDrawBounds)
			{
				Boxes.Emplace(NavigationBoundsData->GetData().GetNavigationBounds(), FColor::White);
			}

			if (DebugInfos.bDebugDrawLayers)
			{
				const auto CorrectedLayerIndex = FMath::Clamp(
					static_cast<int>(DebugInfos.LayerIndexToDraw), 0, LayerCount - 1);
				const auto NodeExtent = NavigationBoundsData->GetData()
				                                            .GetLayer(CorrectedLayerIndex)
				                                            .GetNodeExtent();

				for (const auto& Node : OctreeData.GetLayer(CorrectedLayerIndex).GetNodes())
				{
					const auto Code = Node.MortonCode;

					if (CorrectedLayerIndex == 0)
					{
						if (const auto LeafNodePosition =
							NavigationBoundsData->GetLeafNodePositionFromMortonCode(Code);
							AddVoxelToBoxes(LeafNodePosition, NodeExtent, Node.HasChildren()))
						{
							AddNodeTextInfos(Code, 0, LeafNodePosition);
						}
					}
					else
					{
						const auto Position =
							NavigationBoundsData->GetNodePositionFromLayerAndMortonCode(CorrectedLayerIndex, Code);

						if (AddVoxelToBoxes(Position, NodeExtent, Node.HasChildren()))
						{
							AddNodeTextInfos(Code, CorrectedLayerIndex, Position);
						}
					}
				}
			}
	        
	        // Tactical reasoning visualization
	        if (NavigationData->TacticalSettings.bEnableTacticalReasoning)
	        {
	            const auto& TacticalDebugData = NavigationData->TacticalSettings.TacticalDebugData;
	            
	            // Always ensure we have consolidated data for the superior debug rendering methods
	            EnsureConsolidatedDataForDebugDrawing();
	            
	            // Now we can always use the original, high-quality debug drawing methods
	            if (!NavigationData->GetConsolidatedTacticalData().IsEmpty())
	            {
	                // Draw regions if enabled
	                if (TacticalDebugData.bDebugDrawRegions)
	                {
	                    DebugDrawRegions();
	                }
	                
	                if (TacticalDebugData.bDebugDrawRegionIds)
	                {
	                    DebugDrawRegionIds();
	                }
	                
	                if (TacticalDebugData.bDebugDrawVisibility && TacticalDebugData.VisibilityViewRegionId >= 0)
	                {
	                    DebugDrawVisibility(TacticalDebugData.VisibilityViewRegionId);
	                }
	                
	                if (TacticalDebugData.bDrawBestCover && TacticalDebugData.VisibilityViewRegionId >= 0)
	                {
	                    DebugDrawBestCover(TacticalDebugData.VisibilityViewRegionId);
	                }
	                
	                if (TacticalDebugData.bDebugDrawRegionAdjacency)
	                {
	                    DebugDrawAdjacency();
	                }
	                
	                if (TacticalDebugData.bDebugDrawPortals)
	                {
		                DebugDrawPortals();
	                }
	            }
	            else
	            {
	                UE_LOG(LogNav3D, Warning, TEXT("Failed to convert compact tactical data for debug rendering"));
	            }
	        }
		}
	}
}

void FNav3DMeshSceneProxy::DebugDrawRegionInfo(int32 RegionId)
{
	if (!NavigationData.IsValid())
	{
		return;
	}
	
	// Find the region with the specified ID
	const FNav3DRegion* SelectedRegion = nullptr;
	for (const FNav3DRegion& Region : NavigationData->GetConsolidatedTacticalData().AllLoadedRegions)
	{
		if (Region.Id == RegionId)
		{
			SelectedRegion = &Region;
			break;
		}
	}
	
	if (!SelectedRegion)
	{
		return;
	}
	
	// Draw the region bounds with a special color (bright green)
	FVector RegionCenter = SelectedRegion->Bounds.GetCenter();
	
	// Draw a wireframe box around the selected region
	FColor RegionColor = FColor::Green;
	FVector Min = SelectedRegion->Bounds.Min;
	FVector Max = SelectedRegion->Bounds.Max;
	
	// Draw the 12 edges of the box
	Lines.Emplace(FVector(Min.X, Min.Y, Min.Z), FVector(Max.X, Min.Y, Min.Z), RegionColor);
	Lines.Emplace(FVector(Min.X, Min.Y, Min.Z), FVector(Min.X, Max.Y, Min.Z), RegionColor);
	Lines.Emplace(FVector(Min.X, Min.Y, Min.Z), FVector(Min.X, Min.Y, Max.Z), RegionColor);
	Lines.Emplace(FVector(Max.X, Max.Y, Min.Z), FVector(Min.X, Max.Y, Min.Z), RegionColor);
	Lines.Emplace(FVector(Max.X, Max.Y, Min.Z), FVector(Max.X, Min.Y, Min.Z), RegionColor);
	Lines.Emplace(FVector(Max.X, Max.Y, Min.Z), FVector(Max.X, Max.Y, Max.Z), RegionColor);
	Lines.Emplace(FVector(Min.X, Max.Y, Max.Z), FVector(Min.X, Min.Y, Max.Z), RegionColor);
	Lines.Emplace(FVector(Min.X, Max.Y, Max.Z), FVector(Min.X, Max.Y, Min.Z), RegionColor);
	Lines.Emplace(FVector(Min.X, Max.Y, Max.Z), FVector(Max.X, Max.Y, Max.Z), RegionColor);
	Lines.Emplace(FVector(Max.X, Min.Y, Max.Z), FVector(Min.X, Min.Y, Max.Z), RegionColor);
	Lines.Emplace(FVector(Max.X, Min.Y, Max.Z), FVector(Max.X, Min.Y, Min.Z), RegionColor);
	Lines.Emplace(FVector(Max.X, Min.Y, Max.Z), FVector(Max.X, Max.Y, Max.Z), RegionColor);
	
	// Draw center point
	FColor CenterColor = FColor::Yellow;
	FVector CenterOffset = FVector(5.0f, 0.0f, 0.0f);
	Lines.Emplace(RegionCenter - CenterOffset, RegionCenter + CenterOffset, CenterColor);
	CenterOffset = FVector(0.0f, 5.0f, 0.0f);
	Lines.Emplace(RegionCenter - CenterOffset, RegionCenter + CenterOffset, CenterColor);
	CenterOffset = FVector(0.0f, 0.0f, 5.0f);
	Lines.Emplace(RegionCenter - CenterOffset, RegionCenter + CenterOffset, CenterColor);
	
	// Draw connections to adjacent regions
	if (const FRegionIdArray* AdjacentIds = NavigationData->GetConsolidatedTacticalData().RegionAdjacency.Find(RegionId))
	{
		FColor ConnectionColor = FColor::Cyan;
		for (int32 AdjacentId : AdjacentIds->GetArray())
		{
			// Find the adjacent region
			for (const FNav3DRegion& AdjacentRegion : NavigationData->GetConsolidatedTacticalData().AllLoadedRegions)
			{
				if (AdjacentRegion.Id == AdjacentId)
				{
					FVector AdjacentCenter = AdjacentRegion.Bounds.GetCenter();
					Lines.Emplace(RegionCenter, AdjacentCenter, ConnectionColor);
					break;
				}
			}
		}
	}
	
	// Draw visibility connections
	if (const FRegionIdArray* VisibleIds = NavigationData->GetConsolidatedTacticalData().RegionVisibility.Find(RegionId))
	{
		FColor VisibilityColor = FColor::Magenta;
		for (int32 VisibleId : VisibleIds->GetArray())
		{
			if (VisibleId != RegionId) // Skip self-visibility
			{
				// Find the visible region
				for (const FNav3DRegion& VisibleRegion : NavigationData->GetConsolidatedTacticalData().AllLoadedRegions)
				{
					if (VisibleRegion.Id == VisibleId)
					{
						FVector VisibleCenter = VisibleRegion.Bounds.GetCenter();
						// Draw a dashed line for visibility (approximated with multiple short segments)
						FVector Direction = (VisibleCenter - RegionCenter).GetSafeNormal();
						float Distance = FVector::Dist(RegionCenter, VisibleCenter);
						int32 Segments = FMath::Max(3, FMath::RoundToInt(Distance / 50.0f));
						
						for (int32 i = 0; i < Segments; i += 2)
						{
							float StartT = static_cast<float>(i) / Segments;
							float EndT = static_cast<float>(i + 1) / Segments;
							FVector Start = RegionCenter + Direction * Distance * StartT;
							FVector End = RegionCenter + Direction * Distance * EndT;
							Lines.Emplace(Start, End, VisibilityColor);
						}
						break;
					}
				}
			}
		}
	}
}

FNav3DMeshSceneProxy::~FNav3DMeshSceneProxy()
{
}

SIZE_T FNav3DMeshSceneProxy::GetTypeHash() const
{
	static size_t UniquePointer;
	return reinterpret_cast<size_t>(&UniquePointer);
}

void FNav3DMeshSceneProxy::DebugDrawRegions()
{
	if (!NavigationData.IsValid()) return;
    
	// Convert compact → build for debug drawing with source chunks
	const FConsolidatedTacticalData BuildData = FNav3DTacticalDataConverter::CompactToBuild(
		NavigationData->ConsolidatedCompactTacticalData, NavigationData->GetAllChunkActors());
    
	// Use existing working code with proper bounds
	for (const FNav3DRegion& Region : BuildData.AllLoadedRegions)
	{
		const FColor& RegionColor = RegionColors[Region.Id % 32];
		FVector RegionCenter = Region.Bounds.GetCenter();
		FVector RegionExtent = Region.Bounds.GetExtent();
		Boxes.Emplace(FBox::BuildAABB(RegionCenter, RegionExtent), RegionColor);
	}
}

void FNav3DMeshSceneProxy::DebugDrawRegionIds()
{
    if (!NavigationData.IsValid()) return;
    
    // Convert compact → build for debug drawing
    const FConsolidatedTacticalData BuildData = FNav3DTacticalDataConverter::CompactToBuild(
        NavigationData->ConsolidatedCompactTacticalData, NavigationData->GetAllChunkActors());
    
    for (const FNav3DRegion& Region : BuildData.AllLoadedRegions)
    {
        FVector RegionCenter = Region.Bounds.GetCenter();
        FString RegionIdText = FString::Printf(TEXT("%d"), Region.Id);
        Texts.Emplace(RegionIdText, RegionCenter, FLinearColor::White);
    }
}

void FNav3DMeshSceneProxy::DebugDrawAdjacency()
{
    if (!NavigationData.IsValid()) return;
    
    // Convert compact → build for debug drawing
    const FConsolidatedTacticalData BuildData = FNav3DTacticalDataConverter::CompactToBuild(
        NavigationData->ConsolidatedCompactTacticalData, NavigationData->GetAllChunkActors());
    
    // Use existing working adjacency drawing code
    TSet<TPair<int32, int32>> DrawnConnections;
    
    for (const auto& AdjPair : BuildData.RegionAdjacency)
    {
        const int32 RegionId = AdjPair.Key;
        const FRegionIdArray& AdjacentIds = AdjPair.Value;
        
        // Find the region
        const FNav3DRegion* Region = nullptr;
        for (const FNav3DRegion& R : BuildData.AllLoadedRegions)
        {
            if (R.Id == RegionId)
            {
                Region = &R;
                break;
            }
        }
        
        if (!Region) continue;
        
        FVector RegionCenter = Region->Bounds.GetCenter();
        
        for (int32 AdjacentId : AdjacentIds.RegionIds)
        {
            TPair<int32, int32> Connection(
                FMath::Min(RegionId, AdjacentId),
                FMath::Max(RegionId, AdjacentId)
            );
            
            if (DrawnConnections.Contains(Connection)) continue;
            
            // Find adjacent region
            for (const FNav3DRegion& AdjacentRegion : BuildData.AllLoadedRegions)
            {
                if (AdjacentRegion.Id == AdjacentId)
                {
                    FVector AdjacentCenter = AdjacentRegion.Bounds.GetCenter();
                    Lines.Emplace(RegionCenter, AdjacentCenter, FColor::Black);
                    DrawnConnections.Add(Connection);
                    break;
                }
            }
        }
    }
}

void FNav3DMeshSceneProxy::DebugDrawVisibility(const int32 ViewerRegionId)
{
    if (!NavigationData.IsValid())
    {
        return;
    }
    
    // Convert compact → build for debug drawing
    const FConsolidatedTacticalData BuildData = FNav3DTacticalDataConverter::CompactToBuild(
        NavigationData->ConsolidatedCompactTacticalData, NavigationData->GetAllChunkActors());
    
    // Find the viewer region
    const FNav3DRegion* ViewerRegion = nullptr;
    for (const FNav3DRegion& Region : BuildData.AllLoadedRegions)
    {
        if (Region.Id == ViewerRegionId)
        {
            ViewerRegion = &Region;
            break;
        }
    }
    
    if (!ViewerRegion)
    {
        return;
    }
    
    // Get the center of the viewer region
    FVector ViewerCenter = ViewerRegion->Bounds.GetCenter();
    
    // Access the visibility set from converted legacy data
    const FRegionIdArray* VisibilitySet = BuildData.RegionVisibility.Find(ViewerRegionId);
    
    // Draw lines to ALL regions (not just visible ones) with appropriate colors
    for (const FNav3DRegion& Region : BuildData.AllLoadedRegions)
    {
        // Skip self
        if (Region.Id == ViewerRegionId)
        {
            continue;
        }
        
        // Get the center of the target region
        FVector TargetCenter = Region.Bounds.GetCenter();
        
        // Check if the target region is in the visibility set
        const bool bIsVisible = VisibilitySet && VisibilitySet->Contains(Region.Id);
        
        // Green for visible, Red for not visible
        FColor LineColor = bIsVisible ? FColor(0, 255, 0) : FColor(255, 0, 0);
        
        // Add the line - always draw regardless of visibility
        Lines.Emplace(ViewerCenter, TargetCenter, LineColor);
        
        // Add small sphere at start position
        Spheres.Emplace(25.0f, ViewerCenter, LineColor, SolidMesh);
        
        // Add small sphere at target position
        Spheres.Emplace(25.0f, TargetCenter, LineColor, SolidMesh);
    }
}

bool FNav3DMeshSceneProxy::AddVoxelToBoxes(const FVector& VoxelLocation,
								  const float NodeExtent,
								  const bool IsOccluded)
{
	const auto& DebugInfos = NavigationData->GetDebugData();

	// VoxelLocation is already the center, don't add NodeExtent
	if (DebugInfos.bDebugDrawFreeVoxels && !IsOccluded)
	{
		Boxes.Emplace(FBox::BuildAABB(VoxelLocation, FVector(NodeExtent)),
				 FreeVoxelColor);
		// Store translucent filled surface data for free voxels
		VoxelSurfaces.Emplace(FBox::BuildAABB(VoxelLocation, FVector(NodeExtent)), FreeVoxelColor, 0.01f);
		return true;
	}
	if (DebugInfos.bDebugDrawOccludedVoxels && IsOccluded)
	{
		Boxes.Emplace(FBox::BuildAABB(VoxelLocation, FVector(NodeExtent)), OccludedVoxelColor);
		// Store translucent filled surface data for occluded voxels
		VoxelSurfaces.Emplace(FBox::BuildAABB(VoxelLocation, FVector(NodeExtent)), OccludedVoxelColor, 0.01f);
		return true;
	}

	return false;
}

void FNav3DMeshSceneProxy::GetDynamicMeshElements(const TArray<const FSceneView*>& Views,
	const FSceneViewFamily& ViewFamily,
	const uint32 VisibilityMap,
	FMeshElementCollector& Collector) const
{
	// Let base class draw lines, boxes, spheres, text
	FDebugRenderSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);

	for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ++ViewIndex)
	{
		if ((VisibilityMap & (1 << ViewIndex)) == 0)
		{
			continue;
		}

		FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);
		RenderVoxelSurfaces(PDI, Collector);
	}
}

void FNav3DMeshSceneProxy::AddVolumeTextInfos()
{
	if (!NavigationData.IsValid())
	{
		return;
	}

	const TArray<ANav3DDataChunkActor*> ChunkActors = NavigationData->GetAllChunkActors();
	const TArray<FBox> OriginalVolumes = NavigationData->GetAllDiscoverableVolumes();

	static const TArray VolumeColors = {
		FLinearColor(0.0f, 1.0f, 1.0f),
		FLinearColor::Green,
		FLinearColor::Blue,
		FLinearColor::Yellow,
		FLinearColor(1.0f, 0.5f, 0.0f),
		FLinearColor(0.5f, 0.0f, 1.0f),
		FLinearColor(0.0f, 1.0f, 0.5f),
		FLinearColor(1.0f, 0.0f, 0.5f),
		FLinearColor(0.5f, 1.0f, 0.0f),
		FLinearColor(0.0f, 0.5f, 1.0f),
	};

	for (int32 ChunkIndex = 0; ChunkIndex < ChunkActors.Num(); ++ChunkIndex)
	{
		const ANav3DDataChunkActor* ChunkActor = ChunkActors[ChunkIndex];
		if (!ChunkActor || !IsValid(ChunkActor))
		{
			continue;
		}

		const FBox ChunkBounds = ChunkActor->DataChunkActorBounds;
		const FVector ChunkCenter = ChunkBounds.GetCenter();

		int32 ParentVolumeIndex = INDEX_NONE;
		for (int32 VolumeIndex = 0; VolumeIndex < OriginalVolumes.Num(); ++VolumeIndex)
		{
			if (OriginalVolumes[VolumeIndex].IsInside(ChunkCenter))
			{
				ParentVolumeIndex = VolumeIndex;
				break;
			}
		}

		int32 ChunkIndexInVolume = 0;
		if (ParentVolumeIndex != INDEX_NONE)
		{
			for (int32 PrevChunkIndex = 0; PrevChunkIndex < ChunkIndex; ++PrevChunkIndex)
			{
				const ANav3DDataChunkActor* PrevChunkActor = ChunkActors[PrevChunkIndex];
				if (PrevChunkActor && IsValid(PrevChunkActor))
				{
					const FVector PrevCenter = PrevChunkActor->DataChunkActorBounds.GetCenter();
					if (OriginalVolumes[ParentVolumeIndex].IsInside(PrevCenter))
					{
						ChunkIndexInVolume++;
					}
				}
			}
		}

		FString VolumeText;
		FLinearColor TextColor = FNav3DUtils::GetChunkColorByIndex(ChunkIndex);
		if (ParentVolumeIndex != INDEX_NONE)
		{
			VolumeText = FString::Printf(TEXT("Chunk %d (Vol %d)"), ChunkIndexInVolume, ParentVolumeIndex);
		}
		else
		{
			VolumeText = FString::Printf(TEXT("Chunk %d (Orphan)"), ChunkIndex);
		}

		const FVector ChunkSize = ChunkBounds.GetSize();
		VolumeText += FString::Printf(TEXT("\n%.0f x %.0f x %.0f"), ChunkSize.X, ChunkSize.Y, ChunkSize.Z);

		const FVector TextPosition = ChunkCenter + FVector(0.0f, 0.0f, ChunkBounds.GetExtent().Z * 0.1f);
		Texts.Emplace(VolumeText, TextPosition, TextColor);

		const FColor WireColor = TextColor.ToFColor(true);
		Boxes.Emplace(ChunkBounds, WireColor);
	}

	if (ChunkActors.Num() > 0)
	{
		FVector SummaryPosition = FVector::ZeroVector;
		if (OriginalVolumes.Num() > 0)
		{
			SummaryPosition = OriginalVolumes[0].GetCenter() + FVector(0.0f, 0.0f, OriginalVolumes[0].GetExtent().Z * 1.5f);
		}
		const FString SummaryText = FString::Printf(TEXT("Nav3D Volumes: %d original, %d chunks"), OriginalVolumes.Num(), ChunkActors.Num());
		Texts.Emplace(SummaryText, SummaryPosition, FLinearColor::White);
	}
}

void FNav3DMeshSceneProxy::RenderVoxelSurfaces(FPrimitiveDrawInterface* PDI, FMeshElementCollector& Collector) const
{
	if (VoxelSurfaces.Num() == 0)
	{
		return;
	}

	// Reuse at most two colored material proxies per frame (free vs occluded)
	const FMaterialRenderProxy* BaseProxy = GEngine && GEngine->DebugMeshMaterial
		? GEngine->DebugMeshMaterial->GetRenderProxy()
		: nullptr;
	if (BaseProxy == nullptr)
	{
		return;
	}

	FColoredMaterialRenderProxy* FreeProxy = nullptr;
	FColoredMaterialRenderProxy* OccludedProxy = nullptr;

	auto GetProxyForColor = [&](const FColor& InColor, const float InOpacity) -> FColoredMaterialRenderProxy*
	{
		if (InColor == FreeVoxelColor)
		{
			if (FreeProxy == nullptr)
			{
				FLinearColor Linear = FLinearColor(InColor);
				Linear.A = InOpacity;
				FreeProxy = new FColoredMaterialRenderProxy(BaseProxy, Linear);
				Collector.RegisterOneFrameMaterialProxy(FreeProxy);
			}
			return FreeProxy;
		}
		if (InColor == OccludedVoxelColor)
		{
			if (OccludedProxy == nullptr)
			{
				FLinearColor Linear = FLinearColor(InColor);
				Linear.A = InOpacity;
				OccludedProxy = new FColoredMaterialRenderProxy(BaseProxy, Linear);
				Collector.RegisterOneFrameMaterialProxy(OccludedProxy);
			}
			return OccludedProxy;
		}

		FLinearColor Linear = FLinearColor(InColor);
		Linear.A = InOpacity;
		FColoredMaterialRenderProxy* OneOff = new FColoredMaterialRenderProxy(BaseProxy, Linear);
		Collector.RegisterOneFrameMaterialProxy(OneOff);
		return OneOff;
	};

	for (const FVoxelSurfaceData& Surface : VoxelSurfaces)
	{
		const FColoredMaterialRenderProxy* ColoredProxy = GetProxyForColor(Surface.Color, Surface.Opacity);

		GetBoxMesh(FTransform(Surface.Bounds.GetCenter()).ToMatrixNoScale(),
			   Surface.Bounds.GetExtent(),
			   ColoredProxy,
			   SDPG_World,
			   0, // ViewIndex
			   Collector);
	}
}

void FNav3DMeshSceneProxy::AddNodeTextInfos(const MortonCode NodeMortonCode,
                                            const LayerIndex NodeLayerIndex,
                                            const FVector& NodePosition)
{
	const auto& DebugInfos = NavigationData->GetDebugData();

	static constexpr float VerticalOffsetIncrement = 40.0f;

	auto VerticalOffset = 0.0f;
	if (DebugInfos.bDebugDrawMortonCodes)
	{
		Texts.Emplace(
			FString::Printf(TEXT("%i:%llu"), NodeLayerIndex, NodeMortonCode),
			NodePosition, FLinearColor::Black);
		VerticalOffset += VerticalOffsetIncrement;
	}
	if (DebugInfos.bDebugDrawNodeCoords)
	{
		const FIntVector MortonCoords =
			FIntVector(FNav3DUtils::GetVectorFromMortonCode(NodeMortonCode));
		Texts.Emplace(FString::Printf(TEXT("%d, %d, %d"), MortonCoords.X, MortonCoords.Y, MortonCoords.Z),
		              NodePosition + FVector(0.0f, 0.0f, VerticalOffset),
		              FLinearColor::Black);
	}
}

FPrimitiveViewRelevance
FNav3DMeshSceneProxy::GetViewRelevance(const FSceneView* View) const
{
	const bool bVisible = !!View->Family->EngineShowFlags.Navigation;
	FPrimitiveViewRelevance Result;
	Result.bDrawRelevance = bVisible && IsShown(View);
	Result.bDynamicRelevance = true;
	Result.bSeparateTranslucency = Result.bNormalTranslucency = bVisible && IsShown(View);
	return Result;
}

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST

void FNav3DDebugDrawDelegateHelper::InitDelegateHelper(
	const FNav3DMeshSceneProxy* SceneProxy)
{
	Super::InitDelegateHelper(SceneProxy);

	NavigationData = SceneProxy->NavigationData;
}

void FNav3DDebugDrawDelegateHelper::RegisterDebugDrawDelegateInternal()
{
	if (State == RegisteredState)
	{
		UE_LOG(LogNav3D, Warning, TEXT("Debug draw delegate is already registered"));
		return;
	}
	if (State == InitializedState)
	{
		DebugTextDrawingDelegate = FDebugDrawDelegate::CreateRaw(
			this, &FNav3DDebugDrawDelegateHelper::DrawDebugLabels);
		DebugTextDrawingDelegateHandle = UDebugDrawService::Register(
			TEXT("Navigation"), DebugTextDrawingDelegate);
		State = RegisteredState;
	}
}

void FNav3DDebugDrawDelegateHelper::UnregisterDebugDrawDelegate()
{
	if (State == RegisteredState)
	{
		check(DebugTextDrawingDelegate.IsBound());
		UDebugDrawService::Unregister(DebugTextDrawingDelegateHandle);
		State = InitializedState;
	}
}
#endif

UNav3DNavDataRenderingComponent::UNav3DNavDataRenderingComponent()
{
	UPrimitiveComponent::SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

	bIsEditorOnly = true;
	bSelectable = false;
	bForcesUpdate = false;
}

FPrimitiveSceneProxy* UNav3DNavDataRenderingComponent::CreateSceneProxy()
{
	// Get owner and gather data
	if (ANav3DData* NavData = Cast<ANav3DData>(GetOwner()))
	{
		// Collect all navigation data from chunk actors
		TArray<FNav3DVolumeNavigationData> AllVolumeData;
		for (ANav3DDataChunkActor* ChunkActor : NavData->GetChunkActors())
		{
			if (!ChunkActor) continue;
			
			for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
			{
				if (!Chunk) continue;

				if (const FNav3DVolumeNavigationData* VolumeData = Chunk->GetVolumeNavigationData())
				{
					AllVolumeData.Add(*VolumeData);
				}
			}
		}
		
		// Create proxy data with the collected volume navigation data
		FNav3DMeshSceneProxyData ProxyData(AllVolumeData);

		// Then set the other members
		ProxyData.NavigationData = NavData;
		ProxyData.DebugData = NavData->GetDebugData();
		ProxyData.RenderingComponent = this;

		if (FNav3DMeshSceneProxy* NewSceneProxy = new FNav3DMeshSceneProxy(*this, ProxyData))
		{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
			DebugDrawDelegateManager.InitDelegateHelper(NewSceneProxy);
			DebugDrawDelegateManager.ReregisterDebugDrawDelegate();
#endif
			return NewSceneProxy;
		}
	}

	return nullptr;
}

FBoxSphereBounds UNav3DNavDataRenderingComponent::CalcBounds(
	const FTransform& LocalToWorld) const
{
	FBox BoundingBox(ForceInit);

	if (const ANav3DData* NavigationData = Cast<ANav3DData>(GetOwner()))
	{
		BoundingBox = NavigationData->GetBoundingBox();
	}

	return FBoxSphereBounds(BoundingBox);
}

void UNav3DNavDataRenderingComponent::CreateRenderState_Concurrent(
	FRegisterComponentContext* Context)
{
	Super::CreateRenderState_Concurrent(Context);

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
	DebugDrawDelegateManager.RequestRegisterDebugDrawDelegate(Context);
#endif
}

void UNav3DNavDataRenderingComponent::DestroyRenderState_Concurrent()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
	DebugDrawDelegateManager.UnregisterDebugDrawDelegate();
#endif

	Super::DestroyRenderState_Concurrent();
}

bool UNav3DNavDataRenderingComponent::IsNavigationShowFlagSet(
	const UWorld* World)
{
	bool ShowNavigation;

	const FWorldContext* WorldContext = GEngine->GetWorldContextFromWorld(World);

#if WITH_EDITOR
	if (GEditor != nullptr && WorldContext &&
		WorldContext->WorldType != EWorldType::Game)
	{
		ShowNavigation = WorldContext->GameViewport != nullptr &&
			WorldContext->GameViewport->EngineShowFlags.Navigation;
		if (ShowNavigation == false)
		{
			for (const FEditorViewportClient* CurrentViewport :
			     GEditor->GetAllViewportClients())
			{
				if (CurrentViewport && CurrentViewport->EngineShowFlags.Navigation)
				{
					ShowNavigation = true;
					break;
				}
			}
		}
	}
	else
#endif // WITH_EDITOR
	{
		ShowNavigation = WorldContext && WorldContext->GameViewport &&
			WorldContext->GameViewport->EngineShowFlags.Navigation;
	}

	return ShowNavigation;
}

void FNav3DMeshSceneProxy::DebugDrawBestCover(const int32 ViewerRegionId)
{
    if (!NavigationData.IsValid())
    {
        return;
    }
    
    const TArray<FNav3DRegion>& Regions = NavigationData->GetConsolidatedTacticalData().AllLoadedRegions;
    
    // Find the viewer region
    const FNav3DRegion* ViewerRegion = nullptr;
    for (const FNav3DRegion& Region : Regions)
    {
        if (Region.Id == ViewerRegionId)
        {
            ViewerRegion = &Region;
            break;
        }
    }
    
    if (!ViewerRegion)
    {
        return;
    }
    
    // Get the center of the viewer region - this is our "start position" for finding cover
    FVector StartPosition = ViewerRegion->Bounds.GetCenter();
    
    // Use the same position as the start position, so the path from start to cover position represents fleeing.
    const FVector ObserverPosition = StartPosition;

    // Find best cover position - force a new region since start and observer are in the same region
    TArray<FPositionCandidate> CoverPositions;
	const bool bCoverFound = NavigationData->FindBestLocation(
		StartPosition,
		TArray({ObserverPosition}),
		CoverPositions,
		ETacticalVisibility::TargetOccluded,
		ETacticalDistance::Any,
		ETacticalRegion::Smallest,
		true,
		true);

	if (!bCoverFound)
	{
		UE_LOG(LogNav3D, Warning, TEXT("No cover location found"));
		return;
	}
	
	const FColor DrawColor = FColor::Yellow;
	Spheres.Emplace(200.0f, StartPosition, DrawColor, SolidMesh);
	Spheres.Emplace(200.0f, CoverPositions[0].Position, DrawColor, SolidMesh);
	Lines.Emplace(StartPosition, CoverPositions[0].Position, DrawColor, 5.f);
}

void FNav3DMeshSceneProxy::DebugDrawPortals()
{
    if (!NavigationData.IsValid())
    {
        return;
    }
    
    const FColor PortalColor = FColor::Yellow;

    // Iterate all source chunks from consolidated data
    for (const TWeakObjectPtr<ANav3DDataChunkActor>& ChunkPtr : NavigationData->GetConsolidatedTacticalData().SourceChunks)
    {
        const ANav3DDataChunkActor* SourceChunk = ChunkPtr.Get();
        if (!SourceChunk) { continue; }

        for (const FNav3DChunkAdjacency& Adjacency : SourceChunk->ChunkAdjacency)
        {
            // Use compact portals only
            for (const FCompactPortal& CP : Adjacency.CompactPortals)
            {
	            // Resolve local endpoint in source chunk
                const UNav3DDataChunk* AnyChunk = SourceChunk->Nav3DChunks.Num() > 0 ? SourceChunk->Nav3DChunks[0] : nullptr;
                const FNav3DVolumeNavigationData* Vol = AnyChunk ? AnyChunk->GetVolumeNavigationData() : nullptr;
                const FVector A = Vol ? Vol->GetLeafNodePositionFromMortonCode(CP.Local) : FVector::ZeroVector;

                // Resolve remote endpoint using adjacent chunk
                const ANav3DDataChunkActor* TargetChunk = Adjacency.OtherChunkActor.Get();
                const UNav3DDataChunk* TargetAnyChunk = (TargetChunk && TargetChunk->Nav3DChunks.Num() > 0) ? TargetChunk->Nav3DChunks[0] : nullptr;
                const FNav3DVolumeNavigationData* TargetVol = TargetAnyChunk ? TargetAnyChunk->GetVolumeNavigationData() : nullptr;
                const FVector B = TargetVol ? TargetVol->GetLeafNodePositionFromMortonCode(CP.Remote) : FVector::ZeroVector;

            	auto ProjectToFace = [](const FVector& P, const FBox& BoxBounds, const FVector& FaceNormal) -> FVector
            	{
            		FVector Out = P;
            		if (FaceNormal.X > 0.5f)   Out.X = BoxBounds.Max.X;
            		else if (FaceNormal.X < -0.5f) Out.X = BoxBounds.Min.X;
            		else if (FaceNormal.Y > 0.5f)  Out.Y = BoxBounds.Max.Y;
            		else if (FaceNormal.Y < -0.5f) Out.Y = BoxBounds.Min.Y;
            		else if (FaceNormal.Z > 0.5f)  Out.Z = BoxBounds.Max.Z;
            		else if (FaceNormal.Z < -0.5f) Out.Z = BoxBounds.Min.Z;
            		return Out;
            	};
            	
            	const FVector N = Adjacency.SharedFaceNormal.GetSafeNormal();
            	
                // Lightweight cross markers instead of spheres
	            constexpr float Length = 500.0f;
                const FBox& SrcBounds = SourceChunk->DataChunkActorBounds;
                const FBox TgtBounds = TargetChunk ? TargetChunk->DataChunkActorBounds : FBox(ForceInit);

                const FVector AProj = (!A.IsNearlyZero() && !N.IsNearlyZero()) ? ProjectToFace(A, SrcBounds, N) : A;
                const FVector BProj = (!B.IsNearlyZero() && TargetChunk && !N.IsNearlyZero()) ? ProjectToFace(B, TgtBounds, -N) : B;

                // Slight offset onto the face to avoid z-fighting
	            constexpr float Eps = 1.0f;
                const FVector Aon = AProj + N * Eps;
                const FVector Bon = BProj - N * Eps;

                auto MakeFaceBasis = [](const FVector& Normal, FVector& U, FVector& V)
                {
                    // Snap normal to principal axis to keep basis orthogonal to world axes
                    FVector N = Normal;
                    const float ax = FMath::Abs(N.X);
                    const float ay = FMath::Abs(N.Y);
                    const float az = FMath::Abs(N.Z);
                    if (ax >= ay && ax >= az)
                    {
                        N = FVector((N.X >= 0.f) ? 1.f : -1.f, 0.f, 0.f);
                        U = FVector(0.f, 1.f, 0.f);
                        V = FVector(0.f, 0.f, 1.f);
                    }
                    else if (ay >= ax && ay >= az)
                    {
                        N = FVector(0.f, (N.Y >= 0.f) ? 1.f : -1.f, 0.f);
                        U = FVector(1.f, 0.f, 0.f);
                        V = FVector(0.f, 0.f, 1.f);
                    }
                    else
                    {
                        N = FVector(0.f, 0.f, (N.Z >= 0.f) ? 1.f : -1.f);
                        U = FVector(1.f, 0.f, 0.f);
                        V = FVector(0.f, 1.f, 0.f);
                    }
                };

                FVector UAxis, VAxis; MakeFaceBasis(N, UAxis, VAxis);

                auto DrawSquare = [&](const FVector& C)
                {
                    const FVector P0 = C + UAxis * Length + VAxis * Length;
                    const FVector P1 = C - UAxis * Length + VAxis * Length;
                    const FVector P2 = C - UAxis * Length - VAxis * Length;
                    const FVector P3 = C + UAxis * Length - VAxis * Length;
                    Lines.Emplace(P0, P1, PortalColor, 5.f);
                    Lines.Emplace(P1, P2, PortalColor, 5.f);
                    Lines.Emplace(P2, P3, PortalColor, 5.f);
                    Lines.Emplace(P3, P0, PortalColor, 5.f);
                };

                if (!Aon.IsNearlyZero()) { DrawSquare(Aon); }
                if (!Bon.IsNearlyZero()) { DrawSquare(Bon); }
            }
        }
    }
}
