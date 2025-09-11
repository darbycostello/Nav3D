#include "Nav3DNavDataRenderingComponent.h"
#include "Nav3DData.h"
#include "Nav3DUtils.h"
#include <Debug/DebugDrawService.h>
#include <Engine/CollisionProfile.h>
#include <Materials/Material.h>
#include <Engine/Engine.h>
#include <PrimitiveSceneProxy.h>
#include <libmorton/morton.h>

#include "Nav3D.h"
#include "Materials/MaterialRenderProxy.h"
#include "Nav3DDataChunkActor.h"
#include "Nav3DTacticalActor.h"
#include "Pathfinding/Nav3DCrossVolumeGraph.h"

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
	            
	            if (TacticalDebugData.bDebugDrawAdjacencyGraph)
	            {
	                DebugDrawAdjacency();
	            }
	        }

		// Cross-volume adjacency graph debug rendering
		if (NavigationData->DebugData.bDebugDrawAdjacency)
		{
			DebugDrawCrossVolumeAdjacency();
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
	if (!NavigationData.IsValid())
	{
		return;
	}
    
	const auto& [Regions] = NavigationData->GetTacticalDataAtPosition(FVector::Zero());
    
	for (int32 i = 0; i < Regions.Num(); ++i)
	{
		const FNav3DRegion& Region = Regions[i];
        
		// Map the region ID to a color using modulo to cycle through the available colors
		const FColor& RegionColor = RegionColors[Region.Id % 32];
        
		// Convert from min/max box to center/extent for consistency with voxel rendering
		FVector RegionCenter = Region.Bounds.GetCenter();
		FVector RegionExtent = Region.Bounds.GetExtent() - FVector(10, 10, 10); // Shrink slightly
        
		// Add box for this region using center and extent (same approach as voxels)
		Boxes.Emplace(FBox::BuildAABB(RegionCenter, RegionExtent), RegionColor);
	}
}

void FNav3DMeshSceneProxy::DebugDrawRegionIds()
{
    if (!NavigationData.IsValid())
    {
        return;
    }
    
    const auto& [Regions] = NavigationData->GetTacticalDataAtPosition(FVector::Zero());
    
    for (int32 i = 0; i < Regions.Num(); ++i)
    {
        const FNav3DRegion& Region = Regions[i];
        
        // Draw region ID as text at the center of the region
        FVector RegionCenter = Region.Bounds.GetCenter();
        FString RegionIdText = FString::Printf(TEXT("%d"), Region.Id);
        Texts.Emplace(RegionIdText, RegionCenter, FLinearColor::White);
    }
}

void FNav3DMeshSceneProxy::DebugDrawAdjacency()
{
    if (!NavigationData.IsValid())
    {
        return;
    }
    
    const auto& [Regions] = NavigationData->GetTacticalDataAtPosition(FVector::Zero());
    
    // Keep track of which connections we've already drawn
    TSet<TPair<int32, int32>> DrawnConnections;
    
    for (const FNav3DRegion& Region : Regions)
    {
        // Get the center of this region
        FVector RegionCenter = Region.Bounds.GetCenter();
        
        for (const int32 AdjacentId : Region.AdjacentRegionIds)
        {
            // Create a connection pair (using min/max to ensure consistent ordering)
            TPair<int32, int32> Connection(
                FMath::Min(Region.Id, AdjacentId),
                FMath::Max(Region.Id, AdjacentId)
            );
            
            // Skip if we've already drawn this connection
            if (DrawnConnections.Contains(Connection))
            {
                continue;
            }
            
            // Find the adjacent region to get its center
            const FNav3DRegion* AdjacentRegion = nullptr;
            for (const FNav3DRegion& OtherRegion : Regions)
            {
                if (OtherRegion.Id == AdjacentId)
                {
                    AdjacentRegion = &OtherRegion;
                    break;
                }
            }
            
            if (AdjacentRegion)
            {
                FVector AdjacentCenter = AdjacentRegion->Bounds.GetCenter();
                
                // Draw a line connecting the centers of adjacent regions
                Lines.Emplace(RegionCenter, AdjacentCenter, FColor::Black);
                
                // Add to the set of drawn connections
                DrawnConnections.Add(Connection);
            }
        }
    }
}

void FNav3DMeshSceneProxy::DebugDrawCrossVolumeAdjacency()
{
    if (!NavigationData.IsValid())
    {
        return;
    }

    const bool bDrawPortals = false; // portal drawing removed

    for (const ANav3DTacticalActor* Ta : NavigationData->GetAllTacticalActors())
    {
        if (!Ta) { continue; }
        const FNav3DCrossVolumeGraph& Graph = Ta->GetCrossVolumeGraph();
        const TArray<ANav3DDataChunkActor*>& Actors = Graph.GetCachedChunkActors();

        // Iterate buckets indirectly by sampling each chunk's boundary voxels
        for (int32 ChunkIdx = 0; ChunkIdx < Actors.Num(); ++ChunkIdx)
        {
            const ANav3DDataChunkActor* ChunkActor = Actors[ChunkIdx];
            if (!ChunkActor || ChunkActor->Nav3DChunks.Num() == 0) { continue; }
            const UNav3DDataChunk* Chunk = ChunkActor->Nav3DChunks[0];
            if (!Chunk) { continue; }

            for (const FNav3DEdgeVoxel& Edge : Chunk->BoundaryVoxels)
            {
                if (!Edge.bIsNavigable) { continue; }
                FNav3DVoxelID Id; Id.ChunkIndex = ChunkIdx; Id.VolumeIndex = Edge.VolumeIndex; Id.Layer = Edge.LayerIndex; Id.Morton = Edge.Morton;

                TArray<FNav3DCrossVolumeConnection> Neigh;
                Graph.GetNeighbors(Id, Neigh);
                if (Neigh.Num() == 0) { continue; }

                const FVector FromPos = FNav3DUtils::GetVoxelWorldPosition(Id, Actors);
                for (const FNav3DCrossVolumeConnection& Conn : Neigh)
                {
                    const FVector ToPos = FNav3DUtils::GetVoxelWorldPosition(Conn.RemoteVoxel, Actors);
                    Lines.Emplace(FromPos, ToPos, FColor::Yellow);
                    // portal drawing removed
                }
            }
        }
    }
}

// Portal drawing function removed

void FNav3DMeshSceneProxy::DebugDrawVisibility(const int32 ViewerRegionId)
{
    if (!NavigationData.IsValid())
    {
        return;
    }
    
    const auto& [Regions] = NavigationData->GetTacticalDataAtPosition(FVector::Zero());
    
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
    
    // Get the center of the viewer region
    FVector ViewerCenter = ViewerRegion->Bounds.GetCenter();
    
    // Access the visibility set directly from the region
    const TArray<int32>& VisibilitySet = ViewerRegion->VisibilitySet;
    
    // Draw lines to ALL regions (not just visible ones) with appropriate colors
    for (const FNav3DRegion& Region : Regions)
    {
        // Skip self
        if (Region.Id == ViewerRegionId)
        {
            continue;
        }
        
        // Get the center of the target region
        FVector TargetCenter = Region.Bounds.GetCenter();
        
        // Check if the target region is in the visibility set
        const bool bIsVisible = VisibilitySet.Contains(Region.Id);
        
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
    
    const auto& [Regions] = NavigationData->GetTacticalDataAtPosition(FVector::Zero());
    
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
	
	const FColor DrawColor = FColor::White;
	Spheres.Emplace(200.0f, StartPosition, DrawColor, SolidMesh);
	Spheres.Emplace(200.0f, CoverPositions[0].Position, DrawColor, SolidMesh);
	Lines.Emplace(StartPosition, CoverPositions[0].Position, DrawColor);
}

void FNav3DMeshSceneProxy::DebugDrawOctreeAdjacency(const FNav3DVolumeNavigationData& VolumeData, const int32 MaxLinesToDraw)
{
    const FNav3DData& Data = VolumeData.GetData();
    if (!Data.IsValid()) { return; }

    int32 LinesDrawn = 0;

    // Layer 0: free leaf subnode neighbors
    if (Data.GetLayerCount() > 0)
    {
        const auto& LayerZero = Data.GetLayer(0);
        const auto& LeafNodes = Data.GetLeafNodes();
        for (int32 NodeIdx = 0; NodeIdx < LayerZero.GetNodes().Num() && LinesDrawn < MaxLinesToDraw; ++NodeIdx)
        {
            const auto& Node = LayerZero.GetNode(NodeIdx);
            if (!Node.FirstChild.IsValid()) { continue; }
            const auto& Leaf = LeafNodes.GetLeafNode(Node.FirstChild.NodeIndex);
            // For each free sub-node, draw face-adjacent neighbors in same node
            for (uint8 Sub = 0; Sub < 64 && LinesDrawn < MaxLinesToDraw; ++Sub)
            {
                if (Leaf.IsSubNodeOccluded(Sub)) { continue; }
                FVector APos = VolumeData.GetNodePositionFromAddress(FNav3DNodeAddress(0, Node.FirstChild.NodeIndex, Sub), true);

                // 6 directions within the leaf
                static constexpr int Dx[6] = {1,-1,0,0,0,0};
                static constexpr int Dy[6] = {0,0,1,-1,0,0};
                static constexpr int Dz[6] = {0,0,0,0,1,-1};

                uint_fast32_t Sx, Sy, Sz;
                morton3D_64_decode(Sub, Sx, Sy, Sz);

                for (int d = 0; d < 6 && LinesDrawn < MaxLinesToDraw; ++d)
                {
                    int nx = static_cast<int>(Sx) + Dx[d];
                    int ny = static_cast<int>(Sy) + Dy[d];
                    int nz = static_cast<int>(Sz) + Dz[d];
                    if (nx < 0 || nx > 3 || ny < 0 || ny > 3 || nz < 0 || nz > 3) { continue; }
                    uint64 nsub = morton3D_64_encode(nx, ny, nz);
                    if (Leaf.IsSubNodeOccluded(nsub)) { continue; }
                    FVector BPos = VolumeData.GetNodePositionFromAddress(FNav3DNodeAddress(0, Node.FirstChild.NodeIndex, nsub), true);
                    Lines.Emplace(APos, BPos, FColor::Cyan);
                    LinesDrawn++;
                }

                // parent link (child to parent node center)
                if (LinesDrawn < MaxLinesToDraw)
                {
                    if (Node.FirstChild.IsValid())
                    {
                        FVector ParentPos = VolumeData.GetNodePositionFromLayerAndMortonCode(1, FNav3DUtils::GetParentMortonCode(Node.MortonCode));
                        Lines.Emplace(APos, ParentPos, FColor::White);
                        LinesDrawn++;
                    }
                }
            }
        }
    }

    // Higher layers: free node neighbors and parent links
    for (int32 L = 1; L < Data.GetLayerCount() && LinesDrawn < MaxLinesToDraw; ++L)
    {
        const auto& Layer = Data.GetLayer(L);
        for (int32 NodeIdx = 0; NodeIdx < Layer.GetNodes().Num() && LinesDrawn < MaxLinesToDraw; ++NodeIdx)
        {
            const auto& Node = Layer.GetNode(NodeIdx);
            if (Node.HasChildren()) { continue; } // only free nodes
            // draw neighbors (the data layer stores neighbor links in volume data)
            FNav3DNodeAddress Addr; Addr.LayerIndex = L; Addr.NodeIndex = NodeIdx; Addr.SubNodeIndex = 0;
            TArray<FNav3DNodeAddress> Neigh;
            VolumeData.GetNodeNeighbours(Neigh, Addr);
            const FVector APos = VolumeData.GetNodePositionFromLayerAndMortonCode(L, Node.MortonCode);
            for (const auto& N : Neigh)
            {
                if (LinesDrawn >= MaxLinesToDraw) break;
                if (N.LayerIndex != L) continue; // same layer neighbors only here
                const auto& NNode = Data.GetLayer(N.LayerIndex).GetNode(N.NodeIndex);
                if (NNode.HasChildren()) continue; // neighbor must be free
                const FVector BPos = VolumeData.GetNodePositionFromLayerAndMortonCode(N.LayerIndex, NNode.MortonCode);
                Lines.Emplace(APos, BPos, FColor::Cyan);
                LinesDrawn++;
            }
            // parent link
            if (L + 1 < Data.GetLayerCount() && LinesDrawn < MaxLinesToDraw)
            {
                const MortonCode ParentCode = FNav3DUtils::GetParentMortonCode(Node.MortonCode);
                const FVector ParentPos = VolumeData.GetNodePositionFromLayerAndMortonCode(L + 1, ParentCode);
                Lines.Emplace(APos, ParentPos, FColor::White);
                LinesDrawn++;
            }
        }
    }
}