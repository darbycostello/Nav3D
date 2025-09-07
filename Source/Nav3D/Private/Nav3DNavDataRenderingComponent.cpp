#include "Nav3DNavDataRenderingComponent.h"
#include "Nav3DData.h"
#include "Nav3DUtils.h"
#include <Debug/DebugDrawService.h>
#include <Engine/CollisionProfile.h>
#include <Materials/Material.h>
#include <Engine/Engine.h>
#include <PrimitiveSceneProxy.h>

#include "Materials/MaterialRenderProxy.h"

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
#endif

#if WITH_EDITOR
#include <Editor.h>
#include <EditorViewportClient.h>
#endif

static constexpr FColor OccludedVoxelColor = FColor(128, 0, 0);
static constexpr FColor FreeVoxelColor = FColor(0, 128, 0);

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
	DrawType = SolidAndWireMeshes;
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
	const auto& AllNavigationBoundsData = NavigationData->GetVolumeNavigationData();

	// Reserve for filled voxel surfaces to reduce reallocations
	VoxelSurfaces.Reserve(1000);

	for (const auto& NavigationBoundsData : AllNavigationBoundsData)
	{
		const auto& OctreeData = NavigationBoundsData.GetData();
		const auto LayerCount = OctreeData.GetLayerCount();

		if (LayerCount == 0)
		{
			continue;
		}

		if (DebugInfos.bDebugDrawBounds)
		{
			Boxes.Emplace(NavigationBoundsData.GetData().GetNavigationBounds(), FColor::White);
		}

		if (DebugInfos.bDebugDrawLayers)
		{
			const auto CorrectedLayerIndex = FMath::Clamp(
				static_cast<int>(DebugInfos.LayerIndexToDraw), 0, LayerCount - 1);
			const auto NodeExtent = NavigationBoundsData.GetData()
			                                            .GetLayer(CorrectedLayerIndex)
			                                            .GetNodeExtent();

			for (const auto& Node : OctreeData.GetLayer(CorrectedLayerIndex).GetNodes())
			{
				const auto Code = Node.MortonCode;

				if (CorrectedLayerIndex == 0)
				{
					if (const auto LeafNodePosition =
						NavigationBoundsData.GetLeafNodePositionFromMortonCode(Code);
						AddVoxelToBoxes(LeafNodePosition, NodeExtent, Node.HasChildren()))
					{
						AddNodeTextInfos(Code, 0, LeafNodePosition);
					}
				}
				else
				{
					const auto Position =
						NavigationBoundsData.GetNodePositionFromLayerAndMortonCode(CorrectedLayerIndex, Code);

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
		FColoredMaterialRenderProxy* ColoredProxy = GetProxyForColor(Surface.Color, Surface.Opacity);

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
		Texts.Emplace(FString::Printf(TEXT("%s"), *MortonCoords.ToString()),
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
		// Create proxy data with the volume navigation data reference first
		FNav3DMeshSceneProxyData ProxyData(NavData->GetVolumeNavigationData());

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