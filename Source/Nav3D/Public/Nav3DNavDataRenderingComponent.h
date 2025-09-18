#pragma once

#include "Nav3DTypes.h"
#include <Components/PrimitiveComponent.h>
#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>
#include <Math/GenericOctree.h>
#include "Nav3DData.h"
#include "Nav3DNavDataRenderingComponent.generated.h"

class ANav3DData;
class UNav3DNavDataRenderingComponent;

struct NAV3D_API FNav3DMeshSceneProxyData
{
	FNav3DMeshSceneProxyData()
		: VolumeNavigationData(TArray<FNav3DVolumeNavigationData>())
	{
	}

	FNav3DMeshSceneProxyData(const TArray<FNav3DVolumeNavigationData>& InVolumeNavigationData)
		: VolumeNavigationData(InVolumeNavigationData)
	{
	}

	TWeakObjectPtr<ANav3DData> NavigationData;
	FNav3DVolumeDebugData DebugData;
	const TArray<FNav3DVolumeNavigationData>& VolumeNavigationData;
	TWeakObjectPtr<UNav3DNavDataRenderingComponent> RenderingComponent;
};

class NAV3D_API FNav3DMeshSceneProxy final : public FDebugRenderSceneProxy
{
public:
	friend class FNav3DDebugDrawDelegateHelper;

	void EnsureConsolidatedDataForDebugDrawing() const;
	explicit FNav3DMeshSceneProxy(const UPrimitiveComponent& Component, const FNav3DMeshSceneProxyData& ProxyData);
	virtual ~FNav3DMeshSceneProxy() override;

	virtual SIZE_T GetTypeHash() const override;

protected:
	bool AddVoxelToBoxes(const FVector& VoxelLocation, const float NodeExtent,
	                     const bool IsOccluded);
	void AddNodeTextInfos(const MortonCode NodeMortonCode,
	                      const LayerIndex NodeLayerIndex,
	                      const FVector& NodePosition);
	void AddVolumeTextInfos();
	// Override to draw custom filled geometry for voxels
	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views,
	                                   const FSceneViewFamily& ViewFamily,
	                                   const uint32 VisibilityMap,
	                                   FMeshElementCollector& Collector) const override;

	void RenderVoxelSurfaces(FPrimitiveDrawInterface* PDI, FMeshElementCollector& Collector) const;
    void DebugDrawRegions();
    void DebugDrawRegionIds();
    void DebugDrawAdjacency();
    void DebugDrawVisibility(int32 ViewerRegionId);
    void DebugDrawBestCover(int32 ViewerRegionId);
	void DebugDrawPortals();
	void DebugDrawRegionInfo(int32 RegionId);
	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override;
	
	TWeakObjectPtr<UNav3DNavDataRenderingComponent> RenderingComponent;
	TWeakObjectPtr<ANav3DData> NavigationData;

private:
	// Surface data for filled voxel rendering
	struct FVoxelSurfaceData
	{
		FBox Bounds;
		FColor Color;
		float Opacity;

		FVoxelSurfaceData(const FBox& InBounds, const FColor& InColor, const float InOpacity = 0.1f)
			: Bounds(InBounds), Color(InColor), Opacity(InOpacity)
		{
		}
	};

	mutable TArray<FVoxelSurfaceData> VoxelSurfaces;
};

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
class NAV3D_API FNav3DDebugDrawDelegateHelper final : public FDebugDrawDelegateHelper
{
	using Super = FDebugDrawDelegateHelper;

public:
	FNav3DDebugDrawDelegateHelper() = default;

	void InitDelegateHelper(const FNav3DMeshSceneProxy* SceneProxy);

	virtual void RegisterDebugDrawDelegateInternal() override;
	virtual void UnregisterDebugDrawDelegate() override;

private:
	TWeakObjectPtr<ANav3DData> NavigationData;
};
#endif

UCLASS()
class NAV3D_API UNav3DNavDataRenderingComponent final
	: public UPrimitiveComponent
{
	GENERATED_BODY()

public:
	UNav3DNavDataRenderingComponent();
	void ForceUpdate();
	bool UpdateIsForced() const;
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
	virtual void DestroyRenderState_Concurrent() override;
	static bool IsNavigationShowFlagSet(const UWorld* World);

private:
	uint8 bForcesUpdate : 1;

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
	FNav3DDebugDrawDelegateHelper DebugDrawDelegateManager;
#endif
};

FORCEINLINE void UNav3DNavDataRenderingComponent::ForceUpdate()
{
	bForcesUpdate = true;
}

FORCEINLINE bool UNav3DNavDataRenderingComponent::UpdateIsForced() const
{
	return bForcesUpdate;
}
