#pragma once

#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>
#include <GameFramework/Actor.h>
#include "Nav3DRaycaster.h"
#include "Nav3DRaycasterTest.generated.h"

class UNav3DRaycasterRenderingComponent;
class ANav3DRaycasterTest;
class USphereComponent;
class UNav3DRaycaster;

USTRUCT()
struct NAV3D_API FNav3DRaycasterDebugDrawOptions
{
	GENERATED_BODY()

	FNav3DRaycasterDebugDrawOptions()
		: bEnableDebugDraw(0), bDrawLayerNodes(0), LayerIndexToDraw(0), bDrawMortonCode(0)
	{
	}

	UPROPERTY(EditAnywhere, Category = "Debug")
	uint8 bEnableDebugDraw : 1;

	UPROPERTY(EditAnywhere, Category = "Debug")
	uint8 bDrawLayerNodes : 1;

	UPROPERTY(EditAnywhere, Category = "Debug", meta = (EditCondition = "bDrawLayerNodes",
		ClampMin = "0", UIMin = "0"))
	uint8 LayerIndexToDraw;

	UPROPERTY(EditAnywhere, Category = "Debug")
	uint8 bDrawMortonCode : 1;
};

struct NAV3D_API FNav3DRaycasterSceneProxyData final : TSharedFromThis<FNav3DRaycasterSceneProxyData>
{
	void GatherData(const ANav3DRaycasterTest& RaycasterTest);

	FNav3DRaycasterDebugInfos DebugInfos;
};

class NAV3D_API FNav3DRaycasterSceneProxy final : public FDebugRenderSceneProxy
{
public:
	FNav3DRaycasterSceneProxy(const UPrimitiveComponent& Component,
	                          const FNav3DRaycasterSceneProxyData& ProxyData);

	virtual SIZE_T GetTypeHash() const override;
	virtual FPrimitiveViewRelevance
	GetViewRelevance(const FSceneView* View) const override;

private:
	TWeakObjectPtr<ANav3DRaycasterTest> RaycasterTest;
};

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
class FNav3DRaycasterDebugDrawDelegateHelper final : public FDebugDrawDelegateHelper
{
	using Super = FDebugDrawDelegateHelper;

public:
	FNav3DRaycasterDebugDrawDelegateHelper() = default;

	void InitDelegateHelper(const FNav3DRaycasterSceneProxy* SceneProxy);

	NAV3D_API virtual void RegisterDebugDrawDelegateInternal() override;
	NAV3D_API virtual void UnregisterDebugDrawDelegate() override;
};
#endif

UCLASS()
class NAV3D_API UNav3DRaycasterRenderingComponent final : public UPrimitiveComponent
{
	GENERATED_BODY()

public:
	UNav3DRaycasterRenderingComponent() = default;

	ANav3DRaycasterTest* GetRaycasterTest() const;
	virtual void
	CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
	virtual void DestroyRenderState_Concurrent() override;
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;

	virtual FBoxSphereBounds
	CalcBounds(const FTransform& LocalToWorld) const override;

private:
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
	FNav3DRaycasterDebugDrawDelegateHelper DebugDrawDelegateManager;
#endif
};

UCLASS(meta=(DisplayName="Nav3D Raycaster Test"))
class NAV3D_API ANav3DRaycasterTest final : public AActor
{
	GENERATED_BODY()

public:
	ANav3DRaycasterTest();

	const FNav3DRaycasterDebugInfos& GetDebugInfos() const;
	const FNav3DRaycasterDebugDrawOptions& GetDebugDrawOptions() const;

	FBoxSphereBounds GetBoundingBoxContainingOtherActorAndMe() const;

#if WITH_EDITOR
	virtual void PreEditChange(FProperty* PropertyAboutToChange) override;
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditMove(bool IsFinished) override;
#endif

	virtual void BeginDestroy() override;

private:
	void UpdateDrawing() const;

	UFUNCTION(CallInEditor)
	void DoRaycast();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components", meta = (AllowPrivateAccess = "true"))
	USphereComponent* SphereComponent;

#if WITH_EDITORONLY_DATA
	UPROPERTY(Transient)
	UNav3DRaycasterRenderingComponent* RenderingComponent;
#endif

	UPROPERTY(Instanced, EditAnywhere, Category = "Raycasting")
	UNav3DRaycaster* Raycaster;

	UPROPERTY(EditInstanceOnly, Category = "Raycasting")
	ANav3DRaycasterTest* OtherActor;

	UPROPERTY(EditAnywhere, Category = "Navigation")
	FNavAgentProperties NavAgentProperties;

	UPROPERTY(EditAnywhere, Category = "Raycasting")
	uint8 bUpdatePathAfterMoving : 1;

	UPROPERTY(EditAnywhere, Category = "Debug")
	FNav3DRaycasterDebugDrawOptions DebugDrawOptions;

	FNav3DRaycasterDebugInfos RaycasterDebugInfos;
};

FORCEINLINE const FNav3DRaycasterDebugInfos& ANav3DRaycasterTest::GetDebugInfos() const
{
	return RaycasterDebugInfos;
}

FORCEINLINE const FNav3DRaycasterDebugDrawOptions& ANav3DRaycasterTest::GetDebugDrawOptions() const
{
	return DebugDrawOptions;
}
