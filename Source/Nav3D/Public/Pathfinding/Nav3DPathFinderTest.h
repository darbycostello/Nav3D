#pragma once

#include "PathFinding/Search/Nav3DPathFindingSearch.h"

#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>
#include <GameFramework/Actor.h>
#include "Nav3DPathFinderTest.generated.h"

class FNav3DPathFinder;
class USphereComponent;
class ANav3DPathFinderTest;
class UNav3DPathFindingRenderingComponent;

USTRUCT()
struct NAV3D_API FNav3DPathRenderingDebugDrawOptions
{
	GENERATED_BODY()

	FNav3DPathRenderingDebugDrawOptions()
		: bDrawOnlyWhenSelected(false), bDrawNodes(true), bDrawConnections(true),
		  bDrawCosts(false), bDrawLastProcessedNode(true),
		  bDrawLastProcessedNeighbours(true), bDrawBestPath(true)
	{
	}

	UPROPERTY(EditAnywhere)
	uint8 bDrawOnlyWhenSelected : 1;

	UPROPERTY(EditAnywhere)
	uint8 bDrawNodes : 1;

	UPROPERTY(EditAnywhere)
	uint8 bDrawConnections : 1;

	UPROPERTY(EditAnywhere)
	uint8 bDrawCosts : 1;

	UPROPERTY(EditAnywhere)
	uint8 bDrawLastProcessedNode : 1;

	UPROPERTY(EditAnywhere)
	uint8 bDrawLastProcessedNeighbours : 1;

	UPROPERTY(EditAnywhere)
	uint8 bDrawBestPath : 1;
};

struct NAV3D_API FNav3DPathFindingSceneProxyData final
	: public TSharedFromThis<FNav3DPathFindingSceneProxyData,
	                         ESPMode::ThreadSafe>
{
	void GatherData(const ANav3DPathFinderTest& PathFinderTest);

	FVector StartLocation;
	FVector EndLocation;
	TOptional<EGraphAStarResult> PathFindingResult;
	FNav3DPath NavigationPath;
};

class NAV3D_API FNav3DPathFindingSceneProxy final
	: public FDebugRenderSceneProxy
{
public:
	FNav3DPathFindingSceneProxy(
		const UPrimitiveComponent& Component,
		const FNav3DPathFindingSceneProxyData& ProxyData);

	virtual SIZE_T GetTypeHash() const override;
	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override;
	virtual void
	GetDynamicMeshElements(const TArray<const FSceneView*>& Views,
	                       const FSceneViewFamily& ViewFamily,
	                       uint32 VisibilityMap,
	                       FMeshElementCollector& Collector) const override;

private:
	bool SafeIsActorSelected() const;

	AActor* ActorOwner;
	FNav3DPathRenderingDebugDrawOptions DebugDrawOptions;
	TWeakObjectPtr<ANav3DPathFinderTest> PathFinderTest;
	TWeakObjectPtr<UNav3DPathFindingRenderingComponent> RenderingComponent;
	TArray<TPair<FVector, FVector>> ArrowHeadLocations;
};

UCLASS(meta=(DisplayName="Nav3D PathFinder Test"))
class NAV3D_API ANav3DPathFinderTest final : public AActor
{
	GENERATED_BODY()

public:
	ANav3DPathFinderTest();

#if WITH_EDITOR
	virtual void PreEditChange(FProperty* PropertyToChange) override;
	virtual void PostEditChangeProperty(
		FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditMove(bool IsFinished) override;
#endif

	FVector GetStartLocation() const;
	FVector GetEndLocation() const;
	const FNav3DPathRenderingDebugDrawOptions& GetDebugDrawOptions() const;
	EGraphAStarResult GetPathFindingResult() const;
	const FNav3DPath& GetNavigationPath() const;
	virtual void BeginDestroy() override;

private:
	void UpdateDrawing() const;

	UFUNCTION(CallInEditor, meta = (DisplayName = "Find Path"))
	void FindPath();

	UFUNCTION(CallInEditor, meta = (DisplayName = "Clear Paths"))
	void ClearPaths();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	USphereComponent* SphereComponent;

#if WITH_EDITORONLY_DATA
	UPROPERTY(Transient)
	UNav3DPathFindingRenderingComponent* RenderingComponent;
#endif

	UPROPERTY(EditAnywhere)
	uint8 bUpdatePathAfterMoving : 1;

	UPROPERTY(EditAnywhere)
	FNavAgentProperties NavAgentProperties;

	UPROPERTY(EditAnywhere)
	TSubclassOf<UNavigationQueryFilter> NavigationQueryFilter;

	UPROPERTY(EditAnywhere)
	FNav3DPathRenderingDebugDrawOptions DebugDrawOptions;

	UPROPERTY(EditInstanceOnly)
	ANav3DPathFinderTest* OtherActor;

	FNav3DPath NavigationPath;

	UPROPERTY(VisibleInstanceOnly, AdvancedDisplay)
	FNav3DPathFinderDebugData PathFinderDebugInfos;
	EGraphAStarResult PathFindingResult;
};

FORCEINLINE FVector ANav3DPathFinderTest::GetStartLocation() const
{
	return GetActorLocation();
}

FORCEINLINE FVector ANav3DPathFinderTest::GetEndLocation() const
{
	return OtherActor != nullptr
		       ? OtherActor->GetActorLocation()
		       : FVector::ZeroVector;
}

FORCEINLINE const FNav3DPathRenderingDebugDrawOptions&
ANav3DPathFinderTest::GetDebugDrawOptions() const
{
	return DebugDrawOptions;
}

FORCEINLINE EGraphAStarResult
ANav3DPathFinderTest::GetPathFindingResult() const
{
	return PathFindingResult;
}

FORCEINLINE const FNav3DPath&
ANav3DPathFinderTest::GetNavigationPath() const
{
	return NavigationPath;
}
