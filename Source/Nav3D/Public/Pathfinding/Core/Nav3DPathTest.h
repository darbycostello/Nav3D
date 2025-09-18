#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Pathfinding/Core/Nav3DPath.h"
#include "Pathfinding/Core/Nav3DPathingTypes.h"
#include "DebugRenderSceneProxy.h"
#include "Nav3DPathTest.generated.h"

class ANav3DData;
class USphereComponent;
class UNav3DPathTestRenderingComponent;

USTRUCT()
struct NAV3D_API FNav3DPathTestDebugDrawOptions
{
	GENERATED_BODY()

	FNav3DPathTestDebugDrawOptions()
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

struct NAV3D_API FNav3DPathTestSceneProxyData final
	: public TSharedFromThis<FNav3DPathTestSceneProxyData, ESPMode::ThreadSafe>
{
	void GatherData(const ANav3DPathTest& PathTest);

	FVector StartLocation;
	FVector EndLocation;
	ENavigationQueryResult::Type PathFindingResult;
	FNav3DPath NavigationPath;
};

class NAV3D_API FNav3DPathTestSceneProxy final
	: public FDebugRenderSceneProxy
{
public:
	FNav3DPathTestSceneProxy(
		const UPrimitiveComponent& Component,
		const FNav3DPathTestSceneProxyData& ProxyData);

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
	FNav3DPathTestDebugDrawOptions DebugDrawOptions;
	TWeakObjectPtr<ANav3DPathTest> PathTest;
	TWeakObjectPtr<UNav3DPathTestRenderingComponent> RenderingComponent;
	TArray<TPair<FVector, FVector>> ArrowHeadLocations;
};

UCLASS(Blueprintable, meta=(DisplayName="Nav3D Path Test"))
class NAV3D_API ANav3DPathTest : public AActor
{
	GENERATED_BODY()

public:
	ANav3DPathTest();

#if WITH_EDITOR
	virtual void PreEditChange(FProperty* PropertyToChange) override;
	virtual void PostEditChangeProperty(
		FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditMove(bool IsFinished) override;
#endif

	FVector GetStartLocation() const;
	FVector GetEndLocation() const;
	const FNav3DPathTestDebugDrawOptions& GetDebugDrawOptions() const;
	ENavigationQueryResult::Type GetPathFindingResult() const;
	const FNav3DPath& GetNavigationPath() const;
	virtual void BeginDestroy() override;

	UFUNCTION(CallInEditor, Category="Nav3D", meta = (DisplayName = "Find Path"))
	void FindPath();

	UFUNCTION(CallInEditor, Category="Nav3D", meta = (DisplayName = "Clear Path"))
	void ClearPath();

private:
	void UpdateDrawing() const;

    UPROPERTY(EditAnywhere, Category="Nav3D")
    FNavAgentProperties NavAgentProperties;

	UPROPERTY(EditInstanceOnly, Category="Nav3D")
	ANav3DPathTest* OtherActor;

	UPROPERTY(EditAnywhere, Category="Nav3D")
	ENav3DPathingAlgorithm Algorithm = ENav3DPathingAlgorithm::LazyThetaStar;

	UPROPERTY(EditAnywhere, Category="Nav3D")
	uint8 bUpdatePathAfterMoving : 1;

	UPROPERTY(EditAnywhere, Category="Nav3D")
	FNav3DPathTestDebugDrawOptions DebugDrawOptions;

protected:
	virtual void BeginPlay() override;

private:
    void TryUpdatePath();
    void UpdateReciprocalLink();

	// cached
	FNav3DPath LastPath;
	ENavigationQueryResult::Type LastResult = ENavigationQueryResult::Invalid;
	float TimeSinceLastUpdate = 0.0f;

    // track last request to evaluate completeness
    FVector LastRequestedStart = FVector::ZeroVector;
    FVector LastRequestedEnd = FVector::ZeroVector;
    bool bLastPathReachedTarget = false;

    UPROPERTY(VisibleAnywhere, Category="Nav3D|Test")
    TObjectPtr<USphereComponent> Sphere;

#if WITH_EDITORONLY_DATA
    UPROPERTY(Transient)
    TObjectPtr<UNav3DPathTestRenderingComponent> RenderingComponent;
#endif
};

FORCEINLINE FVector ANav3DPathTest::GetStartLocation() const
{
	return GetActorLocation();
}

FORCEINLINE FVector ANav3DPathTest::GetEndLocation() const
{
	return OtherActor != nullptr
		       ? OtherActor->GetActorLocation()
		       : FVector::ZeroVector;
}

FORCEINLINE const FNav3DPathTestDebugDrawOptions&
ANav3DPathTest::GetDebugDrawOptions() const
{
	return DebugDrawOptions;
}

FORCEINLINE ENavigationQueryResult::Type
ANav3DPathTest::GetPathFindingResult() const
{
	return LastResult;
}

FORCEINLINE const FNav3DPath&
ANav3DPathTest::GetNavigationPath() const
{
	return LastPath;
}


