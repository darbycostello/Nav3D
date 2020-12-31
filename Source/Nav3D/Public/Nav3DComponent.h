#pragma once

#include "Nav3DStructs.h"
#include "Components/ActorComponent.h"
#include "Nav3DComponent.generated.h"

class ANav3DVolume;
struct FNavigationPath;
struct FNav3DOctreeEdge;

DECLARE_DYNAMIC_DELEGATE_OneParam(FFindPathTaskCompleteDynamicDelegate, bool, bPathFound);

UCLASS(BlueprintType, Blueprintable, meta=(BlueprintSpawnableComponent, DisplayName="Nav3D Component"))
class NAV3D_API UNav3DComponent final : public UActorComponent
{
	GENERATED_BODY()

public:

	// The heuristic to use for scoring during pathfinding
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
    ENav3DHeuristic Heuristic = ENav3DHeuristic::Euclidean;
	
	// Making this value greater than 1 will make the algorithm "greedy"
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
	float HeuristicWeight = 5.0f;

	// Making this value greater than 1 will make the algorithm prefer larger-sized nodes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
	float NodeSizePreference = 1.0f;

	// The heuristic to use for scoring during pathfinding
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
	ENav3DPathPruning PathPruning = ENav3DPathPruning::None;

	// Number of times to iterate Catmull-Rom smoothing on navigation paths 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
    int32 PathSmoothing = 3;

	// Whether to debug draw the path from a pathfinding task 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    bool bDebugDrawNavPath;

	// The navigation path debug colour
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    FColor DebugPathColour = FColor(0, 255, 255);

	// The navigation path thickness
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
	float DebugPathLineScale = 10.0f;
	
	UNav3DComponent(const FObjectInitializer& ObjectInitializer);
	FNav3DPathSharedPtr Nav3DPath;
	
	const ANav3DVolume* GetCurrentVolume() const { return Volume; }
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	void ExecutePathFinding(const FNav3DOctreeEdge& StartEdge, const FNav3DOctreeEdge& TargetEdge, const FVector& StartLocation, const FVector& TargetLocation, FNav3DPathFindingConfig Config, FNav3DPath& Path);
	float HeuristicScore(FNav3DOctreeEdge StartEdge, FNav3DOctreeEdge TargetEdge, FNav3DPathFindingConfig Config) const;
	void ApplyPathPruning(FNav3DPath& Path, const FNav3DPathFindingConfig Config) const;
	static void ApplyPathSmoothing(FNav3DPath& Path, FNav3DPathFindingConfig Config);
	void DebugDrawNavPath(const FNav3DPath& Path) const;	
	
	UFUNCTION(BlueprintCallable, Category = "Nav3D")
	void FindPath(
		const FVector& StartLocation,
		const FVector& TargetLocation,
		FFindPathTaskCompleteDynamicDelegate OnComplete,
		ENav3DPathFindingCallResult &Result);
	
	UFUNCTION(BlueprintCallable, Category = "Nav3D")
	void GetPath(TArray<FVector> &Path) const { Nav3DPath->GetPath(Path); }

protected:
	virtual void BeginPlay() override;
	ANav3DVolume* Volume;
	bool VolumeContainsOctree() const;
	bool VolumeContainsOwner() const;
	bool FindVolume();
};
