#pragma once

#include "Nav3DStructs.h"
#include "Components/ActorComponent.h"
#include "Nav3DComponent.generated.h"

DECLARE_DYNAMIC_DELEGATE_OneParam(FFindPathTaskCompleteDynamicDelegate, bool, bPathFound);
DECLARE_DYNAMIC_DELEGATE_OneParam(FFindLineOfSightTaskCompleteDynamicDelegate, bool, bLineOfSightFound);
DECLARE_DYNAMIC_DELEGATE_OneParam(FFindCoverTaskCompleteDynamicDelegate, bool, bLocationFound);

UCLASS(BlueprintType, Blueprintable, meta=(BlueprintSpawnableComponent, DisplayName="Nav3D Component"))
class NAV3D_API UNav3DComponent final : public UActorComponent
{
	friend class ANav3DVolume;
	
	GENERATED_BODY()

public:

	// The heuristic to use for scoring during pathfinding
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
    ENav3DHeuristic Heuristic = ENav3DHeuristic::Manhattan;
	
	// Making this value greater than 1 will make the algorithm "greedy"
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
	float HeuristicWeight = 5.0f;

	// Making this value greater than 1 will make the algorithm prefer larger-sized nodes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
	float NodeSizePreference = 1.0f;

	// The heuristic to use for scoring during pathfinding
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
	ENav3DPathPruning PathPruning = ENav3DPathPruning::WithClearance;

	// Number of times to iterate Catmull-Rom smoothing on navigation paths 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Pathfinding")
    int32 PathSmoothing = 5;

#if WITH_EDITORONLY_DATA
	// Whether to debug draw the pathfinding paths and cover locations 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    bool bDebugDrawEnabled;

	// The navigation path debug colour
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    FColor DebugPathColour = FColor(0, 255, 255);

	// The navigation path thickness
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
	float DebugPathLineScale = 10.0f;

	// Whether to log the pathfinding task process in the console. 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
	bool bDebugLogPathfinding;

	// Whether to log the find cover task process in the console. 
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
	bool bDebugFindCover;
#endif
	
	UNav3DComponent(const FObjectInitializer& ObjectInitializer);
	FNav3DPathSharedPtr Nav3DPath;
	FNav3DCoverLocationSharedPtr Nav3DCoverLocation;

	const ANav3DVolume* GetCurrentVolume() const { return Volume; }
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	void ExecutePathFinding(const FNav3DOctreeEdge& StartEdge, const FNav3DOctreeEdge& TargetEdge, const FVector& StartLocation, const FVector& TargetLocation, FNav3DPathFindingConfig Config, FNav3DPath& Path);
	float HeuristicScore(FNav3DOctreeEdge StartEdge, FNav3DOctreeEdge TargetEdge, FNav3DPathFindingConfig Config) const;
	void AddPathStartLocation(FNav3DPath& Path) const;
	void ApplyPathPruning(FNav3DPath& Path, const FNav3DPathFindingConfig Config) const;
	void ApplyPathLineOfSight(FNav3DPath& Path, AActor* Target, float MinimumDistance) const;
	static void ApplyPathSmoothing(FNav3DPath& Path, FNav3DPathFindingConfig Config);
	void RequestNavPathDebugDraw(const FNav3DPath Path) const;
	void RequestNavCoverLocationDebugDraw(const FNav3DCoverLocation CoverLocation) const;
	void ExecuteFindCover(
		const FVector Location,
		const float Radius,
		const TArray<AActor*> Opponents,
		TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypes,
		const ENav3DCoverSearchType SearchType,
		const bool bPerformLineTraces,
		FNav3DCoverLocation& CoverLocation) const;
	
	UFUNCTION(BlueprintCallable, Category = "Nav3D")
	void FindPath(
		const FVector& StartLocation,
		const FVector& TargetLocation,
		const bool bCheckLineOfSight,
		FFindPathTaskCompleteDynamicDelegate OnComplete,
		ENav3DPathFindingCallResult& Result);

	UFUNCTION(BlueprintCallable, Category = "Nav3D")
	void FindCover(
		FVector SearchOrigin,
		float MaxRadius,
		AActor* Opponent,
		ENav3DCoverSearchType SearchType,
		const bool bPerformLineTraces,
		FFindCoverTaskCompleteDynamicDelegate OnComplete,
		ENav3DFindCoverCallResult& Result
	);

	UFUNCTION(BlueprintCallable, Category = "Nav3D")
    void FindCoverMultipleOpponents(
        FVector SearchOrigin,
        float MaxRadius,
        TArray<AActor*> Opponents,
        ENav3DCoverSearchType SearchType,
        const bool bPerformLineTraces,
        FFindCoverTaskCompleteDynamicDelegate OnComplete,
        ENav3DFindCoverCallResult& Result
    );

	UFUNCTION(BlueprintCallable, Category = "Nav3D")
    void FindLineOfSight(
        const FVector& StartLocation,
        AActor* TargetActor,
        const float MinimumDistance,
        const bool bCheckLineOfSight,
        FFindLineOfSightTaskCompleteDynamicDelegate OnComplete,
        ENav3DFindLineOfSightCallResult& Result);
	
	UFUNCTION(BlueprintCallable, Category = "Nav3D")
	void GetPath(TArray<FVector>& Path) const { Nav3DPath->GetPath(Path); }

	UFUNCTION(BlueprintCallable, Category = "Nav3D")
    void GetCoverLocation(FNav3DCoverLocation& CoverLocation) const { Nav3DCoverLocation->GetLocation(CoverLocation); }

protected:
	virtual void BeginPlay() override;

	UPROPERTY()
	ANav3DVolume* Volume;

	bool VolumeContainsOctree() const;
	bool VolumeContainsOwner() const;
	bool VolumeCoverMapEnabled() const;
	bool VolumeCoverMapExists() const;
	bool FindVolume();
};
