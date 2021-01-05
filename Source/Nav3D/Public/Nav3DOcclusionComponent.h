#pragma once

#include "CoreMinimal.h"
#include "Nav3DStructs.h"
#include "Components/ActorComponent.h"
#include "Nav3DOcclusionComponent.generated.h"

/**
*  This component will consider the owner for dynamic octree occlusion
*/
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="Nav3D Occlusion"))
class NAV3D_API UNav3DOcclusionComponent final : public UActorComponent
{
	friend class ANav3DVolume;
	
	GENERATED_BODY()

public:	
    UNav3DOcclusionComponent();

	// The minimum owner transform delta to trigger an octree update 
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Nav3D")
	float TransformTriggerTolerance = 1.0f;

	// Allow this component's owner to be used with cover map queries
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Nav3D")
	bool bEnableCover = true;

	// Toggle whether this component should be considered for octree occlusion 
	UFUNCTION(BlueprintCallable, Category = "Nav3D")
	void SetOcclusionEnabled(bool bOcclusionEnabled);
	
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    bool UpdateVolatileEdges();
	TArray<FNav3DOctreeEdge>& GetVolatileEdges();
	void ResetCoverLocations() { CoverLocations.Reset(); }
	bool GetCoverEnabled() const { return bEnableCover; }
	bool GetCoverLocations(const int32 NormalIndex, TArray<FVector>& Locations);

protected:
    virtual void BeginPlay() override;
	void RequestUpdate();
	bool FindVolume(ANav3DVolume*& CurrentVolume) const;
	void AddCoverLocations(const int32 NormalIndex, const TArray<FVector> Locations) { CoverLocations.Add(NormalIndex, Locations);}

	UPROPERTY()
	ANav3DVolume* Volume = nullptr;

	// The stored transform of the owner, the delta is used to trigger updates
	UPROPERTY()
	FTransform CachedTransform;

	// Whether this component should be used to update the octree 
	UPROPERTY()
	bool bEnabled = true;

	TArray<FNav3DOctreeEdge> VolatileEdges;
	TMap<int32, TArray<FVector>> CoverLocations;

};
