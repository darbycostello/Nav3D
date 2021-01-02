#pragma once

#include "CoreMinimal.h"
#include "Nav3DStructs.h"
#include "Components/ActorComponent.h"
#include "Nav3DOcclusionComponent.generated.h"

class ANav3DVolume;

/**
*  This component will consider the owner for dynamic octree occlusion
*/
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent, DisplayName="Nav3D Dynamic Occlusion"))
class NAV3D_API UNav3DOcclusionComponent final : public UActorComponent
{
	GENERATED_BODY()

public:	
    UNav3DOcclusionComponent();

	// How often to tick this component 
	UPROPERTY(BlueprintReadWrite, Category = "Nav3D Occlusion Component")
    float TickInterval = 0.5f;

	// The minimum owner transform delta to trigger an octree update 
	UPROPERTY(BlueprintReadWrite, Category = "Nav3D Occlusion Component")
	float TransformTriggerTolerance = 1.0f;

	// Toggle whether this component should be considered for octree occlusion 
	UFUNCTION(BlueprintCallable, Category = "Nav3D")
	void SetOcclusionEnabled(bool bOcclusionEnabled);
	
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    bool UpdateVolatileEdges();
	TArray<FNav3DOctreeEdge>& GetVolatileEdges();

protected:
    virtual void BeginPlay() override;
	void RequestUpdate();
	bool FindVolume(ANav3DVolume*& CurrentVolume) const;

	UPROPERTY()
	ANav3DVolume* Volume = nullptr;

	TArray<FNav3DOctreeEdge> VolatileEdges;

private:

	// The stored transform of the owner, the delta is used to trigger updates
	UPROPERTY()
	FTransform CachedTransform;

	// Whether this component should be used to update the octree 
	UPROPERTY()
    bool bEnabled = true;
};
