#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Nav3DTypes.h"
#include "Nav3DDynamicOcclusion.generated.h"

class ANav3DData;

USTRUCT()
struct FVoxelOcclusionData
{
	GENERATED_BODY()

	FVoxelOcclusionData()
	{
	}

	TMap<LeafIndex, uint64> OccludedLeafNodes;
	FTransform CachedTransform;

	void Reset()
	{
		OccludedLeafNodes.Reset();
		CachedTransform = FTransform::Identity;
	}
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class NAV3D_API UNav3DDynamicOcclusion : public UActorComponent
{
	GENERATED_BODY()

public:
	UNav3DDynamicOcclusion();
	virtual void OnRegister() override;
	void AttemptRegistration();
	void UpdateSpatiallyLoaded(const ANav3DData* NavData) const;
	virtual void OnUnregister() override;
	void RegisterOwner(ANav3DData* NavData);
	void UnregisterOwner(ANav3DData* NavData);
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

private:
	UPROPERTY()
	TArray<TObjectPtr<ANav3DData>> RegisteredNavData;
	TMap<const ANav3DData*, FVoxelOcclusionData> OcclusionDataMap;
	FBox PreviousBounds;
	FCollisionShape CollisionShape;
};
