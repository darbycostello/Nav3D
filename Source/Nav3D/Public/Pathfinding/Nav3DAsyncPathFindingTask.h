#pragma once
#include "Nav3DData.h"
#include "Nav3DPath.h"
#include "GameplayTask.h"
#include "GameplayTasksComponent.h"
#include "GameFramework/NavMovementComponent.h"
#include "Nav3DAsyncPathFindingTask.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FNav3DPathfindingCompleteDelegate, bool, bSuccess, const FNav3DPathData&,
                                             PathData);

UCLASS()
class NAV3D_API UNav3DAsyncPathfindingTask : public UGameplayTask
{
	// Required so task doesn't get garbage collected while async work is happening
	virtual void OnDestroy(bool bInOwnerFinished) override
	{
		// Only allow destruction after task is done
		if (bTaskComplete)
		{
			Super::OnDestroy(bInOwnerFinished);
		}
	}

	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable)
	FNav3DPathfindingCompleteDelegate OnComplete;

	UFUNCTION(BlueprintCallable, Category="Nav3D",
		meta=(WorldContext="WorldContextObject", BlueprintInternalUseOnly="true"))
	static UNav3DAsyncPathfindingTask* FindPathAsync(
		UObject* WorldContextObject,
		const FVector StartLoc,
		const FVector EndLoc,
		AActor* PathfindingContext,
		TSubclassOf<UNavigationQueryFilter> FilterClass = nullptr);

	virtual void Activate() override;

private:
	UPROPERTY()
	TWeakObjectPtr<UGameplayTasksComponent> TaskOwner;

	UPROPERTY()
	TWeakObjectPtr<UNavMovementComponent> MovementComponent;

	TWeakObjectPtr<UWorld> WorldPtr;
	TSubclassOf<UNavigationQueryFilter> QueryFilterClass;
	bool bTaskComplete = false;
	FVector StartLocation;
	FVector EndLocation;
	FNavAgentProperties NavAgentProperties;
	TSharedPtr<FNav3DPath> PathResult;
};
