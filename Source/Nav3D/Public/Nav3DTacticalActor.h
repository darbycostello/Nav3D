// ReSharper disable CppUEBlueprintCallableFunctionUnused
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ActorPartition/PartitionActor.h"
#include "Pathfinding/Nav3DCrossVolumeGraph.h"
#include "Tactical/Nav3DTacticalTypes.h"
#include "Nav3DTacticalActor.generated.h"

UCLASS(NotPlaceable, BlueprintType)
class NAV3D_API ANav3DTacticalActor : public APartitionActor
{
	GENERATED_BODY()

public:
	explicit ANav3DTacticalActor(const FObjectInitializer& ObjectInitializer);

	// Tactical data (single source of truth)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Tactical")
	FNav3DTacticalData TacticalData;

	// Spatial bounds
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Tactical")
	FBox TacticalActorBounds;

	// Owning Nav3D bounds volume this tactical actor represents
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Tactical")
	FBox OwningVolumeBounds;
	
	// Build state management
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bIsBuilt = false;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bIsBuilding = false;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bNeedsRebuild = false;

	// Cross-volume adjacency graph (always built; serialized via owner Serialize)
	FNav3DCrossVolumeGraph CrossVolumeGraph;

	// Build cross-volume graph from a list of chunk actors
	void BuildCrossVolumeGraph(const TArray<ANav3DDataChunkActor*>& ChunkActors) { CrossVolumeGraph.BuildGraph(ChunkActors); }

	// Accessor
	const FNav3DCrossVolumeGraph& GetCrossVolumeGraph() const { return CrossVolumeGraph; }

	// Tactical data access
	UFUNCTION(BlueprintCallable, Category="Tactical")
	bool ContainsPoint(const FVector& Point) const;
	
	UFUNCTION(BlueprintCallable, Category="Tactical")
	bool GetRegionContainingPoint(const FVector& Point, FNav3DRegion& OutRegion) const;

	virtual uint32 GetDefaultGridSize(UWorld* InWorld) const override;
	virtual void GetActorBounds(bool bOnlyCollidingComponents, FVector& OutOrigin, FVector& OutBoxExtent, bool bIncludeFromChildActors) const override;
	void RegisterWithNavigationSystem();
	void UnregisterFromNavigationSystem();
	
#if WITH_EDITOR
	virtual void GetStreamingBounds(FBox& OutRuntimeBounds, FBox& OutEditorBounds) const override;
	void SetTacticalActorBounds(const FBox& InBounds);
	
	UFUNCTION(CallInEditor, meta=(DisplayName="Rebuild Tactical Data", Category="Tactical"))
	void RebuildTacticalData() const;
#endif

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Serialize(FArchive& Ar) override;
    virtual void PostLoad() override;

	void AddTacticalDataToWorld();
	void RemoveTacticalDataFromWorld();
};
