#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Nav3DDataChunk.h"
#include "Nav3DTypes.h"
#include "ActorPartition/PartitionActor.h"
#include "Nav3DDataChunkActor.generated.h"

UCLASS(NotPlaceable, BlueprintType)
class NAV3D_API ANav3DDataChunkActor : public APartitionActor
{
	GENERATED_BODY()

public:
	ANav3DDataChunkActor(const FObjectInitializer& ObjectInitializer);

	// Navigation data (single source of truth)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation")
	TArray<TObjectPtr<UNav3DDataChunk>> Nav3DChunks;

	// Spatial bounds
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation")
	FBox DataChunkActorBounds;

	// Owning Nav3D bounds volume this chunk was generated from
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation")
	FBox OwningVolumeBounds;
	
	// Adjacency (baked during build)
	UPROPERTY(VisibleAnywhere, Category="Navigation")
	TArray<FNav3DChunkAdjacency> ChunkAdjacency;

	// Fast portal lookup (transient, rebuilt at runtime)
	TMap<int32, TMultiMap<uint64, FNav3DVoxelConnection>> PortalLookup;
	
	// Universal chunk management (works in all scenarios)
	void InitializeForStandardLevel(); // Non-world-partition setup
	void InitializeForWorldPartition(); // World partition setup
	
	// Navigation data access
	UFUNCTION(BlueprintCallable, Category="Navigation")
	bool ContainsPoint(const FVector& Point) const;
	
	UFUNCTION(BlueprintCallable, Category="Navigation")
	const UNav3DDataChunk* GetChunkContainingPoint(const FVector& Point) const;
	
	// Build state management
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bIsBuilt = false;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bIsBuilding = false;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bNeedsRebuild = false;

	virtual uint32 GetDefaultGridSize(UWorld* InWorld) const override;
	virtual void GetActorBounds(bool bOnlyCollidingComponents, FVector& OutOrigin, FVector& OutBoxExtent, bool bIncludeFromChildActors) const override;
	void RegisterWithNavigationSystem();
	void UnregisterFromNavigationSystem();
	
#if WITH_EDITOR
	virtual void GetStreamingBounds(FBox& OutRuntimeBounds, FBox& OutEditorBounds) const override;
	void SetDataChunkActorBounds(const FBox& InBounds);
	
	UFUNCTION(CallInEditor, meta=(DisplayName="Rebuild This Chunk", Category="Navigation"))
	void RebuildNavigationData() const;
#endif

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	void AddNav3DChunkToWorld();
	void RemoveNav3DChunkFromWorld();
};


