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
	
	virtual void Serialize(FArchive& Ar) override;
	virtual void PostLoad() override;

	// Navigation data (single source of truth)
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation")
	TArray<TObjectPtr<UNav3DDataChunk>> Nav3DChunks;

	// Spatial bounds
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Navigation")
	FBox DataChunkActorBounds;
	
	// Adjacency (baked during build)
	UPROPERTY(VisibleAnywhere, Category="Navigation")
	TArray<FNav3DChunkAdjacency> ChunkAdjacency;
	
	// Compact tactical data (new format, serialized with the chunk actor)
	UPROPERTY(VisibleAnywhere, Category="Tactical")
	FCompactTacticalData CompactTacticalData;

	// Store compact regions built directly from coordinates
	UPROPERTY()
	TArray<FCompactRegion> CompactRegions;

	// Boundary connection interfaces to neighboring chunks (serialized)
	UPROPERTY(VisibleAnywhere, Category="Tactical")
	TMap<FVector, FChunkConnectionInterface> ConnectionInterfaces;

	// Fast portal lookup removed; iterate CompactPortals via ChunkAdjacency instead
	
	// Universal chunk management (works in all scenarios)
	void InitializeForStandardLevel(); // Non-world-partition setup
	void InitializeForWorldPartition(); // World partition setup
	
	// Navigation data access
	UFUNCTION(BlueprintCallable, Category="Navigation")
	bool ContainsPoint(const FVector& Point) const;
	
	// Tactical data access
	UFUNCTION(BlueprintCallable, Category="Tactical")
	bool HasTacticalData() const { return !CompactTacticalData.IsEmpty(); }
	
	UFUNCTION(BlueprintCallable, Category="Tactical")
	bool HasCompactTacticalData() const { return !CompactTacticalData.IsEmpty(); }
	
	UFUNCTION(BlueprintCallable, Category="Tactical")
	int32 GetTacticalRegionCount() const 
	{ 
		return CompactTacticalData.Regions.Num(); 
	}

	// Get region by index (works with both old and new formats)
	const FCompactRegion* GetTacticalRegion(uint8 RegionIndex) const
	{
		return CompactTacticalData.GetRegion(RegionIndex);
	}
	
	// Check if this chunk is adjacent to another chunk
	bool IsAdjacentToChunk(const ANav3DDataChunkActor* OtherChunk, float Tolerance = 10.0f) const;

	/** Clear all tactical data from this chunk (called before tactical rebuild) */
	void ClearTacticalData();
	
	// Build state management
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bIsBuilt = false;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bIsBuilding = false;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bNeedsRebuild = false;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Build Status")
	bool bNeedsTacticalDataBuild = false;

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


