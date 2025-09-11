#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Nav3DWorldSubsystem.generated.h"

class ANav3DDataChunkActor;

USTRUCT()
struct FNav3DSpatialCell
{
	GENERATED_BODY()

	TArray<TWeakObjectPtr<ANav3DDataChunkActor>> Actors;
};

UCLASS()
class NAV3D_API UNav3DWorldSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	// Configurable via settings later; defaults derived from grid size
	UPROPERTY(EditAnywhere, Category="Nav3D")
	float CellSize = 50000.0f;

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	void RegisterChunkActor(ANav3DDataChunkActor* Actor);
	void UnregisterChunkActor(ANav3DDataChunkActor* Actor);

	// Query neighbor actors whose bounds intersect the search box
	void QueryActorsInBounds(const FBox& Bounds, TArray<ANav3DDataChunkActor*>& Out) const;

private:
	mutable FCriticalSection Mutex;
	TMap<FIntVector, FNav3DSpatialCell> Grid;

	FIntVector ToCell(const FVector& P) const
	{
		return FIntVector(
			FMath::FloorToInt(P.X / CellSize),
			FMath::FloorToInt(P.Y / CellSize),
			0);
	}

	void GetCellsOverlapping(const FBox& Bounds, TArray<FIntVector>& Out) const;
};


