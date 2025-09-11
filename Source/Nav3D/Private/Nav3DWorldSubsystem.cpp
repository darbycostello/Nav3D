#include "Nav3DWorldSubsystem.h"
#include "Nav3DDataChunkActor.h"

void UNav3DWorldSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	CellSize = 25600.0f;
}

void UNav3DWorldSubsystem::Deinitialize()
{
	FScopeLock Lock(&Mutex);
	Grid.Reset();
}

void UNav3DWorldSubsystem::RegisterChunkActor(ANav3DDataChunkActor* Actor)
{
	if (!Actor) return;
	FScopeLock Lock(&Mutex);
	const FBox& B = Actor->DataChunkActorBounds;
	TArray<FIntVector> Cells; GetCellsOverlapping(B, Cells);
	for (const FIntVector& C : Cells)
	{
		Grid.FindOrAdd(C).Actors.Add(Actor);
	}
}

void UNav3DWorldSubsystem::UnregisterChunkActor(ANav3DDataChunkActor* Actor)
{
	if (!Actor) return;
	FScopeLock Lock(&Mutex);
	for (auto& Pair : Grid)
	{
		Pair.Value.Actors.RemoveAllSwap([Actor](const TWeakObjectPtr<ANav3DDataChunkActor>& Ptr){ return !Ptr.IsValid() || Ptr.Get() == Actor; });
	}
}

void UNav3DWorldSubsystem::QueryActorsInBounds(const FBox& Bounds, TArray<ANav3DDataChunkActor*>& Out) const
{
	FScopeLock Lock(&Mutex);
	TArray<FIntVector> Cells; GetCellsOverlapping(Bounds, Cells);
	for (const FIntVector& C : Cells)
	{
		if (const FNav3DSpatialCell* Cell = Grid.Find(C))
		{
			for (const auto& Weak : Cell->Actors)
			{
				if (ANav3DDataChunkActor* A = Weak.Get())
				{
					if (A->DataChunkActorBounds.Intersect(Bounds))
					{
						Out.Add(A);
					}
				}
			}
		}
	}
}

void UNav3DWorldSubsystem::GetCellsOverlapping(const FBox& Bounds, TArray<FIntVector>& Out) const
{
	if (!Bounds.IsValid) return;
	const FIntVector Min = ToCell(Bounds.Min);
	const FIntVector Max = ToCell(Bounds.Max);
	for (int32 x = Min.X; x <= Max.X; ++x)
	{
		for (int32 y = Min.Y; y <= Max.Y; ++y)
		{
			Out.Add(FIntVector(x,y,0));
		}
	}
}


