#include "Nav3DDynamicOcclusion.h"
#include "Nav3D.h"
#include "Nav3DData.h"
#include "NavigationSystem.h"
#include "Nav3DDataChunkActor.h"

UNav3DDynamicOcclusion::UNav3DDynamicOcclusion()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickInterval = 0.0f;
	UActorComponent::SetComponentTickEnabled(true);
}

void UNav3DDynamicOcclusion::OnRegister()
{
	Super::OnRegister();

	// First disable default navigation affect
	if (const AActor* Owner = GetOwner())
	{
		TArray<UPrimitiveComponent*> PrimitiveComponents;
		Owner->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
		for (UPrimitiveComponent* PrimComponent : PrimitiveComponents)
		{
			if (PrimComponent)
			{
				PrimComponent->SetCanEverAffectNavigation(false);
			}
		}
	}

	AttemptRegistration();
}

void UNav3DDynamicOcclusion::AttemptRegistration()
{
	RegisteredNavData.Empty();
	OcclusionDataMap.Empty();

	bool bFoundNavData = false;

	if (UWorld* World = GetWorld())
	{
		if (const UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
		{
			// Important: Log the state of the navigation system
			UE_LOG(LogNav3D, Verbose, TEXT("[%s] Attempting registration - NavSystem: %s, NavData count: %d"),
			       *GetOwner()->GetActorNameOrLabel(),
			       *NavSys->GetName(),
			       NavSys->NavDataSet.Num());

			for (ANavigationData* NavData : NavSys->NavDataSet)
			{
				if (ANav3DData* Nav3DData = Cast<ANav3DData>(NavData))
				{
					UE_LOG(LogNav3D, Verbose, TEXT("[%s] Found Nav3DData: %s"),
					       *GetOwner()->GetActorNameOrLabel(),
					       *Nav3DData->GetName());

					UpdateSpatiallyLoaded(Nav3DData);
					RegisterOwner(Nav3DData);
					bFoundNavData = true;
				}
			}
		}
		else
		{
			UE_LOG(LogNav3D, Error, TEXT("[%s] Navigation system not found"),
			       *GetOwner()->GetActorNameOrLabel());
		}
	}
	else
	{
		UE_LOG(LogNav3D, Error, TEXT("[%s] World not found"),
		       *GetOwner()->GetActorNameOrLabel());
	}

	if (!bFoundNavData)
	{
		UE_LOG(LogNav3D, Warning, TEXT("[%s] Nav3DData not found, will retry registration..."),
		       *GetOwner()->GetActorNameOrLabel());

		// Set a timer to retry registration
		FTimerHandle RetryHandle;
		GetWorld()->GetTimerManager().SetTimer(RetryHandle, this,
		                                       &UNav3DDynamicOcclusion::AttemptRegistration, 0.5f, false);
	}
	else
	{
		UE_LOG(LogNav3D, Verbose, TEXT("[%s] Registration complete - Map Size: %d"),
		       *GetOwner()->GetActorNameOrLabel(),
		       OcclusionDataMap.Num());
	}
}

void UNav3DDynamicOcclusion::UpdateSpatiallyLoaded(const ANav3DData* NavData) const
{
	// GetIsSpatiallyLoaded/SetIsSpatiallyLoaded are editor-only APIs in some UE5 versions
	// Skip this functionality in non-editor builds
#if WITH_EDITOR
	if (!NavData)
	{
		return;
	}

	if (AActor* Owner = GetOwner())
	{
		const bool bOwnerSpatiallyLoaded = Owner->GetIsSpatiallyLoaded();
		const bool bNavDataSpatiallyLoaded = NavData->GetIsSpatiallyLoaded();

		if (bOwnerSpatiallyLoaded != bNavDataSpatiallyLoaded)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("[%s] spatially loaded: %hhd, Nav3DData spatially loaded: %hhd"),
			       *GetOwner()->GetActorNameOrLabel(), bOwnerSpatiallyLoaded, bNavDataSpatiallyLoaded);

			Owner->SetIsSpatiallyLoaded(bNavDataSpatiallyLoaded);
		}
	}
#endif
}

void UNav3DDynamicOcclusion::OnUnregister()
{
	if (UWorld* World = GetWorld())
	{
		if (const UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World))
		{
			for (ANavigationData* NavData : NavSys->NavDataSet)
			{
				if (ANav3DData* Nav3DData = Cast<ANav3DData>(NavData))
				{
					UnregisterOwner(Nav3DData);
				}
			}
		}
	}

	Super::OnUnregister();
}

void UNav3DDynamicOcclusion::RegisterOwner(ANav3DData* NavData)
{
	if (!NavData)
	{
		UE_LOG(LogNav3D, Verbose, TEXT("[%s] RegisterOwner called with null NavData"),
		       *GetOwner()->GetActorNameOrLabel());
		return;
	}

	UE_LOG(LogNav3D, Verbose, TEXT("[%s] RegisterOwner - NavData: %s"),
	       *GetOwner()->GetActorNameOrLabel(),
	       *NavData->GetName());

	if (!RegisteredNavData.Contains(NavData))
	{
		RegisteredNavData.Add(NavData);

		const FTransform InitialTransform = GetOwner()->GetActorTransform();
		const FBox InitialBounds = GetOwner()->GetComponentsBoundingBox(true);

		FVoxelOcclusionData OcclusionData;
		OcclusionData.CachedTransform = InitialTransform;
		OcclusionDataMap.Add(NavData, OcclusionData);
		PreviousBounds = InitialBounds;

		// Log before registration attempt
		UE_LOG(LogNav3D, Verbose, TEXT("[%s] Calling RegisterDynamicOccluder on NavData %s"),
		       *GetOwner()->GetActorNameOrLabel(),
		       *NavData->GetName());

		NavData->RegisterDynamicOccluder(GetOwner());

		TArray<FBox> InitialDirtyAreas;
		InitialDirtyAreas.Add(InitialBounds);
		NavData->RebuildDirtyBounds(InitialDirtyAreas);

		UE_LOG(LogNav3D, Verbose, TEXT("[%s] Registration verification - Map Size: %d, Bounds: %s"),
		       *GetOwner()->GetActorNameOrLabel(),
		       OcclusionDataMap.Num(),
		       *InitialBounds.ToString());
	}
}

void UNav3DDynamicOcclusion::UnregisterOwner(ANav3DData* NavData)
{
	if (!NavData)
	{
		return;
	}

	UE_LOG(LogNav3D, Verbose, TEXT("Unregistering actor %s with NavData %s"),
	       *GetOwner()->GetActorNameOrLabel(), *NavData->GetName());
	RegisteredNavData.Remove(NavData);
	OcclusionDataMap.Remove(NavData);
	NavData->UnregisterDynamicOccluder(GetOwner());
}

void UNav3DDynamicOcclusion::TickComponent(const float DeltaTime, const ELevelTick TickType,
                                           FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (RegisteredNavData.Num() > 0)
	{
		const FTransform CurrentTransform = GetOwner()->GetActorTransform();
		const FBox CurrentBounds = GetOwner()->GetComponentsBoundingBox(true);

		// Check if transform changed
		bool bTransformChanged = false;
		for (const auto& Pair : OcclusionDataMap)
		{
			if (!Pair.Value.CachedTransform.Equals(CurrentTransform))
			{
				bTransformChanged = true;
				break;
			}
		}

		if (bTransformChanged)
		{
			UE_LOG(LogNav3D, Verbose, TEXT("[%s] Transform changed, updating Nav3DData..."),
			       *GetOwner()->GetActorNameOrLabel());

			for (ANav3DData* NavData : RegisteredNavData)
			{
				if (!NavData)
				{
					continue;
				}

				// First verify the occluder is still registered
				TArray<const AActor*> CurrentOccluders;
				for (ANav3DDataChunkActor* ChunkActor : NavData->GetChunkActors())
				{
					if (!ChunkActor) continue;
					
					for (const UNav3DDataChunk* Chunk : ChunkActor->Nav3DChunks)
					{
						if (!Chunk) continue;

						if (const FNav3DVolumeNavigationData* VolumeData = Chunk->GetVolumeNavigationData())
						{
							for (const auto& Occluder : VolumeData->DynamicOccluders)
							{
								if (const AActor* OccluderActor = Occluder.Get())
								{
									CurrentOccluders.AddUnique(OccluderActor);
								}
							}
						}
					}
				}

				if (!CurrentOccluders.Contains(GetOwner()))
				{
					UE_LOG(LogNav3D, Verbose, TEXT("[%s] Occluder not found in Nav3DData, re-registering..."),
					       *GetOwner()->GetActorNameOrLabel());
					NavData->RegisterDynamicOccluder(GetOwner());
				}

				TArray<FBox> DirtyAreas;
				DirtyAreas.Add(PreviousBounds);
				DirtyAreas.Add(CurrentBounds);

				NavData->RebuildDirtyBounds(DirtyAreas);

				if (FVoxelOcclusionData* OcclusionData = OcclusionDataMap.Find(NavData))
				{
					OcclusionData->CachedTransform = CurrentTransform;
				}
			}

			PreviousBounds = CurrentBounds;
		}
	}
}
