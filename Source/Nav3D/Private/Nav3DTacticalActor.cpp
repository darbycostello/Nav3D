#include "Nav3DTacticalActor.h"
#include "Nav3DData.h"
#include "Engine/World.h"
#include "Engine/Level.h"

ANav3DTacticalActor::ANav3DTacticalActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryActorTick.bCanEverTick = false;
	// Don't call virtual functions in constructor - move to BeginPlay
}

bool ANav3DTacticalActor::ContainsPoint(const FVector& Point) const
{
	return TacticalActorBounds.IsInsideXY(Point);
}

bool ANav3DTacticalActor::GetRegionContainingPoint(const FVector& Point, FNav3DRegion& OutRegion) const
{
	for (const FNav3DRegion& Region : TacticalData.Regions)
	{
		if (Region.Bounds.IsInside(Point))
		{
			OutRegion = Region;
			return true;
		}
	}
	return false;
}

uint32 ANav3DTacticalActor::GetDefaultGridSize(UWorld* InWorld) const
{
	// Use the same grid size as chunk actors for consistency
	return 102400; // 1000m grid size
}

void ANav3DTacticalActor::GetActorBounds(bool bOnlyCollidingComponents, FVector& OutOrigin, FVector& OutBoxExtent, bool bIncludeFromChildActors) const
{
	OutOrigin = TacticalActorBounds.GetCenter();
	OutBoxExtent = TacticalActorBounds.GetExtent();
}

// ReSharper disable once CppMemberFunctionMayBeStatic
void ANav3DTacticalActor::RegisterWithNavigationSystem()
{
	// Tactical actors don't need to register with the navigation system
	// They are managed by ANav3DData directly
}

// ReSharper disable once CppMemberFunctionMayBeStatic
void ANav3DTacticalActor::UnregisterFromNavigationSystem()
{
	// Tactical actors don't need to unregister from the navigation system
	// They are managed by ANav3DData directly
}

#if WITH_EDITOR
void ANav3DTacticalActor::GetStreamingBounds(FBox& OutRuntimeBounds, FBox& OutEditorBounds) const
{
	OutRuntimeBounds = TacticalActorBounds;
	OutEditorBounds = TacticalActorBounds;
}

void ANav3DTacticalActor::SetTacticalActorBounds(const FBox& InBounds)
{
	TacticalActorBounds = InBounds;
	
	// Update actor location to match bounds center
	SetActorLocation(TacticalActorBounds.GetCenter());
}

void ANav3DTacticalActor::RebuildTacticalData() const
{
	if (ANav3DData* NavData = Cast<ANav3DData>(GetOwner()))
	{
		NavData->RebuildTacticalData();
	}
}
#endif

void ANav3DTacticalActor::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
	// Persist cross-volume graph buckets for on-demand adjacency across sessions
	CrossVolumeGraph.Serialize(Ar);
}

void ANav3DTacticalActor::PostLoad()
{
	Super::PostLoad();
	// Rebind cached chunk actors after load
	if (ANav3DData* NavData = Cast<ANav3DData>(GetOwner()))
	{
		CrossVolumeGraph.SetCachedChunkActors(NavData->GetAllChunkActors());
	}
}

void ANav3DTacticalActor::BeginPlay()
{
	Super::BeginPlay();
	SetActorHiddenInGame(true);
	SetActorEnableCollision(false);
	RegisterWithNavigationSystem();
	// Cross-volume graph is built by ANav3DData upon registration
}

void ANav3DTacticalActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	UnregisterFromNavigationSystem();
	Super::EndPlay(EndPlayReason);
}

void ANav3DTacticalActor::AddTacticalDataToWorld()
{
	RegisterWithNavigationSystem();
}

void ANav3DTacticalActor::RemoveTacticalDataFromWorld()
{
	UnregisterFromNavigationSystem();
}
