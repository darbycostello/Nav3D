#include "Nav3DOcclusionComponent.h"
#include "Nav3DVolume.h"
#include "Kismet/GameplayStatics.h"

UNav3DOcclusionComponent::UNav3DOcclusionComponent() {
	PrimaryComponentTick.bCanEverTick = true;
	bWantsInitializeComponent = true;
}

bool UNav3DOcclusionComponent::UpdateVolatileEdges() {
	if (!bEnabled) return false;
	if (!Volume) FindVolume(Volume);
	if (!Volume) return false;
	VolatileEdges = Volume->CalculateVolatileEdges(GetOwner());
	return true;
}

TArray<FNav3DOctreeEdge>& UNav3DOcclusionComponent::GetVolatileEdges() {
	return VolatileEdges;
}

void UNav3DOcclusionComponent::BeginPlay() {
	Super::BeginPlay();
	CachedTransform = GetOwner()->GetActorTransform();
	if (bEnabled) RequestUpdate();
}

void UNav3DOcclusionComponent::RequestUpdate() {
	if (!Volume) FindVolume(Volume);
	if (Volume) Volume->RequestOctreeUpdate(this);
}

bool UNav3DOcclusionComponent::FindVolume(ANav3DVolume*& CurrentVolume) const {
	TArray<AActor*> Volumes;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANav3DVolume::StaticClass(), Volumes);

	for (auto& NavVolume : Volumes) {
		CurrentVolume = Cast<ANav3DVolume>(NavVolume);
		if (CurrentVolume->IsValidLowLevel() && Volume->IsWithinBounds(GetOwner()->GetActorLocation())) {
			return true;
		}
	}
	return false;
}

bool UNav3DOcclusionComponent::GetCoverLocations(const int32 NormalIndex, TArray<FVector>& Locations) {
	if (CoverLocations.Contains(NormalIndex)) {
		Locations = CoverLocations[NormalIndex];
		return true;
	}
	return false;
}

void UNav3DOcclusionComponent::SetOcclusionEnabled(const bool bOcclusionEnabled) {
	if (bOcclusionEnabled && !bEnabled || !bOcclusionEnabled && bEnabled) {
		RequestUpdate();
	}
	bEnabled = bOcclusionEnabled;
	if (!bEnabled) {
		bEnableCover = false;
	}
}

void UNav3DOcclusionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (bEnabled) {
		
		if (!GetOwner()->GetActorTransform().Equals(CachedTransform, TransformTriggerTolerance)) {
			RequestUpdate();
			CachedTransform = GetOwner()->GetActorTransform();
		}
	}
}