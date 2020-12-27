#include "Nav3DOcclusionComponent.h"
#include "Nav3DVolume.h"
#include "Kismet/GameplayStatics.h"

UNav3DOcclusionComponent::UNav3DOcclusionComponent() {
	PrimaryComponentTick.bCanEverTick = true;
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
	SetComponentTickInterval(TickInterval);
	CachedTransform = GetOwner()->GetActorTransform();
	if (bEnabled) RequestUpdate();
}

void UNav3DOcclusionComponent::RequestUpdate() {
	if (!Volume) FindVolume(Volume);
	if (Volume) {
		Volume->AddDirtyOcclusionComponent(this);
	}
}

bool UNav3DOcclusionComponent::FindVolume(ANav3DVolume*& CurrentVolume) const {
	TArray<AActor*> Volumes;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANav3DVolume::StaticClass(), Volumes);

	for (auto& NavVolume : Volumes) {
		CurrentVolume = Cast<ANav3DVolume>(NavVolume);
		if (CurrentVolume->IsValidLowLevel() && Volume->EncompassesPoint(GetOwner()->GetActorLocation())) {
			return true;
		}
	}
	return false;
}

void UNav3DOcclusionComponent::SetOcclusionEnabled(const bool bOcclusionEnabled) {
	if (bOcclusionEnabled && !bEnabled || !bOcclusionEnabled && bEnabled) {
		RequestUpdate();
	}
	bEnabled = bOcclusionEnabled;
}

void UNav3DOcclusionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (bEnabled) {
		if (GetOwner()->GetActorTransform().Equals(CachedTransform, TransformTriggerTolerance)) {
			RequestUpdate();
			CachedTransform = GetOwner()->GetActorTransform();
		}
	}
}
