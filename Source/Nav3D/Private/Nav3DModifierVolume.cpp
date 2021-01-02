#include "Nav3DModifierVolume.h"
#include "Nav3DVolume.h"
#include "DrawDebugHelpers.h"
#include "Components/BrushComponent.h"
#include "Kismet/GameplayStatics.h"

ANav3DModifierVolume::ANav3DModifierVolume(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer) {
	GetBrushComponent()->Mobility = EComponentMobility::Movable;
	PrimaryActorTick.bCanEverTick = false;
}

void ANav3DModifierVolume::OnConstruction(const FTransform &Transform)
{
	Super::OnConstruction(Transform);
	Initialise();
}

void ANav3DModifierVolume::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
	Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ANav3DModifierVolume::EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
	Super::EditorApplyRotation(DeltaRotation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ANav3DModifierVolume::EditorApplyScale( const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown )
{
	Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

#if WITH_EDITOR

void ANav3DModifierVolume::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) {
	Super::PostEditChangeProperty(PropertyChangedEvent);
	FProperty* Property = PropertyChangedEvent.Property;
	const FString PropertyName = Property != nullptr ? Property->GetFName().ToString() : "";

	const TSet<FString> CriticalProperties = {
		"Scale",
		"Priority",
		"Enabled"};
	const TSet<FString> DebugProperties = {
		"bDisplayVolumeBounds",
		"bDisplayOverlaps",
		"VolumeBoundsColor",
		"OverlapColor"};
	if (CriticalProperties.Contains(PropertyName)) {

		// Update any overlapping Nav3D volumes
		TArray<ANav3DVolume*> Volumes;
		TArray<FBox> Overlaps;
		GetOverlappingVolumes(Volumes, Overlaps);
		for (auto& Volume: Volumes)
		{
			if (Volume->IsValidLowLevel()) Volume->Initialise();
		}
	}
	Initialise();
}

void ANav3DModifierVolume::PostEditUndo() {
	Super::PostEditUndo();
	Initialise();
}

void ANav3DModifierVolume::DebugDrawModifierVolume() const
{
	if (!GetWorld() || !bDisplayVolumeBounds) return;
	const FBox Box = GetBoundingBox();
	DebugDrawBoundsMesh(Box, VolumeBoundsColour);
	if (bDisplayOverlaps) DebugDrawOverlaps();
}

void ANav3DModifierVolume::DebugDrawOverlaps() const
{
	TArray<ANav3DVolume*> Volumes;
	TArray<FBox> Overlaps;
	GetOverlappingVolumes(Volumes, Overlaps);
	if (!bDisplayOverlaps || Overlaps.Num() == 0) return;
	for (auto& Box: Overlaps)
	{
		DebugDrawBoundsMesh(Box, OverlapColour);
	}
}

#endif


void ANav3DModifierVolume::Initialise() const
{
	TArray<ANav3DVolume*> Volumes;
	TArray<FBox> Overlaps;
	GetOverlappingVolumes(Volumes, Overlaps);
}


void ANav3DModifierVolume::DebugDrawBoundsMesh(const FBox Box, const FColor Colour) const { 
	const TArray<FVector> Vertices = {
		{Box.Min.X, Box.Min.Y, Box.Min.Z}, {Box.Max.X, Box.Min.Y, Box.Min.Z}, {Box.Max.X, Box.Min.Y, Box.Max.Z}, {Box.Min.X, Box.Min.Y, Box.Max.Z},
		{Box.Min.X, Box.Max.Y, Box.Min.Z}, {Box.Max.X, Box.Max.Y, Box.Min.Z}, {Box.Max.X, Box.Max.Y, Box.Max.Z}, {Box.Min.X, Box.Max.Y, Box.Max.Z}};
	const TArray<int32> Indices = { 0, 1, 2, 0, 2, 3, 5, 4, 7, 5, 7, 6, 3, 2, 6, 3, 6, 7, 5, 4, 0, 5, 0, 1, 0, 4, 7, 0, 7, 3, 5, 1, 2, 5, 2, 6};
	DrawDebugMesh(GetWorld(), Vertices, Indices, Colour, true, -1.0, 0);
}

void ANav3DModifierVolume::GetOverlappingVolumes(TArray<ANav3DVolume*> &Volumes, TArray<FBox> &Overlaps) const
{
	TArray<AActor*> Nav3DVolumeActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ANav3DVolume::StaticClass(), Nav3DVolumeActors);
	for (auto& Actor: Nav3DVolumeActors)
	{
		ANav3DVolume* Nav3DVolumeActor = Cast<ANav3DVolume>(Actor);
		if (Nav3DVolumeActor) {
			const FBox Bounds = GetBoundingBox();
			const FBox VolumeBounds = Nav3DVolumeActor->GetBoundingBox();
			if (VolumeBounds.Intersect(Bounds))
			{
				FBox Overlap = Bounds.Overlap(VolumeBounds);
				Overlaps.Add(Overlap);
				Volumes.Add(Nav3DVolumeActor);
			}
		}
	}
}

FBox ANav3DModifierVolume::GetBoundingBox() const
{
	const FBoxSphereBounds Bounds = GetBrushComponent()->CalcBounds(ActorToWorld());
	return Bounds.GetBox();
}