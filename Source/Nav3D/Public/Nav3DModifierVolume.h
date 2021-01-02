#pragma once
#include "GameFramework/Volume.h"
#include "Nav3DModifierVolume.generated.h"

/**
*  Volume used to modify the leaf quantization level in an overlapping Nav3D Volume
*/
UCLASS(Blueprintable, meta=(DisplayName = "Nav3D Modifier Volume"))
class NAV3D_API ANav3DModifierVolume : public AVolume
{
	friend class ANav3DVolume;
	
	GENERATED_BODY()

public:

    ANav3DModifierVolume(const FObjectInitializer& ObjectInitializer);

	// This value is used to override the leaf voxel quantization in the overlapping Nav3D volume
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0", ClampMax = "2"), DisplayName = "Leaf LOD", Category = "Nav3D")
    int32 LeafLOD = 0;

	// If two Nav3D Modifier volumes overlap, the one with the highest priority will be used  
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0", ClampMax = "100"), DisplayName = "Priority", Category = "Nav3D")
    int32 Priority = 0;

	// Whether to use this volume to modify overlapping Nav3D volumes  
	UPROPERTY(EditAnywhere, DisplayName = "Enabled", Category = "Nav3D")
    bool bEnabled = true;

	// Show the entire volume bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    bool bDisplayVolumeBounds = false;

	// Show the overlapping areas between this volume and any Nav3D volumes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    bool bDisplayOverlaps = false;
	
	// The colour for leaf occlusion debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    FColor VolumeBoundsColour = FColor(255, 0, 255, 64);

	// The colour for overlaps between modifier volume and any Nav3D volumes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    FColor OverlapColour = FColor(255, 0, 0, 64);

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditUndo() override;
#endif
	
	virtual void OnConstruction(const FTransform &Transform) override;
	virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyScale( const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown ) override;
	bool GetEnabled() const { return bEnabled; }
	int32 GetPriority() const { return Priority; }
	uint8 GetLOD() const { return LeafLOD; }
	FBox GetBoundingBox() const;

private:
	void Initialise() const;
	void GetOverlappingVolumes(TArray<ANav3DVolume*> &Volumes, TArray<FBox> &Overlaps) const;
	void DebugDrawModifierVolume() const;
	void DebugDrawOverlaps() const;
	void FlushDebugDraw() const;
	void DebugDrawBoundsMesh(FBox Box, FColor Colour) const;
};