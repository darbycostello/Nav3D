#pragma once
#include "GameFramework/Volume.h"
#include "Nav3DModifierVolume.generated.h"

/**
*  Volume used to modify the properties of an overlapping Nav3D Volume
*/
UCLASS(Blueprintable, meta=(DisplayName = "Nav3D Modifier Volume"))
class NAV3D_API ANav3DModifierVolume : public AVolume
{
	friend class ANav3DVolume;
	
	GENERATED_BODY()

public:

    ANav3DModifierVolume(const FObjectInitializer& ObjectInitializer);

	// This value will be added to the calculated path cost for any points that lie within the modifier boundary  
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0", ClampMax = "100"), Category = "Nav3D")
    float PathCostModifier = 1;

	// Enabling this will prevent any locations within the volume being used for cover map queries.
	UPROPERTY(EditAnywhere, Category = "Nav3D")
	bool bInvalidateCoverLocations = false;
	
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

#if WITH_EDITOR
	virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyScale( const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown ) override;
#endif

	bool GetEnabled() const { return bEnabled; }
	float GetPathCost() const { return bEnabled ? PathCostModifier : 0.f; }
	FBox GetBoundingBox() const;

private:
	void Initialise();
	void GetOverlappingVolumes(TArray<ANav3DVolume*> &Volumes, TArray<FBox> &Overlaps) const;

#if WITH_EDITOR
	void DebugDrawModifierVolume() const;
	void DebugDrawOverlaps() const;
	void DebugDrawBoundsMesh(FBox Box, FColor Colour) const;
#endif
};