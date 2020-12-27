#pragma once

#include "Nav3DModifierVolume.h"
#include "Nav3DOcclusionComponent.h"
#include "Nav3DStructs.h"
#include "GameFramework/Volume.h"
#include "Nav3DVolume.generated.h"

/**
 *  Volume contains the octree and methods required for 3D navigation
 */
UCLASS(Blueprintable)
class NAV3D_API ANav3DVolume : public AVolume
{
	GENERATED_BODY()

public:

	ANav3DVolume(const FObjectInitializer& ObjectInitializer);

	// The size of the volume. Voxel exponent will be calculated based on this and the voxel size
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0.000001"), DisplayName = "Volume Size", Category = "Nav3D|Volume")
    float VolumeSize = 200.0f;

	// The minimum size of a leaf voxel in the X, Y and Z dimensions. Nav3D LOD volumes can quantize this value
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0.000001"), DisplayName = "Minimum Voxel Size", Category = "Nav3D|Volume")
    float VoxelSize = 200.0f;

	// Which collision channel to use for object tracing during octree generation
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Generation")
    TEnumAsByte<ECollisionChannel> CollisionChannel;

	// The minimum distance away from any object traces to apply during octree generation
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Generation")
    float Clearance = 0.f;

	// This value is used to quantize leaf voxels. Nav3D LOD volumes will override the default set here 
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0", ClampMax = "2"), DisplayName = "Default Leaf LOD", Category = "Nav3D|Generation")
    int32 DefaultLeafLOD = 0;

	// Draw distance for debug lines
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    float DebugDistance = 10000.f;

	// Show all debug messages in console
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    bool bDebugLogging = false;

	// Show the entire volume bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    bool bDisplayVolumeBounds = false;

	// The colour for the volume bounds debug draw
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    FColor VolumeBoundsColour = FColor(0, 128, 0, 64);
	
	// Show the octree voxel node bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")
    bool bDisplayLayers = false;
	
	// Show the octree layer bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")
    bool bDisplayLeafs = false;

	// Show the occluded octree voxel sub-leaf bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")
    bool bDisplayLeafOcclusion = false;
	
	// Show voxels with intersections or volatile interactions with Nav3D Occlusion components
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")
    bool bDisplayDynamicOcclusion = false;

	// The colours for debug drawing each layer
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")
    TArray<FColor> LayerColours = { FColor::Blue, FColor::Black, FColor::Cyan, FColor::Green, FColor::Magenta };

	// The colour for leaf occlusion debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")
    FColor LeafOcclusionColour = FColor::Yellow;

	// The colour for highlight debug drawing, such as with dynamic occlusion and navigation paths
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")
    FColor HighlightColour = FColor::Red;

	// The scaling factor for morton code debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")	
    float LineScale = 1.0f;
	
	// Show the morton codes within each voxel
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Morton Codes")
    bool bDisplayMortonCodes = false;

	// The colour for morton code debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Morton Codes")
    FColor MortonCodeColour = FColor::Magenta;
	
	// The scaling factor for morton code debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Morton Codes")
    float MortonCodeScale = 1.0f;

	// The number of voxel subdivisions calculated to meet the desired voxel size. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Info")
    float VoxelExponent = 3.0f;

	// The number of layers created by the octree generation. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Info")
    uint8 NumLayers = 0;

	// The number of bytes used to store the generated octree. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Info")
    int32 NumBytes = 0;

#if WITH_EDITOR
	void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	void PostEditUndo() override;
#endif 

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void OnConstruction(const FTransform &Transform) override;
	virtual void PostRegisterAllComponents() override;
	virtual void PostUnregisterAllComponents() override;
	virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyScale( const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown ) override;
	void Initialise();
	bool ShouldTickIfViewportsOnly() const override { return true; }
	void Serialize(FArchive& Ar) override;
	bool BuildOctree();
	void DebugDrawOctree();
	FBox GetBoundingBox() const;
	bool GetEdge(const FVector& Location, FNav3DOctreeEdge& Edge);
	void GetVolumeExtents(const FVector& Location, int32 LayerIndex, FIntVector& Extents) const;
	bool OctreeValid() const { return NumLayers > 0; }
	const TArray<FNav3DOctreeNode>& GetLayer(uint8 LayerIndex) const { return Octree.Layers[LayerIndex]; };
	const FNav3DOctreeNode& GetNode(const FNav3DOctreeEdge& Edge) const;
	const FNav3DOctreeLeaf& GetLeaf(int32 LeafIndex) const;
	bool GetEdgeLocation(const FNav3DOctreeEdge& Edge, FVector& Location) const;
	bool GetNodeLocation(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const;
	bool GetNodeLocation(FNav3DOctreeEdge Edge, FVector& Location);
	bool GetNodeLocation(FNav3DOctreeEdge Edge, uint8 LOD, FVector& Location);
	void GetAdjacentLeafs(const FNav3DOctreeEdge& Edge, TArray<FNav3DOctreeEdge>& AdjacentEdges) const;
	void GetAdjacentEdges(const FNav3DOctreeEdge& Edge, TArray<FNav3DOctreeEdge>& AdjacentEdges) const;
	float GetVoxelScale(uint8 LayerIndex) const;
	TArray<FNav3DOctreeEdge> CalculateVolatileEdges(const AActor* Actor) const;
	void AddDirtyOcclusionComponent(UNav3DOcclusionComponent* OcclusionComponent);
		
private:
	FNav3DOctree Octree;
	TArray<float> VoxelHalfSizes;
	
	UPROPERTY()
    TArray<UNav3DOcclusionComponent*> DirtyOcclusionComponents;

	UPROPERTY()
	TArray<ANav3DModifierVolume*> ModifierVolumes;

	TArray<TSet<uint_fast64_t>> Occluded;
	FVector Origin;
	FVector VolumeExtent;
	const FIntVector Directions[6] = {{1, 0, 0}, {-1,0,0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
	const int32 NodeOffsets[6][4] = {{0, 4, 2, 6}, {1, 3, 5, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}};
	const int32 LeafOffsets[6][16] = {{0, 2, 16, 18, 4, 6, 20, 22, 32, 34, 48, 50, 36, 38, 52, 54}, {9, 11, 25, 27, 13, 15, 29, 31, 41, 43, 57, 59, 45, 47, 61, 63},
		{0, 1, 8, 9, 4, 5, 12, 13, 32, 33, 40, 41, 36, 37, 44, 45}, {18, 19, 26, 27, 22, 23, 30, 31, 50, 51, 58, 59, 54, 55, 62, 63},
		{0, 1, 8, 9, 2, 3, 10, 11, 16, 17, 24, 25, 18, 19, 26, 27}, {36, 37, 44, 45, 38, 39, 46, 47, 52, 53, 60, 61, 54, 55, 62, 63}};

	TArray<FNav3DOctreeNode>& GetLayer(const uint8 LayerIndex) { return Octree.Layers[LayerIndex]; };
	void UpdateBounds();
	void UpdateVoxelSize();
	void UpdateVolumeExtents();
	void RasterizeInitial();
	void RasterizeLayer(uint8 LayerIndex);
	void RasterizeLeaf(FVector NodeLocation, int32 LeafIndex, uint8 LOD);
	void BuildEdges(uint8 LayerIndex);
	bool FindEdge(uint8 LayerIndex, int32 NodeIndex, uint8 Direction, FNav3DOctreeEdge& Edge);
	bool GetMortonCodeIndex(uint8 LayerIndex, uint_fast64_t MortonCode, int32& CodeIndex) const;
	bool IsOccluded(const FVector& Location, float Size) const;
	int32 GetLayerNodeCount(uint8 LayerIndex) const;
	int32 GetSegmentNodeCount(uint8 LayerIndex) const;
	bool InDebugRange(FVector Location) const;
	bool GetNodeIndex(uint8 LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const;
	int32 GetInsertIndex(uint8 LayerIndex, uint_fast64_t MortonCode) const;
	void UpdateOctree();
	void UpdateNode(FNav3DOctreeEdge Edge);
	void UpdateLeaf(const FVector& Location, int32 LeafIndex);
	void FlushDebugDraw() const;
	void DebugDrawVolume() const;
	void DebugDrawVoxel(FVector Location, FVector Extent, FColor Colour) const;
	void DebugDrawMortonCode(FVector Location, FString String, FColor Colour) const;
	void DebugDrawOccludedLeafs();
	void DebugDrawVolatileNodes();
	void DebugDrawBoundsMesh(FBox Box, FColor Colour) const;
	void UpdateModifierVolumes();
	FColor GetLayerColour(const int32 LayerIndex) const { return LayerColours[FMath::Clamp(LayerIndex, 0, LayerColours.Num()-1)]; }
	uint8 GetLocationLOD(FVector WorldLocation);
};
