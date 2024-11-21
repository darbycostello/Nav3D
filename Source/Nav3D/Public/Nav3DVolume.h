#pragma once

#include "Nav3DModifierVolume.h"
#include "Nav3DOcclusionComponent.h"
#include "Nav3DStructs.h"
#include "GameFramework/Volume.h"
#include "Nav3DVolume.generated.h"

DECLARE_DELEGATE(FNav3DUpdateOctreeDelegate);

/**
 *  Volume contains the octree and methods required for 3D navigation
 */
UCLASS(Blueprintable, meta=(DisplayName = "Nav3D Volume"))
class NAV3D_API ANav3DVolume : public AVolume
{
	GENERATED_BODY()

public:

	ANav3DVolume(const FObjectInitializer& ObjectInitializer);

	// The size of the volume. This will be approximated in order to support the requested voxel size
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0.000001"), DisplayName = "Desired Volume Size", Category = "Nav3D|Volume")
    float VolumeSize = 200.0f;

	// The minimum size of a leaf voxel in the X, Y and Z dimensions.
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0.000001"), DisplayName = "Minimum Voxel Size", Category = "Nav3D|Volume")
    float VoxelSize = 200.0f;

	// Which collision channel to use for object tracing during octree generation
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Generation")
    TEnumAsByte<ECollisionChannel> CollisionChannel;

	// The minimum distance away from any object traces to apply during octree generation
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Generation")
    float Clearance = 0.f;

	// How often to tick this actor to perform dynamic updates
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Generation")
	float TickInterval = 0.1f;

	// Whether to use a cover map with this volume. Requires an octree rebuild after modifying
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Cover Map")
	bool bEnableCoverMap = true;
	
	// The minimum object radius to be considered for cover
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Cover Map")
	float MinimumObjectRadius = 500.0f;

	// Cover locations with the same normal must be at least this distance apart
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Cover Map")
	float MinimumDensity = 500.0f;
	
	// Draw distance for debug lines
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging")
    float DebugDistance = 50000.f;

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

	// Show adjacency edges between each node
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")
    bool bDisplayEdgeAdjacency = false;

	// The scaling factor for debug line drawing. Set to zero for fastest performance
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels")	
    float LineScale = 0.0f;

	// The colours for debug drawing each layer. Colours added will be spread across a gradient
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels|Colours")
    TArray<FColor> LayerColours = { FColor::Magenta, FColor::Blue, FColor::Cyan, FColor::Green };

	// The colour for leaf occlusion debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Voxels|Colours")
    FColor LeafOcclusionColour = FColor::Yellow;

	// Show the morton codes within each voxel
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Morton Codes")
    bool bDisplayMortonCodes = false;

	// The colour for morton code debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Morton Codes")
    FColor MortonCodeColour = FColor::Magenta;
	
	// The scaling factor for morton code debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Morton Codes")
    float MortonCodeScale = 1.0f;

	// Show the cover map locations and direction normals. Also applies to occlusion component cover
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Nav3D|Debugging|Cover Map")
	bool bDisplayCoverMap = false;
	
	// The number of voxel subdivisions calculated to meet the desired voxel size. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Info")
    int32 VoxelExponent = 6;

	// The actual volume size calculated to meet the desired volume size and voxel size. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Info")
	float ActualVolumeSize = 200.0f;

	// The number of layers created by the octree generation. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Info")
    uint8 NumLayers = 0;

	// The number of bytes used to store the generated octree. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Info")
    int32 NumBytes = 0;

	UFUNCTION()
	void UpdateTaskComplete();

#if WITH_EDITOR
    void FlushDebugDraw() const;
    void RequestOctreeDebugDraw();

	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditUndo() override;
#endif 

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void OnConstruction(const FTransform &Transform) override;
	virtual void PostRegisterAllComponents() override;
	virtual void PostUnregisterAllComponents() override;

#if WITH_EDITOR
	virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyScale( const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown ) override;
#endif

	virtual void Serialize(FArchive& Ar) override;
	void Initialise();
	bool BuildOctree();
	void UpdateOctree();
    void ClearAllDebugNavPaths();
	void AddDebugNavPath(const FNav3DDebugPath DebugPath);
	void AddDebugLocation(const FNav3DDebugLocation DebugLocation);
	void AddModifierVolume(ANav3DModifierVolume* ModifierVolume);
	void LockOctree() { bOctreeLocked = true; }
	void UnlockOctree() { bOctreeLocked = false; }
	FBox GetBoundingBox() const;
	bool GetEdge(const FVector& Location, FNav3DOctreeEdge& Edge);
	bool FindAccessibleEdge(FVector& Location, FNav3DOctreeEdge& Edge);
	void GetPathCost(FVector& Location, float& Cost);
	int32 GetCoverNormalIndex(FVector Normal) const;
	FVector GetCoverNormal(const int32 NormalIndex) const { return CoverNormals[NormalIndex];}
	bool GetCoverLocationValid(FVector& Location) const;
	void GetVolumeExtents(const FVector& Location, int32 LayerIndex, FIntVector& Extents) const;
	void GetMortonVoxel(const FVector& Location, int32 LayerIndex, FIntVector& MortonLocation) const;
	bool OctreeValid() const { return NumLayers > 0 && Octree.Layers.Num() == NumLayers; }
	const TArray<FNav3DOctreeNode>& GetLayer(uint8 LayerIndex) const { return Octree.Layers[LayerIndex]; };
	const FNav3DOctreeNode& GetNode(const FNav3DOctreeEdge& Edge) const;
	bool EdgeNodeIsValid(const FNav3DOctreeEdge& Edge) const;
	bool GetEdgeLocation(const FNav3DOctreeEdge& Edge, FVector& Location) const;
	bool GetNodeLocation(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const;
	bool GetNodeLocation(FNav3DOctreeEdge Edge, FVector& Location);
	void GetAdjacentLeafs(const FNav3DOctreeEdge& Edge, TArray<FNav3DOctreeEdge>& AdjacentEdges) const;
	void GetAdjacentEdges(const FNav3DOctreeEdge& Edge, TArray<FNav3DOctreeEdge>& AdjacentEdges) const;
	TArray<ANav3DModifierVolume*> GetModifierVolumes() const { return ModifierVolumes; }
	float GetVoxelScale(uint8 LayerIndex) const;
	bool IsWithinBounds(const FVector Location) const { return GetBoundingBox().IsInside(Location); }
	TArray<FNav3DOctreeEdge> CalculateVolatileEdges(const AActor* Actor) const;
	void RequestOctreeUpdate(UNav3DOcclusionComponent* OcclusionComponent);
	bool CoverMapValid() const { return CoverMap.Nodes.Num() > 0; }
	bool CoverMapContainsActor(const FName ActorName) const { return CoverMap.Nodes.Contains(ActorName); }
	TArray<FVector> GetCoverMapNodeLocations(const FName ActorName, const int32 NormalIndex) { return CoverMap.Nodes[ActorName].Locations[NormalIndex]; }
	
private:
	FNav3DOctree Octree;
	FNav3DOctree CachedOctree;
	TArray<float> VoxelHalfSizes;
	bool bOctreeLocked = false;
	FNav3DUpdateOctreeDelegate OnUpdateComplete;
	FNav3DCoverMap CoverMap;
	
#if WITH_EDITOR
	bool bDebugDrawRequested;
	TArray<FNav3DDebugEdge> DebugEdges;
	TArray<FNav3DDebugPath> DebugPaths;
	TArray<FNav3DDebugLocation> DebugLocations;
#endif

	UPROPERTY()
	TArray<UNav3DOcclusionComponent*> OcclusionComponents;

	UPROPERTY()
	TArray<ANav3DModifierVolume*> ModifierVolumes;

	TArray<TSet<uint_fast64_t>> Occluded;
	FVector VolumeOrigin;
	FVector VolumeExtent;
	FCollisionQueryParams CollisionQueryParams;
	bool bUpdateRequested;
	const FIntVector Directions[6] = {{1, 0, 0}, {-1,0,0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
	const int32 NodeOffsets[6][4] = {{0, 4, 2, 6}, {1, 3, 5, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}};
	const int32 LeafOffsets[6][16] = {{0, 2, 16, 18, 4, 6, 20, 22, 32, 34, 48, 50, 36, 38, 52, 54}, {9, 11, 25, 27, 13, 15, 29, 31, 41, 43, 57, 59, 45, 47, 61, 63},
		{0, 1, 8, 9, 4, 5, 12, 13, 32, 33, 40, 41, 36, 37, 44, 45}, {18, 19, 26, 27, 22, 23, 30, 31, 50, 51, 58, 59, 54, 55, 62, 63},
		{0, 1, 8, 9, 2, 3, 10, 11, 16, 17, 24, 25, 18, 19, 26, 27}, {36, 37, 44, 45, 38, 39, 46, 47, 52, 53, 60, 61, 54, 55, 62, 63}};
	const FVector CoverNormals[26] = {{-0.57735f, -0.57735f, -0.57735f}, {-0.7071f, 0.f, -0.7071f}, {-0.57735f, 0.57735f, -0.57735f},
		{-0.7071f, -0.7071f, 0.f}, {-1.f, 0.f, 0.f}, {-0.7071f, 0.7071f, 0.f},
		{-0.57735f, -0.57735f, 0.57735f}, {-0.7071f, 0.f, 0.7071f}, {-0.57735f, 0.57735f, 0.57735f},
		{0.f, -0.7071f, -0.7071}, {0.f, 0.f, -1.f}, {0.f, 0.7071f, -0.7071f},
		{0.f, -1.f, 0.f}, {0.f, 1.f, 0.f},
		{0.f, -0.7071f, 0.7071f}, {0.f, 0.f, 1.f}, {0.f, 0.7071f, 0.7071f},
		{0.57735f, -0.57735f, -0.57735f}, {0.7071f, 0.f, -0.7071f}, {0.57735f, 0.57735f, -0.57735f},
		{0.7071f, -0.7071f, 0.f}, {1.f, 0.f, 0.f}, {0.7071f, 0.7071f, 0.f},
		{0.57735f, -0.57735f, 0.57735f}, {0.7071f, 0.f, 0.7071f}, {0.57735f, 0.57735f, 0.57735f}};
	TArray<FNav3DOctreeNode>& GetLayer(const uint8 LayerIndex) { return Octree.Layers[LayerIndex]; };

#if WITH_EDITOR
	void UpdateVolume();
#endif

	void RasterizeInitial();
	void RasterizeLayer(uint8 LayerIndex);
	void RasterizeLeaf(FVector NodeLocation, int32 LeafIndex);
	void BuildEdges(uint8 LayerIndex);
	bool FindEdge(uint8 LayerIndex, int32 NodeIndex, uint8 Direction, FNav3DOctreeEdge& Edge, const FVector& NodeLocation);
	bool IsOccluded(const FVector& Location, float Size) const;
	bool IsOccluded(const FVector& Location, float Size, TArray<FOverlapResult>& OverlapResults) const;
	void UpdateCoverMap(FVector Location, TArray<FOverlapResult>& Overlaps);
	void UpdateOcclusionComponentCover(FVector Location, TArray<FOverlapResult>& Overlaps) const;
	int32 GetLayerNodeCount(uint8 LayerIndex) const;
	int32 GetSegmentNodeCount(uint8 LayerIndex) const;
	bool InDebugRange(FVector Location) const;
	bool GetNodeIndex(uint8 LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const;
	int32 GetInsertIndex(uint8 LayerIndex, uint_fast64_t MortonCode) const;
	float GetActualVolumeSize() const { return FMath::Pow(2.f, VoxelExponent) * (VoxelSize * 4); }
	void UpdateNode(FNav3DOctreeEdge Edge);
	void UpdateLeaf(const FVector& Location, int32 LeafIndex);

#if WITH_EDITOR
	void DebugDrawOctree();
	void DebugDrawVolume() const;
	void DebugDrawVoxel(FVector Location, FVector Extent, FColor Colour) const;
	void DebugDrawSphere(const FVector Location, const float Radius, const FColor Colour) const;
	void DebugDrawMortonCode(FVector Location, FString String, FColor Colour) const;
	void DebugDrawLeafOcclusion();
	void DebugDrawEdgeAdjacency() const;
	void DebugDrawBoundsMesh(FBox Box, FColor Colour) const;
	void DebugDrawNavPaths() const;
	void DebugDrawNavLocations() const;
	void DebugDrawModifierVolumes() const;
	void DebugDrawCoverMapLocations() const;
	void DebugDrawOcclusionComponentCover() const;
#endif

	void VerifyModifierVolumes();
	FColor GetLayerColour(const int32 LayerIndex) const;
	TArray<AActor*> GatherOcclusionActors();
};