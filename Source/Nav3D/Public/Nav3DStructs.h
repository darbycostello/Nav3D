#pragma once

#include "CoreMinimal.h"
#include "Nav3D/Private/libmorton/morton.h"
#include "Nav3DStructs.generated.h"

USTRUCT(BlueprintType)
struct NAV3D_API FNav3DPathPoint {
	
	GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    FVector Location;

	UPROPERTY(BlueprintReadWrite)
    int32 Layer;
	
	FNav3DPathPoint() :
        Location(FVector::ZeroVector),
        Layer(-1) {}
	FNav3DPathPoint(const FVector& Location, const int32 LayerIndex):
        Location(Location),
        Layer(LayerIndex) {}
};

USTRUCT(BlueprintType)
struct NAV3D_API FNav3DPath {

	GENERATED_BODY()
	
	UPROPERTY(BlueprintReadWrite)
    TArray<FNav3DPathPoint> Points;
	
	void Add(const FNav3DPathPoint& Point) { Points.Add(Point); }
	void Empty() { Points.Empty(); }
	TArray<FNav3DPathPoint> GetPoints() const { return Points; }
	void SetPoints(const TArray<FNav3DPathPoint> NewPoints) { Points = NewPoints; }
	void GetPath(TArray<FVector> &Path) { for (const auto& Point: Points) { Path.Add(Point.Location); } }
};
typedef TSharedPtr<FNav3DPath, ESPMode::ThreadSafe> FNav3DPathSharedPtr;

USTRUCT(BlueprintType)
struct NAV3D_API FNav3DDebugPath {

	GENERATED_BODY()
	
	UPROPERTY(BlueprintReadWrite)
	TArray<FVector> Points;

	UPROPERTY(BlueprintReadWrite)
	FColor Colour;

	UPROPERTY(BlueprintReadWrite)
	float LineScale;

	FNav3DDebugPath(): Colour(FColor::Black), LineScale(0) {}
};

USTRUCT(BlueprintType)
struct NAV3D_API FNav3DDebugLocation {

	GENERATED_BODY()
	
	UPROPERTY(BlueprintReadWrite)
	FVector Location;

	UPROPERTY(BlueprintReadWrite)
	FColor Colour;

	UPROPERTY(BlueprintReadWrite)
	float LineScale;

	FNav3DDebugLocation(): Location(FVector::ZeroVector), Colour(FColor::Black), LineScale(0) {}
};

UENUM()
enum class ENav3DHeuristic: uint8 {
	Manhattan UMETA(DisplayName="Manhattan"),
    Euclidean UMETA(DisplayName="Euclidean")
};

UENUM()
enum class ENav3DPathFindingCallResult: uint8 {
	Success UMETA(DisplayName="Call Success", ToolTip="Find path task was called successfully."),
	Reachable UMETA(DisplayName="Target in line of sight", ToolTip="Find path unnecessary. Target is already reachable."),
	NoVolume UMETA(DisplayName="Volume not found", ToolTip="Nav3D component owner is not inside a Nav3D volume."),
    NoOctree UMETA(DisplayName="Octree not found", ToolTip="Nav3D octree has not been built."),
	NoStart UMETA(DisplayName="Start edge not found", ToolTip="Failed to find start edge."),
	NoTarget UMETA(DisplayName="Target edge not found", ToolTip="Failed to find target edge.")
};

UENUM()
enum class ENav3DFindLineOfSightCallResult: uint8 {
	Success UMETA(DisplayName="Call Success", ToolTip="Line of sight task was called successfully."),
    Visible UMETA(DisplayName="Target in line of sight", ToolTip="Find Line of sight unnecessary. Target is already visible."),
    NoVolume UMETA(DisplayName="Volume not found", ToolTip="Nav3D component owner is not inside a Nav3D volume."),
    NoOctree UMETA(DisplayName="Octree not found", ToolTip="Nav3D octree has not been built."),
    NoStart UMETA(DisplayName="Start edge not found", ToolTip="Failed to find start edge."),
    NoTarget UMETA(DisplayName="Target edge not found", ToolTip="Failed to find target edge.")
};

UENUM()
enum class ENav3DPathPruning: uint8 {
	None UMETA(DisplayName="None", ToolTip="Do not use path pruning."),
    WithClearance UMETA(DisplayName="With clearance", ToolTip="Use path pruning with actor radius as clearance."),
    WithoutClearance UMETA(DisplayName="Without clearance", ToolTip="Use path pruning without clearance.")
};

UENUM()
enum class ENav3DCoverSearchType: uint8 {
	Nearest UMETA(DisplayName="Find nearest", ToolTip="Find the nearest viable cover location to the target."),
    Furthest UMETA(DisplayName="Find furthest", ToolTip="Find the furthest cover location from the target."),
    Random UMETA(DisplayName="Find random", ToolTip="Select a random cover location from all viable options.")
};

UENUM()
enum class ENav3DFindCoverCallResult: uint8 {
	Success UMETA(DisplayName="Call Success", ToolTip="Find cover task was called successfully."),
    NoVolume UMETA(DisplayName="Volume not found", ToolTip="Nav3D component owner is not inside a Nav3D volume."),
	NoOctree UMETA(DisplayName="Octree not found", ToolTip="Nav3D octree has not been built."),
    CoverMapNotEnabled UMETA(DisplayName="Cover map not enabled", ToolTip="Nav3D cover map is not enabled for this volume."),
	CoverMapInvalid UMETA(DisplayName="Cover map invalid", ToolTip="Nav3D cover map has no entries or is not valid for this volume.")
};

USTRUCT(BlueprintType)
struct NAV3D_API FNav3DPathFindingConfig
{
	GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    float EstimateWeight;

	UPROPERTY(BlueprintReadWrite)
    float NodeSizePreference;

	UPROPERTY(BlueprintReadWrite)
    ENav3DHeuristic Heuristic;

	UPROPERTY(BlueprintReadWrite)
	ENav3DPathPruning PathPruning;
	
	UPROPERTY(BlueprintReadWrite)
    int32 PathSmoothing;

	FNav3DPathFindingConfig() :
		EstimateWeight(5.0f),
		NodeSizePreference(1.0f),
		Heuristic(ENav3DHeuristic::Euclidean),
		PathPruning(ENav3DPathPruning::None),
		PathSmoothing(3) {}
};

struct NAV3D_API FNav3DOctreeEdge {
	uint8 LayerIndex:4;
	uint_fast32_t NodeIndex:22;
	uint8 SubNodeIndex:6;

	FNav3DOctreeEdge() :
		LayerIndex(15),
		NodeIndex(0),
		SubNodeIndex(0) {}

	FNav3DOctreeEdge(const uint8 LayerIndex, const uint_fast32_t NodeIndex, const uint8 SubNodeIndex) :
		LayerIndex(LayerIndex),
		NodeIndex(NodeIndex),
		SubNodeIndex(SubNodeIndex) {}

	uint8 GetLayerIndex() const { return LayerIndex; }
	void SetLayerIndex(const uint8 NewLayerIndex) { LayerIndex = NewLayerIndex; }

	uint_fast32_t GetNodeIndex() const { return NodeIndex; }
	void SetNodeIndex(const uint_fast32_t NewNodeIndex) { NodeIndex = NewNodeIndex; }

	uint8 GetSubNodeIndex() const { return SubNodeIndex; }
	void SetSubNodeIndex(const uint8 NewSubIndex) { SubNodeIndex = NewSubIndex; }

	bool IsValid() const { return LayerIndex != 15; }
	void Invalidate() { LayerIndex = 15; }
	bool operator==(const FNav3DOctreeEdge& OtherEdge) const { return memcmp(this, &OtherEdge, sizeof(FNav3DOctreeEdge)) == 0; }
	bool operator!=(const FNav3DOctreeEdge& OtherEdge) const { return !(*this == OtherEdge); }
	static FNav3DOctreeEdge GetInvalidEdge() { return FNav3DOctreeEdge(15, 0, 0); }
	FString ToString() const { return FString::Printf(TEXT("%i:%i:%i"), LayerIndex, NodeIndex, SubNodeIndex); }
};

FORCEINLINE uint32 GetTypeHash(const FNav3DOctreeEdge& Edge) { return *(uint32*)&Edge; }

FORCEINLINE FArchive& operator <<(FArchive& Archive, FNav3DOctreeEdge& Edge) {
	Archive.Serialize(&Edge, sizeof(FNav3DOctreeEdge));
	return Archive;
}

struct NAV3D_API FNav3DOctreeLeaf {
	uint_fast64_t SubNodes = 0;
	
	bool GetSubNodeAt(uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z) const {
		const uint_fast64_t MortonCode = 0;
		morton3D_64_decode(MortonCode, X, Y, Z);
		return (SubNodes & 1ULL << morton3D_64_encode(X, Y, Z)) != 0;
	}

	void SetSubNodeAt(uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z) {
		const uint_fast64_t MortonCode = 0;
		morton3D_64_decode(MortonCode, X, Y, Z);
		SubNodes |= 1ULL << morton3D_64_encode(X, Y, Z);
	}
	void SetSubNode(const uint8 Index) { SubNodes |= 1ULL << Index; }
	bool GetSubNode(const uint_fast64_t MortonCode) const { return (SubNodes & 1ULL << MortonCode) != 0; }
	void ClearSubNode(const uint8 Index) { SubNodes &= !(1ULL << Index); }
	bool IsOccluded() const { return SubNodes == -1; }
	bool IsEmpty() const { return SubNodes == 0; }
};

FORCEINLINE FArchive& operator<<(FArchive& Archive, FNav3DOctreeLeaf& Leaf)
{
	Archive << Leaf.SubNodes;
	return Archive;
}

struct NAV3D_API FNav3DOctreeNode
{
	uint_fast64_t MortonCode;
	FNav3DOctreeEdge Parent;
	FNav3DOctreeEdge FirstChild;
	FNav3DOctreeEdge AdjacentEdges[6];

	FNav3DOctreeNode() :
		MortonCode(0),
		Parent(FNav3DOctreeEdge::GetInvalidEdge()),
		FirstChild(FNav3DOctreeEdge::GetInvalidEdge()) {}

	bool HasChildren() const { return FirstChild.IsValid(); }
};

FORCEINLINE FArchive& operator <<(FArchive& Ar, FNav3DOctreeNode& Node) {
	Ar << Node.MortonCode;
	Ar << Node.Parent;
	Ar << Node.FirstChild;

	for (int32 I = 0; I < 6; I++) {
		Ar << Node.AdjacentEdges[I];
	}

	return Ar;
}

struct NAV3D_API FNav3DOctree {
	TArray<TArray<FNav3DOctreeNode>> Layers;
	TArray<FNav3DOctreeLeaf> Leafs;

	void Reset() {
		Layers.Empty();
		Leafs.Empty();
	}

	int32 GetSize() {
		int Size = 0;
		Size += Leafs.Num() * sizeof(FNav3DOctreeLeaf);
		for (int32 I = 0; I < Layers.Num(); I++) {
			Size += Layers[I].Num() * sizeof(FNav3DOctreeNode);
		}
		return Size;
	}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FNav3DOctree& Octree) {
	Ar << Octree.Layers;
	Ar << Octree.Leafs;
	return Ar;
}

USTRUCT(BlueprintType)
struct NAV3D_API FNav3DCoverLocation {

	GENERATED_BODY()

	// The actor providing cover
	UPROPERTY(BlueprintReadWrite)
	AActor* Actor;

	// Location of the cover point
	UPROPERTY(BlueprintReadWrite)
	FVector Location;

	// Quantized normal between the actor providing cover and the cover location
	UPROPERTY(BlueprintReadWrite)
	FVector Normal;

	FNav3DCoverLocation(): Actor(nullptr), Location(FVector::ZeroVector), Normal(FVector::ZeroVector) {}

	FNav3DCoverLocation(AActor* Actor, const FVector Location, const FVector Normal):
	Actor(Actor), Location(Location), Normal(Normal){}

	void GetLocation(FNav3DCoverLocation& CoverLocation) const {
		CoverLocation = FNav3DCoverLocation(Actor, Location, Normal);
	}
};
typedef TSharedPtr<FNav3DCoverLocation, ESPMode::ThreadSafe> FNav3DCoverLocationSharedPtr;

struct NAV3D_API FNav3DCoverMapNode {
	TMap<int32, TArray<FVector>> Locations;
	
	FNav3DCoverMapNode() {}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FNav3DCoverMapNode& Node) {
	Ar << Node.Locations;
	return Ar;
}

struct NAV3D_API FNav3DCoverMap {
	TMap<FName, FNav3DCoverMapNode> Nodes;

	void Reset() {
		Nodes.Empty();
	}

	int32 GetSize() const {
		int32 Size = Nodes.Num() * sizeof(FNav3DCoverMapNode);
		for (auto& Node: Nodes) {
			for (auto& Index: Node.Value.Locations) {
				Size += Node.Value.Locations[Index.Key].Num() * sizeof(FVector);
			}
		}
		return Size;
	}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FNav3DCoverMap& CoverMap) {
	Ar << CoverMap.Nodes;
	return Ar;
}

struct NAV3D_API FNav3DDebugEdge {

	FVector Start;
	FVector End;
	uint8 LayerIndex;

	FNav3DDebugEdge() :
        Start(FVector::ZeroVector),
		End(FVector::ZeroVector),
        LayerIndex(0) {}

	FNav3DDebugEdge(const FVector Start, const FVector End, const uint8 LayerIndex) :
        Start(Start),
        End(End),
        LayerIndex(LayerIndex) {}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FNav3DDebugEdge& DebugEdge) {
	Ar << DebugEdge.Start;
	Ar << DebugEdge.End;
	Ar << DebugEdge.LayerIndex;
	return Ar;
}