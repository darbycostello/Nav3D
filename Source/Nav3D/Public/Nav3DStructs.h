#pragma once

#include "CoreMinimal.h"
#include "Nav3D/Private/libmorton/morton.h"

enum NAV3D_API EPathFindingHeuristic { Manhattan, Euclidean };

struct NAV3D_API FNav3DOctreeEdge {
	uint8 LayerIndex:4;
	uint_fast32_t NodeIndex:22;
	uint8 SubNodeIndex:6;

	FNav3DOctreeEdge() :
		LayerIndex(15),
		NodeIndex(0),
		SubNodeIndex(0)
	{}

	FNav3DOctreeEdge(const uint8 LayerIndex, const uint_fast32_t NodeIndex, const uint8 SubNodeIndex) :
		LayerIndex(LayerIndex),
		NodeIndex(NodeIndex),
		SubNodeIndex(SubNodeIndex)
	{}

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
	uint8 LOD = 0;
	
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
	void SetLOD(const int32 NewLOD) { LOD = FMath::Clamp(NewLOD, 0, 2); }
	uint8 GetLOD() const { return LOD; } 
	void SetSubNode(const uint8 Index) { SubNodes |= 1ULL << Index; }
	bool GetSubNode(const uint_fast64_t Index) const { return (SubNodes & 1ULL << Index) != 0; }
	void ClearSubNode(const uint8 Index) { SubNodes &= !(1ULL << Index); }
	bool IsOccluded() const { return SubNodes == -1; }
	bool IsEmpty() const { return SubNodes == 0; }
};

FORCEINLINE FArchive& operator<<(FArchive& Archive, FNav3DOctreeLeaf& Leaf)
{
	Archive << Leaf.SubNodes;
	Archive << Leaf.LOD;
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
		FirstChild(FNav3DOctreeEdge::GetInvalidEdge())
	{}

	bool IsDefault() const { return MortonCode == 0; }
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

struct NAV3D_API FNav3DPathPoint {
    FVector PointLocation;
    int32 PointLayer;
	
	FNav3DPathPoint() :
		PointLocation(FVector()),
		PointLayer(-1)
	{}
	FNav3DPathPoint(const FVector& Location, const uint8 LayerIndex) :
		PointLocation(Location),
		PointLayer(LayerIndex)
	{}
};

typedef TSharedPtr<struct FNav3DPath, ESPMode::ThreadSafe> FNav3DPathSharedPtr;
struct NAV3D_API FNav3DPath {	
	bool bIsReady = false;
	TArray<FNav3DPathPoint> PathPoints;
	
	void AddPoint(const FNav3DPathPoint& PathPoint) { PathPoints.Add(PathPoint); }
	void Reset() { PathPoints.Empty(); }
	TArray<FNav3DPathPoint>& GetPathPoints() { return PathPoints; }
	void SetPathPoints(const TArray<FNav3DPathPoint> NewPoints) { PathPoints = NewPoints; }
	bool IsReady() const { return bIsReady; }
	void SetIsReady(const bool bEnabled) { bIsReady = bEnabled; }
	void GetPath(TArray<FVector> &Path) {
		for (const auto& Point: PathPoints) {
			Path.Add(Point.PointLocation);
		}
	}
};

struct NAV3D_API FNav3DPathFindingConfig
{
	float EstimateWeight;
	float NodeSizeCompensation;
	EPathFindingHeuristic Heuristic;
	int32 PathSmoothing;

	FNav3DPathFindingConfig() :
		EstimateWeight(5.0f),
		NodeSizeCompensation(1.0f),
		Heuristic(Euclidean),
		PathSmoothing(3) {
	}
};