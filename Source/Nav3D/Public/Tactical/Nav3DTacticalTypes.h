#pragma once

#include "Nav3DTacticalTypes.generated.h"

class FNav3DVolumeNavigationData;

// Visibility relationship between observers and targets
UENUM(BlueprintType)
enum class ETacticalVisibility : uint8
{
    TargetVisible    UMETA(DisplayName = "Target Visible to Observer"),
    MutuallyVisible  UMETA(DisplayName = "Mutually Visible"),
    TargetOccluded   UMETA(DisplayName = "Target Occluded from Observer"),
    MutuallyOccluded UMETA(DisplayName = "Mutually Occluded")
};

// Distance preference for tactical queries
UENUM(BlueprintType)
enum class ETacticalDistance : uint8
{
    Any     UMETA(DisplayName = "Any Distance"),
    Closest UMETA(DisplayName = "Closest"),
    Median  UMETA(DisplayName = "Median Distance"),
    Furthest UMETA(DisplayName = "Furthest")
};

// Region size preference for tactical queries
UENUM(BlueprintType)
enum class ETacticalRegion : uint8
{
    Any      UMETA(DisplayName = "Any Size"),
    Smallest UMETA(DisplayName = "Smallest"),
    Median   UMETA(DisplayName = "Medium Sized"),
    Largest  UMETA(DisplayName = "Largest")
};

// Struct to hold information about position candidates
USTRUCT(BlueprintType)
struct FPositionCandidate
{
    GENERATED_BODY()

    // Region ID this candidate belongs to
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    int32 RegionId;

    // Position in world space
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    FVector Position;

    // Path distance from start position (through region graph)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    float PathDistance;

    // Direct distance (as the crow flies) from start position
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    float DirectDistance;

    // Region size (volume)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    float RegionSize;

    // Overall score (higher is better)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Nav3D|Tactical")
    float Score;

    // Constructor
    FPositionCandidate()
        : RegionId(-1)
        , Position(FVector::ZeroVector)
        , PathDistance(0.0f)
        , DirectDistance(0.0f)
        , RegionSize(0.0f)
        , Score(0.0f)
    {}
};

// A region is a box-shaped volume of free space
USTRUCT(BlueprintType)
struct FNav3DRegion
{
    GENERATED_BODY()

    // Unique identifier for this region
    UPROPERTY()
    int32 Id;

    // The region's bounds in world space
    UPROPERTY()
    FBox Bounds;

    // SVO layer index this region belongs to
    UPROPERTY()
    int32 LayerIndex;

    // List of adjacent region IDs
    UPROPERTY()
    TArray<int32> AdjacentRegionIds;

    UPROPERTY()
    TArray<int32> VisibilitySet;

    FNav3DRegion()
        : Id(-1)
        , Bounds(ForceInit)
        , LayerIndex(-1) {}

    FNav3DRegion(const int32 InId, const FBox& InBounds, const int32 InLayerIndex)
        : Id(InId)
        , Bounds(InBounds)
        , LayerIndex(InLayerIndex) {}

    friend FArchive& operator<<(FArchive& Ar, FNav3DRegion& Region)
    {
        Ar << Region.Id;
        Ar << Region.Bounds;
        Ar << Region.LayerIndex;
        Ar << Region.AdjacentRegionIds;
        Ar << Region.VisibilitySet;
    
        return Ar;
    }

    friend FArchive& operator<<(FArchive& Ar, FNav3DRegion& Region);
};

// Helper struct for region construction
struct FNav3DRegionBuilder
{
    // Region ID
    int32 Id;
    
    // SVO layer index
    int32 LayerIndex;
    
    // Min/max coordinates in grid space
    FIntVector MinCoord;
    FIntVector MaxCoord;
    
    // Set of Morton codes contained in this region
    TSet<uint64> MortonCodes;
    
    // Set of adjacent region IDs
    TSet<int32> AdjacentRegionIds;

    // Convert to final FNav3DRegion
    FNav3DRegion ToRegion(const FNav3DVolumeNavigationData* VolumeData) const;
};

struct FBoxRegion
{
    FIntVector Min;
    FIntVector Max;
    int32 Id;
    int32 LayerIndex;
    
    FBoxRegion() : Min(0, 0, 0), Max(0, 0, 0), Id(-1), LayerIndex(-1) {}
    
    FBoxRegion(const int32 InId, const FIntVector InMin, const FIntVector InMax, const int32 InLayerIndex)
        : Min(InMin), Max(InMax), Id(InId), LayerIndex(InLayerIndex) {}
    
    // Convert to a region builder
    FNav3DRegionBuilder ToRegionBuilder(const TArray<TPair<uint64, FIntVector>>& FreeVoxels) const;
    
    // Check if a coordinate is contained in this box
    bool Contains(const FIntVector& Coord) const;
    
    // Get the volume of this box in cells
    int32 GetVolume() const;
};

USTRUCT(BlueprintType)
struct FNav3DTacticalData
{
    GENERATED_BODY()

    // All regions in the system
    UPROPERTY()
    TArray<FNav3DRegion> Regions;

    friend FArchive& operator<<(FArchive& Ar, FNav3DTacticalData& TacticalData);

    bool IsRegionVisibilityMatch(int32 ViewerRegionId, int32 TargetRegionId, ETacticalVisibility VisibilityType) const;

    // Add a region ID to a region's visibility set
    void AddToVisibilitySet(const int32 ViewerRegionId, const int32 VisibleRegionId);

    // Find which region contains a position
    int32 FindContainingRegion(const FVector& Position) const;

    friend FArchive& operator<<(FArchive& Ar, FNav3DTacticalData& TacticalData)
    {
        // Serialize the regions array
        Ar << TacticalData.Regions;
        return Ar;
    }
};