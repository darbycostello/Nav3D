#pragma once

#include "CoreMinimal.h"
#include "Nav3DTypes.h"
#include "Nav3DRaycaster.generated.h"

// Forward declarations
class FNav3DVolumeNavigationData;

// Structs previously from the base class
struct FNav3DRaycastHit
{
    FNav3DRaycastHit()
       : bBlockingHit(false)
         , Distance(MAX_flt)
         , ImpactPoint(FVector::ZeroVector)
         , ImpactNormal(FVector::ZeroVector)
         , OccludedVoxelCount(0)
    {
    }

    bool bBlockingHit;
    float Distance;
    FVector ImpactPoint;
    FVector ImpactNormal;
    FNav3DNodeAddress NodeAddress;
    int32 OccludedVoxelCount;
};

struct FNav3DRaycasterTraversedNode
{
    FNav3DRaycasterTraversedNode() = default;

    FNav3DRaycasterTraversedNode(const FNav3DNodeAddress& NodeAddress, const bool IsOccluded)
       : NodeAddress(NodeAddress), bIsOccluded(IsOccluded)
    {
    }

    FNav3DNodeAddress NodeAddress;
    bool bIsOccluded;
};

struct FNav3DRaycasterDebugInfos
{
    FVector RaycastStartLocation;
    FVector RaycastEndLocation;
    bool Result;
    TArray<FNav3DRaycasterTraversedNode> TraversedNodes;
    TArray<FNav3DRaycasterTraversedNode> TraversedLeafNodes;
    TArray<FNav3DRaycasterTraversedNode> TraversedLeafSubNodes;
    const FNav3DVolumeNavigationData* NavigationData;
};

class NAV3D_API FNav3DRaycasterProcessor
{
public:
    virtual ~FNav3DRaycasterProcessor() = default;

    virtual void Initialize(const FNav3DVolumeNavigationData* NavigationData, const FVector From, const FVector To)
    {
    }

    virtual void SetResult(bool Result)
    {
    }

    virtual void AddTraversedNode(FNav3DNodeAddress NodeAddress, bool IsOccluded)
    {
    }

    virtual void AddTraversedLeafSubNode(FNav3DNodeAddress NodeAddress, bool IsOccluded)
    {
    }
};

class NAV3D_API FNav3DRaycasterProcessor_GenerateDebugInfos final : public FNav3DRaycasterProcessor
{
public:
    explicit FNav3DRaycasterProcessor_GenerateDebugInfos(
       FNav3DRaycasterDebugInfos& DebugInfos);

    virtual void Initialize(const FNav3DVolumeNavigationData* NavigationData,
                            const FVector From, const FVector To) override;
    virtual void SetResult(bool Result) override;
    virtual void AddTraversedNode(FNav3DNodeAddress NodeAddress,
                                  bool IsOccluded) override;
    virtual void AddTraversedLeafSubNode(FNav3DNodeAddress NodeAddress,
                                         bool IsOccluded) override;

private:
    FNav3DRaycasterDebugInfos& DebugInfos;
};

UCLASS(NotBlueprintable, EditInlineNew)
class NAV3D_API UNav3DRaycaster : public UObject
{
    GENERATED_BODY()

public:
    // Methods from the former base class
    bool Trace(const FNav3DVolumeNavigationData& VolumeNavigationData,
               const FVector& From, const FVector& To) const;

    bool Trace(const FNav3DVolumeNavigationData& VolumeNavigationData,
               const FVector& From, const FVector& To,
               FNav3DRaycastHit& OutHit) const;
               
    bool TraceCountingOccludedVoxels(const FNav3DVolumeNavigationData& VolumeNavigationData, 
                                    const FVector& From, const FVector& To, 
                                    FNav3DRaycastHit& OutHit) const;
                                    
    int32 CountOccludedVoxelsAlongRay(const FNav3DVolumeNavigationData& VolumeNavigationData, 
                                     const FVector& From, const FVector& To) const;

    void SetProcessor(const TSharedPtr<FNav3DRaycasterProcessor>& NewProcessor);
    
    static UWorld* GetWorldContext();

private:
    // Helper methods specific to octree implementation
    bool TraceInternal(const FNav3DVolumeNavigationData& VolumeNavigationData,
                      const FVector& From, const FVector& To,
                      FNav3DRaycastHit& OutHit) const;
                      
    // Modified version of TraceInternal that supports voxel counting
    bool TraceInternal(const FNav3DVolumeNavigationData& VolumeNavigationData,
                      const FVector& From, const FVector& To,
                      FNav3DRaycastHit& OutHit,
                      bool bCountAllOccludedVoxels) const;

    struct FOctreeRay
    {
       FOctreeRay(float Tx0, float Tx1, float Ty0, float Ty1, float Tz0, float Tz1);
       bool Intersects() const;
       bool IsInRange(float MaxSize) const;
       float Tx0, Tx1, Txm, Ty0, Ty1, Tym, Tz0, Tz1, Tzm;
    };

    struct FRaycastState
    {
       FRaycastState(const FVector& From, const FVector& To)
          : RayOrigin(From)
            , RayDirection((To - From).GetSafeNormal())
            , OriginalRayDirection((To - From).GetSafeNormal())
            , RaySize((To - From).Size())
            , A(0)
       {
       }

       FVector RayOrigin;
       FVector RayDirection;
       FVector OriginalRayDirection; // Store original ray direction for ray-box intersection
       float RaySize;
       uint8 A; // Direction bit flags
    };

    static uint8 GetFirstNodeIndex(const FOctreeRay& Ray);
    static uint8 GetNextNodeIndex(float Txm, int32 X, float Tym, int32 Y, float Tzm, int32 Z);
    static FVector CalculateImpactNormal(const FVector& ImpactPoint, const FVector& NodeCenter);

    bool DoesRayIntersectOccludedNode(const FOctreeRay& Ray,
                                      const FNav3DNodeAddress& NodeAddress,
                                      const FNav3DVolumeNavigationData& Data,
                                      const FRaycastState& RayState,
                                      FNav3DRaycastHit& OutHit,
                                      bool bCountAllOccludedVoxels = false) const;

    bool DoesRayIntersectOccludedNormalNode(const FOctreeRay& Ray,
                                            const FNav3DNodeAddress& NodeAddress,
                                            const FNav3DVolumeNavigationData& Data,
                                            const FRaycastState& RayState,
                                            FNav3DRaycastHit& OutHit,
                                            bool bCountAllOccludedVoxels = false) const;

    static bool DoesRayIntersectOccludedLeaf(const FOctreeRay& Ray,
                                             const FNav3DNodeAddress& NodeAddress,
                                             const FNav3DVolumeNavigationData& Data,
                                             const FRaycastState& RayState,
                                             FNav3DRaycastHit& OutHit,
                                             bool bCountAllOccludedVoxels = false);

    // Processor for debugging (formerly in base class)
    TSharedPtr<FNav3DRaycasterProcessor> Processor;

    UPROPERTY(EditAnywhere)
    uint8 bShowLineOfSightTraces : 1;
};