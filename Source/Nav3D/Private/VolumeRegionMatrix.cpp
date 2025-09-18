#include "Nav3DTypes.h"

void FVolumeRegionMatrix::SetRegionReference(const uint8 LocalRegionID, const uint16 TargetVolumeID, const uint8 TargetRegionID)
{
    check(LocalRegionID < 64);
    check(TargetVolumeID < 1024);
    check(TargetRegionID < 64);

    const uint16 Key = EncodeKey(LocalRegionID, TargetVolumeID);
    uint64& RegionMask = SparseReferences.FindOrAdd(Key);
    RegionMask |= (1ULL << TargetRegionID);
}

bool FVolumeRegionMatrix::HasReference(const uint8 LocalRegionID, const uint16 TargetVolumeID, const uint8 TargetRegionID) const
{
    check(LocalRegionID < 64);
    check(TargetVolumeID < 1024);
    check(TargetRegionID < 64);

    const uint16 Key = EncodeKey(LocalRegionID, TargetVolumeID);
    if (const uint64* RegionMask = SparseReferences.Find(Key))
    {
        return (*RegionMask & (1ULL << TargetRegionID)) != 0;
    }
    return false;
}

uint64 FVolumeRegionMatrix::GetReferenceMask(const uint8 LocalRegionID, const uint16 TargetVolumeID) const
{
    check(LocalRegionID < 64);
    check(TargetVolumeID < 1024);

    const uint16 Key = EncodeKey(LocalRegionID, TargetVolumeID);
    return SparseReferences.FindRef(Key);
}

void FVolumeRegionMatrix::SetReferenceMask(const uint8 LocalRegionID, const uint16 TargetVolumeID, const uint64 RegionMask)
{
    check(LocalRegionID < 64);
    check(TargetVolumeID < 1024);

    if (RegionMask == 0)
    {
        const uint16 Key = EncodeKey(LocalRegionID, TargetVolumeID);
        SparseReferences.Remove(Key);
    }
    else
    {
        const uint16 Key = EncodeKey(LocalRegionID, TargetVolumeID);
        SparseReferences.FindOrAdd(Key) = RegionMask;
    }
}

void FVolumeRegionMatrix::ClearRegionReferences(const uint8 LocalRegionID)
{
    check(LocalRegionID < 64);

    for (auto It = SparseReferences.CreateIterator(); It; ++It)
    {
        uint8 DecodedLocalRegionID;
        uint16 DecodedTargetVolumeID;
        DecodeKey(It.Key(), DecodedLocalRegionID, DecodedTargetVolumeID);
        
        if (DecodedLocalRegionID == LocalRegionID)
        {
            It.RemoveCurrent();
        }
    }
}

void FVolumeRegionMatrix::GetReferencedVolumes(const uint8 LocalRegionID, TArray<uint16>& OutVolumeIDs) const
{
    check(LocalRegionID < 64);

    OutVolumeIDs.Reset();
    
    for (const auto& Pair : SparseReferences)
    {
        uint8 DecodedLocalRegionID;
        uint16 DecodedTargetVolumeID;
        DecodeKey(Pair.Key, DecodedLocalRegionID, DecodedTargetVolumeID);
        
        if (DecodedLocalRegionID == LocalRegionID)
        {
            OutVolumeIDs.Add(DecodedTargetVolumeID);
        }
    }
}
