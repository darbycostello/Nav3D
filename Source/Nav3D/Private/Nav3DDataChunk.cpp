#include "Nav3DDataChunk.h"
#include "Nav3DTypes.h"

void UNav3DDataChunk::Serialize(FArchive& Archive)
{
	Super::Serialize(Archive);

	ENav3DVersion Version = ENav3DVersion::Latest;
	Archive << Version;
	auto N3dSizeBytes = 0;
	const auto N3dSizePosition = Archive.Tell();

	Archive << N3dSizeBytes;

	if (Archive.IsLoading())
	{
		if (Version < ENav3DVersion::MinCompatible)
		{
			// incompatible, just skip over this data
			Archive.Seek(N3dSizePosition + N3dSizeBytes);
			return;
		}
	}

	auto VolumeCount = NavigationData.Num();
	Archive << VolumeCount;
	if (Archive.IsLoading())
	{
		NavigationData.Reset(VolumeCount);
		NavigationData.SetNum(VolumeCount);
	}

	for (auto Index = 0; Index < VolumeCount; Index++)
	{
		NavigationData[Index].Serialize(Archive, Version);
	}

	// Serialize boundary voxels (Morton-coded)
	int32 BoundaryCount = BoundaryVoxels.Num();
	Archive << BoundaryCount;
	if (Archive.IsLoading())
	{
		BoundaryVoxels.Reset(BoundaryCount);
		BoundaryVoxels.SetNum(BoundaryCount);
	}
	for (int32 i = 0; i < BoundaryCount; ++i)
	{
		Archive << BoundaryVoxels[i].Morton;
		Archive << BoundaryVoxels[i].AdjacentChunkVoxels;
		if (Archive.IsSaving())
		{
			uint8 Flag = BoundaryVoxels[i].bIsNavigable ? 1 : 0;
			Archive << Flag;
		}
		else
		{
			uint8 Flag = 0;
			Archive << Flag;
			BoundaryVoxels[i].bIsNavigable = Flag != 0;
		}
	}
	if (Archive.IsLoading())
	{
		MortonToBoundaryIndex.Empty(BoundaryVoxels.Num());
		for (int32 i = 0; i < BoundaryVoxels.Num(); ++i)
		{
			MortonToBoundaryIndex.Add(BoundaryVoxels[i].Morton, i);
		}
	}

	if (Archive.IsSaving())
	{
		const auto CurrentPosition = Archive.Tell();

		N3dSizeBytes = CurrentPosition - N3dSizePosition;

		Archive.Seek(N3dSizePosition);
		Archive << N3dSizeBytes;
		Archive.Seek(CurrentPosition);
	}
}

void UNav3DDataChunk::AddNavigationData(FNav3DVolumeNavigationData& NavData)
{
	NavData.SetInNavigationDataChunk(true);
	NavigationData.Emplace(NavData);
}

void UNav3DDataChunk::ReleaseNavigationData() { NavigationData.Reset(); }

const FNav3DVolumeNavigationData* UNav3DDataChunk::GetVolumeNavigationData() const
{
	return NavigationData.Num() > 0 ? &NavigationData[0] : nullptr;
}

FBox UNav3DDataChunk::GetBounds() const
{
	if (NavigationData.Num() > 0)
	{
		return NavigationData[0].GetVolumeBounds();
	}
	return FBox(ForceInit);
}
