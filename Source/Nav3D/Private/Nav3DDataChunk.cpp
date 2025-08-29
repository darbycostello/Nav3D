#include "Nav3DDataChunk.h"
#include "Nav3DVersion.h"

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
