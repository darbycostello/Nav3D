#include "Pathfinding/Core/INav3DPathfinder.h"
#include "Logging/LogMacros.h"

static const TCHAR* ToStringVerbosity(ENav3DPathingLogVerbosity Verbosity)
{
	switch (Verbosity)
	{
	case ENav3DPathingLogVerbosity::Silent: return TEXT("Silent");
	case ENav3DPathingLogVerbosity::Standard: return TEXT("Standard");
	case ENav3DPathingLogVerbosity::Detailed: return TEXT("Detailed");
	case ENav3DPathingLogVerbosity::Verbose: return TEXT("Verbose");
	default: return TEXT("Unknown");
	}
}

void INav3DPathfinder::LogPathfindingStart(const FNav3DPathingRequest& Request, const FString& AlgorithmName)
{
	if (Request.LogVerbosity == ENav3DPathingLogVerbosity::Silent)
	{
		return;
	}
	UE_LOG(LogTemp, Display, TEXT("Nav3D %s start: (%s) -> (%s), Verbosity=%s"),
		*AlgorithmName,
		*Request.StartLocation.ToString(),
		*Request.EndLocation.ToString(),
		ToStringVerbosity(Request.LogVerbosity));
}

void INav3DPathfinder::LogPathfindingResult(ENavigationQueryResult::Type Result, int32 PathPointCount, const FString& AlgorithmName)
{
	UE_LOG(LogTemp, Display, TEXT("Nav3D %s result: %d points, Result=%d"), *AlgorithmName, PathPointCount, (int32)Result);
}

void INav3DPathfinder::LogAlgorithmProgress(const FNav3DPathingRequest& Request, const FString& Message)
{
	if (Request.LogVerbosity < ENav3DPathingLogVerbosity::Detailed)
	{
		return;
	}
	UE_LOG(LogTemp, Verbose, TEXT("Nav3D algo: %s"), *Message);
}