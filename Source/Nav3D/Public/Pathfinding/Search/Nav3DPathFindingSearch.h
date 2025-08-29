// ReSharper disable CppUE4CodingStandardNamingViolationWarning
#pragma once

#include "Pathfinding/Nav3DPathFindingProcessor.h"
#include "Pathfinding/Nav3DPathFindingTypes.h"
#include "Nav3DVolumeNavigationData.h"
#include "Nav3DPathFindingSearch.generated.h"

enum class NAV3D_API ENav3DPathFindingState : uint8
{
	Init,
	ProcessNode,
	ProcessNeighbour,
	Ended
};

UCLASS(HideDropdown, NotBlueprintable, EditInlineNew)
class NAV3D_API UNav3DPathFindingSearch : public UObject
{
	GENERATED_BODY()

public:
	virtual ENavigationQueryResult::Type GetPath(
		FNav3DPath& NavigationPath,
		const FNav3DPathFindingParameters& Params) const;
	virtual TSharedPtr<FNav3DPathStepper> GetDebugPathStepper(
		FNav3DPathFinderDebugData& DebugData,
		const FNav3DPathFindingParameters Params) const;
};
