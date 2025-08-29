#include "Pathfinding/Search/Nav3DThetaStar.h"
#include "Nav3DUtils.h"
#include "Pathfinding/Nav3DPathBuilder.h"
#include "Pathfinding/Search/Nav3DPathFindingSearch.h"
#include "Pathfinding/Stepper/Nav3DPathDebug.h"

ENavigationQueryResult::Type UNav3DThetaStar::GetPath(
	FNav3DPath& NavigationPath,
	const FNav3DPathFindingParameters& Params) const
{
	FNav3DPathStepperThetaStar Stepper(Params, ThetaStarParameters);
	const auto PathBuilder = MakeShared<FNav3DPathBuilder>(NavigationPath, Stepper);

	Stepper.AddProcessor(PathBuilder);

	int Iterations = 0;

	EGraphAStarResult Result = SearchFail;
	while (Stepper.Step(Result) ==
		ENav3DPathStepperStatus::MustContinue)
	{
		Iterations++;
	}

	return FNav3DUtils::GraphAStarResultToNavigationTypeResult(Result);
}

TSharedPtr<FNav3DPathStepper> UNav3DThetaStar::GetDebugPathStepper(
	FNav3DPathFinderDebugData& DebugData,
	const FNav3DPathFindingParameters Params) const
{
	auto Stepper = MakeShared<FNav3DPathStepperThetaStar>(
		Params, ThetaStarParameters);
	const auto DebugPath =
		MakeShared<FNav3DPathDebug>(DebugData, Stepper.Get());
	Stepper->AddProcessor(DebugPath);

	return Stepper;
}
