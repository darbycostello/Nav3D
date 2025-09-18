#include "Pathfinding/Utils/Nav3DPathSmoothing.h"

void FNav3DPathSmoothing::SmoothPath(FNav3DPath& Path, const int32 Subdivisions)
{
	if (Subdivisions <= 0)
	{
		return;
	}

	auto OldPoints = Path.GetPathPoints();
	auto OldCosts = Path.GetPathPointCosts();

	// Need at least 2 points to smooth
	if (OldPoints.Num() < 2)
	{
		return;
	}

	auto& PathPoints = Path.GetPathPoints();
	auto& PathPointCosts = Path.GetPathPointCosts();

	// Create virtual end points for smooth interpolation
	OldPoints.Insert(2 * (OldPoints[0].Location - OldPoints[1].Location), 0);
	OldPoints.Emplace(2 * (OldPoints.Last().Location - OldPoints.Last(1).Location));

	const auto NewSize = (OldPoints.Num() - 3) * Subdivisions;
	PathPoints.Reset(NewSize);
	PathPointCosts.Reset(NewSize);

	for (auto Index = 1; Index < OldPoints.Num() - 2; ++Index)
	{
		for (auto Alpha = 0; Alpha < Subdivisions; ++Alpha)
		{
			PathPoints.Emplace(GetPoint(OldPoints[Index - 1], OldPoints[Index],
			                            OldPoints[Index + 1], OldPoints[Index + 2],
			                            static_cast<float>(Alpha) / Subdivisions));

			PathPointCosts.Add(OldCosts[Index - 1] / Subdivisions);
		}
	}
}

float FNav3DPathSmoothing::GetT(const float T, const float Alpha, const FVector& P0,
                                const FVector& P1)
{
	const auto D = P1 - P0;
	const auto A = D | D; // Dot product
	const auto B = FMath::Pow(A, Alpha * 0.5f);
	return B + T;
}

FVector FNav3DPathSmoothing::GetPoint(const FVector& P0, const FVector& P1,
                                      const FVector& P2, const FVector& P3,
                                      float T, const float Alpha)
{
	constexpr auto T0 = 0.0f;
	const auto T1 = GetT(T0, Alpha, P0, P1);
	const auto T2 = GetT(T1, Alpha, P1, P2);
	const auto T3 = GetT(T2, Alpha, P2, P3);

	T = FMath::Lerp(T1, T2, T);

	const auto A1 = (T1 - T) / (T1 - T0) * P0 + (T - T0) / (T1 - T0) * P1;
	const auto A2 = (T2 - T) / (T2 - T1) * P1 + (T - T1) / (T2 - T1) * P2;
	const auto A3 = (T3 - T) / (T3 - T2) * P2 + (T - T2) / (T3 - T2) * P3;

	const auto B1 = (T2 - T) / (T2 - T0) * A1 + (T - T0) / (T2 - T0) * A2;
	const auto B2 = (T3 - T) / (T3 - T1) * A2 + (T - T1) / (T3 - T1) * A3;

	return (T2 - T) / (T2 - T1) * B1 + (T - T1) / (T2 - T1) * B2;
}
