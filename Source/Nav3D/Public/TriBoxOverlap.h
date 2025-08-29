#pragma once

#include "CoreMinimal.h"
#include "Math/Vector.h"

namespace Nav3D::TriBoxOverlapUtils
{
	constexpr int32 GTbo_X = 0;
	constexpr int32 GTbo_Y = 1;
	constexpr int32 GTbo_Z = 2;

#define TBO_FINDMINMAX(x0, x1, x2, min, max) \
    min = max = x0;   \
    if(x1<min) min=x1;\
    else if(x1>max) max=x1;\
    if(x2<min) min=x2;\
    else if(x2>max) max=x2;

	static void Cross(float Dest[3], const float V1[3], const float V2[3])
	{
		Dest[0] = V1[1] * V2[2] - V1[2] * V2[1];
		Dest[1] = V1[2] * V2[0] - V1[0] * V2[2];
		Dest[2] = V1[0] * V2[1] - V1[1] * V2[0];
	}

	static float Dot(const float V1[3], const float V2[3])
	{
		return (V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]);
	}

	static void Sub(float Dest[3], const float V1[3], const float V2[3])
	{
		Dest[0] = V1[0] - V2[0];
		Dest[1] = V1[1] - V2[1];
		Dest[2] = V1[2] - V2[2];
	}

	static int32 PlaneBoxOverlap(const float Normal[3], const float Vert[3], const float MaxBox[3])
	{
		float VMin[3], VMax[3];
		for (int32 q = GTbo_X; q <= GTbo_Z; q++)
		{
			const float v = Vert[q];
			if (Normal[q] > 0.0f)
			{
				VMin[q] = -MaxBox[q] - v;
				VMax[q] = MaxBox[q] - v;
			}
			else
			{
				VMin[q] = MaxBox[q] - v;
				VMax[q] = -MaxBox[q] - v;
			}
		}
		if (Dot(Normal, VMin) > 0.0f)
		{
			return 0;
		}
		if (Dot(Normal, VMax) >= 0.0f)
		{
			return 1;
		}
		return 0;
	}
}

bool TriBoxOverlap(const FVector& BoxCenter, const FVector& BoxHalfSize, const FVector& TriVert0,
                   const FVector& TriVert1, const FVector& TriVert2);
