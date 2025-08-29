#include "TriBoxOverlap.h"
#include "CoreMinimal.h"
#include "Math/Vector.h"

using namespace Nav3D::TriBoxOverlapUtils;

static void FVectorToFloatArray(const FVector& Vec, float Array[3])
{
	Array[0] = Vec.X;
	Array[1] = Vec.Y;
	Array[2] = Vec.Z;
}

bool TriBoxOverlap(const FVector& BoxCenter, const FVector& BoxHalfSize, const FVector& TriVert0,
                   const FVector& TriVert1, const FVector& TriVert2)
{
	float V0[3], V1[3], V2[3];
	float Min, Max, Normal[3], E0[3], E1[3], E2[3];
	float BoxCenterArray[3], BoxHalfSizeArray[3];

	FVectorToFloatArray(BoxCenter, BoxCenterArray);
	FVectorToFloatArray(BoxHalfSize, BoxHalfSizeArray);

	// Move everything so that the box center is at the origin
	float TriVert0Array[3], TriVert1Array[3], TriVert2Array[3];
	FVectorToFloatArray(TriVert0, TriVert0Array);
	FVectorToFloatArray(TriVert1, TriVert1Array);
	FVectorToFloatArray(TriVert2, TriVert2Array);

	Sub(V0, TriVert0Array, BoxCenterArray);
	Sub(V1, TriVert1Array, BoxCenterArray);
	Sub(V2, TriVert2Array, BoxCenterArray);

	// Compute triangle edges
	Sub(E0, V1, V0); // Tri edge 0
	Sub(E1, V2, V1); // Tri edge 1
	Sub(E2, V0, V2); // Tri edge 2

	// Nine tests first
	Cross(Normal, E0, V0);
	if (!PlaneBoxOverlap(Normal, V0, BoxHalfSizeArray))
	{
		return false;
	}

	Cross(Normal, E1, V1);
	if (!PlaneBoxOverlap(Normal, V1, BoxHalfSizeArray))
	{
		return false;
	}

	Cross(Normal, E2, V2);
	if (!PlaneBoxOverlap(Normal, V2, BoxHalfSizeArray))
	{
		return false;
	}

	// Test overlap in {x,y,z}-directions
	// Find Min, Max of the triangle in each direction, and test for overlap in that direction

	// Test in X-direction
	TBO_FINDMINMAX(V0[GTbo_X], V1[GTbo_X], V2[GTbo_X], Min, Max);
	if (Min > BoxHalfSizeArray[GTbo_X] || Max < -BoxHalfSizeArray[GTbo_X])
	{
		return false;
	}

	// Test in Y-direction
	TBO_FINDMINMAX(V0[GTbo_Y], V1[GTbo_Y], V2[GTbo_Y], Min, Max);
	if (Min > BoxHalfSizeArray[GTbo_Y] || Max < -BoxHalfSizeArray[GTbo_Y])
	{
		return false;
	}

	// Test in Z-direction
	TBO_FINDMINMAX(V0[GTbo_Z], V1[GTbo_Z], V2[GTbo_Z], Min, Max);
	if (Min > BoxHalfSizeArray[GTbo_Z] || Max < -BoxHalfSizeArray[GTbo_Z])
	{
		return false;
	}

	// Test if the box intersects the plane of the triangle
	Cross(Normal, E0, E1);

	if (!PlaneBoxOverlap(Normal, V0, BoxHalfSizeArray))
	{
		return false;
	}

	return true;
}
