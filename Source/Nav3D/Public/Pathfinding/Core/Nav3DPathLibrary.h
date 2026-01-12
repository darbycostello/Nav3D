#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Nav3DPathLibrary.generated.h"

UCLASS()
class NAV3D_API UNav3DPathLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// Simplified Blueprints-friendly pathfinder - just get a path from A to B
	UFUNCTION(BlueprintCallable, Category = "Nav3D|Pathfinding", meta = (WorldContext = "WorldContextObject"))
	// ReSharper disable once CppUEBlueprintCallableFunctionUnused
	static bool FindNav3DPath(
		const UObject* WorldContextObject,
		FVector StartLocation,
		FVector EndLocation,
		float AgentRadius,
		TArray<FVector>& OutPathPoints);
};