#pragma once

#include "CoreMinimal.h"
#include "Nav3DPathingTypes.generated.h"

class UNav3DPathTraversalCostCalculator;
class UNav3DPathHeuristicCalculator;

UENUM(BlueprintType)
enum class ENav3DPathingAlgorithm : uint8
{
    // Standard A* pathfinding - guaranteed shortest path but follows voxel centers (jaggy paths)
    AStar UMETA(DisplayName = "A*"),
    
    // Theta* with line-of-sight optimization - smoother paths with direct shortcuts when possible
    ThetaStar UMETA(DisplayName = "Theta*"),
    
    // Lazy Theta* - best balance of performance and path quality, deferred line-of-sight checking
    LazyThetaStar UMETA(DisplayName = "Lazy Theta*")
};

UENUM(BlueprintType)
enum class ENav3DPathingLogVerbosity : uint8
{
    // No pathfinding logs - for production builds
    Silent UMETA(DisplayName = "Silent"),
    
    // Basic start/end/result logging - minimal performance impact
    Standard UMETA(DisplayName = "Standard"),
    
    // Algorithm progress and iteration logging - moderate performance impact
    Detailed UMETA(DisplayName = "Detailed"),
    
    // Full debug logging with all internal operations - high performance impact
    Verbose UMETA(DisplayName = "Verbose")
};

USTRUCT(BlueprintType)
struct NAV3D_API FNav3DPathingRequest
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	FVector StartLocation = FVector::ZeroVector;

	UPROPERTY(BlueprintReadWrite)
	FVector EndLocation = FVector::ZeroVector;

	UPROPERTY(BlueprintReadWrite)
	const class ANav3DData* NavData = nullptr;

	UPROPERTY(BlueprintReadWrite)
	FNavAgentProperties AgentProperties;

	UPROPERTY(BlueprintReadWrite)
	ENav3DPathingAlgorithm Algorithm = ENav3DPathingAlgorithm::LazyThetaStar;

	UPROPERTY(BlueprintReadWrite)
	bool bSmoothPath = true;

	UPROPERTY(BlueprintReadWrite)
	int32 SmoothingSubdivisions = 2;

	UPROPERTY(BlueprintReadWrite)
	ENav3DPathingLogVerbosity LogVerbosity = ENav3DPathingLogVerbosity::Standard;

	// Cost calculation system
	UPROPERTY(BlueprintReadWrite)
	UNav3DPathTraversalCostCalculator* CostCalculator = nullptr;

	// Heuristic calculation system
	UPROPERTY(BlueprintReadWrite)
	UNav3DPathHeuristicCalculator* HeuristicCalculator = nullptr;

	// Heuristic scaling factor (goal bias)
	UPROPERTY(BlueprintReadWrite)
	float HeuristicScale = 1.0f;

	// Node size compensation for hierarchical optimization
	UPROPERTY(BlueprintReadWrite)
	bool bUseNodeSizeCompensation = false;
};

USTRUCT()
struct NAV3D_API FNav3DQueryFilterSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Instanced)
	UNav3DPathTraversalCostCalculator* TraversalCostCalculator = nullptr;

	UPROPERTY(EditAnywhere, Instanced)
	UNav3DPathHeuristicCalculator* HeuristicCalculator = nullptr;

	UPROPERTY(EditDefaultsOnly)
	float HeuristicScale = 1.0f;

	// If set to true, this will lower the cost of traversing bigger nodes, and
	// make the PathFinding more favorable traversing them
	UPROPERTY(EditDefaultsOnly)
	uint8 bUseNodeSizeCompensation : 1 = 0;

	UPROPERTY(EditDefaultsOnly)
	uint8 bSmoothPaths : 1 = 0;

	// How many intermediate points we will generate between the points returned
	// by the PathFinding in order to smooth the curve (the bigger, the smoother)
	UPROPERTY(EditDefaultsOnly, meta = (EditCondition = "bSmoothPaths == true"))
	int SmoothingSubdivisions = 10.0f;
};