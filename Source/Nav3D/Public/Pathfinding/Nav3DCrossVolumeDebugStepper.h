#pragma once

#include "CoreMinimal.h"
#include "Nav3DTypes.h"

class ANav3DDataChunkActor;
class FNav3DVolumeNavigationData;

/**
 * Debug stepper for cross-volume pathfinding diagnostics
 * Provides step-by-step visualization and debugging of cross-volume pathfinding failures
 */
class NAV3D_API FNav3DCrossVolumeDebugStepper
{
public:
	struct FDebugStep
	{
		FString StepName;
		FString Description;
		FVector Position;
		FNav3DNodeAddress NodeAddress;
		bool bSuccess = false;
		FString FailureReason;
		float Timestamp = 0.0f;
		
		FDebugStep() = default;
		FDebugStep(const FString& InStepName, const FString& InDescription, const FVector& InPosition, bool bInSuccess, const FString& InFailureReason = TEXT(""))
			: StepName(InStepName), Description(InDescription), Position(InPosition), bSuccess(bInSuccess), FailureReason(InFailureReason)
		{
			Timestamp = FPlatformTime::Seconds();
		}
	};

	struct FDebugSession
	{
		FString SessionId;
		FVector StartLocation;
		FVector EndLocation;
		TArray<FDebugStep> Steps;
		bool bCompleted = false;
		FString FinalResult;
		float TotalTime = 0.0f;
		
		FDebugSession() = default;
		FDebugSession(const FString& InSessionId, const FVector& InStart, const FVector& InEnd)
			: SessionId(InSessionId), StartLocation(InStart), EndLocation(InEnd)
		{
		}
	};

	// Start a new debug session
	static FDebugSession* StartDebugSession(const FString& SessionId, const FVector& StartLocation, const FVector& EndLocation);
	
	// Add a debug step to the current session
	static void AddDebugStep(FDebugSession* Session, const FString& StepName, const FString& Description, 
	                        const FVector& Position, bool bSuccess, const FString& FailureReason = TEXT(""));
	
	// Add a debug step with node address information
	static void AddDebugStep(FDebugSession* Session, const FString& StepName, const FString& Description,
	                        const FVector& Position, const FNav3DNodeAddress& NodeAddress, bool bSuccess, const FString& FailureReason = TEXT(""));
	
	// Complete a debug session
	static void CompleteDebugSession(FDebugSession* Session, bool bSuccess, const FString& FinalResult);
	
	// Log debug session summary
	static void LogDebugSessionSummary(const FDebugSession* Session);
	
	// Get debug session by ID
	static FDebugSession* GetDebugSession(const FString& SessionId);
	
	// Clear all debug sessions
	static void ClearAllDebugSessions();
	
	// Export debug session to file
	static bool ExportDebugSessionToFile(const FDebugSession* Session, const FString& FilePath);
	static FString GenerateSessionId();
	
private:
	static TMap<FString, FDebugSession*> ActiveDebugSessions;
};
