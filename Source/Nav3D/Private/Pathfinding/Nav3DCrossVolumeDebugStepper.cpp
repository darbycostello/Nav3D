#include "Pathfinding/Nav3DCrossVolumeDebugStepper.h"
#include "Nav3D.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/DateTime.h"

TMap<FString, FNav3DCrossVolumeDebugStepper::FDebugSession*> FNav3DCrossVolumeDebugStepper::ActiveDebugSessions;

FNav3DCrossVolumeDebugStepper::FDebugSession* FNav3DCrossVolumeDebugStepper::StartDebugSession(
	const FString& SessionId, const FVector& StartLocation, const FVector& EndLocation)
{
	UE_LOG(LogNav3D, Verbose, TEXT("CrossVolumeDebugStepper: Starting debug session '%s' from %s to %s"), 
	       *SessionId, *StartLocation.ToString(), *EndLocation.ToString());
	
	FDebugSession* Session = new FDebugSession(SessionId, StartLocation, EndLocation);
	ActiveDebugSessions.Add(SessionId, Session);
	
	// Add initial step
	AddDebugStep(Session, TEXT("SessionStart"), TEXT("Cross-volume pathfinding session started"), 
	             StartLocation, true);
	
	return Session;
}

void FNav3DCrossVolumeDebugStepper::AddDebugStep(FDebugSession* Session, const FString& StepName, 
	const FString& Description, const FVector& Position, bool bSuccess, const FString& FailureReason)
{
	if (!Session)
	{
		UE_LOG(LogNav3D, Warning, TEXT("CrossVolumeDebugStepper: Cannot add debug step - null session"));
		return;
	}
	
	FDebugStep Step(StepName, Description, Position, bSuccess, FailureReason);
	Session->Steps.Add(Step);
	
	UE_LOG(LogNav3D, Verbose, TEXT("CrossVolumeDebugStepper: [%s] %s - %s at %s (Success: %s)"), 
	       *Session->SessionId, *StepName, *Description, *Position.ToString(), 
	       bSuccess ? TEXT("Yes") : TEXT("No"));
	
	if (!bSuccess && !FailureReason.IsEmpty())
	{
		UE_LOG(LogNav3D, Warning, TEXT("CrossVolumeDebugStepper: [%s] %s - Failure: %s"), 
		       *Session->SessionId, *StepName, *FailureReason);
	}
}

void FNav3DCrossVolumeDebugStepper::AddDebugStep(FDebugSession* Session, const FString& StepName,
	const FString& Description, const FVector& Position, const FNav3DNodeAddress& NodeAddress, 
	bool bSuccess, const FString& FailureReason)
{
	if (!Session)
	{
		UE_LOG(LogNav3D, Warning, TEXT("CrossVolumeDebugStepper: Cannot add debug step - null session"));
		return;
	}
	
	FDebugStep Step(StepName, Description, Position, bSuccess, FailureReason);
	Step.NodeAddress = NodeAddress;
	Session->Steps.Add(Step);
	
	UE_LOG(LogNav3D, Verbose, TEXT("CrossVolumeDebugStepper: [%s] %s - %s at %s (Layer: %d, Node: %d) (Success: %s)"), 
	       *Session->SessionId, *StepName, *Description, *Position.ToString(), 
	       NodeAddress.LayerIndex, NodeAddress.NodeIndex, bSuccess ? TEXT("Yes") : TEXT("No"));
	
	if (!bSuccess && !FailureReason.IsEmpty())
	{
		UE_LOG(LogNav3D, Warning, TEXT("CrossVolumeDebugStepper: [%s] %s - Failure: %s"), 
		       *Session->SessionId, *StepName, *FailureReason);
	}
}

void FNav3DCrossVolumeDebugStepper::CompleteDebugSession(FDebugSession* Session, bool bSuccess, const FString& FinalResult)
{
	if (!Session)
	{
		UE_LOG(LogNav3D, Warning, TEXT("CrossVolumeDebugStepper: Cannot complete debug session - null session"));
		return;
	}
	
	Session->bCompleted = true;
	Session->FinalResult = FinalResult;
	Session->TotalTime = FPlatformTime::Seconds() - Session->Steps[0].Timestamp;
	
	// Add final step
	AddDebugStep(Session, TEXT("SessionComplete"), 
	             FString::Printf(TEXT("Cross-volume pathfinding session completed (Success: %s)"), 
	                            bSuccess ? TEXT("Yes") : TEXT("No")), 
	             Session->EndLocation, bSuccess, bSuccess ? TEXT("") : FinalResult);
	
	UE_LOG(LogNav3D, Verbose, TEXT("CrossVolumeDebugStepper: Completed debug session '%s' in %.3f seconds (Success: %s)"), 
	       *Session->SessionId, Session->TotalTime, bSuccess ? TEXT("Yes") : TEXT("No"));
	
	// Log summary
	LogDebugSessionSummary(Session);
}

void FNav3DCrossVolumeDebugStepper::LogDebugSessionSummary(const FDebugSession* Session)
{
	if (!Session)
	{
		return;
	}
	
	UE_LOG(LogNav3D, Log, TEXT("=== Cross-Volume Debug Session Summary ==="));
	UE_LOG(LogNav3D, Log, TEXT("Session ID: %s"), *Session->SessionId);
	UE_LOG(LogNav3D, Log, TEXT("Start: %s"), *Session->StartLocation.ToString());
	UE_LOG(LogNav3D, Log, TEXT("End: %s"), *Session->EndLocation.ToString());
	UE_LOG(LogNav3D, Log, TEXT("Total Time: %.3f seconds"), Session->TotalTime);
	UE_LOG(LogNav3D, Log, TEXT("Total Steps: %d"), Session->Steps.Num());
	UE_LOG(LogNav3D, Log, TEXT("Completed: %s"), Session->bCompleted ? TEXT("Yes") : TEXT("No"));
	UE_LOG(LogNav3D, Log, TEXT("Final Result: %s"), *Session->FinalResult);
	
	int32 SuccessCount = 0;
	int32 FailureCount = 0;
	
	for (const FDebugStep& Step : Session->Steps)
	{
		if (Step.bSuccess)
		{
			SuccessCount++;
		}
		else
		{
			FailureCount++;
			UE_LOG(LogNav3D, Log, TEXT("  FAILED: %s - %s"), *Step.StepName, *Step.FailureReason);
		}
	}
	
	UE_LOG(LogNav3D, Log, TEXT("Successful Steps: %d"), SuccessCount);
	UE_LOG(LogNav3D, Log, TEXT("Failed Steps: %d"), FailureCount);
	UE_LOG(LogNav3D, Log, TEXT("========================================="));
}

FNav3DCrossVolumeDebugStepper::FDebugSession* FNav3DCrossVolumeDebugStepper::GetDebugSession(const FString& SessionId)
{
	if (FDebugSession** Session = ActiveDebugSessions.Find(SessionId))
	{
		return *Session;
	}
	return nullptr;
}

void FNav3DCrossVolumeDebugStepper::ClearAllDebugSessions()
{
	UE_LOG(LogNav3D, Verbose, TEXT("CrossVolumeDebugStepper: Clearing %d debug sessions"), ActiveDebugSessions.Num());
	
	for (auto& Pair : ActiveDebugSessions)
	{
		delete Pair.Value;
	}
	ActiveDebugSessions.Empty();
}

bool FNav3DCrossVolumeDebugStepper::ExportDebugSessionToFile(const FDebugSession* Session, const FString& FilePath)
{
	if (!Session)
	{
		UE_LOG(LogNav3D, Warning, TEXT("CrossVolumeDebugStepper: Cannot export null session"));
		return false;
	}
	
	FString Output;
	Output += FString::Printf(TEXT("Cross-Volume Pathfinding Debug Session\n"));
	Output += FString::Printf(TEXT("=====================================\n"));
	Output += FString::Printf(TEXT("Session ID: %s\n"), *Session->SessionId);
	Output += FString::Printf(TEXT("Start Location: %s\n"), *Session->StartLocation.ToString());
	Output += FString::Printf(TEXT("End Location: %s\n"), *Session->EndLocation.ToString());
	Output += FString::Printf(TEXT("Total Time: %.3f seconds\n"), Session->TotalTime);
	Output += FString::Printf(TEXT("Completed: %s\n"), Session->bCompleted ? TEXT("Yes") : TEXT("No"));
	Output += FString::Printf(TEXT("Final Result: %s\n"), *Session->FinalResult);
	Output += FString::Printf(TEXT("Total Steps: %d\n\n"), Session->Steps.Num());
	
	Output += FString::Printf(TEXT("Step Details:\n"));
	Output += FString::Printf(TEXT("============\n"));
	
	for (int32 i = 0; i < Session->Steps.Num(); ++i)
	{
		const FDebugStep& Step = Session->Steps[i];
		Output += FString::Printf(TEXT("%d. %s\n"), i + 1, *Step.StepName);
		Output += FString::Printf(TEXT("   Description: %s\n"), *Step.Description);
		Output += FString::Printf(TEXT("   Position: %s\n"), *Step.Position.ToString());
		Output += FString::Printf(TEXT("   Node Address: Layer=%d, Node=%d, SubNode=%llu\n"), 
		                          Step.NodeAddress.LayerIndex, Step.NodeAddress.NodeIndex, Step.NodeAddress.SubNodeIndex);
		Output += FString::Printf(TEXT("   Success: %s\n"), Step.bSuccess ? TEXT("Yes") : TEXT("No"));
		if (!Step.bSuccess && !Step.FailureReason.IsEmpty())
		{
			Output += FString::Printf(TEXT("   Failure Reason: %s\n"), *Step.FailureReason);
		}
		Output += FString::Printf(TEXT("   Timestamp: %.3f\n\n"), Step.Timestamp);
	}
	
	bool bSuccess = FFileHelper::SaveStringToFile(Output, *FilePath);
	if (bSuccess)
	{
		UE_LOG(LogNav3D, Log, TEXT("CrossVolumeDebugStepper: Exported debug session to %s"), *FilePath);
	}
	else
	{
		UE_LOG(LogNav3D, Warning, TEXT("CrossVolumeDebugStepper: Failed to export debug session to %s"), *FilePath);
	}
	
	return bSuccess;
}

FString FNav3DCrossVolumeDebugStepper::GenerateSessionId()
{
	FDateTime Now = FDateTime::Now();
	return FString::Printf(TEXT("CrossVol_%04d%02d%02d_%02d%02d%02d_%03d"), 
	                       Now.GetYear(), Now.GetMonth(), Now.GetDay(),
	                       Now.GetHour(), Now.GetMinute(), Now.GetSecond(), 
	                       Now.GetMillisecond());
}
