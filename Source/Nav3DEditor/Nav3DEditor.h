#pragma once

#include "CoreMinimal.h"
#include "UnrealEd.h"

DECLARE_LOG_CATEGORY_EXTERN(LogNav3DEditor, Log, All)

class FNav3DEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

};