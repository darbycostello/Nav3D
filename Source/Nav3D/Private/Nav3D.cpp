
#include "Nav3D.h"

#if WITH_EDITOR
DEFINE_LOG_CATEGORY(LogNav3D)
#endif

#define LOCTEXT_NAMESPACE "FNav3DModule"

void FNav3DModule::StartupModule()
{
#if WITH_EDITOR
	UE_LOG(LogNav3D, Warning, TEXT("Nav3D: Module Startup"));
#endif
}

void FNav3DModule::ShutdownModule()
{
#if WITH_EDITOR
	UE_LOG(LogNav3D, Warning, TEXT("Nav3D: Module Shutdown"));
#endif
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FNav3DModule, Nav3D)