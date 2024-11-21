#include "Nav3DEditor/Nav3DEditor.h"
#include "Nav3DVolumeProperties.h"
#include "Nav3DModifierVolumeProperties.h"
// #include "PropertyEditor/Public/PropertyEditorModule.h"
#include "PropertyEditorModule.h"
IMPLEMENT_GAME_MODULE(FNav3DEditorModule, Nav3DEditor);
DEFINE_LOG_CATEGORY(LogNav3DEditor)
#define LOCTEXT_NAMESPACE "Nav3DEditor"

void FNav3DEditorModule::StartupModule()
{
	UE_LOG(LogNav3DEditor, Warning, TEXT("Nav3DEditor: Module Startup"));
	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	PropertyModule.RegisterCustomClassLayout("Nav3DVolume", FOnGetDetailCustomizationInstance::CreateStatic(&FNav3DVolumeProperties::MakeInstance));
	PropertyModule.RegisterCustomClassLayout("Nav3DModifierVolume", FOnGetDetailCustomizationInstance::CreateStatic(&FNav3DModifierVolumeProperties::MakeInstance));
}

void FNav3DEditorModule::ShutdownModule()
{
	UE_LOG(LogNav3DEditor, Warning, TEXT("Nav3DEditor: Module Shutdown"));
	FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
}

#undef LOCTEXT_NAMESPACE