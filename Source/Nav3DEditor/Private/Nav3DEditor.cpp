#include "Nav3DEditor/Public/Nav3DEditor.h"
#include "Nav3DDataDetailCustomization.h"
#include "PropertyEditorModule.h"
#include "Nav3D/Public/Nav3DData.h"
#include "Modules/ModuleManager.h"

IMPLEMENT_GAME_MODULE(FNav3DEditorModule, Nav3DEditor);
DEFINE_LOG_CATEGORY(LogNav3DEditor)
#define LOCTEXT_NAMESPACE "Nav3DEditor"

void FNav3DEditorModule::StartupModule()
{
	UE_LOG(LogNav3DEditor, Verbose, TEXT("Nav3DEditor: Module Startup"));
	
	// Add the Nav3D filter button to the Details panel
	FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
	{
		const TSharedRef<FPropertySection> Section = PropertyModule.FindOrCreateSection("Actor", "Nav3D", LOCTEXT("Nav3D", "Nav3D"));
		Section->AddCategory("Nav3D");
	}

	// Register detail customization
	PropertyModule.RegisterCustomClassLayout(
		ANav3DData::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FNav3DDataDetailCustomization::MakeInstance)
	);
	
	PropertyModule.NotifyCustomizationModuleChanged();
}

void FNav3DEditorModule::ShutdownModule()
{
	UE_LOG(LogNav3DEditor, Verbose, TEXT("Nav3DEditor: Module Shutdown"));
	
	// Unregister detail customization
	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
		PropertyModule.UnregisterCustomClassLayout(ANav3DData::StaticClass()->GetFName());
	}
}

#undef LOCTEXT_NAMESPACE
