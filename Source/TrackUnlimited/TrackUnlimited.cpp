#include "Modules/ModuleManager.h"
#include "UI/TUStyle.h"

// THE STYLE SET IS REGISTERED BY THE MODULE, not lazily by the first widget that
// wants it. Slate keeps style sets in a global registry keyed by name, so
// registration is a lifecycle concern rather than a widget's: one owner, one
// place to look, and an unregister on the way out that a lazy singleton never
// gets to do.
class FTrackUnlimitedModule : public FDefaultGameModuleImpl
{
public:
	virtual void StartupModule() override
	{
		FTUStyle::Initialise();
	}

	virtual void ShutdownModule() override
	{
		FTUStyle::Shutdown();
	}
};

IMPLEMENT_PRIMARY_GAME_MODULE(FTrackUnlimitedModule, TrackUnlimited, "TrackUnlimited");
