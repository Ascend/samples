#include "atlas_utils.h"

using namespace std;

AtlasApp& CreateAtlasAppInstance() {
	return AtlasApp::getInstance();
}

AtlasApp& GetAtlasAppInstance() {
	return AtlasApp::getInstance();
}

int SendMessage(int dest, int msgId, std::shared_ptr<void> data) {
	AtlasApp& app = AtlasApp::getInstance();
        return app.SendMessage(dest, msgId, data);
}

int GetAtlasThreadIdByName(const string& threadName) {
	AtlasApp& app =  AtlasApp::getInstance();
	return app.GetAtlasThreadIdByName(threadName);
}
