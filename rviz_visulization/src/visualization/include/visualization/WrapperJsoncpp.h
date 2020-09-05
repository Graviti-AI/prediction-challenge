#pragma once
#include <string>
#include <vector>

namespace Json {
	class Value;
}

class WrapperJsoncpp {
public:

	std::string getRequestStringNewSession();
	std::string getRequestStringContinue(int sessionId);
	std::string getRequestStringGetAgents(int sessionId);
	//std::string getRequestStringGetAIPedAgents(int sessionId);
	std::string getRequestStringGetPedestrianAgents(int sessionId);
	std::string sendPedestrianInput(int sessionId, float x, float y, float yaw);
	int parseRespondSessionId(std::string respond);
};
