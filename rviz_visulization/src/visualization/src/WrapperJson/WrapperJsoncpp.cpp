#include "visualization/WrapperJsoncpp.h"
#include <memory>
#include "json/json.h"
#include <json/reader.h>
#include <json/writer.h>
#include <sstream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> /* struct hostent */
#include <arpa/inet.h> /* inet_ntop */
#include <iostream>
#include "unistd.h"
using std::string;

bool GetHostInfo(std::string& hostName, std::string& Ip) {
	char name[256];
	gethostname(name, sizeof(name));
	hostName = name;
	
	struct hostent* host = gethostbyname(name);
	char ipStr[32];
	const char* ret = inet_ntop(host->h_addrtype, host->h_addr_list[0], ipStr, sizeof(ipStr));
	if (NULL==ret) {
		std::cout << "hostname transform to ip failed";
		return false;
	}
	Ip = ipStr;
	return true;
}

std::string Json2String(const Json::Value &val) {
	std::stringstream ss;
	Json::StreamWriterBuilder builder;
	builder["commentStyle"] = "None";
	builder["indentation"] = "";
	std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
	writer->write(val, &ss);
	string request = ss.str();
	return request;
}

Json::Value String2Json(const std::string &str) {
	Json::CharReaderBuilder builder;
	Json::CharReader *reader = builder.newCharReader();
	Json::Value root;
	string err;
	bool ret = reader->parse(str.c_str(), str.c_str() + str.length(), &root, &err);
	delete reader;
	if (ret) {
		return root;
	}
	else {
		throw "parsing error";
	}
}



Json::Value Vector2Json(const std::vector<double> &vec) {
	Json::Value ret(Json::arrayValue);
	for (double temp : vec) {
		Json::Value val = temp;
		ret.append(val);
	}
	return ret;
}

std::string WrapperJsoncpp::getRequestStringNewSession() {
	Json::Value root;
 	std::string hostName;
        std::string Ip;
        bool ret = GetHostInfo(hostName, Ip);
	root["sender"] = "rendering";
	root["command"] = "newSession";
	root["IP"] = Ip.c_str();
	return Json2String(root);
}
std::string WrapperJsoncpp::getRequestStringContinue(int sessionId) {
	Json::Value root;
	root["sender"] = "rendering";
	root["command"] = "continue";
	root["sessionId"] = sessionId;
	return Json2String(root);
}
std::string WrapperJsoncpp::getRequestStringGetAgents(int sessionId) {
	Json::Value root;
	root["sender"] = "rendering";
	root["command"] = "getAgents";
	root["sessionId"] = sessionId;
	return Json2String(root);
}
/*
std::string WrapperJsoncpp::getRequestStringGetAIPedAgents(int sessionId) {
	Json::Value root;
	root["sender"] = "rendering";
	root["command"] = "getAIPedAgents";
	root["sessionId"] = sessionId;
	return Json2String(root);
}
*/
std::string WrapperJsoncpp::getRequestStringGetPedestrianAgents(int sessionId) {
	Json::Value root;
	root["sender"] = "rendering";
	root["command"] = "getPedestrianAgents";
	root["sessionId"] = sessionId;
	return Json2String(root);
}

std::string WrapperJsoncpp::sendPedestrianInput(int sessionId, float x, float y, float yaw) {
	std::vector<double> input{ x,y,yaw };
	Json::Value root;
	root["sender"] = "rendering";
	root["command"] = "updateInput";
	root["sessionId"] = sessionId;
	root["input"] = Vector2Json(input);
	return Json2String(root);
}

int WrapperJsoncpp::parseRespondSessionId(std::string respond) {

	Json::Value root = String2Json(respond);

	if (root["respondStatus"].asBool()) {
		return root["sessionId"].asInt();
	}
	else {
		return -1;
	}

}
