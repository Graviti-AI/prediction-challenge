#include "ros/ros.h"
#include <iostream>
#include "unistd.h"
#include "visualization/WrapperJsoncpp.h"
#include <string>
#include <visualization/vehicleinfo.h>
#include "visualization/vehicleinfomation.h"
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
#include <sys/time.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> /* struct hostent */
#include <arpa/inet.h> /* inet_ntop */
#include "json/json.h"
#include <json/reader.h>
#include <json/writer.h>
#define SERVER_PORT 8082
#define BUFF_LEN 10240

std::string respondRendering(const std::string &requestJson) {
    Json::Reader reader;
    Json::Value root;

    Json::Value ret;


    try {

        bool isParsingSuccessful = reader.parse(requestJson, root);

        if (isParsingSuccessful && root["sender"].asString() == "rendering") {
            ret["respondStatus"] = true;

            std::string command = root["command"].asString();

            if (command == "newSession") { // if the client connects for the first time, and requests a new session

                //mutex.lock();

                int sessionId = 0;//this->sessionIdCounter++;
                ret["sessionId"] = sessionId;

                //Session *session = new Session();
                //session->sessionId = sessionId;
                //session->sessionType = SessionType::Rendering;

                //this->sessionAgentMap[session] = nullptr;

                //mutex.unlock();

                std::cout << "request new session" << std::endl;

            } 

        } else {
            ret["respondStatus"] = false;
            ret["errorMessage"] = "bad json format";
        }

    } catch (const std::runtime_error &e) {
        ret["respondStatus"] = false;
        ret["errorMessage"] = e.what();
    } catch (...) {
        ret["respondStatus"] = false;
        ret["errorMessage"] = "unknown error";
    }

    Json::FastWriter fastWriter;
    std::string output = fastWriter.write(ret);
    return output;
}

void handle_udp_msg(int fd)
{
    char buf[BUFF_LEN];  //接收缓冲区，1024字节
    socklen_t len;
    int count;
    struct sockaddr_in clent_addr;  //clent_addr用于记录发送方的地址信息

    while(1){
        memset(buf, 0, BUFF_LEN);
        len = sizeof(clent_addr);
        count = recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, &len);

        if(count == -1)
        {
            printf("recieve data fail!\n");
            return;
        }

        printf("client: %s\n",buf);
	std::string requestJson;
        std::string sendJson;
	requestJson += buf;

	sendJson = respondRendering(requestJson);
	sendto(fd, sendJson.c_str(), BUFF_LEN, 0, (struct sockaddr*)&clent_addr, len);
    	std::cout<<sendJson<<std::endl;
	if (sendJson != "{\"errorMessage\":\"bad json format\",\"respondStatus\":false}") break;
    }

    memset(buf, 0, BUFF_LEN);
    len = sizeof(clent_addr);
    count = recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, &len);
    if(count == -1)
    {
        printf("recieve data fail!\n");
        return;
    }

    printf("client: %s\n",buf);
    memset(buf, 0, BUFF_LEN);
    sprintf(buf, "Connection has established!");
    sendto(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, len);
    std::cout<<buf<<std::endl;

    while(1)
    {
	std::string p="OK";
        memset(buf, 0, BUFF_LEN);
        //sprintf(buf, p);
	//buf =  p.data()
        sendto(fd, p.data(), BUFF_LEN, 0, (struct sockaddr*)&clent_addr, len);

    }
}


/*
    server:
            socket-->bind-->recvfrom-->sendto-->close
*/

int main(int argc, char* argv[])
{
    int server_fd, ret;
    struct sockaddr_in ser_addr; 

    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(server_fd < 0)
    {
        printf("create socket fail!\n");
        return -1;
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY); //IP地址，需要进行网络序转换，INADDR_ANY：本地地址
    ser_addr.sin_port = htons(SERVER_PORT);  //端口号，需要网络序转换

    ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
    {
        printf("socket bind fail!\n");
        return -1;
    }

    handle_udp_msg(server_fd);   //处理接收到的数据

    close(server_fd);
    return 0;
}
