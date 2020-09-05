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
using namespace std;

#define PORT 8086
#define BUFF_LEN 62500
#define SERVER_IP "127.0.0.1"

WrapperJsoncpp wrapperJsoncpp;
bool if_connect = false;
int My_id;
int counter=0;
std::string result;

std::vector<double> Json2Vector(const Json::Value &val) {
	std::vector<double> ret;
	for (auto iter = val.begin(); iter != val.end(); iter++) {
		ret.push_back(iter->asDouble());
	}
	return ret;
}

void udp_msg_sender(int fd, struct sockaddr* dst)
{

    socklen_t len;
    struct sockaddr_in src;
    char buf[BUFF_LEN] = "building UDP!\n";
    if (!if_connect){

	//get session id
        len = sizeof(*dst);
	std::string p;
	p=wrapperJsoncpp.getRequestStringNewSession();
	sendto(fd, p.c_str(), BUFF_LEN, 0, dst, len);
	recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&src, &len);
    	printf("server:%s\n\n",buf);
	My_id = wrapperJsoncpp.parseRespondSessionId(buf);
        if_connect = true; 
    }
    if (counter>800){
	std::cout<<"Continue"<<std::endl;
	//Continue
        len = sizeof(*dst);
	std::string p;
	p=wrapperJsoncpp.getRequestStringContinue(My_id);
	sendto(fd, p.c_str(), BUFF_LEN, 0, dst, len);
	recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&src, &len);
    	printf("server:%s\n",buf);
	//My_id = wrapperJsoncpp.parseRespondSessionId(buf);
	counter =0;
    }
    memset(buf, 0, BUFF_LEN);
    //printf("waiting responce!\n");
    recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&src, &len);
    printf("server:%s\n\n",buf);
    result = buf;
}


int64_t getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000 + tv.tv_usec /1000;
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "client");
    ros::NodeHandle n;
    ros::Publisher pub7 = n.advertise<visualization::vehicleinfo>("vehicle_state",1);

    ros::Rate loop_rate(50);
    int i;
    long int length;
    
    
    int client_fd;
    struct sockaddr_in ser_addr;

    client_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(client_fd < 0)
    {
        printf("create socket fail!\n");
        return -1;
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    //ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);  //注意网络序转换
    ser_addr.sin_port = htons(PORT);  //注意网络序转换



    Json::Reader reader;
    Json::Value root;

    while(ros::ok())
    {
	visualization::vehicleinfo ob_visual;
        counter++;
        udp_msg_sender(client_fd, (struct sockaddr*)&ser_addr);
        bool isParsingSuccessful = reader.parse(result, root);
	if (!isParsingSuccessful) continue;
	if (root["respondStatus"].asBool()) {
		std::cout<<root["respondStatus"]<<std::endl;
		continue;
	}
	
	Json::Value agentInfo=root["agentInfo"];
	cout<<"ok1"<<endl;
	for (auto oneagent : agentInfo){
	    if (oneagent["state"].empty()) continue;
	    std::vector<double> state = Json2Vector(oneagent["state"]);
	    visualization::vehicleinfomation one_obcar;
	    one_obcar.X = state[0]; // /100;
	    one_obcar.Y = state[1]; // /100;
	    one_obcar.psi = state[2];
	    one_obcar.lx = 4;
	    one_obcar.ly = 2;
	    ob_visual.point.push_back(one_obcar);
	}
	pub7.publish(ob_visual);

        //loop_rate.sleep();

    }

    close(client_fd);

    return 0;
}

