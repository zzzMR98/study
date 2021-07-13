#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <arpa/inet.h>
#include <iostream>
#include "TypeDef.h"
#include <vector>

#define BUFF_LEN 64
using namespace std;
class udpSend{
private:

	int server_fd;
	struct sockaddr_in dst_addr;

public:

    udpSend(std::string address,int client_port);
    ~udpSend();
    void sendMsgs(vector<moving_object_millimeter> &Obj_Send);
};

