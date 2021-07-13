#include "UdpServer.h"

void udpSend::sendMsgs(vector<moving_object_millimeter> &Obj_Send)
{
	socklen_t len;
	len = sizeof(dst_addr);
	moving_object_millimeter buf[BUFF_LEN];
//	int send_len = sendto(server_fd, (char*) &Obj_Send, sizeof(Obj_Send) + 1, 0,
//			(struct sockaddr*) &dst_addr, len);
	for (int ii = 0; ii < BUFF_LEN - 1; ii++)
	{
		buf[ii] = Obj_Send[ii];
		std::cout << "ffffffffffffff" << buf[ii].x << buf[ii].y << std::endl;
		int send_len = sendto(server_fd,  (const void *)buf, BUFF_LEN, 0, (struct sockaddr*) &dst_addr, len);
		if (send_len < 0)
		{
			perror("[ERROR]cannot send Obj_Send data!");
		}
	}
}
udpSend::udpSend(std::string address,int client_port){

    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(server_fd < 0)
    {
        printf("create socket fail!\n");
        //return -1;
    }
    memset(&dst_addr, 0, sizeof(dst_addr));
    dst_addr.sin_family = AF_INET;
    char* addr = (char*)address.data();//
    dst_addr.sin_addr.s_addr = inet_addr(addr);//IP地址，需要进行网络序转换，INADDR_ANY：本地地址
//    std::cout<<"111111111111111111"<<addr<<inet_addr(addr)<<std::endl;
//    dst_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dst_addr.sin_port = htons(client_port);  //端口号，需要网络序转换
}

udpSend::~udpSend(){
    close(server_fd);
}
