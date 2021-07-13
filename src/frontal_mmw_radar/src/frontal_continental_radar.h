/*==================================================================
 *    Function    ：receive udp data of continental radar and parse them
 *    相关说明：
 *    作者    ：  mdj
 *    创建日期    ：20180719
 *    修改记录：
/*==================================================================*/
#ifndef FRONTAL_CONTINENTAL_RADAR_H_
#define FRONTAL_CONTINENTAL_RADAR_H_
//C/C++
#include <cstdio>
#include <string.h>
#include <iostream>
#include <string>
#include <vector>
//UDP socket
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
//project headers
#include "TypeDef.h" //radar and vehicle information struct

#define HIGER
//mdj_begin
#ifdef HIGER
//const definition
const int FRONTAL_RADAR_PORT = 4001;
const int FRONTAL_RADAR_LISTEN_PORT = 8002;   //local listen port
#endif
//mdj_end

const short RADAR_DATA_BUFFER = 650;   //buffer 1000 bytes

class FrontalContinentalRadar {
public:
	//constructors
	FrontalContinentalRadar();
	~FrontalContinentalRadar();

	bool Init();   //init socket and bind socket
	bool Update();   //update socket to receive data, should be called in a loop
	continental_radar_target radar_target_data();   //get radar target data
private:
	void Proc_Radar_Data(); //important! parse radar data

	//mdj_begin
	bool Send_RadarCfg_Info();   //send configuration information to ARS radar
	bool Send_ObjectfilterCfg_Info(); //send Object filter configuration information to ARS radar
	//mdj_end

	//udp socket
	int radar_socket_; //socket used to communicate with radar,send or receive data from radar
	sockaddr_in myaddr_; //listen port info
	sockaddr_in remaddr_; //remote address
	socklen_t remaddrlen_;
	int recv_len_; //recvfrom function return, indicate receive how many bytes in a UDP package
	//address struct for send vehicle info

	//radar data parsing
	unsigned char radar_data_buf_[RADAR_DATA_BUFFER];
	continental_radar_target radar_target_data_;  //final output

};

#endif /*FRONTAL_CONTINENTAL_RADAR_H_*/
