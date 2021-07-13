#include "frontal_continental_radar.h"
#include <arpa/inet.h>
#include <algorithm>
#include <math.h>

//mdj_begin
#ifdef HIGER
const char* FRONTAL_RADAR_IP = "192.168.0.24";
#endif
//mdj_end

#define PI 3.1415926535898
const double toRAD = PI/180.0;
FrontalContinentalRadar::FrontalContinentalRadar() {
	memset((char*) &myaddr_, 0, sizeof(myaddr_));
	memset(&remaddr_, 0, sizeof(remaddr_));
	remaddr_.sin_addr.s_addr = inet_addr(FRONTAL_RADAR_IP); //inet_addr函数需要一个字符串作为其参数，该字符串指定了以点分十进制格式表示的IP地址（例如：192.168.0.16）。而且inet_addr函数会返回一个适合分配给S_addr的u_long类型的数值。
	remaddr_.sin_family = AF_INET;   //ip地址家族
	remaddr_.sin_port = htons(FRONTAL_RADAR_PORT); //将多字节整数类型的数据，从主机的字节顺序转化为网络字节顺序（大端）

	//reset the radar struct
	memset(&radar_target_data_, 0, sizeof(radar_target_data_));
	memset(radar_data_buf_, 0, sizeof(radar_data_buf_));   //reset buffer

}

FrontalContinentalRadar::~FrontalContinentalRadar(){

}
bool FrontalContinentalRadar::Init() {
	//1)create radar socket
	//socket是应用层和传输层之间的抽象层
	//初始化创建socket对象，通常是第一个调用的socket函数。 成功时，返回非负数的socket描述符；失败是返回-1
	//AF_INET决定了要用ipv4地址（32位的）与端口号（16位的）的组合
	//SOCK_STREAM -- TCP类型，保证数据顺序及可靠性
	//SOCK_DGRAM --  UDP类型，不保证数据接收的顺序，非可靠连接
	if ((radar_socket_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("[ERROR]cannot create radar socket!");
		return false;
	}
	//2)build connecion address
	myaddr_.sin_family = AF_INET;
	myaddr_.sin_port = htons(FRONTAL_RADAR_LISTEN_PORT);
	myaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
	//enable address reuse
	int ret, on = 1;
	//原型int setsockopt( int socket, int level, int option_name,const void *option_value, size_t ，ption_len);
	//第一个参数socket是套接字描述符,第二个参数level是被设置的选项的级别，如果想要在套接字级别上设置选项，就必须把level设置为 SOL_SOCKET
	//option_name指定准备设置的选项，option_name可以有哪些取值，这取决于level
	//SO_REUSEADDR，打开或关闭地址复用功能
	//当option_value不等于0时，打开，否则，关闭。它实际所做的工作是置sock->sk->sk_reuse为1或0。
	ret = setsockopt(radar_socket_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)); // closesocket（一般不会立即关闭而经历TIME_WAIT的过程）后想继续重用该socket：

	//3)bind socket to specific address
	//bind()：绑定socket到本地地址和端口，通常由服务端调用
	//原型：int bind(int sockfd, const struct sockaddr* myaddr, socklen_t addrlen)
	//功能描述：将创建的socket绑定到指定的IP地址和端口上，通常是第二个调用的socket接口。返回值：0 -- 成功，-1 -- 出错。
	//当socket函数返回一个描述符时，只是存在于其协议族的空间中，并没有分配一个具体的协议地址（这里指IPv4/IPv6和端口号的组合），bind函数可以将一组固定的地址绑定到sockfd上。
	if (bind(radar_socket_, (sockaddr*) &myaddr_, sizeof(myaddr_)) < 0) {
		perror("[ERROR]cannot bind the radar socket!");
		close(radar_socket_);
		return false;
	}

	return true;
}

bool FrontalContinentalRadar::Update() {
	//mdj_begin
	Send_RadarCfg_Info();
	Send_ObjectfilterCfg_Info();
	//mdj_end

	//receive radar socket data
	remaddrlen_ = sizeof(remaddr_);
	//recvfrom（）函数原型为：int recvfrom（int sockfd，void *buf，int len，unsigned int lags，struct sockaddr *from，int *fromlen）from是一个struct sockaddr类型的变量，该变量保存源机的IP地址及端口号。fromlen常置为sizeof （struct sockaddr）
	//当recvfrom（）返回时，fromlen包含实际存入from中的数据字节数。Recvfrom（）函数返回接收到的字节数或当出现错误时返回－1，并置相应的errno
	memset(radar_data_buf_, 0, sizeof(radar_data_buf_));
	recv_len_ = recvfrom(radar_socket_, radar_data_buf_, RADAR_DATA_BUFFER, 0,
			(struct sockaddr*) &remaddr_, &remaddrlen_);
	printf("recv length is %d \n", recv_len_);
	printf("remote address is %s and %d\n", inet_ntoa(remaddr_.sin_addr),
			ntohs(remaddr_.sin_port));
	if (recv_len_ < 0) {
		perror("[ERROR]cannot receive radar data!");
		close(radar_socket_);
		return false;
	} else {
		Proc_Radar_Data();
	}
	return true;
}

bool FrontalContinentalRadar::Send_RadarCfg_Info()
{
	printf("[INFO] <frontal_continental_radar> send radarcfg info to radar...\n");
	unsigned char send_buf_200[13];
	memset(send_buf_200, 0, sizeof(send_buf_200));
	send_buf_200[0] = 0b00001000;
	send_buf_200[1] = 0x00;
	send_buf_200[2] = 0x00;
	send_buf_200[3] = 0x02;
	send_buf_200[4] = 0x00;
	send_buf_200[5] = (0x1 | (0x1 << 1) | (0x1 << 3) | (0x1 << 4) | (0x1 << 5));   //RadarCfg_MaxDistance_valid | RadarCfg_SensorID_valid | RadarCfg_OutputType_valid | RadarCfg_SendQuality_valid | RadarCfg_SendExtInfo_valid
	send_buf_200[6] = 0x19;  //RadarCfg_MaxDistance     无符号字符存储时不占用符号位
	send_buf_200[7] = 0x00;  //RadarCfg_MaxDistance = 200m  range_resolution= 0.42m
	send_buf_200[9] = (0x01 | (0x01 << 3));   //RadarCfg_SensorID = 1 | RadarCfg_OutputType = 1
	send_buf_200[10] = (0x01 << 2);   //RadarCfg_SendQuality = 1
	//发送
	int send_len = sendto(radar_socket_, send_buf_200, sizeof(send_buf_200), 0, (sockaddr*) &remaddr_, sizeof(remaddr_));
	if (send_len < 0) {
		perror("[ERROR]cannot send send_buf_200 data!");
	}
	return true;
}

bool FrontalContinentalRadar::Send_ObjectfilterCfg_Info()
{
	printf("[INFO] <frontal_continental_radar> send Objectfiltercfg info to radar...\n");
	unsigned char send_buf_212[13];
	memset(send_buf_212, 0, sizeof(send_buf_212));
	send_buf_212[0] = 0b00001000;
	send_buf_212[1] = 0x00;
	send_buf_212[2] = 0x00;
	send_buf_212[3] = 0x02;
	send_buf_212[4] = 0x12;
	send_buf_212[5] = ((0x1 << 1) | (0x1 << 2) | (0x1 << 7));
	for(int i = 0 ; i < 15 ; i++)
	{
		switch(i)
		{
		case 0:
			send_buf_212[5] = (0x0 << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = 0 | FilterCfg_Type = 1
			send_buf_212[8] = 0x00;
			send_buf_212[9] = 0x40;  //FilterCfg_Max_NofObj = 64
			break;
		case 1:
			send_buf_212[5] = (0x1 << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = 1 | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x00;  //FilterCfg_Min_Distance = 0m
			send_buf_212[8] = 0x03;
			send_buf_212[9] = 0xE8; //FilterCfg_Max_Distance = 100m   Radial distance in m [r = sqrt(x2 + y2)]
			break;
		case 2:
			send_buf_212[5] = (0x2 << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = 2 | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0xC8;  //FilterCfg_Min_Azimuth = -45deg
			send_buf_212[8] = 0x0E;
			send_buf_212[9] = 0xD8; //FilterCfg_Max_Azimuth = 45deg   Azimuth angle in degree [a = atan(y/x)]
			break;
		case 3:
			send_buf_212[5] = (0x3 << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = 3 | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x00;  //FilterCfg_Min_VrelOncome = 0m/s
			send_buf_212[8] = 0x0C;
			send_buf_212[9] = 0x67;  //FilterCfg_Max_VrelOncome = 100m/s
			break;
		case 4:
			send_buf_212[5] = (0x4 << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = 4 | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x00;  //FilterCfg_Min_VrelDepart = 0m/s
			send_buf_212[8] = 0x0C;
			send_buf_212[9] = 0x67;  //FilterCfg_Max_VrelDepart = 100m/s
			break;
		case 8:
			send_buf_212[5] = (0x8 << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = 8 | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x02;  //FilterCfg_Min_ProbExists = 50%
			send_buf_212[8] = 0x00;
			send_buf_212[9] = 0x07;  //FilterCfg_Max_ProbExists = 100%
			break;
		case 9:
			send_buf_212[5] = (0x9 << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = 9 | FilterCfg_Type = 1
			send_buf_212[6] = 0x07;
			send_buf_212[7] = 0xCE;  //FilterCfg_Min_Y = -10m
			send_buf_212[8] = 0x08;
			send_buf_212[9] = 0x64;  //FilterCfg_Max_Y = 20m
			break;
		case 10:
			send_buf_212[5] = (0xA << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = A | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x00;  //FilterCfg_Min_X = 0m
			send_buf_212[8] = 0x0B;
			send_buf_212[9] = 0xB8;  //FilterCfg_Max_X = 100m
			break;
		case 11:
			send_buf_212[5] = (0xB << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = B | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x00;  //FilterCfg_Min_VYRightLeft = 0m/s
			send_buf_212[8] = 0x0C;
			send_buf_212[9] = 0x67;  //FilterCfg_Max_VYRightLeft = 100m/s
			break;
		case 12:
			send_buf_212[5] = (0xC << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = C | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x00;  //FilterCfg_Min_VXOncome = 0m/s
			send_buf_212[8] = 0x0C;
			send_buf_212[9] = 0x67;  //FilterCfg_Max_VXOncome = 100m/s
			break;
		case 13:
			send_buf_212[5] = (0xD << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = D | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x00;  //FilterCfg_Min_VYLeftRight = 0m/s
			send_buf_212[8] = 0x0C;
			send_buf_212[9] = 0x67;  //FilterCfg_Max_VYLeftRight = 100m/s
			break;
		case 14:
			send_buf_212[5] = (0xE << 3); //FilterCfg_valid | FilterCfg_Active | FilterCfg_Index = E | FilterCfg_Type = 1
			send_buf_212[6] = 0x00;
			send_buf_212[7] = 0x00;  //FilterCfg_Min_VXDepart = 0m/s
			send_buf_212[8] = 0x0C;
			send_buf_212[9] = 0x67;  //FilterCfg_Max_VXDepart = 100m/s
			break;
		}
		//发送
		int send_len = sendto(radar_socket_, send_buf_212, sizeof(send_buf_212),
				0, (sockaddr*) &remaddr_, sizeof(remaddr_));
		if (send_len < 0) {
			perror("[ERROR]cannot send send_buf_212 data!");
		}
		return true;
	}

}

void FrontalContinentalRadar::Proc_Radar_Data() {
	int can_frame_count = recv_len_ / 13;   //each can frame contains 13 bytes
	printf("recv_len_ is %d ========can_frame_count is %d \n", recv_len_,
			can_frame_count);
	for (int i = 0; i < can_frame_count; ++i) { //a udp data frame may contain numbers of CAN frames
		unsigned char* buf = &(radar_data_buf_[i * 13]); // 因为一帧CAN报文为108位，大概为13字节 ，而一帧UDP可能含有多帧CAN报文
		unsigned int tmpCanID = 0;
		unsigned char tmpdata[8] = { 0 };
		tmpCanID = buf[1] << 24 | buf[2] << 16 | buf[3] << 8 | buf[4]; //”|“与"<<"为位运算符，"<<"为左移运算符，”|“为或运算符
		tmpdata[0] = buf[5];
		tmpdata[1] = buf[6];
		tmpdata[2] = buf[7];
		tmpdata[3] = buf[8];
		tmpdata[4] = buf[9];
		tmpdata[5] = buf[10];
		tmpdata[6] = buf[11];
		tmpdata[7] = buf[12];   //表示的是8个字节中的数据，数组的一位存放一个字节的数

		if (tmpCanID == 0x211) {
			//RadarState_MaxDistanceCfg

			unsigned short temp_1 = tmpdata[1];
			unsigned short temp_2 = tmpdata[2] & 0xC0;
			radar_target_data_.MaxDistanceCfg = ((temp_1 << 2) | (temp_2 >> 6))
					* 2;

			printf(
					"RadarState_MaxDistanceCfg is ++++++++++++++++++++++++++++++++++++++++++%d \n",
					radar_target_data_.MaxDistanceCfg);

			//RadarState_SensorID
			temp_1 = tmpdata[4] & 0x07;
			radar_target_data_.SensorID = temp_1;
			printf(
					"RadarState_SensorID is ++++++++++++++++++++++++++++++++++++++++++%d \n",
					radar_target_data_.SensorID);

			//RadarState_OutputTypeCfg
			temp_1 = tmpdata[5] & 0x0C;
			radar_target_data_.OutputTypeCfg = temp_1  >> 2;
			printf(
					"RadarState_OutputTypeCfg is ++++++++++++++++++++++++++++++++++++++++++%d \n",
					radar_target_data_.OutputTypeCfg);

			//RadarState_SendQualityCfg
			temp_1 = tmpdata[5] & 0x10;
			radar_target_data_.SendQualityCfg = temp_1 >> 4;
			printf(
					"RadarState_SendQualityCfg is ++++++++++++++++++++++++++++++++++++++++++%d \n",
					radar_target_data_.SendQualityCfg);

			//RadarState_SendExtInfoCfg
			temp_1 = tmpdata[5] & 0x20;
			radar_target_data_.SendExtInfoCfg = temp_1 >> 5;
			printf(
					"RadarState_SendExtInfoCfg is ++++++++++++++++++++++++++++++++++++++++++%d \n",
					radar_target_data_.SendExtInfoCfg);
		}

		if (tmpCanID == 0x213) {
			//FilterState_NofObjectFilterCfg
			unsigned short temp_1 = tmpdata[1] & 0xF8;
			radar_target_data_.NofObjectFilterCfg = temp_1 >> 3;
			printf(
					"FilterState_NofObjectFilterCfg is ++++++++++++++++++++++++++++++++++++++++++%d \n",
					radar_target_data_.NofObjectFilterCfg);
		}

		//FilterState_Cfg
		if (tmpCanID == 0x214) {
			unsigned short temp_1 = tmpdata[0] & 0x0078;
			unsigned short temp_D1;
			unsigned short temp_D2;
			unsigned short temp_S1;
			unsigned short temp_S2;
			switch (temp_1) {   //不能在switch中定义变量
			case 0:
				//FilterState_Max_NofObj
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_NofObj = (temp_S1 << 8) | temp_S2;
				printf(
						"RadarState_Max_NofObj is ++++++++++++++++++++++++++++++++++++++++++%d \n",
						radar_target_data_.Max_NofObj);
				break;
			case 1:
				//FilterState_Min_Distance
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_Distance = ((temp_D1 << 8) | temp_D2)
						* 0.1f;
				printf(
						"RadarState_Min_Distance is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_Distance);
				//FilterState_Max_Distance
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_Distance = ((temp_S1 << 8) | temp_S2)
						* 0.1f;
				printf(
						"RadarState_Max_Distance is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_Distance);
				break;
			case 2:
				//FilterState_Min_Azimuth
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_Azimuth = (((temp_D1 << 8) | temp_D2)
						* 0.025f) - 50;
				printf(
						"RadarState_Min_Azimuth is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_Azimuth);
				//FilterState_Max_Azimuth
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_Azimuth = (((temp_S1 << 8) | temp_S2)
						* 0.025f) - 50;
				printf(
						"RadarState_Max_Azimuth is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_Azimuth);
				break;
			case 3:
				//FilterState_Min_VrelOncome
				temp_D1 = tmpdata[1] & 0x0F;
			    temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_VrelOncome = ((temp_D1 << 8) | temp_D2)
						* 0.0315f;
				printf(
						"RadarState_Min_VrelOncome is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_VrelOncome);
				//FilterState_Max_VrelOncome
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_VrelOncome = ((temp_S1 << 8) | temp_S2)
						* 0.0315f;
				printf(
						"RadarState_Max_VrelOncome is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_VrelOncome);
				break;
			case 4:
				//FilterState_Min_VrelDepart
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_VrelDepart = ((temp_D1 << 8) | temp_D2)
						* 0.0315f;
				printf(
						"RadarState_Min_VrelDepart is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_VrelDepart);
				//FilterState_Max_VrelDepart
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_VrelOncome = ((temp_S1 << 8) | temp_S2)
						* 0.0315f;
				printf(
						"RadarState_Max_VrelOncome is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_VrelOncome);
				break;
			case 8:
				//FilterState_Min_ProbExists
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_ProbExists = ((temp_D1 << 8) | temp_D2);
				printf(
						"RadarState_Min_ProbExists is ++++++++++++++++++++++++++++++++++++++++++%d \n",
						radar_target_data_.Min_ProbExists);
				//FilterState_Max_ProbExists
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_ProbExists = ((temp_S1 << 8) | temp_S2);
				printf(
						"RadarState_Max_ProbExists is ++++++++++++++++++++++++++++++++++++++++++%d \n",
						radar_target_data_.Max_ProbExists);
				break;
			case 9:
				//FilterState_Min_Y
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_Y = (((temp_D1 << 8) | temp_D2) * 0.2f)
						- 409.5f;
				printf(
						"RadarState_Min_Y is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_Y);
				//FilterState_Max_Y
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_Y = (((temp_S1 << 8) | temp_S2) * 0.2f)
						- 409.5f;
				printf(
						"RadarState_Max_Y is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_Y);
				break;
			case 10:
				//FilterState_Min_X
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_X = (((temp_D1 << 8) | temp_D2) * 0.2f)
						- 500;
				printf(
						"RadarState_Min_X is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_X);
				//FilterState_Max_X
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_X = (((temp_S1 << 8) | temp_S2) * 0.2f)
						- 500;
				printf(
						"RadarState_Max_X is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_X);
				break;
			case 11:
				//FilterState_Min_VYRightLeft
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_VYRightLeft = ((temp_D1 << 8) | temp_D2)
						* 0.0315f;
				printf(
						"RadarState_Min_VYRightLeft is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_VYRightLeft);
				//FilterState_Max_VYRightLeft
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_VYRightLeft = ((temp_S1 << 8) | temp_S2)
						* 0.0315f;
				printf(
						"RadarState_Max_VYRightLeft is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_VYRightLeft);
				break;
			case 12:
				//FilterState_Min_VXOncome
				temp_D1 = tmpdata[1] & 0x0F;
			    temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_VXOncome = ((temp_D1 << 8) | temp_D2)
						* 0.0315f;
				printf(
						"RadarState_Min_VXOncome is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_VXOncome);
				//FilterState_Max_VXOncome
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_VXOncome = ((temp_S1 << 8) | temp_S2)
						* 0.0315f;
				printf(
						"RadarState_Max_VXOncome is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_VXOncome);
				break;
			case 13:
				//FilterState_Min_VYLeftRight
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_VYLeftRight = ((temp_D1 << 8) | temp_D2)
						* 0.0315f;
				printf(
						"RadarState_Min_VYLeftRight is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_VYLeftRight);
				//FilterState_Max_VYLeftRight
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_VYLeftRight = ((temp_S1 << 8) | temp_S2)
						* 0.0315f;
				printf(
						"RadarState_Max_VYLeftRight is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_VYLeftRight);
				break;
			case 14:
				//FilterState_Min_VXDepart
				temp_D1 = tmpdata[1] & 0x0F;
				temp_D2 = tmpdata[2] & 0xFF;
				radar_target_data_.Min_VXDepart = ((temp_D1 << 8) | temp_D2)
						* 0.0315f;
				printf(
						"RadarState_Min_VXDepart is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Min_VXDepart);
				//FilterState_Max_VXDepart
				temp_S1 = tmpdata[3] & 0x0F;
				temp_S2 = tmpdata[4] & 0xFF;
				radar_target_data_.Max_VXDepart = ((temp_S1 << 8) | temp_S2)
						* 0.0315f;
				printf(
						"RadarState_Max_VXDepart is ++++++++++++++++++++++++++++++++++++++++++%f \n",
						radar_target_data_.Max_VXDepart);
				break;
			}
		}

		if (tmpCanID == 0x61A) {
			//Object_NofObjects
			unsigned short temp_D1 = tmpdata[0];
			radar_target_data_.NofObjects = temp_D1;   //目标数量，如果目标数量是n，那么所有目标的编号不一定就是从1连续排到n，有可能中间会有不连续，且最大编号也有可能大于n
			printf(
					"Object_NofObjects is ++++++++++++++++++++++++++++++++++++++++++%d \n",
					radar_target_data_.NofObjects);
		}

		if (tmpCanID == 0x61B) {
			//Object general information
//			continental_detection_array是一个结构体类型的数组
			unsigned short temp_D1 = tmpdata[0];
			radar_target_data_.continental_detection_array[temp_D1].target_ID =
					temp_D1 + 1;
			unsigned short temp_S1 = tmpdata[1];
			unsigned short temp_S2 = tmpdata[2] & 0xF8;
			radar_target_data_.continental_detection_array[temp_D1].x =
					(((temp_S1 << 5) | (temp_S2 >> 3)) * 0.2) - 500;
			unsigned short temp_H1 = tmpdata[2] & 0x07;
			unsigned short temp_H2 = tmpdata[3] & 0xFF;
			radar_target_data_.continental_detection_array[temp_D1].y =
					(((temp_H1 << 8) | temp_H2) * 0.2) - 204.6;
			unsigned short temp_N1 = tmpdata[4];
			unsigned short temp_N2 = tmpdata[5] & 0xC0;
			radar_target_data_.continental_detection_array[temp_D1].Relative_xv =
					(((temp_N1 << 2) | (temp_N2 >> 6)) * 0.25) - 128;
			unsigned short temp_Q1 = tmpdata[5] & 0x3F;
			unsigned short temp_Q2 = tmpdata[6] & 0xE0;
			radar_target_data_.continental_detection_array[temp_D1].Relative_yv =
					(((temp_Q1 << 3) | (temp_Q2 >> 5)) * 0.25) - 64;
			printf(
					"target_ID is %d========x is %f========y is %f========Relative_xv is %f========Relative_yv is %f\n",
					radar_target_data_.continental_detection_array[temp_D1].target_ID,
					radar_target_data_.continental_detection_array[temp_D1].x,
					radar_target_data_.continental_detection_array[temp_D1].y,
					radar_target_data_.continental_detection_array[temp_D1].Relative_xv,
					radar_target_data_.continental_detection_array[temp_D1].Relative_yv);
		}

//		if (tmpCanID == 0x61C) {
//			//Object quality information
//			unsigned short temp_D1 = tmpdata[0];
//			radar_target_data_.continental_detection_array[temp_D1].target_ID =
//					temp_D1 + 1;
//			unsigned short temp_S1 = tmpdata[1] & 0xF8;
//			radar_target_data_.continental_detection_array[temp_D1].index_deviation_x =
//					temp_S1 >> 3;   //仅仅是获得了偏差的索引，还需要处理来获得具体的偏差值，这里还没处理
//			unsigned short temp_H1 = tmpdata[1] & 0x07;
//			unsigned short temp_H2 = tmpdata[2] & 0xC0;
//			radar_target_data_.continental_detection_array[temp_D1].index_deviation_y =
//					((temp_H1 << 2) | (temp_H2 >> 6));
//			unsigned short temp_N1 = tmpdata[2] & 0x3E;
//			radar_target_data_.continental_detection_array[temp_D1].index_deviation_xv =
//					temp_N1 >> 1;
//			unsigned short temp_Q1 = tmpdata[2] & 0x01;
//			unsigned short temp_Q2 = tmpdata[3] & 0xF0;
//			radar_target_data_.continental_detection_array[temp_D1].index_deviation_yv =
//					((temp_Q1 << 4) | (temp_H2 >> 4));
//			printf(
//					"target_ID is %d========index_deviation_x is %f========index_deviation_y is %f========index_deviation_xv is %f========index_deviation_yv is %f\n",
//					radar_target_data_.continental_detection_array[temp_D1].target_ID,
//					radar_target_data_.continental_detection_array[temp_D1].index_deviation_x,
//					radar_target_data_.continental_detection_array[temp_D1].index_deviation_y,
//					radar_target_data_.continental_detection_array[temp_D1].index_deviation_xv,
//					radar_target_data_.continental_detection_array[temp_D1].index_deviation_yv);
//		}

		if (tmpCanID == 0x61D) {
			//Object extended information
			unsigned short temp_D1 = tmpdata[0];
			radar_target_data_.continental_detection_array[temp_D1].target_ID =
					temp_D1 + 1;
			unsigned short temp_S1 = tmpdata[1] & 0xFF;
			unsigned short temp_S2 = tmpdata[2] & 0xE0;
			radar_target_data_.continental_detection_array[temp_D1].Relative_acc_x =
					(((temp_S1 << 3) | (temp_S2 >> 5)) * 0.01) - 10 ;
			unsigned short temp_H1 = tmpdata[2] & 0x1F;
			unsigned short temp_H2 = tmpdata[3] & 0xF0;
			radar_target_data_.continental_detection_array[temp_D1].Relative_acc_y =
					(((temp_H1 << 4) | (temp_H2 >> 4)) * 0.01) - 2.5;
			unsigned short temp_N1 = tmpdata[3] & 0x07;
			radar_target_data_.continental_detection_array[temp_D1].Obiect_class =
					temp_N1;
			unsigned short temp_Q1 = tmpdata[4] & 0xFF;
			unsigned short temp_Q2 = tmpdata[5] & 0xC0;
			radar_target_data_.continental_detection_array[temp_D1].angle =
					(((temp_Q1 << 2) | (temp_H2 >> 6)) * 0.4) - 180;
			unsigned short temp_J1 = tmpdata[6] & 0xFF;
			radar_target_data_.continental_detection_array[temp_D1].Object_Length =
					temp_J1 * 0.2;
			unsigned short temp_M1 = tmpdata[3] & 0xFF;
			radar_target_data_.continental_detection_array[temp_D1].Object_Width =
					temp_M1 * 0.2;
			printf(
					"target_ID is %d========Relative_acc_x is %f========Relative_acc_y is %f========Obiect_class is %d========angle is %f========Object_Length is %f========Object_Width is %f\n",
					radar_target_data_.continental_detection_array[temp_D1].target_ID,
					radar_target_data_.continental_detection_array[temp_D1].Relative_acc_x,
					radar_target_data_.continental_detection_array[temp_D1].Relative_acc_y,
					radar_target_data_.continental_detection_array[temp_D1].Obiect_class,
					radar_target_data_.continental_detection_array[temp_D1].angle,
					radar_target_data_.continental_detection_array[temp_D1].Object_Length,
					radar_target_data_.continental_detection_array[temp_D1].Object_Width);
		}

  }

}

continental_radar_target FrontalContinentalRadar::radar_target_data() {
	return this->radar_target_data_;
}
