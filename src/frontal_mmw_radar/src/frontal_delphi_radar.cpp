#include "frontal_delphi_radar.h"
#include <arpa/inet.h>
#include <algorithm>
#include <math.h>

#ifdef HUACHEN
const char* FRONTAL_RADAR_IP = "192.168.0.45";
#endif

#ifdef TOYOTA
const char* FRONTAL_RADAR_IP = "192.168.0.12";
#endif

using std::vector;
#define PI 3.1415926535898
const double toRAD = PI/180.0;
FrontalDelphiRadar::FrontalDelphiRadar(){
  memset((char*)&myaddr_,0,sizeof(myaddr_));
  memset(&remaddr_,0,sizeof(remaddr_));
  remaddr_.sin_addr.s_addr = inet_addr(FRONTAL_RADAR_IP);   //inet_addr函数需要一个字符串作为其参数，该字符串指定了以点分十进制格式表示的IP地址（例如：192.168.0.16）。而且inet_addr函数会返回一个适合分配给S_addr的u_long类型的数值。
  remaddr_.sin_family = AF_INET;   //ip地址家族
  remaddr_.sin_port = htons(FRONTAL_RADAR_PORT);   //将多字节整数类型的数据，从主机的字节顺序转化为网络字节顺序（大端）

  //initialize the parsing parameters
  radar_target_CAN_ID_vec_.resize(64);
  //迭代器类似与指针类型
  //++it  令it指向容器的下一个元素
  //容器和string有迭代器类型同时拥有返回迭代器的成员
  for(vector<unsigned int>::iterator it=radar_target_CAN_ID_vec_.begin();it!=radar_target_CAN_ID_vec_.end();++it){
    int count = 0x500+(it-radar_target_CAN_ID_vec_.begin());   //因为协议中动态目标地址是从0x500开始的   以十进制输出
    *it = count;   //这个容器中的每个元素用来存放地址，只是改变了容器内部的元素，没有改变迭代器，迭代器依然是从容器的第一个元素过滤到最后一个元素的下一个位置
  }

  //reset the radar struct
  memset(&radar_target_data_,0,sizeof(radar_target_data_));
  memset(radar_data_buf_,0,sizeof(radar_data_buf_));//reset buffer

}

FrontalDelphiRadar::~FrontalDelphiRadar(){

}
bool FrontalDelphiRadar::Init(){
  //1)create radar socket
	//socket是应用层和传输层之间的抽象层
	//初始化创建socket对象，通常是第一个调用的socket函数。 成功时，返回非负数的socket描述符；失败是返回-1
	//AF_INET决定了要用ipv4地址（32位的）与端口号（16位的）的组合
	//SOCK_STREAM -- TCP类型，保证数据顺序及可靠性
    //SOCK_DGRAM --  UDP类型，不保证数据接收的顺序，非可靠连接
	if((radar_socket_=socket(AF_INET,SOCK_DGRAM,0))<0){
    perror("[ERROR]cannot create radar socket!");
    return false;
  }
  //2)build connecion address
  myaddr_.sin_family=AF_INET;
  myaddr_.sin_port = htons(FRONTAL_RADAR_LISTEN_PORT);
  myaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
  //enable address reuse
  int ret,on=1;
  //原型int setsockopt( int socket, int level, int option_name,const void *option_value, size_t ，ption_len);
  //第一个参数socket是套接字描述符,第二个参数level是被设置的选项的级别，如果想要在套接字级别上设置选项，就必须把level设置为 SOL_SOCKET
  //option_name指定准备设置的选项，option_name可以有哪些取值，这取决于level
  //SO_REUSEADDR，打开或关闭地址复用功能
  //当option_value不等于0时，打开，否则，关闭。它实际所做的工作是置sock->sk->sk_reuse为1或0。
  ret = setsockopt(radar_socket_,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));   // closesocket（一般不会立即关闭而经历TIME_WAIT的过程）后想继续重用该socket：

  //3)bind socket to specific address
  //bind()：绑定socket到本地地址和端口，通常由服务端调用
  //原型：int bind(int sockfd, const struct sockaddr* myaddr, socklen_t addrlen)
  //功能描述：将创建的socket绑定到指定的IP地址和端口上，通常是第二个调用的socket接口。返回值：0 -- 成功，-1 -- 出错。
  //当socket函数返回一个描述符时，只是存在于其协议族的空间中，并没有分配一个具体的协议地址（这里指IPv4/IPv6和端口号的组合），bind函数可以将一组固定的地址绑定到sockfd上。
  if(bind(radar_socket_,(sockaddr*)&myaddr_,sizeof(myaddr_))<0){
    perror("[ERROR]cannot bind the radar socket!");
    close(radar_socket_);
    return false;
  }


  return true;
}

bool FrontalDelphiRadar::Update(){
  //send initial data to radar
  //Send_Triggle_Signal();
  //Send vehicle info
  Send_Vehicle_Info();

  //receive radar socket data
  remaddrlen_ = sizeof(remaddr_);
  //recvfrom（）函数原型为：int recvfrom（int sockfd，void *buf，int len，unsigned int lags，struct sockaddr *from，int *fromlen）from是一个struct sockaddr类型的变量，该变量保存源机的IP地址及端口号。fromlen常置为sizeof （struct sockaddr）
  //当recvfrom（）返回时，fromlen包含实际存入from中的数据字节数。Recvfrom（）函数返回接收到的字节数或当出现错误时返回－1，并置相应的errno
  memset(radar_data_buf_,0,sizeof(radar_data_buf_));

  recv_len_ = recvfrom(radar_socket_,radar_data_buf_,RADAR_DATA_BUFFER,0,(struct sockaddr*)&remaddr_,&remaddrlen_);
  //recvform为UDP数据的接收函数，通过此函数直接把数据存进radar_data_buf数组，然后又通过recvform
  //函数直接得出每一帧CAN报文所包含的字节数，赋值给recv_len
  //radar_data_buf_
  printf("recv length is %d \n",recv_len_);
  printf("remote address is %s and %d\n",inet_ntoa(remaddr_.sin_addr),ntohs(remaddr_.sin_port));
  if(recv_len_<0){
    perror("[ERROR]cannot receive radar data!");
    close(radar_socket_);
    return false;
  }
  else
  {
    Proc_Radar_Data();
  }
  return true;
}

bool FrontalDelphiRadar::Send_Triggle_Signal(){    //没调用
  int static count = 0;
  ++count;
  if(count == 20)//发送20次触发信号
  {
    return true;
  }
  //1)specify remote address
  sockaddr_in send_addr;
  memset(&send_addr,0,sizeof(send_addr));
  send_addr.sin_addr.s_addr = inet_addr(FRONTAL_RADAR_IP);//IP
  send_addr.sin_family = AF_INET;
  send_addr.sin_port = htons(FRONTAL_RADAR_PORT); //port
  unsigned char FI=0b00001000;
  unsigned char tmpCanID1=0b00000000;
  unsigned char tmpCanID2=0b00000000;
  unsigned char tmpCanID3=0b00000100;
  unsigned char tmpCanID4=0b11110001;
  unsigned char tmpNum=0b00000000;
  //  quint8 tmpNum2 = 0b10111111;
  unsigned char tmpNum2 = 0xbf;

  //2)fill send buffer
  //send data buffer
  unsigned char send_buf[13];
  /*ID = 0X04f1 DATA = 00 00 00 00 00 00 BF 00*/
  send_buf[0]=FI;
  send_buf[1]=tmpCanID1;
  send_buf[2]=tmpCanID2;
  send_buf[3]=tmpCanID3;
  send_buf[4]=tmpCanID4;
  send_buf[5]=tmpNum;
  send_buf[6]=tmpNum;
  send_buf[7]=tmpNum;
  send_buf[8]=tmpNum;
  send_buf[9]=tmpNum;
  send_buf[10]=tmpNum;
  send_buf[11]=tmpNum2;
  send_buf[12]=tmpNum;
  int send_len = sendto(radar_socket_,send_buf,sizeof(send_buf),0,(sockaddr*)&remaddr_,sizeof(remaddr_));
  if(send_len<0){
    perror("[ERROR]cannot send initializiton data!");
    return false;
  }
  return true;
}

bool FrontalDelphiRadar::Send_Vehicle_Info(){
	printf("[INFO] <frontal_delphi_radar> send vehicle info to radar...\n");
  //vehicle info value assignment
   //车速
   int speed_can = (int)(self_vehicle_info_.vehicle_speed/0.0625f+0.5f);   //+0.5f啥意思
   unsigned char bfsign = 0; //车速方向，0——向前，1——向后
   if(self_vehicle_info_.vehicle_speed<0) bfsign = 1;
   radar_target_data_.vehicle_speed_origin = self_vehicle_info_.vehicle_speed;
   //方向盘转角
   float steer_phi = self_vehicle_info_.steering_angle;
   unsigned char steersign = steer_phi>0?1:0;
   short steer_can = (short)abs(steer_phi);


   //横摆角速度
   float yawrate_phi = self_vehicle_info_.yaw_rate; //0.2用来调整偏差，根据实际情况设定
   if(yawrate_phi<-128)
   {
     yawrate_phi = -128;
   }
   else if(yawrate_phi>127.9375)
   {
     yawrate_phi = 127;
   }
   radar_target_data_.yaw_rate_origin =  self_vehicle_info_.yaw_rate;
   short yawrate_can = (short)(yawrate_phi/0.0625f+0.5f);   //四舍五入取比yawrate_phi/0.0625f大的最小整数
   //转弯半径
   int radius;
   if(abs(yawrate_phi/180.0f*PI)<1.0f/8191)
   {
     if(yawrate_phi<0) radius = -8192;
     else              radius = 8192;
   }
   else{
     radius = (int)(1.0f/(yawrate_phi/180.0f*PI)+0.5f);
   }
   //Send info to ID 0x4F0
   unsigned char FI=0b00001000;
   unsigned char tmpCanID1=0b00000000;   //0b说明这段数字是二进制
   unsigned char tmpCanID2=0b00000000;
   unsigned char tmpCanID3=0b00000100; //04
   unsigned char tmpCanID4=0b11110000; //F0
   unsigned char send_buf_4F0[13];
   memset(send_buf_4F0,0,sizeof(send_buf_4F0));
   send_buf_4F0[0]=FI;
   send_buf_4F0[1]=tmpCanID1;
   send_buf_4F0[2]=tmpCanID2;
   send_buf_4F0[3]=tmpCanID3;
   send_buf_4F0[4]=tmpCanID4;
   send_buf_4F0[5]=(speed_can>>3); //车速, m/s  第0字节存车速的低8位
   send_buf_4F0[6]=(((speed_can&0x07)<<5)|((yawrate_can>>8)&0x0F)|(bfsign<<4));//横摆角速度，行驶方向   第1字节存车速的高3位 横摆角速度的高4位 车速方向的1位
   send_buf_4F0[7]=((yawrate_can)&0xFF);//横摆角速度
   send_buf_4F0[8]=(0x80|(radius>>8));//横摆角速度有效位,转弯半径,0x80代表横摆角速度有效，0x00代表无效
   send_buf_4F0[9]=(radius&0xFF);//转弯半径
   send_buf_4F0[10]=(0x00|(steersign<<6)|(steer_can>>5));//方向盘转角有效位，方向盘转角方向，方向盘转角
   send_buf_4F0[11]=((steer_can&0x1F)<<3);
   //发送
   int send_len = sendto(radar_socket_,send_buf_4F0,sizeof(send_buf_4F0),0,(sockaddr*)&remaddr_,sizeof(remaddr_));
   if(send_len<0){
     perror("[ERROR]cannot send send_buf_4F0 data!");
   }


   //Send info to ID 0x4F1
   unsigned char send_buf_4F1[13];
   memset(send_buf_4F1,0,sizeof(send_buf_4F1));
   send_buf_4F1[0]=0b00001000;   //标准帧与扩展帧标志位  0B00001000 bit3=1 扩展ID   bit3=0 标准ID
   send_buf_4F1[1]=0x00;
   send_buf_4F1[2]=0x00;
   send_buf_4F1[3]=0x04;
   send_buf_4F1[4]=0xF1;
   send_buf_4F1[10]= 0;//横向安装偏差为0,
   send_buf_4F1[11]=(1<<7)|(1<<6)|63;//雷达辐射命令位置1，阻塞关闭位置1，最大跟踪目标数64
   send_buf_4F1[12] =(1<<5);//速度有效位置
   //发送
   send_len = sendto(radar_socket_,send_buf_4F1,sizeof(send_buf_4F1),0,(sockaddr*)&remaddr_,sizeof(remaddr_));
   if(send_len<0){
     perror("[ERROR]cannot send send_buf_4F1 data!");
   }
   //Send info to ID 0x5F2
   unsigned char send_buf_5F2[13];
   memset(send_buf_5F2,0,sizeof(send_buf_5F2));
   send_buf_5F2[0] = 0b00001000;
   send_buf_5F2[1]=0x00;
   send_buf_5F2[2]=0x00;
   send_buf_5F2[3]=0x05;
   send_buf_5F2[4]=0xF2;
   send_buf_5F2[7] = (10>>1);//长距离模式的角度为10度
   send_buf_5F2[8] = ((10&0x01)<<7)|45;//短距离模式的角度是45度
   send_buf_5F2[9] = 55; //雷达的安装高度 cm
   //发送
   send_len = sendto(radar_socket_,send_buf_5F2,sizeof(send_buf_5F2),0,(sockaddr*)&remaddr_,sizeof(remaddr_));
   if(send_len<0){
     perror("[ERROR]cannot send send_buf_5F2 data!");
   }

   return true;


}

void FrontalDelphiRadar::Proc_Radar_Data(){
  int can_frame_count = recv_len_/13;//each can frame contains 13 bytes
  printf("recv_len_ is %d ========can_frame_count is %d \n",recv_len_,can_frame_count);
  for(int i=0;i<can_frame_count;++i){//a udp data frame may contain numbers of CAN frames
    unsigned char* buf = &(radar_data_buf_[i*13]);   // 因为一帧CAN报文为108位，大概为13字节 ，而一帧UDP可能含有多帧CAN报文
    unsigned int tmpCanID = 0;
    unsigned char tmpdata[8] = {0};
    tmpCanID=buf[1]<<24|buf[2]<<16|buf[3]<<8|buf[4];                                                                   //”|“与"<<"为位运算符，"<<"为左移运算符，”|“为或运算符
    tmpdata[0]=buf[5];tmpdata[1]=buf[6];tmpdata[2]=buf[7];
    tmpdata[3]=buf[8];tmpdata[4]=buf[9];tmpdata[5]=buf[10];
    tmpdata[6]=buf[11];tmpdata[7]=buf[12];   //表示的是8个字节中的数据，数组的一位存放一个字节的数

    /*******************************/
    /*parsing the radar data we want*/
    /*******************************/
    //get the vehicle info from ESR to validate we have send vehicle info to ESR indeed
    //Delphi Electronically Scanning Radar (ESR)

    if(tmpCanID == 0x4E0){
    	//CAN_TX_YAW_RATE_CALC
    	unsigned short temp_A1 = tmpdata[5];
    	unsigned short temp_A2 = tmpdata[6]&0x00F0;                                                  //0x00F0这个是值不是地址
    	unsigned short temp_A = temp_A1&0x0080;   //判断第6个字节的最高位是否为1，是因为有些值可能为负，最高位为符号位   是不是有误，temp_A2还应该右移4位
    	if(temp_A == 0)
    		{
    		radar_target_data_.ESR_yaw_rate = ((temp_A1<<4)|(temp_A2))*0.0625f;
    	}
    	if(temp_A == 0x0080){
    		unsigned short temp_a0 = ((temp_A1<<4)|temp_A2);
    		unsigned short temp_a1 = ~temp_a1;   //～是按位取反运算符
    		unsigned short temp_a2 = (temp_a1&0x07FF)+1;
    		radar_target_data_.ESR_yaw_rate = -(temp_a2*0.0625f);
    	}

    	//CAN_TX_Vehicle_Speed_CALC
    	temp_A1 = tmpdata[6]&0x0007;
    	temp_A2 = tmpdata[7];
    	radar_target_data_.ESR_vehicle_speed = ((temp_A1<<8)|(temp_A2))*0.0625f;

    	printf("yaw_rate from ESR is ++++++++++++++++++++++++++++++++++++++++++%f \n",radar_target_data_.ESR_yaw_rate);
    	printf("vehicle_speed from ESR is ++++++++++++++++++++++++++++++++++%f \n",radar_target_data_.ESR_vehicle_speed);
    }
    //get the most dangerous target ID
    unsigned short TrackID_1 = 0,TrackID_2 = 0;
    if(tmpCanID == 0x4E3){
      TrackID_1 = tmpdata[1];//动态ACC目标
      TrackID_2 = tmpdata[7];//静态ACC目标
    }
    radar_target_data_.ACC_Target_ID = TrackID_1; //choose dynamic ACC target first
    if(TrackID_1 == 0){
      radar_target_data_.ACC_Target_ID = TrackID_2;
    }
    //get the target CAN ID data
    //tmpCanID 要查找的元素,类型要与vector<>类型一致
    //返回的是一个迭代器指针
    vector<unsigned int>::iterator iter = find(radar_target_CAN_ID_vec_.begin(),radar_target_CAN_ID_vec_.end(),tmpCanID);
    if(iter!=radar_target_CAN_ID_vec_.end()){ //obtain valid radar target data
      int m = iter-radar_target_CAN_ID_vec_.begin();//the m-th target, m = 0~63
      radar_target_data_.delphi_detection_array[m].target_ID = m+1; //target_ID = 1~64
      //target status
      unsigned short temp_S1 = tmpdata[1];
      unsigned short temp_S2 = temp_S1&0x00E0;
      radar_target_data_.delphi_detection_array[m].status = temp_S2>>5;
      printf("mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm is %d \n",m);
      unsigned short temp_D1;
      unsigned short temp_D2;
      unsigned short temp_V1;
      unsigned short temp_V2;
      unsigned short temp_V3;
      //range  Unit: m
      temp_D1 = tmpdata[2];
      temp_D2 = tmpdata[3];
      radar_target_data_.delphi_detection_array[m].range = ((temp_D2)|((temp_D1&0x0007)<<8))*0.1f;
      printf("range is %f \n",radar_target_data_.delphi_detection_array[m].range);

      //range_rate  Unit: m/s
      temp_V1 = tmpdata[6];
      temp_V2 = tmpdata[7];
      unsigned short  temp_V=temp_V1&0x0020;   //二进制的第一位表示符号位，1表示负。0表示正
      if (temp_V==0)
      {
        radar_target_data_.delphi_detection_array[m].v =(temp_V2|((temp_V1&0x003F)<<8))*0.01f;//Unit: m/s
      }
      if (temp_V==0x0020)
      {
        unsigned short temp_0=((temp_V1&0x003F) <<8)|temp_V2;
        unsigned short temp_1=~temp_0;   //真值对应的是原码  -128的机器码原码实际是：110000000，反码101111111，补码110000000，截取低8位即10000000，表示的是一个负数。
        unsigned short temp_2=(temp_1 & 0x1FFF)+1;   //数值在计算机中是以补码的形式存储的
        radar_target_data_.delphi_detection_array[m].v=-(temp_2*0.01f);//Unit: m/s
      }
      printf("range rate is %f \n",radar_target_data_.delphi_detection_array[m].v);

      //angle  Unit:degree
      unsigned short temp_A1=tmpdata[1];
      unsigned short temp_A2=tmpdata[2];
      unsigned short temp_A3=temp_A1&0x0010;
      if (temp_A3==0)
      {
        radar_target_data_.delphi_detection_array[m].angle=(((temp_A1&0x000F)<<5)|((temp_A2&0x00F8)>>3))*0.1f;   //左移5是因为angle在第二字节占5位
      }
      if(temp_A3==0x0010)
      {
        unsigned short temp_3=((temp_A1&0x000F)<<5)|((temp_A2&0x00F8)>>3);
        unsigned short temp_4=~temp_3;
        unsigned short temp_5=(temp_4& 0x01FF)+1;   //补码转原码，按位取反加1   已知补码求原码： 最高位如果是1的话（负数），那么除了最高位之外的取反，然后加1得原码。
        radar_target_data_.delphi_detection_array[m].angle=-temp_5*0.1f;   //真值是由原码得到的
      }
//      printf("angle is %f \n",radar_target_data_.delphi_detection_array[m].angle);

      //calculate x,y   Unit: m
      radar_target_data_.delphi_detection_array[m].x=radar_target_data_.delphi_detection_array[m].range*sin(radar_target_data_.delphi_detection_array[m].angle*toRAD);
      radar_target_data_.delphi_detection_array[m].y=radar_target_data_.delphi_detection_array[m].range*cos(radar_target_data_.delphi_detection_array[m].angle*toRAD);
    }//end if(iter!=radar_target_CAN_ID_vec_.end())

#if 1 //解析一些运动属性
    if(tmpCanID==0x540)
    {
      ////先解析Group_ID
      unsigned short temp_A1 = tmpdata[0];
      unsigned short temp_A2 = temp_A1&0x000F; //取出后四位
      switch(temp_A2)   //说明一帧地址为0x540的CAN中，一次只会有一组，每组有7个运动目标，所以才用条件选择语句，如果同时有64个，就要用循环语句了
      {
      case 0://第0组
        for(int j = 0;j<7;++j)
        {
          //解析Moving状态
          unsigned short temp_D1 = (tmpdata[j+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;
          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;

        }
        break;
      case 1://第1组
        for(int j = 7;j<14;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-7+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-7+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-7+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 2://第2组
        for(int j = 14;j<21;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-14+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-14+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-14+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 3://第3组
        for(int j = 21;j<28;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-21+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-21+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-21+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 4://第4组
        for(int j = 28;j<35;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-28+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-28+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-28+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 5://第5组
        for(int j = 35;j<42;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-35+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-35+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-35+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 6://第6组
        for(int j = 42;j<49;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-42+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-42+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-42+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 7://第7组
        for(int j = 49;j<56;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-49+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-49+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-49+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 8://第8组
        for(int j = 56;j<63;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-56+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short  temp_S1 = (tmpdata[j-56+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-56+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 9://第9组
        unsigned short temp_D1 = (tmpdata[1])&0x0020;
        unsigned short temp_D2 = temp_D1>>5;
        radar_target_data_.delphi_detection_array[63].moving=temp_D2;

        //解析Movable_fast状态
        unsigned short temp_S1 = (tmpdata[1])&0x0080;
        unsigned short temp_S2 = temp_S1>>7;
        radar_target_data_.delphi_detection_array[63].moving_fast=temp_S2;
        //解析Movable_slow状态
        temp_S1 = (tmpdata[1])&0x0040;
        temp_S2 = temp_S1>>6;
        radar_target_data_.delphi_detection_array[63].moving_slow=temp_S2;
        break;
      }//end switch
    }//end if(tmpCanID==0x540)
#endif

  }

}

delphi_radar_target FrontalDelphiRadar::radar_target_data(){
  return this->radar_target_data_;
}

//没调用
Vehicle_Info FrontalDelphiRadar::vehicle_info(){
  return this->self_vehicle_info_;
}

//没调用
void FrontalDelphiRadar::set_self_vehicle_info(const double& yaw_rate,const double& vehicle_speed,const double& steering_angle){
  self_vehicle_info_.yaw_rate = yaw_rate;
  self_vehicle_info_.vehicle_speed = vehicle_speed;
  self_vehicle_info_.steering_angle = steering_angle;
}

void FrontalDelphiRadar::set_self_vehicle_info(const Vehicle_Info& vehicle_info){
	printf("[INFO] <frontal_delphi_radar> ESR has received vehicle info, vehicle speed is %.6f,yaw rate is %.6f\n",
			vehicle_info.vehicle_speed,
			vehicle_info.yaw_rate);
  self_vehicle_info_ = vehicle_info;
}

