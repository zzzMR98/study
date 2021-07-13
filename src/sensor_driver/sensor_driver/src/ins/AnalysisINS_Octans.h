
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <math.h>
#include <fstream>
#include <unistd.h>

#include  <iostream>


#define BUFSIZE 4096

typedef unsigned char                 UCHAR;
/******************INS  Model output***************************************/

 struct struct_INS
{

	long	lTimeStamp;			// 时间戳(Unit:ms)
	double dHeading;
	double dPitch;
	double dRoll;
};


class CAnalysisINS
{
   public:
    struct_INS  INSData_struct;
    CAnalysisINS(); 
    ~CAnalysisINS(); 
    bool Init(int port);
    void Update();
   


  private:
    void OctansData_HEHDT(UCHAR * OctansData,int n);
	void OctansData_PHTRO(UCHAR * OctansData,int n);

    sockaddr_in myaddr; /* our address */
    sockaddr_in remaddr; /* remote address */
    socklen_t addrlen; /* length of addresses */

    int recvlen; /* # bytes received */
    int fd; /* our socket */
    unsigned char buf[BUFSIZE]; /* receive buffer */

//    std::fstream data_backup;
};
