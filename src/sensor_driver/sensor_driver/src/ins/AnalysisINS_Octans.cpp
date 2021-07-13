#include "AnalysisINS_Octans.h"
#include <stdlib.h>

CAnalysisINS::CAnalysisINS()
{

}

CAnalysisINS::~CAnalysisINS()
{
//	data_backup.close();
}

bool CAnalysisINS::Init(int port)
{
    /*addrlen = sizeof(myaddr); /* length of addresses */
    /* create a UDP socket */
    /*if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
	perror("cannot create socket\n");
	return false;
    }
    /* bind the socket to any valid IP address and a specific port */
 /*   memset((char*)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr =htonl(INADDR_ANY);
    myaddr.sin_port = htons(port);
    if(bind(fd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
	perror("bind failed INS");
	return false;
    }
   return true;*/
    addrlen = sizeof(myaddr); /* length of addresses */
    /* create a UDP socket */
    if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
	perror("cannot create socket\n");
	return false;
    }
    /* bind the socket to any valid IP address and a specific port */
    memset((char*)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr =htonl(INADDR_ANY);
    myaddr.sin_port = htons(port);
    if(bind(fd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
	perror("bind failed ins");
	return false;
    }
   return true;


  }




void CAnalysisINS::Update()
{
	   recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&myaddr, &addrlen);
		if ( (buf[recvlen-2]=='$')&&(buf[recvlen-1]=='H')&&(buf[2-2]=='E')&&(buf[3-2]=='H')&&(buf[4-2]=='D')&&(buf[5-2]=='T'))
	    {
	    		OctansData_HEHDT(buf,recvlen);
	    }
	    else if((buf[recvlen-2]=='$')&&(buf[recvlen-1]=='P')&&(buf[2-2]=='H')&&(buf[3-2]=='T')&&(buf[4-2]=='R')&&(buf[5-2]=='O'))
		{
			OctansData_PHTRO(buf, recvlen);
		}
}
void CAnalysisINS::OctansData_HEHDT(UCHAR * OctansData,int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
//	UINT  unTemp = 0;
	long  lTemp = 0;

	int i = 0;
	int j = 0;
	int OctansCommaNumber = 0;
	char OctansDataTemp[15];

	memset(OctansDataTemp,0,15);

	while(i++ <= n)
	{

		if(OctansData[i] == ',')		//找到逗号
		{
			j = 0;
			OctansCommaNumber++;
			switch(OctansCommaNumber)
			{
			case 1:	// $HEHDT
				memset(OctansDataTemp,0,15);
				break;
			case 2: // Course over ground

				dTemp = atof(OctansDataTemp);

				INSData_struct.dHeading=dTemp;

//				strTemp.Format("   %f",dTemp);
				memset(OctansDataTemp,0,15);
				break;
			case 3: // Magnetic course over ground
				memset(OctansDataTemp,0,15);
				break;
			case 4: //
				memset(OctansDataTemp,0,15);
				break;

			default: break;
			}
		}
		else
		{
			if(OctansCommaNumber <=7 )
				OctansDataTemp[j++] = OctansData[i];
		}
	}
	// save data in file (GPVTG.txt)
	{
/*	FILE *fp;

	fp  = fopen( "Octans.txt","a+");//打开文件
		if( fp )
		{
			fprintf(fp,"%s\n",OctansData);
			fclose(fp);

		}*/
	}
}


void CAnalysisINS::OctansData_PHTRO(UCHAR * OctansData,int n)
{

	double dTemp = 0.0;
	double dTemp1 = 0.0,dTemp2 = 0.0;
	char  unTemp = 0;
	long  lTemp = 0;

	int i = 0;
	int j = 0;
	int OctansCommaNumber = 0;
	char OctansDataTemp[15];

	memset(OctansDataTemp,0,15);

	while(i++ <= n)
	{

		if(OctansData[i] == ',')		//找到逗号
		{
			j = 0;
			OctansCommaNumber++;
			switch(OctansCommaNumber)
			{
				case 1:	// $PHTRO
					memset(OctansDataTemp,0,15);
					break;
				case 2: // pitch in degree

					dTemp = atof(OctansDataTemp);

					INSData_struct.dPitch = dTemp;

//				strTemp.Format("   %f",dTemp);
					memset(OctansDataTemp,0,15);
					break;
				case 3: // M for bow up, P for bow down
					unTemp = OctansDataTemp[0];
					if(unTemp == 'P')
						INSData_struct.dPitch = - INSData_struct.dPitch;
					memset(OctansDataTemp,0,15);
					break;
				case 4: // roll in degree
					dTemp = atof(OctansDataTemp);
					INSData_struct.dPitch = dTemp;
					memset(OctansDataTemp,0,15);
					break;
				case 5:// B for port down, T for port up
					unTemp = OctansDataTemp[0];
					if(unTemp == 'T')
						INSData_struct.dPitch = - INSData_struct.dPitch;
				default: break;
			}
		}
		else
		{
			if(OctansCommaNumber <=7 )
				OctansDataTemp[j++] = OctansData[i];
		}
	}
	// save data in file (GPVTG.txt)
	{
/*	FILE *fp;

	fp  = fopen( "Octans.txt","a+");//打开文件
		if( fp )
		{
			fprintf(fp,"%s\n",OctansData);
			fclose(fp);

		}*/
	}
}