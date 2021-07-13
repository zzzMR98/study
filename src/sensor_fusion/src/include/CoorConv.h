#include <cmath>

#define CPI 3.14159265358979

/* Ellipsoid model constants (actual values here are for WGS84) */
#define sm_a  6378137.0
#define sm_b  6356752.314
#define sm_EccSquared  6.69437999013e-03
#define UTMScaleFactor  0.9996

typedef struct tagUTMCorr 
{
	double x;
	double y;
}UTMCoor;

typedef struct tagWGS84Corr
{
	double lat;
	double log;
}WGS84Corr;
/*
* DegToRad
*
* Converts degrees to radians.
*
*/
class UTMconv
{
public:
	UTMconv();
	~UTMconv();

	double DegToRad(double deg);
	double RadToDeg(double rad);
	double ArcLengthOfMeridian(double phi);
	double UTMCentralMeridian(int zone);
	double FootpointLatitude(double y);
	void MapLatLonToXY(double phi, double lambda, double lambda0, UTMCoor &xy);
	void MapXYToLatLon(double x, double y, double lambda0, WGS84Corr &philambda);
	void LatLonToUTMXY(double lat, double lon, int zone, UTMCoor &xy);
	void UTMXYToLatLon(double x, double y, int zone, bool southhemi, WGS84Corr &latlon);

private:
	
};





