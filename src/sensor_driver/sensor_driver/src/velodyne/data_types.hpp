#pragma once

#include <vector>
#include <assert.h>
#include <boost/math/special_functions/round.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <assert.h>
#include "Eigen/Core"

#ifdef _WINDOWS
typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
typedef __int16 int16_t;

uint64_t
htobe64( uint64_t input)
{
	uint64_t rval;
	uint8_t *data = (uint8_t *)&rval;

	data[0] = input >> 56;
	data[1] = input >> 48;
	data[2] = input >> 40;
	data[3] = input >> 32;
	data[4] = input >> 24;
	data[5] = input >> 16;
	data[6] = input >> 8;
	data[7] = input >> 0;

	return rval;
}

uint64_t
be64toh( uint64_t input)
{
	return (htobe64(input));
}

#endif

template<class T = unsigned char>
class OGMData {
public:
	OGMData() :
			initflag(false) {
	}
	;
	OGMData(float height, float width, float resolution, float y = 0, float x =
			0) :
			initflag(false) {
		initogm(height, width, resolution, y, x);
	}
	OGMData(const OGMData<T>& ogmdata) :
			initflag(false) {

		//*this = ogmdata;
		initogm(ogmdata.ogmheight, ogmdata.ogmwidth, ogmdata.ogmresolution,
				ogmdata.vehicle_y, ogmdata.vehicle_x);
		timestamp = ogmdata.timestamp;
		updateflag = ogmdata.updateflag;
		memcpy(this->ogm, ogmdata.ogm, ogmdata.ogmcell_size * sizeof(T));

	}
	virtual ~OGMData() {
		if (initflag) {
			//delete [] tempogm;
			delete[] ogm;
		}
	}
	float ogmwidth; //-20 to 20
	float ogmheight; //-20 to 60
	float ogmresolution;
	int ogmwidth_cell;
	int ogmheight_cell;
	int ogmcell_size;
	float vehicle_x;
	float vehicle_y;
	//unsigned char* tempogm ;
	T* ogm;
	bool initflag;
	bool updateflag;
	boost::uint64_t timestamp;
	void initogm(float height, float width, float resolution, float y = 0,
			float x = 0) {
		if (initflag)
			delete[] ogm;
		ogmwidth = width; //-20 to 20
		ogmheight = height; //-20 to 60
		ogmresolution = resolution;
		ogmwidth_cell = boost::math::round(ogmwidth / ogmresolution) + 1;
		ogmheight_cell = boost::math::round(ogmheight / ogmresolution) + 1;
		ogmcell_size = ogmwidth_cell * ogmheight_cell;
		setvehiclepos(x, y);
		//tempogm = new unsigned char[ogmcell_size];
		ogm = new T[ogmcell_size];
		memset(ogm, 0, ogmcell_size * sizeof(T));
		initflag = true;
		timestamp = 0;
		updateflag = false;
	}

	void setvehiclepos(float x, float y) {
		vehicle_x = x;
		vehicle_y = y;
	}

	static void jointogm(const OGMData<T>& src1, const OGMData<T>& src2,
			OGMData<T>& dst) {
//          assert(src1.ogmresolution==src2.ogmresolution);
//          assert(src1.ogmresolution==dst.ogmresolution);
		for (int row = 0; row < dst.ogmheight_cell; row++) {
			double pointy = row * dst.ogmresolution - dst.vehicle_y;
			for (int col = 0; col < dst.ogmwidth_cell; col++) {
				double pointx = col * dst.ogmresolution - dst.vehicle_x;

				int row1 = (pointy + src1.vehicle_y) / src1.ogmresolution;
				int col1 = (pointx + src1.vehicle_x) / src1.ogmresolution;
				if (row1 >= 0 && row1 < src1.ogmheight_cell && col1 >= 0
						&& col1 < src1.ogmwidth_cell) {
					dst.ogm[row * dst.ogmwidth_cell + col] = src1.ogm[row1
							* src1.ogmwidth_cell + col1];
				}

				int row2 = (pointy + src2.vehicle_y) / src2.ogmresolution;
				int col2 = (pointx + src2.vehicle_x) / src2.ogmresolution;
				if (row2 >= 0 && row2 < src2.ogmheight_cell && col2 >= 0
						&& col2 < src2.ogmwidth_cell) {
					dst.ogm[row * dst.ogmwidth_cell + col] = src2.ogm[row2
							* src2.ogmwidth_cell + col2];
				}

			}
		}
	}

	static void decomposeogm(OGMData<T>& src1, OGMData<T>& src2,
			const OGMData<T>& dst) {
//          assert(src1.ogmresolution==src2.ogmresolution);
//          assert(src1.ogmresolution==dst.ogmresolution);
		for (int row = 0; row < dst.ogmheight_cell; row++) {
			double pointy = row * dst.ogmresolution - dst.vehicle_y;
			for (int col = 0; col < dst.ogmwidth_cell; col++) {
				T val = dst.ogm[row * dst.ogmwidth_cell + col];
				double pointx = col * dst.ogmresolution - dst.vehicle_x;

				int row1 = (pointy + src1.vehicle_y) / src1.ogmresolution;
				int col1 = (pointx + src1.vehicle_x) / src1.ogmresolution;
				if (row1 >= 0 && row1 < src1.ogmheight_cell && col1 >= 0
						&& col1 < src1.ogmwidth_cell) {
					src1.ogm[row1 * src1.ogmwidth_cell + col1] = val;
				}

				int row2 = (pointy + src2.vehicle_y) / src2.ogmresolution;
				int col2 = (pointx + src2.vehicle_x) / src2.ogmresolution;
				if (row2 >= 0 && row2 < src2.ogmheight_cell && col2 >= 0
						&& col2 < src2.ogmwidth_cell) {
					src2.ogm[row2 * src2.ogmwidth_cell + col2] = val;
				}

			}
		}
	}

	bool withinLimit(int x,int y){
		if(x>=0&&x<ogmwidth_cell&&y>=0&&y<ogmheight_cell){
			return true;
		}
		else
			return false;
	}

	bool withinLimit(double x,double y){
		if(x>=0&&x<ogmwidth&&y>=0&&y<ogmheight){
			return true;
		}
		else
			return false;
	}


	void updateStamp() {
		timestamp = getCurrentStamp64();
		updateflag = true;

	}

	bool timeIsValid(boost::uint64_t thredshold = 200){ //ms
		if (!updateflag)
			return false;
		boost::uint64_t timediff = getCurrentStamp64() - timestamp;
		//std::cout<<"timediff:"<<timediff<<std::endl;
		if (timediff < thredshold)
			return true;
		else {
			updateflag = false;
			return false;
		}
	}
	boost::uint64_t getCurrentStamp64() {//ms
		boost::posix_time::ptime epoch(
				boost::gregorian::date(1970, boost::gregorian::Jan, 1));
		boost::posix_time::time_duration time_from_epoch =
				boost::posix_time::microsec_clock::universal_time() - epoch;
		//boost::posix_time::second_clock::universal_time() - epoch;

		//return time_from_epoch.total_microseconds();

		return time_from_epoch.total_milliseconds();
		//return time_from_epoch.total_seconds();
	}

	OGMData<T>& operator =(const OGMData<T>& ogmdata) {

		if (this == &ogmdata)
			return *this;
		initogm(ogmdata.ogmheight, ogmdata.ogmwidth, ogmdata.ogmresolution,
				ogmdata.vehicle_y, ogmdata.vehicle_x);
		timestamp = ogmdata.timestamp;
		updateflag = ogmdata.updateflag;
		memcpy(this->ogm, ogmdata.ogm, ogmdata.ogmcell_size * sizeof(T));

		return *this;

	}

};

class OGMFilter: public OGMData<unsigned char> {
public:
	OGMFilter(float height, float width, float resolution, float y = 0,
			float x = 0,unsigned char filterthreshold = 1)
	:OGMData(height,width,resolution, y,x)
	,filterthreshold_(filterthreshold){
		memset(ogm,0,ogmcell_size);

	}
	bool addPoint(const float& x,const float& y) {
		int col = (x + vehicle_x)/ogmresolution;
		int row = (y + vehicle_y)/ogmresolution;
		int index = row*ogmwidth_cell+col;
		if(withinLimit(col,row)&&ogm[index]<filterthreshold_)
		{
			ogm[index]++;
			return true;
		}
		else
			return false;
	}
private:
	const unsigned char filterthreshold_;
};

