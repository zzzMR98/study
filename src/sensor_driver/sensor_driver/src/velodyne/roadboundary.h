#include "data_types.hpp"
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

struct POINT_2F{
    double x;
    double y;
    float height;
    float angle;
    float layer;
    int index;
    float distance;
    float weight;
};

struct CloudPointSection{
    float height;
    float distance;
    float angle;
    float anglerange;
    int passablepointnum;
    int unkonwnpointnum;
    POINT_2F minpoint;
    POINT_2F maxpoint;

};
class CloudPointSections{
    public:
    CloudPointSections(){
    };
    ~CloudPointSections(){};
/*
   inline void merge(int num1 , int num2)
    {
        int num;
        if(num1==num2)
        {
            return;
        }
        else if(num1 <num2)
        {
            sections[num1].maxpoint = sections[num2].maxpoint;
            sections[num1].height = (sections[num1].height + sections[num2].height)/2;
            sections[num1].distance = (sections[num1].distance + sections[num2].distance)/2;
            sections[num1].passablepointnum = sections[num1].passablepointnum + sections[num2].passablepointnum;
            sections[num1].unkonwnpointnum = sections[num1].unkonwnpointnum + sections[num2].unkonwnpointnum;

            sections.erase(sections.begin()+num1+1,sections.begin()+num2+1);
            num = num1;
        }
        else
        {
            num=num2;
            sections[num1].minpoint = sections[num2].minpoint;
            sections[num1].height = (sections[num1].height + sections[num2].height)/2;
            sections[num1].distance = (sections[num1].distance + sections[num2].distance)/2;
            sections[num1].passablepointnum = sections[num1].passablepointnum + sections[num2].passablepointnum;
            sections[num1].unkonwnpointnum = sections[num1].unkonwnpointnum + sections[num2].unkonwnpointnum;

            sections.erase(sections.begin()+num2,sections.begin()+num1);

        }

    }
*/
//    int layer_;
    std::vector<CloudPointSection> sections;
};


struct RoadNode{
    int index;
    float angle;
    float anglebegin;
    float angleend;
    float anglerange;
    float distance;
    float height;
    float lastdistance;
    float lastheight;
    bool valid;   //
};

class RoadPath{
public:
    RoadPath(int maxsize=32)
    {
		nodes.reserve(maxsize);
        validnum.reserve(3);
    }
    RoadPath(const RoadPath& temp)
    {
		nodes.reserve(temp.nodes.capacity());
        nodes = temp.nodes;
        validnum = temp.validnum;
    }

    RoadPath& operator=(const RoadPath& temp)
    {
        if(this == &temp)
            return *this;
		nodes.reserve(temp.nodes.capacity());
        nodes = temp.nodes;
        validnum = temp.validnum;
        return *this;
    }
    bool operator>(const RoadPath& temp) const
    {
//        std::cout<<">"<<std::endl;
        assert(temp.nodes.size()>0&&nodes.size()>0);
        assert(temp.validnum.size()>0&&validnum.size()>0);
        if(validnum.at(0)>temp.validnum.at(0))
            return true;
        else if(validnum.at(0)<temp.validnum.at(0))
            return false;
        if(nodes.at(validnum.at(0)).anglerange > temp.nodes.at(temp.validnum.at(0)).anglerange)
            return true;
        else
            return false;
    }

    bool operator<(const RoadPath& temp) const
    {
        assert(temp.nodes.size()>0&&nodes.size()>0);
        assert(temp.validnum.size()>0&&validnum.size()>0);

        if(validnum.at(0)<temp.validnum.at(0))
            return true;
        else if(validnum.at(0)>temp.validnum.at(0))
            return false;
        if(nodes.at(validnum.at(0)).anglerange < temp.nodes.at(temp.validnum.at(0)).anglerange)
            return true;
        else
            return false;
    }

    static bool angleendsort(const RoadPath& temp1,const RoadPath& temp2)
    {
        if(temp1.nodes.at(temp1.validnum.at(0)).angleend < temp2.nodes.at(temp2.validnum.at(0)).angleend)
            return true;
        else
            return false;
    }
    std::vector<RoadNode> nodes;
    std::vector<int> validnum;
    double prob;

};

class RoadPaths{
public:
    std::vector<RoadPath> paths;
};

struct BoundaryPose
{
    BoundaryPose()
    {
        clear();
    }
    inline void clear()
    {
        a0 = 0;
        a1 = 0;
        a2 = 0;
        matchrate = 0;
        weight = 0;
        style = 0;
        theta = 0;

    }
	double a0;
	double a1;
	double a2;
    double theta;
    double matchrate;
	double weight;
    int style;
};
struct RoadBoundary{

    inline void clear()
    {
        points.clear();
        pose.clear();
    }
	std::vector<POINT_2F> points;
    BoundaryPose pose;
};

class RoadBoundaries{
public:
    RoadBoundaries()
    {
        clear();
        fs_boundary.open("boundary.txt",std::ios::out);
        mainroadkalman_inited = false;
        mainroad_failed_count = 0;

    }
    ~RoadBoundaries()
    {
        fs_boundary.close();
    }
    void clear();
    void RotatePoint(double srcx,double srcy,double& dstx,double& dsty,double theta);
    float Cal_road_boudary_RANSAC(int iteration_num,float threshold_scale,RoadBoundary& boundary,int sample_num,float bias_threshold,int boud_style,int lineposition = 0);
    bool getRoadBoundary(const RoadPaths& roadpaths,const std::vector<CloudPointSections>& cloudsections ,double front_angle );
    void MainBoundaryKalman();
    float Cal_road_boudary_LSM(RoadBoundary& boundary_point,int style);
    RoadBoundary mainroad_leftboundary;
    RoadBoundary mainroad_rightboundary;
    std::vector<RoadBoundary> boundaries;
    bool left_found;
    bool right_found;
    //Kalman
	CvKalman* mainroadkalman;
	bool mainroadkalman_inited;
    int mainroad_failed_count;
    std::fstream fs_boundary;
};


