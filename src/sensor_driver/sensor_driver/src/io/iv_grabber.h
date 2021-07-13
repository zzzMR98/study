
#include <pcl/pcl_config.h>

#ifndef __PCL_IO_IV_IVGrabber__
#define __PCL_IO_IV_IVGrabber__

// needed for the IVGrabber interface / observers
#include <map>
#include <iostream>
#include <string>
#include <typeinfo>
#include <vector>
#include <sstream>
#include <pcl/pcl_macros.h>
#include <pcl/io/boost.h>
#include <pcl/exceptions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct hdlPoint
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;
    float laserID;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (hdlPoint,           // here we assume a XYZ + "test" (as fields)
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (float, laserID, laserID)
        )


#define HAVE_PCAP
#define HDL_Grabber_toRadians(x) ((x) * M_PI / 180.0)

#ifndef _WINDOWS
    typedef unsigned char           UCHAR;
    typedef unsigned char          UINT8;
    typedef unsigned short int UINT16;
    typedef unsigned int UINT;
    typedef long long int LONGLONG;
    typedef unsigned char uchar;

#define strncpy_s(dest,len,src,count) strncpy(dest,src,count)
#define sprintf_s(dest,len,format,args...) sprintf(dest,format,args)
#define sscanf_s sscanf
#define strcpy_s(dst,len,src) strcpy(dst,src)
#define _strdup(src) strdup(src)
#define strtok_s(tok,del,ctx) strtok(tok,del)
#endif

namespace pcl
{
struct CalibrationValue{
    double alfa;
    double beta;
    double gama;
    double x_offset;
    double y_offset;
    double z_offset;

};

/// \brief 计算标定矩阵
///
/// \param calibvalue　标定值
/// \param transform_matrix_calibration　标定矩阵
template<typename T>
void get_transform_matrix(const CalibrationValue& calibvalue,Eigen::Matrix<T, 4, 4>& transform_matrix_calibration)
{

	std::cout<< calibvalue.alfa <<" "<<calibvalue.beta<<" "<<calibvalue.gama<<std::endl;
	double alfa = calibvalue.alfa*M_PI/180;
	double beta = calibvalue.beta*M_PI/180;
	double gama = calibvalue.gama*M_PI/180;//-164

	double x_offset = calibvalue.x_offset;
	double y_offset = calibvalue.y_offset;
	double z_offset = calibvalue.z_offset;

  transform_matrix_calibration = Eigen::Matrix<T, 4, 4>::Identity();
  transform_matrix_calibration(0,0) = cos(beta)*cos(gama) - sin(beta)*sin(alfa)*sin(gama);
  transform_matrix_calibration(1,0) = cos(beta)*sin(gama) + sin(beta)*sin(alfa)*cos(gama);
  transform_matrix_calibration(2,0) = -cos(alfa)*sin(beta);

  transform_matrix_calibration(0,1) = -cos(alfa)*sin(gama);
  transform_matrix_calibration(1,1) = cos(alfa)*cos(gama);
  transform_matrix_calibration(2,1) = sin(alfa);

  transform_matrix_calibration(0,2) = sin(beta)*cos(gama) + cos(beta)*sin(alfa)*sin(gama);
  transform_matrix_calibration(1,2) = sin(beta)*sin(gama) - cos(beta)*sin(alfa)*cos(gama);
  transform_matrix_calibration(2,2) = cos(alfa)*cos(beta);

  transform_matrix_calibration(3,0) = 0;
  transform_matrix_calibration(3,1) = 0;
  transform_matrix_calibration(3,2) = 0;
  transform_matrix_calibration(3,3) = 1.0;

  transform_matrix_calibration(0,3) = x_offset;
  transform_matrix_calibration(1,3) = y_offset;
  transform_matrix_calibration(2,3) = z_offset;
  std::cout<<"transform_matrix_calibration:"<<std::endl;
  std::cout<<transform_matrix_calibration<<std::endl;
  std::cout<<transform_matrix_calibration(2,0) <<" "<<transform_matrix_calibration(2,1)<<std::endl;
}

/** \brief Grabber for the Velodyne High-Definition-Laser (HDL)
 * \author Keven Ring <keven@mitre.org>
 * \ingroup io
 */

struct LaserData{
    int number;
    double angle;
};

  /** \brief IVGrabber interface for PCL 1.x device drivers
    * \author Suat Gedikli <gedikli@willowgarage.com>
    * \ingroup io
    */
  class PCL_EXPORTS IVGrabber
  {
    public:

      /** \brief Constructor. */
      IVGrabber () : signals_ (), connections_ (), shared_connections_ () {}

      /** \brief virtual desctructor. */
      virtual inline ~IVGrabber () throw ();

      /** \brief registers a callback function/method to a signal with the corresponding signature
        * \param[in] callback: the callback function/method
        * \return Connection object, that can be used to disconnect the callback method from the signal again.
        */
      template<typename T> boost::signals2::connection
      registerCallback (const boost::function<T>& callback);

      /** \brief indicates whether a signal with given parameter-type exists or not
        * \return true if signal exists, false otherwise
        */
      template<typename T> bool
      providesCallback () const;

      /** \brief For devices that are streaming, the streams are started by calling this method.
        *        Trigger-based devices, just trigger the device once for each call of start.
        */
      virtual void
      start () = 0;

      /** \brief For devices that are streaming, the streams are stopped.
        *        This method has no effect for triggered devices.
        */
      virtual void
      stop () = 0;

      /** \brief returns the name of the concrete subclass.
        * \return the name of the concrete driver.
        */
      virtual std::string
      getName () const = 0;

      /** \brief Indicates whether the IVGrabber is streaming or not. This value is not defined for triggered devices.
        * \return true if IVGrabber is running / streaming. False otherwise.
        */
      virtual bool
      isRunning () const = 0;

      /** \brief returns fps. 0 if trigger based. */
      virtual float
      getFramesPerSecond () const = 0;

      virtual void setCalibration(const CalibrationValue& value) = 0;
      virtual void setDataFromExtern() = 0;

      virtual bool externenqueueHDLPacket (const unsigned char *data,std::size_t bytesReceived) = 0;
      virtual void setMinimumDistanceThreshold(float & minThreshold) = 0;

      virtual void setMaximumDistanceThreshold(float & maxThreshold) = 0;

    protected:

      virtual void
      signalsChanged () { }

      template<typename T> boost::signals2::signal<T>*
      find_signal () const;

      template<typename T> int
      num_slots () const;

      template<typename T> void
      disconnect_all_slots ();

      template<typename T> void
      block_signal ();

      template<typename T> void
      unblock_signal ();

      inline void
      block_signals ();

      inline void
      unblock_signals ();

      template<typename T> boost::signals2::signal<T>*
      createSignal ();
      static const int MAX_NUM_LASERS = 64;
      std::map<std::string, boost::signals2::signal_base*> signals_;
      std::map<std::string, std::vector<boost::signals2::connection> > connections_;
      std::map<std::string, std::vector<boost::signals2::shared_connection_block> > shared_connections_;
    public:
    std::map<double,int> map_tanangle_index;//sort by angle
    LaserData indexmaptable[MAX_NUM_LASERS];
  } ;

  IVGrabber::~IVGrabber () throw ()
  {
    for (std::map<std::string, boost::signals2::signal_base*>::iterator signal_it = signals_.begin (); signal_it != signals_.end (); ++signal_it)
      delete signal_it->second;
  }

  template<typename T> boost::signals2::signal<T>*
  IVGrabber::find_signal () const
  {
    typedef boost::signals2::signal<T> Signal;

    std::map<std::string, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find (typeid (T).name ());
    if (signal_it != signals_.end ())
      return (dynamic_cast<Signal*> (signal_it->second));

    return (NULL);
  }

  template<typename T> void
  IVGrabber::disconnect_all_slots ()
  {
    typedef boost::signals2::signal<T> Signal;

    if (signals_.find (typeid (T).name ()) != signals_.end ())
    {
      Signal* signal = dynamic_cast<Signal*> (signals_[typeid (T).name ()]);
      signal->disconnect_all_slots ();
    }
  }

  template<typename T> void
  IVGrabber::block_signal ()
  {
    if (connections_.find (typeid (T).name ()) != connections_.end ())
      for (std::vector<boost::signals2::shared_connection_block>::iterator cIt = shared_connections_[typeid (T).name ()].begin (); cIt != shared_connections_[typeid (T).name ()].end (); ++cIt)
        cIt->block ();
  }

  template<typename T> void
  IVGrabber::unblock_signal ()
  {
    if (connections_.find (typeid (T).name ()) != connections_.end ())
      for (std::vector<boost::signals2::shared_connection_block>::iterator cIt = shared_connections_[typeid (T).name ()].begin (); cIt != shared_connections_[typeid (T).name ()].end (); ++cIt)
        cIt->unblock ();
  }

  void
  IVGrabber::block_signals ()
  {
    for (std::map<std::string, boost::signals2::signal_base*>::iterator signal_it = signals_.begin (); signal_it != signals_.end (); ++signal_it)
      for (std::vector<boost::signals2::shared_connection_block>::iterator cIt = shared_connections_[signal_it->first].begin (); cIt != shared_connections_[signal_it->first].end (); ++cIt)
        cIt->block ();
  }

  void
  IVGrabber::unblock_signals ()
  {
    for (std::map<std::string, boost::signals2::signal_base*>::iterator signal_it = signals_.begin (); signal_it != signals_.end (); ++signal_it)
      for (std::vector<boost::signals2::shared_connection_block>::iterator cIt = shared_connections_[signal_it->first].begin (); cIt != shared_connections_[signal_it->first].end (); ++cIt)
        cIt->unblock ();
  }

  template<typename T> int
  IVGrabber::num_slots () const
  {
    typedef boost::signals2::signal<T> Signal;

    // see if we have a signal for this type
    std::map<std::string, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find (typeid (T).name ());
    if (signal_it != signals_.end ())
    {
      Signal* signal = dynamic_cast<Signal*> (signal_it->second);
      return (static_cast<int> (signal->num_slots ()));
    }
    return (0);
  }

  template<typename T> boost::signals2::signal<T>*
  IVGrabber::createSignal ()
  {
    typedef boost::signals2::signal<T> Signal;

    if (signals_.find (typeid (T).name ()) == signals_.end ())
    {
      Signal* signal = new Signal ();
      signals_[typeid (T).name ()] = signal;
      return (signal);
    }
    return (0);
  }

  template<typename T> boost::signals2::connection
  IVGrabber::registerCallback (const boost::function<T> & callback)
  {
    typedef boost::signals2::signal<T> Signal;
    if (signals_.find (typeid (T).name ()) == signals_.end ())
    {
      std::stringstream sstream;

      sstream << "no callback for type:" << typeid (T).name ();
      /*
      sstream << "registered Callbacks are:" << std::endl;
      for( std::map<std::string, boost::signals2::signal_base*>::const_iterator cIt = signals_.begin ();
           cIt != signals_.end (); ++cIt)
      {
        sstream << cIt->first << std::endl;
      }*/

      PCL_THROW_EXCEPTION (pcl::IOException, "[" << getName () << "] " << sstream.str ());
      //return (boost::signals2::connection ());
    }
    Signal* signal = dynamic_cast<Signal*> (signals_[typeid (T).name ()]);
    boost::signals2::connection ret = signal->connect (callback);

    connections_[typeid (T).name ()].push_back (ret);
    shared_connections_[typeid (T).name ()].push_back (boost::signals2::shared_connection_block (connections_[typeid (T).name ()].back (), false));
    signalsChanged ();
    return (ret);
  }

  template<typename T> bool
  IVGrabber::providesCallback () const
  {
    if (signals_.find (typeid (T).name ()) == signals_.end ())
      return (false);
    return (true);
  }

} // namespace

#endif
