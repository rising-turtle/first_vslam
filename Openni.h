#ifndef OPENNI_H
#define OPENNI_H
//#include "pre_header.h"
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/surface/gp3.h>
//#include <pcl/features/normal_3d.h>
//#include <Windows.h>
#include <boost/thread/thread.hpp>
#include <fstream>

using namespace std;

//#include "Viewer.h"

//#define sleep(x) Sleep((x)*1000)

//static pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr g_cloud; // used to obtain cloud from Kinect

class SimpleOpenNIViewer
{
public:

	SimpleOpenNIViewer ():new_cloud(false),_interface(new pcl::OpenNIGrabber()){
		// just for log_debug
		//logfile.open("D:\\Openni.log",ios::app);
	}
	~SimpleOpenNIViewer(){
		if(_interface)
			_interface->stop();
		if(logfile.is_open())
			logfile.close();
	}

	// used to construct XYZRGB from image and DepthImage
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToXYZRGBPointCloud(const boost::shared_ptr<openni_wrapper::Image> &image,
		const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image, float constant);

	// callback function used for VisualSlam and HybridSlam
	void kinect_frame_cb(const boost::shared_ptr<openni_wrapper::Image>& image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float constant){
		mutex_.lock();
			g_cloud = convertToXYZRGBPointCloud(image,depth_image,constant);
			g_image = image;
			g_depthimage = depth_image;
			new_cloud = true;
		mutex_.unlock();
	}

void startframe(){
		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>& , const boost::shared_ptr<openni_wrapper::DepthImage>& , float )> f =
			boost::bind (&SimpleOpenNIViewer::kinect_frame_cb,this,_1,_2,_3);
		_interface->registerCallback(f);
		_interface->start();
	}

bool getframe(boost::shared_ptr<openni_wrapper::Image> &image, boost::shared_ptr<openni_wrapper::DepthImage> &depth_image,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudpoint
		)
		{
		bool ret;
		mutex_.lock();
		if(!new_cloud){ // no new cloud point
			ret = false;	
			mutex_.unlock();
		}
		else{ // there is new cloud
			cloudpoint = g_cloud;

			if(g_depthimage->getTimeStamp() != g_image->getTimeStamp())
				cout<<"not the same"<<endl;


			image=g_image;
			depth_image = g_depthimage;
			ret = true;
			new_cloud = false;
			mutex_.unlock();
		}
		return ret;
	}	
	// callback function used for FPFHSlam obtain only XYZRGB without image
	void cloud_cb_points(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
		mutex_.lock ();
		// If this frame is valid
		/*if(isvalidframe(cloud))
		{}*/

	/*	static int count = 0;
		if(count == 0){
			start_t = ::GetTickCount();
		}
		end_t = ::GetTickCount();
		if(count != 0)
			logfile<<count<<"th grabber cost :"<<end_t - start_t<<endl;*/
	
		g_cloud =boost::const_pointer_cast<pcl::PointCloud<pcl::PointXYZRGB> >(cloud);
		new_cloud = true;
	/*	start_t = ::GetTickCount();
		count ++;*/

		//if(count == 60)  _interface->stop();
		mutex_.unlock ();
	}

	bool getCloudPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& g_cloudpoint){
		bool ret ;
		mutex_.lock();
		if(!new_cloud){ // no new cloud point
			ret = false;	
			mutex_.unlock();
		}
		else{ // there is new cloud
			g_cloudpoint = g_cloud;
			ret = true;
			new_cloud = false;
			mutex_.unlock();
		}
		return ret;
	}

	//start grab frames from Kinect
	void start ()
	{
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_points, this, _1);

		_interface->registerCallback (f);

		_interface->start ();

	//	interface->stop ();
	}
	
	//grabber object 
	pcl::Grabber* _interface;

	//thread synchronization
	boost::mutex mutex_;
	bool new_cloud ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud;
	boost::shared_ptr<openni_wrapper::Image> g_image;
    boost::shared_ptr<openni_wrapper::DepthImage> g_depthimage;

	// for format transformation 
	/*boost::shared_ptr<xn::ImageMetaData> imageMetaData;
	boost::shared_ptr<xn::DepthMetaData> depthMetaData;*/

	// just for Log_debug 
	ofstream logfile;
	// record time_span for openni
	double start_t;
	double end_t;
};

//simpleopenni v;
//v.start;
//
//if(v.getframe())


#endif