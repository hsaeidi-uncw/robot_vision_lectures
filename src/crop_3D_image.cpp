#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace cv_bridge;


// raw input image
Mat img;
// pointer for reading ros images via cv brdige
CvImagePtr cv_ptr;


// initialization check to see if the first image is received or not
bool image_received = false;

bool pcl_received = false;

// max x and y coordinates take from the camera
int width, height;

// callback function for the input image
void get_img(const sensor_msgs::Image &  _data){
	// read the rectified rgb input image from the camera
	cv_ptr = toCvCopy(_data,"mono8");
	// take the image part and put it in a mat variable in opencv format
	img = cv_ptr->image;
	// get the width and height
	width = _data.width;
	height = _data.height;
	
	image_received = true;
}


pcl::PointCloud<pcl::PointXYZ> raw_pcl;

void get_pcl(const sensor_msgs::PointCloud2Ptr& cloud){

	// convert the ROS message to PCL
	pcl::fromROSMsg(*cloud, raw_pcl);

	pcl_received = true;
}



int main(int argc, char * argv[]){
	ros::init(argc,argv,"crop_3D_image");
	ros::NodeHandle nh_;
	// subscriber for reading the filtered 2D image
	ros::Subscriber cam_sub = nh_.subscribe("/ball_2D", 1, get_img);
	// subscriber for reading the registered 3D point cloud
	ros::Subscriber pcl_sub = nh_.subscribe("/camera/depth_registered/points",1,get_pcl);
	
	// publisher for writing the cropped pcl
	ros::Publisher pub_pcl= nh_.advertise<sensor_msgs::PointCloud2>( "/cropped_3D_ball", 10 );
	int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);
	
	int seq = 0;
	while(ros::ok()){
		if (pcl_received){
			pcl::PointCloud<pcl::PointXYZ> cropped_pcl;

			cropped_pcl = raw_pcl;
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.seq = seq++;
			header.frame_id = std::string( "camera_depth_optical_frame" );
		
			cropped_pcl.header = pcl_conversions::toPCL( header );
			pub_pcl.publish( cropped_pcl );
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;	
}
