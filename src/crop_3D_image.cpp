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
	// raise the flag
	pcl_received = true;
}

pcl::PointCloud<pcl::PointXYZ> crop_pcl(pcl::PointCloud<pcl::PointXYZ>& _raw_pcl, cv::Mat& filtered_2D_img){
	// define an empty point cloud
	pcl::PointCloud<pcl::PointXYZ> tmp_pcl;
	// a 3D point for filtering the ball image
	pcl::PointXYZ point_buffer_usedInAtMethod; 
	// loop through the rows and columns
	for (int row = 0; row < filtered_2D_img.rows; row++){
		for (int col = 0; col < filtered_2D_img.cols; col++){
			// if there is a pixel on the ball (from the ball_2D topic add the corresponding point to the point cloud)
			if (filtered_2D_img.at<uchar>(row,col) == 255){
				point_buffer_usedInAtMethod = _raw_pcl.at(col , row);
				tmp_pcl.push_back(point_buffer_usedInAtMethod);
			}
		
		}
		
	}
	return tmp_pcl;

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
		if (pcl_received && image_received){
			pcl::PointCloud<pcl::PointXYZ> cropped_pcl;
			// crop the pointcloud
			cropped_pcl = crop_pcl(raw_pcl, img);
			//update the message and publish it
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.seq = seq++;
			header.frame_id = std::string( "camera_color_optical_frame" );
			cropped_pcl.header = pcl_conversions::toPCL( header );
			pub_pcl.publish( cropped_pcl );
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;	
}
