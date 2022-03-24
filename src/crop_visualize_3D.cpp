#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <robot_vision_lectures/XYZarray.h>
#include <robot_vision_lectures/SphereParams.h>
#include <geometry_msgs/Point.h>

#include <cmath>

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

robot_vision_lectures::SphereParams sphere_params;

void get_sphere_params(const robot_vision_lectures::SphereParams& _params){
	//read the parameters and save them
	sphere_params = _params;
	std::cout << sphere_params.xc << " " << sphere_params.yc << " " << sphere_params.zc << " "<< sphere_params.radius << std::endl;
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

robot_vision_lectures::XYZarray convert_pcl(pcl::PointCloud<pcl::PointXYZ>& _pcl){
	robot_vision_lectures::XYZarray xyz;
	// define a pointer to loop throught the pcl data
	pcl::PointCloud<pcl::PointXYZ>::iterator pi = _pcl.begin();

    	xyz.points.clear();
    	geometry_msgs::Point point; 
       // convert the pcl data to point and add them to the array
	for( ; pi != _pcl.end(); pi++ ) {
	
		if( 0 < fabs( pi->x ) && 0 < fabs( pi->y ) && 0 < fabs( pi->z ) ){
			point.x = pi->x;
			point.y = pi->y;
			point.z = pi->z;
			
			xyz.points.push_back(point); 
		}
	}
	
	return xyz;
}


pcl::PointCloud<pcl::PointXYZ> visualize_sphere(const robot_vision_lectures::SphereParams& _params){
	// define an empty point cloud
	pcl::PointCloud<pcl::PointXYZ> tmp_pcl;
	pcl::PointXYZ tmp_point;
	// using the spherical coordinates to produce xyz using the coordinates of the center and the radius
	// example of math: https://tutorial.math.lamar.edu/classes/calciii/SphericalCoords.aspx
	// break the theta and phi values to 20 chunks between 0 and 2*Pi
	int theta_segments = 20;
	int phi_segments = 20;
	for (int theta_ind = 0; theta_ind < theta_segments; theta_ind++){
		for (int phi_ind = 0; phi_ind < phi_segments; phi_ind++){
			// calculate current theta and phi on the grid
			float theta = (float) theta_ind/theta_segments*2*M_PI;
			float phi = (float) phi_ind/phi_segments*2*M_PI;
			// use the param values and update the points
			tmp_point.x = _params.xc + _params.radius*sin(phi)*cos(theta);
			tmp_point.y = _params.yc + _params.radius*sin(phi)*sin(theta);
			tmp_point.z = _params.zc + _params.radius*cos(phi); 
			tmp_pcl.push_back(tmp_point);
		}
		
	}
	return tmp_pcl;

}


pcl::PointCloud<pcl::PointXYZ> visualize_center(const robot_vision_lectures::SphereParams& _params){
	// define an empty point cloud
	pcl::PointCloud<pcl::PointXYZ> tmp_pcl;
	pcl::PointXYZ tmp_point;
	// use the param values and update the points
	tmp_point.x = _params.xc;
	tmp_point.y = _params.yc;
	tmp_point.z = _params.zc; 
	tmp_pcl.push_back(tmp_point);

	return tmp_pcl;

}


int main(int argc, char * argv[]){
	ros::init(argc,argv,"crop_3D_image");
	ros::NodeHandle nh_;
	// subscriber for reading the filtered 2D image
	ros::Subscriber cam_sub = nh_.subscribe("/ball_2D", 1, get_img);
	// subscriber for reading the registered 3D point cloud
	ros::Subscriber pcl_sub = nh_.subscribe("/camera/depth_registered/points",1,get_pcl);

	// subscriber for reading shpere fit parameters (x, y, z) of the centery and the radius
	ros::Subscriber sphere_sub = nh_.subscribe("/sphere_params",1, get_sphere_params);
		
	// publisher for the cropped pcl
	ros::Publisher pub_pcl= nh_.advertise<sensor_msgs::PointCloud2>( "/pcl_cropped_ball", 1 );
	
	// publisher for the ball fit pcl
	ros::Publisher pub_fit= nh_.advertise<sensor_msgs::PointCloud2>( "/ball_3D_fit", 1 );
	
	// publisher for writing the ball fit pcl
	ros::Publisher pub_center= nh_.advertise<sensor_msgs::PointCloud2>( "/ball_center", 1 );
	
	// publisher for writing the cropped xyz
	ros::Publisher pub_xyz= nh_.advertise<robot_vision_lectures::XYZarray>( "/xyz_cropped_ball", 1 );
	
	int loop_freq = 2;
	ros::Rate loop_rate(loop_freq);
	
	
	int seq = 0;
	
	robot_vision_lectures::XYZarray xyz;
	// define a pointcloud for showing the cropped pcl
	pcl::PointCloud<pcl::PointXYZ> cropped_pcl;	
	// define a pointcloud for showing the ball fit in 3D
	pcl::PointCloud<pcl::PointXYZ> ball_fit_pcl;	
	// define a pointcloud for showing the center of the ball fit
	pcl::PointCloud<pcl::PointXYZ> ball_center_pcl;

	while(ros::ok()){
		if (pcl_received && image_received){
			//clear the current pcl
			cropped_pcl.clear();
			// crop the pointcloud
			cropped_pcl = crop_pcl(raw_pcl, img);
			//clear the current pcl
			ball_fit_pcl.clear();
			// update the fit based on the parameters
			ball_fit_pcl = visualize_sphere(sphere_params);				
			//clear the current pcl
			ball_center_pcl.clear();
			// update the fit based on the parameters
			ball_center_pcl = visualize_center(sphere_params);				
		
			//update the message and publish it
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.seq = seq++;
			header.frame_id = std::string( "camera_color_optical_frame" );
			cropped_pcl.header = pcl_conversions::toPCL( header );
			pub_pcl.publish( cropped_pcl );
			ball_fit_pcl.header = pcl_conversions::toPCL( header );
			pub_fit.publish( ball_fit_pcl );
			ball_center_pcl.header = pcl_conversions::toPCL( header );
			pub_center.publish( ball_center_pcl );
			// update the xyz array and publish it
			xyz.points.clear();
			xyz = convert_pcl(cropped_pcl);
			pub_xyz.publish(xyz);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;	
}
