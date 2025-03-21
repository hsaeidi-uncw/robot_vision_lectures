#include<ros/ros.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;


// raw input image
Mat img;
// pointer for reading ros images via cv brdige
CvImagePtr cv_ptr;


// initialization check to see if the first image is received or not
bool initialized = false;

// max x and y coordinates take from the camera
double width, height;


// callback function for the input image
void get_img(const sensor_msgs::Image &  _data){
	// read the raw rgb input image from the camera
	cv_ptr = toCvCopy(_data,"rgb8");
	// take the image part and put it in a mat variable in opencv format
	img = cv_ptr->image;
	// get the width and height
	width = _data.width;
	height = _data.height;
	// raise the flag
	initialized = true;
}

int main(int argc, char * argv[]){
	ros::init(argc,argv,"flip_image_example");
	ros::NodeHandle nh;
	
	// subscribe to the raw rgb input 
	ros::Subscriber cam_sub = nh.subscribe("/camera/color/image_raw",2,get_img);
	
	//publisher for checking the images and debugging them
	ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/flipped_image",1);
	
	// set the loop frequency equal to the camera input
	int loop_freq = 30;
	ros::Rate loop_rate(loop_freq);
	
	// define a variable for the modifed image
	Mat modified_img;
	// pointer for reading ros images via cv brdige
	CvImagePtr modified_cv_ptr;
	while(ros::ok()){
		// if an image is received from the camera process it 
		if(initialized){
			cout << "Image received!\n";
			flip(img, modified_img, 0); // an example of modifying an image
			modified_cv_ptr = cv_ptr; // copy the pointer settings
            modified_cv_ptr->image = modified_img;// update the image
            modified_cv_ptr->encoding = "rgb8"; // update the encoding if needed
			image_pub.publish(modified_cv_ptr->toImageMsg()); // publish the message
		}
		ros::spinOnce();
		loop_rate.sleep();
	}	
    return 0;
}

