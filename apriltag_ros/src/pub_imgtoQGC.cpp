#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::string udp_host;
  std::string pipeline;
  cv::VideoWriter writer;
  int img_width;  
  int img_height;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/tag_detections_image", 1, 
      &ImageConverter::imageCb, this);

    nh_.getParam("/imgtoqgc/udp_host",udp_host);
    nh_.getParam("/imgtoqgc/img_width",img_width);
    nh_.getParam("/imgtoqgc/img_height",img_height);
    std::cout << "udp_host: " << udp_host << std::endl;
    pipeline = "appsrc ! videoconvert ! x264enc noise-reduction=10000 tune=zerolatency byte-stream=true bitrate=1000 threads=4 ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=" + udp_host + " port=5600";
    writer.open(pipeline, 0, (double)30, cv::Size(img_width, img_height), true);

    cv::namedWindow("Image reuslt");
  }

  ~ImageConverter()
  {
    cv::destroyWindow("Image reuslt");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_WARN("cv_bridge exception: %s", e.what());
      return;
    }
    // try
    // {
    //   writer.write(cv_ptr->image);
    // }
    // catch(const std::exception& e)
    // {
    //   ROS_WARN("cv_bridge exception: %s" , e.what());
    // }
    
    // Update GUI Window
    writer.write(cv_ptr->image);
    cv::imshow("Image reuslt", cv_ptr->image);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imgtoqgc");
  ImageConverter ic;
  ros::spin();
  return 0;
}