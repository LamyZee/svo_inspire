#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/file_reader.h>
#include <vikit/params_helper.h>
#include <vikit/camera_loader.h>
#include <vikit/abstract_camera.h>
#include <vikit/blender_utils.h>
#include <vikit/sample.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
//#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>

#define ROW 480
#define COL 640
using namespace std;
std::string IMAGE_TOPIC;

std::mutex i_buf;
vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;


class BenchmarkNode
{
  vk::AbstractCamera* cam_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void addImage(const cv::Mat& image, double timestamp);
  svo::FrameHandlerMono* vo_;
};

BenchmarkNode::BenchmarkNode()
{
    cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
    vo_ = new svo::FrameHandlerMono(cam_);
    vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
    delete vo_;
    delete cam_;
}

void BenchmarkNode::addImage(const cv::Mat& image, double timestamp)
{
    if(!image.empty() && !first_image_flag){
        //load image.
        vo_->addImage(image, timestamp);

        if(vo_->lastFrame() != NULL){
/*            std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                      << "#Features: " << vo_->lastNumObservations() << " \t"
                      << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";*/
            // access the pose of the camera via vo_->lastFrame()->T_f_w_.
        }
    }

}


BenchmarkNode* svo_ = new BenchmarkNode();

void process(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(first_image_flag){
        first_image_flag = false;
        return;
    }

    last_image_time = img_msg->header.stamp.toSec();

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    const cv::Mat image_ = ptr->image.rowRange(0, ROW);

    svo_->addImage(image_, last_image_time);

/*    ROS_INFO("Frame-Id: %d ", svo_->vo_->lastFrame()->id_ );
    ROS_INFO("#Features: %d ", svo_->vo_->lastNumObservations());
    ROS_INFO("Proc. Time: %f ms \n", svo_->vo_->lastProcessingTime()*1000);
    std::cout << "Frame-Id: " << svo_->vo_->lastFrame()->id_ << " \t"
              << "#Features: " << svo_->vo_->lastNumObservations() << " \t"
              << "Proc. Time: " << svo_->vo_->lastProcessingTime()*1000 << "ms \n";*/
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg){
    i_buf.lock();
    //img_buf.push(img_msg);
    process(img_msg);
    i_buf.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    //readParameters(n);

    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

/*    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);*/
    ros::spin();
    return 0;
}
