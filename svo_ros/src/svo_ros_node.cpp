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
#define COL 752
using namespace std;
std::string IMAGE_TOPIC = "/cam0/image_raw";

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
public:
    BenchmarkNode();
    ~BenchmarkNode();
    void addImage(const cv::Mat& _image, double timestamp);
    vk::AbstractCamera* cam_;
    svo::FrameHandlerMono* vo_;
    svo::Visualizer visualizer_;
};

BenchmarkNode::BenchmarkNode()
{
    visualizer_.T_world_from_vision_ = Sophus::SE3(
    vk::rpy2dcm(Eigen::Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                                vk::getParam<double>("svo/init_ry", 0.0),
                                vk::getParam<double>("svo/init_rz", 0.0))),
    Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                    vk::getParam<double>("svo/init_ty", 0.0),
                    vk::getParam<double>("svo/init_tz", 0.0)));
    cam_ = new vk::PinholeCamera(752, 480, 461.6, 460.3, 363.0, 248.1, 
        0.2917, 0.08228, 5.333e-05, -1.578e-04, 0.f);
    vo_ = new svo::FrameHandlerMono(cam_);
    vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
    delete vo_;
    delete cam_;
}

bool publish_markers_ = true;
bool publish_dense_input_ = true;

void BenchmarkNode::addImage(const cv::Mat& _image, double timestamp)
{
    if(!_image.empty() && !first_image_flag){
        //load image.
#if 0        
        cv::Mat image;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(_image, image);
#endif        
        vo_->addImage(_image, timestamp);

        if(vo_->lastFrame() != NULL){
/*            std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                      << "#Features: " << vo_->lastNumObservations() << " \t"
                      << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";*/
            // access the pose of the camera via vo_->lastFrame()->T_f_w_.
        }
        visualizer_.publishMinimal(image, vo_->lastFrame(), *vo_, timestamp);

#if 0        
        if(publish_markers_ && vo_->stage() != svo::FrameHandlerBase::STAGE_PAUSED)
            visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

        if(publish_dense_input_)
            visualizer_.exportToDense(vo_->lastFrame());
#endif    
        }
}

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

    
    static BenchmarkNode* svo_ = new BenchmarkNode();
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
    ros::init(argc, argv, "svo");
    ros::NodeHandle n;


    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    //readParameters(n);

    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

/*    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);*/
    ros::spin();
    return 0;
}
