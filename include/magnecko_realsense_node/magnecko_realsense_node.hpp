#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/header.hpp"

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "depth_metrics.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
class MagneckoRealsenseBroadcaster : public rclcpp::Node
{
  public:
    MagneckoRealsenseBroadcaster();
    ~MagneckoRealsenseBroadcaster();
  private:
    // using StatePublisher = realtime_tools::RealtimePublisher<magnecko_realsense_msgs::msg::DepthCameraRange>;
    using StatePublisherNormal = realtime_tools::RealtimePublisher<geometry_msgs::msg::Quaternion>;
    using StatePublisherPointCloud = realtime_tools::RealtimePublisher<sensor_msgs::msg::PointCloud2>;
    using RvizVectorPublisher = realtime_tools::RealtimePublisher<visualization_msgs::msg::Marker>;
    using ImagePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>;

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr normal_distance_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr normal_vector_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_rviz_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    
    std::unique_ptr<StatePublisherNormal> realtime_publisher_norm_dist_;
    std::unique_ptr<StatePublisherPointCloud> realtime_publisher_pointcloud_;
    std::unique_ptr<RvizVectorPublisher> realtime_publisher_rviz_points_;
    std::unique_ptr<RvizVectorPublisher> realtime_publisher_rviz_arrow_;
    std::unique_ptr<ImagePublisher> realtime_publisher_image_;

    std::vector<rs2::pipeline> pipelines_;
    rs2::context ctx_;
    rs2::pointcloud pointcloud_;
    rs2::pipeline_profile profile_;
    Eigen::Vector4f plane_parameters_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points);
    Eigen::Vector3f convert_depth_to_phys_coord(float x, float y, float depth_distance);

    // create time variables for the callbacks
    rclcpp::TimerBase::SharedPtr normal_vector_timer_;

    // callback functions
    void normal_vector_callback();
    void shut_down_sequence();
};