#include "magnecko_realsense_node/magnecko_realsense_node.hpp"

MagneckoRealsenseBroadcaster::MagneckoRealsenseBroadcaster() : Node("magnecko_realsense_broadcaster")
{
  // register on_shutdown() sequence

  // setting up intel realsense camera
  plane_parameters_ = {0, 0, 0, 0};
  
  // Capture serial numbers before opening streaming
  std::vector<std::string>              serials;
  for (auto&& dev : ctx_.query_devices())
      serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

  // Start a streaming pipe per each connected device
  for (auto&& serial : serials)
  {
      rs2::pipeline pipe(ctx_);
      rs2::config cfg;
      cfg.enable_device(serial);
      // cfg.enable_all_streams();
      try
      {
        profile_ = pipe.start(cfg);
      } catch (const rs2::error & e)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), e.what());
      }
      pipelines_.emplace_back(pipe);
  }

  pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud2", rclcpp::SystemDefaultsQoS());
  realtime_publisher_pointcloud_ = std::make_unique<StatePublisherPointCloud>(pointcloud_publisher_);
  
  // create publishers
  normal_distance_publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/magnecko_camera/normalAndDistance", rclcpp::SystemDefaultsQoS());
  realtime_publisher_norm_dist_ = std::make_unique<StatePublisherNormal>(normal_distance_publisher_);

  points_rviz_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("~/pointsRviz", rclcpp::SystemDefaultsQoS());
  realtime_publisher_rviz_points_ = std::make_unique<RvizVectorPublisher>(points_rviz_publisher_);

  normal_vector_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("~/normalVectorRviz", rclcpp::SystemDefaultsQoS());
  realtime_publisher_rviz_arrow_ = std::make_unique<RvizVectorPublisher>(normal_vector_publisher_);

  image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image", rclcpp::SystemDefaultsQoS());
  realtime_publisher_image_ = std::make_unique<ImagePublisher>(image_publisher_);

  // create wall timers
  normal_vector_timer_ = this->create_wall_timer(100ms, std::bind(&MagneckoRealsenseBroadcaster::normal_vector_callback, this));

  realtime_publisher_pointcloud_->lock();
  realtime_publisher_norm_dist_->lock();
  realtime_publisher_rviz_points_->lock();
  realtime_publisher_rviz_arrow_->lock();
  realtime_publisher_image_->lock();
  
  realtime_publisher_pointcloud_->msg_.header.frame_id = "depth_camera";
  realtime_publisher_rviz_arrow_->msg_.points.resize(2);

  realtime_publisher_pointcloud_->unlock();
  realtime_publisher_norm_dist_->unlock();
  realtime_publisher_rviz_points_->unlock();
  realtime_publisher_rviz_arrow_->unlock();
  realtime_publisher_image_->unlock();

  RCLCPP_INFO(this->get_logger(), "Realsense is activated!");

  // test if we get frames
  rs2::frameset frames;
  bool gets_frames = false;
  while (!gets_frames && rclcpp::ok())
  {
    try
    {
      frames = pipelines_.at(0).wait_for_frames(5000U);
      gets_frames = true;
    }
    catch(const rs2::error & e)
    {
      RCLCPP_WARN(this->get_logger(), e.what());
      try
      {
        pipelines_.at(0).stop();
      } catch (rs2::error & e)
      {
        RCLCPP_ERROR(this->get_logger(), e.what());
      }
      RCLCPP_INFO(this->get_logger(), "Please unplug and then plug in camera. You have 10 seconds!");
      rclcpp::sleep_for(10000ms);
      try 
      {
        pipelines_.at(0).start();
      } catch (rs2::error & error)
      {
        RCLCPP_ERROR(this->get_logger(), e.what());
      }
    }
  }

  if (gets_frames)
    RCLCPP_INFO(this->get_logger(), "We receive frames!!!"); 
}

MagneckoRealsenseBroadcaster::~MagneckoRealsenseBroadcaster()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Class destructed!");
}

void MagneckoRealsenseBroadcaster::shut_down_sequence()
{
  for (auto pipe : pipelines_)
  {
    pipe.stop();
  }
  std::cout << "stopped pipeline" << std::endl;
}

void MagneckoRealsenseBroadcaster::normal_vector_callback()
{
  rs2::depth_quality::plane plane;
  std::vector<rs2::depth_quality::float3> points_on_plane;
  if (realtime_publisher_norm_dist_ && realtime_publisher_norm_dist_->trylock())
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      
      // Getting pointcloud from intel realsense
      rs2::frameset frames = pipelines_.at(0).wait_for_frames();
      
      rs2::depth_frame depth = frames.get_depth_frame();
      rs2::video_frame color = frames.get_color_frame();

      pointcloud_.map_to(color);
      rs2::points points = pointcloud_.calculate(depth);

      auto pcl_points = MagneckoRealsenseBroadcaster::points_to_pcl(points);
      
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(pcl_points);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.0, 1.0);
      pass.filter(*cloud_filtered);

      std::vector<rs2::vertex> vertices(cloud_filtered->points.size());
      for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
      {
          vertices[i].x = cloud_filtered->points[i].x;
          vertices[i].y = cloud_filtered->points[i].y;
          vertices[i].z = cloud_filtered->points[i].z;
      }

      rs2::depth_quality::float3 data_point;
      Eigen::Vector3f middle_point = MagneckoRealsenseBroadcaster::convert_depth_to_phys_coord(depth.get_width()/2, depth.get_height()/2, depth.get_distance(depth.get_width()/2, depth.get_height()/2));
      data_point.x = middle_point(0);
      data_point.y = middle_point(1);
      data_point.z = middle_point(2);
      points_on_plane.push_back(data_point);
      double radius = 0.05; // search radius of 5 cm
      for (size_t point_idx = 0; point_idx < vertices.size(); ++point_idx)
      {
          double condition = (vertices[point_idx].x - middle_point(0))*(vertices[point_idx].x - middle_point(0)) + (vertices[point_idx].y - middle_point(1))*(vertices[point_idx].y - middle_point(1));
          if (condition <= radius*radius)
          {
              if (std::abs(vertices[point_idx].z - middle_point(2)) < 0.05)
              {
                  rs2::depth_quality::float3 data_point;
                  data_point.x = vertices[point_idx].x;
                  data_point.y = vertices[point_idx].y;
                  data_point.z = vertices[point_idx].z;
                  points_on_plane.push_back(data_point);
              }
              
          }
      }
      plane = rs2::depth_quality::plane_from_points(points_on_plane);
          
      realtime_publisher_norm_dist_->msg_.w = depth.get_distance(depth.get_width()/2, depth.get_height()/2);
      realtime_publisher_norm_dist_->msg_.x = plane.a;
      realtime_publisher_norm_dist_->msg_.y = plane.b;
      realtime_publisher_norm_dist_->msg_.z = plane.c;

      realtime_publisher_norm_dist_->unlockAndPublish();
  }

  if (realtime_publisher_rviz_points_ && realtime_publisher_rviz_points_->trylock())
  {
      realtime_publisher_rviz_points_->msg_.header.frame_id = "depth_camera";
      realtime_publisher_rviz_points_->msg_.header.stamp = this->now();
      realtime_publisher_rviz_points_->msg_.ns = "normal_vector";
      realtime_publisher_rviz_points_->msg_.id = 0;
      realtime_publisher_rviz_points_->msg_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      realtime_publisher_rviz_points_->msg_.action = visualization_msgs::msg::Marker::ADD;
      realtime_publisher_rviz_points_->msg_.points.resize(points_on_plane.size());
      for (size_t i = 0; i < points_on_plane.size(); ++i)
      {
          geometry_msgs::msg::Point p;
          p.x = points_on_plane.at(i).x;
          p.y = points_on_plane.at(i).y;
          p.z = points_on_plane.at(i).z;
          realtime_publisher_rviz_points_->msg_.points[i] = p;
      }
      realtime_publisher_rviz_points_->msg_.scale.x = 0.1;
      realtime_publisher_rviz_points_->msg_.scale.y = 0.1;
      realtime_publisher_rviz_points_->msg_.scale.z = 0.1;
      realtime_publisher_rviz_points_->msg_.color.a = 1.0; // Don't forget to set the alpha!
      realtime_publisher_rviz_points_->msg_.color.r = 0.0;
      realtime_publisher_rviz_points_->msg_.color.g = 1.0;
      realtime_publisher_rviz_points_->msg_.color.b = 0.0;

      realtime_publisher_rviz_points_->unlockAndPublish();
  }

  if (realtime_publisher_pointcloud_ && realtime_publisher_pointcloud_->trylock())
  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      
      // Getting pointcloud from intel realsense
      rs2::frameset frames = pipelines_.at(0).wait_for_frames();
      rs2::depth_frame depth = frames.get_depth_frame();
      rs2::video_frame color = frames.get_color_frame();

      pointcloud_.map_to(color);
      rs2::points points = pointcloud_.calculate(depth);

      auto pcl_points = MagneckoRealsenseBroadcaster::points_to_pcl(points);
      
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(pcl_points);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.0, 5.0);
      pass.filter(*cloud_filtered);

      pcl::toROSMsg(*cloud_filtered, realtime_publisher_pointcloud_->msg_);
      realtime_publisher_pointcloud_->msg_.header.frame_id = "depth_camera";
      realtime_publisher_pointcloud_->unlockAndPublish();
  }

  if (realtime_publisher_rviz_arrow_ && realtime_publisher_rviz_arrow_->trylock())
  {
      realtime_publisher_rviz_arrow_->msg_.header.frame_id = "depth_camera";
      realtime_publisher_rviz_arrow_->msg_.header.stamp = this->now();
      realtime_publisher_rviz_arrow_->msg_.ns = "normal_vector";
      realtime_publisher_rviz_arrow_->msg_.id = 0;
      realtime_publisher_rviz_arrow_->msg_.type = visualization_msgs::msg::Marker::ARROW;
      realtime_publisher_rviz_arrow_->msg_.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point start_point;
      start_point.x = 0;
      start_point.y = 0;
      start_point.z = 0;
      realtime_publisher_rviz_arrow_->msg_.points[0] = start_point;
      geometry_msgs::msg::Point end_point;
      end_point.x = plane.a;
      end_point.y = plane.b;
      end_point.z = plane.c;
      realtime_publisher_rviz_arrow_->msg_.points[1] = end_point;
      realtime_publisher_rviz_arrow_->msg_.scale.x = 0.01;
      realtime_publisher_rviz_arrow_->msg_.scale.y = 0.03;
      realtime_publisher_rviz_arrow_->msg_.scale.z = 0.01;
      realtime_publisher_rviz_arrow_->msg_.color.a = 1.0; // Don't forget to set the alpha!
      realtime_publisher_rviz_arrow_->msg_.color.r = 0.0;
      realtime_publisher_rviz_arrow_->msg_.color.g = 0.0;
      realtime_publisher_rviz_arrow_->msg_.color.b = 1.0;

      realtime_publisher_rviz_arrow_->unlockAndPublish();
  }

  if (realtime_publisher_image_ && realtime_publisher_image_->trylock())
  {
      rs2::frameset data = pipelines_.at(0).wait_for_frames();
      rs2::frame color_frame = data.get_color_frame();
      rs2::depth_frame depth = data.get_depth_frame();

      // Query frame size (width and height)
      const int w = color_frame.as<rs2::video_frame>().get_width();
      const int h = color_frame.as<rs2::video_frame>().get_height();

      // Creating OpenCV Matrix from a color image
      cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

      // convert original image to BGR, since Lab is only available from BGR
      cv::Mat output_bgr;
      cv::cvtColor(color, output_bgr, cv::COLOR_BGRA2BGR);

      // First blur to reduce noise prior to color space conversion
      cv::medianBlur(output_bgr, output_bgr, 3);

      // convert to lab color space, we only need to check one channel (a-channel) for red here
      cv::Mat output_lab;
      cv::cvtColor(output_bgr, output_lab, cv::COLOR_BGR2HSV);

      // Threshold the Lab image, keep only the red pixels
      // Possible yellow threshold: [20, 110, 170][255, 140, 215]
      // Possible blue threshold: [20, 115, 70][255, 145, 120]
      cv::Mat output_lab_red;
      // RCLCPP_INFO_STREAM(get_node()->get_logger(), output_lab.type());
      // RESULT: CV_8UC3
      uint8_t low_data[3] = {20, 150, 150};
      uint8_t up_data[3] = {190, 255, 255};
      cv::Mat lower_bound = cv::Mat(1, 3, CV_8UC3, low_data);
      cv::Mat upper_bound = cv::Mat(1, 3, CV_8UC3, up_data);
      cv::inRange(output_lab, cv::Scalar(20, 150, 150), cv::Scalar(190, 255, 255), output_lab_red);

      // Second blur to reduce more noise, easier circle detection
      cv::GaussianBlur(output_lab_red, output_lab_red, cv::Size(5, 5), 2, 2);

      // Use the Hough transform to detect red circles in the image
      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(output_lab_red, circles, CV_HOUGH_GRADIENT, 1, output_lab_red.rows/8, 100, 18, 5, 60);

      // determine position of red circles and draw them into the image
      std::vector<Eigen::Vector3f> dot_positions;
      if (circles.size() > 0)
      {
          for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
          {
              int x_pixel = std::round(circles[current_circle][0]);
              int y_pixel = std::round(circles[current_circle][1]);
              float distance = depth.get_distance(x_pixel, y_pixel);

              Eigen::Vector3f position_red_circle = MagneckoRealsenseBroadcaster::convert_depth_to_phys_coord(x_pixel, y_pixel, distance);
              dot_positions.push_back(position_red_circle);
              RCLCPP_INFO_STREAM(this->get_logger(), position_red_circle);
              
              cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
              int radius = std::round(circles[current_circle][2]);

              cv::circle(color, center, radius, cv::Scalar(0, 255, 0), 5);
          }
      }

      // qr detection
      // auto depth_stream = profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
      // auto intrinsics = depth_stream.get_intrinsics();

      // auto qr = cv::QRCodeDetector();
      // cv::Mat points;
      // bool qr_detected = qr.detect(output_bgr, points);

      // float qr_edges_list[12] = {0,0,0, 0,1,0, 1,1,0, 1,0,0};
      // int qr_shape[3] = {4, 1, 3};
      // cv::Mat qr_edges = cv::Mat(3, qr_shape, CV_32F, qr_edges_list);
      
      // float cmatrix_data[9] = {intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1};
      // cv::Mat cmatrix = cv::Mat(3, 3, CV_32F, cmatrix_data);

      // float distortion_data[5] = {intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]};
      // cv::Mat distortion = cv::Mat(1, 5, CV_32F, distortion_data);

      // cv::Mat rotation_vector;
      // cv::Mat translation_vector;
      // bool returned_orientation = cv::solvePnP(qr_edges, points, cmatrix, distortion, rotation_vector, translation_vector);
      // // if (points.rows > 0 && qr_detected)
      // //     RCLCPP_INFO_STREAM(get_node()->get_logger(), points.at<float>(0, 0) << " " << points.at<float>(0, 1));
      // if (points.rows > 0 && qr_detected)
      // {
      //     for (size_t current_point = 0; current_point < points.rows; ++current_point)
      //     {
      //         int x_pixel = std::round(points.at<float>(current_point, 0));
      //         int y_pixel = std::round(points.at<float>(current_point, 1));
      //         float distance = depth.get_distance(x_pixel, y_pixel);

      //         Eigen::Vector3f position_red_circle = MagneckoRealsenseBroadcaster::convert_depth_to_phys_coord(x_pixel, y_pixel, distance);
      //         RCLCPP_INFO_STREAM(get_node()->get_logger(), position_red_circle);
              
      //         cv::Point center(std::round(points.at<float>(current_point, 0)), std::round(points.at<float>(current_point, 1)));
      //         int radius = 1;

      //         cv::circle(color, center, radius, cv::Scalar(0, 255, 0), 5);
      //     }
      // }
      cv_bridge::CvImage img_bridge;
      std_msgs::msg::Header header;
      sensor_msgs::msg::Image image;
      header.stamp = this->now();
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, color);
      img_bridge.toImageMsg(realtime_publisher_image_->msg_);

      realtime_publisher_image_->unlockAndPublish();
  }
}

Eigen::Vector3f MagneckoRealsenseBroadcaster::convert_depth_to_phys_coord(float x, float y, float depth_distance)
{
    rs2_intrinsics intrinsics = profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    float point[3];
    float pixel[2];
    pixel[0] = x;
    pixel[1] = y;
    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_distance);
    Eigen::Vector3f result;
    result(0) = point[0];
    result(1) = point[1];
    result(2) = point[2];
    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MagneckoRealsenseBroadcaster::points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagneckoRealsenseBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
