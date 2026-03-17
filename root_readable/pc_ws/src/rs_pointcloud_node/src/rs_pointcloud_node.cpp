#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <librealsense2/rs.hpp>

class RSPointCloudNode : public rclcpp::Node
{
public:
  RSPointCloudNode() : Node("rs_pointcloud_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/points", 10);

    // RealSense 파이프라인 설정
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 15);

    pipe_.start(cfg);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RSPointCloudNode::publish_pointcloud, this)
    );

    RCLCPP_INFO(this->get_logger(), "RS PointCloud node started");
  }

private:
  void publish_pointcloud()
  {
    rs2::frameset frames;
    if (!pipe_.poll_for_frames(&frames))
      return;

    auto depth = frames.get_depth_frame();
    auto color = frames.get_color_frame();

    rs2::pointcloud pc;
    pc.map_to(color);
    rs2::points points = pc.calculate(depth);

    const rs2::vertex* verts = points.get_vertices();
    const rs2::texture_coordinate* tex = points.get_texture_coordinates();
    size_t point_count = points.size();

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "camera_link";
    msg.header.stamp = this->now();

    msg.height = 1;
    msg.width = point_count;
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(point_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

    const uint8_t* color_data =
      reinterpret_cast<const uint8_t*>(color.get_data());

    int w = color.get_width();
    int h = color.get_height();

    for (size_t i = 0; i < point_count; ++i)
    {
      if (verts[i].z <= 0) continue;

      *iter_x = verts[i].x;
      *iter_y = verts[i].y;
      *iter_z = verts[i].z;

      int u = static_cast<int>(tex[i].u * w);
      int v = static_cast<int>(tex[i].v * h);

      int idx = (v * w + u) * 3;

      *iter_r = color_data[idx];
      *iter_g = color_data[idx + 1];
      *iter_b = color_data[idx + 2];

      ++iter_x; ++iter_y; ++iter_z;
      ++iter_r; ++iter_g; ++iter_b;
    }

    publisher_->publish(msg);
  }

  rs2::pipeline pipe_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RSPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
