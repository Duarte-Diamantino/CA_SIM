#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <cmath>

// Parameters
static constexpr double MAX_RAW_SPEED_CMD = 23070.0;    // raw velocity units
static constexpr double V_MAX             = 3.0;        // m/s
static constexpr double PRED_DT           = 0.1;        // simulation step (s)
static constexpr double PRED_TIME         = 4.0;        // horizon (s)
static constexpr int    NV_SAMPLES        = 8;          // number of linear velocity samples
static constexpr int    NW0_SAMPLES       = 6;          // number of initial ω samples
static constexpr int    NA_SAMPLES        = 6;          // number of angular acceleration α samples

class SpiralTrajectoryPublisher : public rclcpp::Node
{
public:
  SpiralTrajectoryPublisher()
  : Node("spiral_trajectory_publisher")
  {
    // Subscribe to odometry
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/vesc/odom", 10,
      std::bind(&SpiralTrajectoryPublisher::odomCallback, this, std::placeholders::_1)
    );
    // Publisher for spiral trajectories
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/predicted_spirals", 10
    );
    // Timer to periodically generate spirals
    timer_ = create_wall_timer(
      std::chrono::milliseconds(int(PRED_DT*1000)),
      std::bind(&SpiralTrajectoryPublisher::publishSpirals, this)
    );
  }

private:
  // Robot pose
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  bool have_odom_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    auto &q = msg->pose.pose.orientation;
    yaw_ = std::atan2(
      2*(q.w*q.z + q.x*q.y),
      1 - 2*(q.y*q.y + q.z*q.z)
    );
    have_odom_ = true;
  }

  void publishSpirals() {
    if (!have_odom_) return;

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(NV_SAMPLES * NW0_SAMPLES * NA_SAMPLES);
    auto now = this->now();
    const std::string frame = "vesc/odom";
    int id = 0;

    // Sample linear velocity, initial angular velocity, angular acceleration
    for (int iv = 0; iv < NV_SAMPLES; ++iv) {
      double v = V_MAX * iv / (NV_SAMPLES - 1);
      for (int iw0 = 0; iw0 < NW0_SAMPLES; ++iw0) {
        double w0 = -M_PI/6 + 2*(M_PI/6)*iw0/(NW0_SAMPLES-1);
        for (int ia = 0; ia < NA_SAMPLES; ++ia) {
          double alpha = -0.5 + ia*(1.0/(NA_SAMPLES-1)); // rad/s^2

          // Simulate spiral trajectory
          double sx = x_, sy = y_, syaw = yaw_, w = w0;
          int steps = int(PRED_TIME / PRED_DT);

          visualization_msgs::msg::Marker m;
          m.header.frame_id = frame;
          m.header.stamp = now;
          m.ns = "spiral";
          m.id = id++;
          m.type = visualization_msgs::msg::Marker::LINE_STRIP;
          m.action = visualization_msgs::msg::Marker::ADD;
          m.scale.x = 0.005;  // line thickness
          m.color.r = iv / double(NV_SAMPLES - 1);
          m.color.g = iw0 / double(NW0_SAMPLES - 1);
          m.color.b = ia / double(NA_SAMPLES - 1);
          m.color.a = 1.0;

          geometry_msgs::msg::Point p;
          for (int s = 0; s <= steps; ++s) {
            p.x = sx;
            p.y = sy;
            p.z = 0.0;
            m.points.push_back(p);
            // integrate dynamics
            sx += v * std::cos(syaw) * PRED_DT;
            sy += v * std::sin(syaw) * PRED_DT;
            syaw += w * PRED_DT;
            w += alpha * PRED_DT;
          }
          ma.markers.push_back(m);
        }
      }
    }

    marker_pub_->publish(ma);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpiralTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
