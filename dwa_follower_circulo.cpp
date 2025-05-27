#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <cmath>

// Parameters
static constexpr double V_MAX             = 3.0;         // m/s
static constexpr double MAX_RAW_SPEED_CMD = 23070.0;    // raw velocity units
static constexpr double W_MAX_RAD         = M_PI/6.0;    // ±30°/s = π/6 rad/s
static constexpr double PRED_DT           = 0.1;        // simulation step (s)
static constexpr double PRED_TIME         = 2.0;        // prediction horizon (s)
static constexpr int    NV_SAMPLES        = 5;          // number of v samples
static constexpr int    NW_SAMPLES        = 7;          // number of ω samples

class DwaTrajectoryDrawer : public rclcpp::Node
{
public:
  DwaTrajectoryDrawer()
  : Node("dwa_trajectory_drawer")
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/vesc/odom", 10,
      std::bind(&DwaTrajectoryDrawer::odomCallback, this, std::placeholders::_1)
    );

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/predicted_trajectories", 10
    );

    timer_ = create_wall_timer(
      std::chrono::milliseconds(int(PRED_DT * 1000)),
      std::bind(&DwaTrajectoryDrawer::publishTrajectories, this)
    );
  }

private:
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  bool have_odom_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_   = msg->pose.pose.position.x;
    y_   = msg->pose.pose.position.y;
    auto &q = msg->pose.pose.orientation;
    yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );
    have_odom_ = true;
  }

  void publishTrajectories() {
    if (!have_odom_) return;

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(NV_SAMPLES * NW_SAMPLES);
    auto now = this->now();
    const std::string frame = "vesc/odom";
    int id = 0;

    for (int iv = 0; iv < NV_SAMPLES; ++iv) {
      double v_raw = MAX_RAW_SPEED_CMD * iv / (NV_SAMPLES - 1);
      double v = (v_raw / MAX_RAW_SPEED_CMD) * V_MAX;  // convert to m/s
      for (int iw = 0; iw < NW_SAMPLES; ++iw) {
        double w = -W_MAX_RAD + 2.0 * W_MAX_RAD * iw / (NW_SAMPLES - 1);

        // simulate trajectory
        std::vector<std::pair<double,double>> traj;
        traj.reserve(int(PRED_TIME / PRED_DT) + 1);
        double sx = x_, sy = y_, syaw = yaw_;
        traj.emplace_back(sx, sy);
        int steps = int(PRED_TIME / PRED_DT);
        for (int k = 0; k < steps; ++k) {
          sx   += v * std::cos(syaw) * PRED_DT;
          sy   += v * std::sin(syaw) * PRED_DT;
          syaw += w * PRED_DT;
          traj.emplace_back(sx, sy);
        }

        // create marker
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = now;
        m.ns = "dwa_traj";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.005;  // thinner lines
        m.color.r = v / V_MAX;
        m.color.g = std::abs(w) / W_MAX_RAD;
        m.color.b = 1.0 - (v / V_MAX);
        m.color.a = 1.0;

        for (auto &pt : traj) {
          geometry_msgs::msg::Point p;
          p.x = pt.first;
          p.y = pt.second;
          p.z = 0.0;
          m.points.push_back(p);
        }
        ma.markers.push_back(m);
      }
    }
    marker_pub_->publish(ma);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaTrajectoryDrawer>());
  rclcpp::shutdown();
  return 0;
}
