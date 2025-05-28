#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

struct Pt { double x, y; };

// DWA parameters
double constexpr INITIAL_HORIZON   = 2.0;      // s
int    constexpr NV                = 5;        // linear velocity samples
int    constexpr NW                = 7;        // angular velocity samples
double constexpr DT                = 0.1;      // s

double constexpr V_MAX_MS          = 3.0;      // m/s
double constexpr W_MAX_RAD         = M_PI/6.0; // ±30°/s

double constexpr GOAL_TOL          = 0.05;     // m

double constexpr MAX_RAW_SPEED_CMD = 23070.0; // raw units

// Servo mapping constants
static constexpr double SERVO_CENTER = 0.5304;
static constexpr double SERVO_MAX    = 0.94299;
static constexpr double SERVO_MIN    = 0.11781;

double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

class DwaController : public rclcpp::Node {
public:
  DwaController()
  : Node("dwa_controller"), have_odom_(false), have_goal_(false)
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/vesc/odom", 10,
      std::bind(&DwaController::odomCb, this, std::placeholders::_1)
    );
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal", 10,
      std::bind(&DwaController::goalCb, this, std::placeholders::_1)
    );
    obs_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/obstacles_marker", 10,
      std::bind(&DwaController::obsCb, this, std::placeholders::_1)
    );

    speed_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/commands/motor/speed", 10
    );
    steer_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/commands/servo/position", 10
    );
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/predicted_trajectories", 10
    );

    timer_ = create_wall_timer(
      std::chrono::milliseconds(int(DT*1000)),
      std::bind(&DwaController::update, this)
    );
  }

private:
  double x_, y_, yaw_;
  bool have_odom_, have_goal_;
  double goal_x_, goal_y_;
  std::vector<Pt> obstacles_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_, steer_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr m) {
    x_ = m->pose.pose.position.x;
    y_ = m->pose.pose.position.y;
    auto &q = m->pose.pose.orientation;
    yaw_ = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    have_odom_ = true;
  }

  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr m) {
    goal_x_ = m->pose.position.x;
    goal_y_ = m->pose.position.y;
    have_goal_ = true;
    RCLCPP_INFO(get_logger(), "Goal set: [%.2f, %.2f]", goal_x_, goal_y_);
  }

  void obsCb(const visualization_msgs::msg::MarkerArray::SharedPtr m) {
    obstacles_.clear();
    for (auto &mk : m->markers) {
      obstacles_.push_back({mk.pose.position.x, mk.pose.position.y});
    }
  }

  void update() {
    if (!have_odom_ || !have_goal_) return;

    double best_v = 0.0, best_w = 0.0;
    std::vector<Pt> best_traj;
    double best_cost = -1e9;

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(NV * NW + 1);
    auto now = this->now();
    const std::string frame = "vesc/odom";
    int id = 0;

    // simulate all
    int steps = int(INITIAL_HORIZON / DT);
    for (int iv = 0; iv < NV; ++iv) {
      double v = V_MAX_MS * iv / (NV - 1);
      for (int iw = 0; iw < NW; ++iw) {
        double w = W_MAX_RAD * (1.0 - 2.0*iw/(NW-1));  // flipped sign for correct turn

        double sx = x_, sy = y_, syaw = yaw_;
        std::vector<Pt> traj;
        traj.reserve(steps+1);
        double min_d = 1e9;
        for (int k = 0; k <= steps; ++k) {
          traj.push_back({sx, sy});
          for (auto &o : obstacles_) {
            min_d = std::min(min_d, std::hypot(sx-o.x, sy-o.y));
          }
          sx   += v * std::cos(syaw) * DT;
          sy   += v * std::sin(syaw) * DT;
          syaw += w * DT;
        }
        double dg = std::hypot(sx-goal_x_, sy-goal_y_);
        double cost = -dg + min_d;
        

        // draw all in gray
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = now;
        m.ns = "all_traj";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.005;
        m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.6;
        for (auto &pt : traj) {
          geometry_msgs::msg::Point p; p.x = pt.x; p.y = pt.y; p.z = 0.0;
          m.points.push_back(p);
        }
        ma.markers.push_back(m);

        if (cost > best_cost) {
          best_cost = cost;
          best_v = v;
          best_w = w;
          best_traj = traj;
        }
      }
    }
    // draw best in turquoise
    visualization_msgs::msg::Marker mb;
    mb.header.frame_id = frame;
    mb.header.stamp = now;
    mb.ns = "best_traj";
    mb.id = id++;
    mb.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mb.action = visualization_msgs::msg::Marker::ADD;
    mb.scale.x = 0.01;
    mb.color.r = 0.0; mb.color.g = 1.0; mb.color.b = 1.0; mb.color.a = 1.0;
    for (auto &pt : best_traj) {
      geometry_msgs::msg::Point p; p.x = pt.x; p.y = pt.y; p.z = 0.0;
      mb.points.push_back(p);
    }
    ma.markers.push_back(mb);

    marker_pub_->publish(ma);

    // send commands
    std_msgs::msg::Float64 sp, st;
    sp.data = std::clamp(best_v / V_MAX_MS * MAX_RAW_SPEED_CMD, 0.0, MAX_RAW_SPEED_CMD);
    speed_pub_->publish(sp);
    RCLCPP_INFO(get_logger(), "Publishing speed: %.2f raw", sp.data);
    double span = (SERVO_MAX - SERVO_CENTER);
    st.data = std::clamp(SERVO_CENTER + (-best_w / W_MAX_RAD) * span,
                         SERVO_MIN, SERVO_MAX);
    // note flipped sign here for correct steering
    steer_pub_->publish(st);
    RCLCPP_INFO(get_logger(), "Publishing steer: %.3f", st.data);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaController>());
  rclcpp::shutdown();
  return 0;
}
