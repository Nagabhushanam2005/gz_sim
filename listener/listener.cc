#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/transport/Node.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <fstream>
#include <iostream>
#include <atomic>

class WamVDragLogger {
public:
  WamVDragLogger(double max_sim_time = 50.0) : 
    mass(350.0),
    max_sim_time(max_sim_time),
    logging_active(true) {
    
    // Initialize Gazebo Transport subscribers
    node.Subscribe("/model/wam-v/odometry", &WamVDragLogger::odomCallback, this);
    node.Subscribe("/world/waves/model/wam-v/link/imu_link/sensor/imu_sensor/imu", 
                  &WamVDragLogger::imuCallback, this);
    node.Subscribe("/clock", &WamVDragLogger::clockCallback, this);

    // Open log files
    odom_file.open("odom.dat");
    angles_file.open("angles.dat");
    force_file.open("force.dat");
    pose_file.open("pose.dat");

    // Write headers
    odom_file << "# <sim_time> <wave_height>\n";
    angles_file << "# <sim_time> <error_angle_deg>\n";
    force_file << "# <sim_time> <Fx> <Fy> <Fz>\n";
    pose_file << "# <sim_time> <x> <y> <z> <qx> <qy> <qz> <qw>\n";

    std::cout << "Logging started. Will log for " << max_sim_time << " seconds of simulation time.\n";
  }

  ~WamVDragLogger() {
    odom_file.close();
    angles_file.close();
    force_file.close();
    pose_file.close();
    std::cout << "Logging completed after " << current_sim_time << " seconds.\n";
  }

  void clockCallback(const gz::msgs::Clock &_msg) {
    current_sim_time = _msg.sim().sec() + _msg.sim().nsec() * 1e-9;
    
    // Check if we should stop logging
    if (current_sim_time >= max_sim_time && logging_active) {
      logging_active = false;
      std::cout << "Reached maximum simulation time. Stopping logging.\n";
    }
  }

  void odomCallback(const gz::msgs::Odometry &_msg) {
    if (!logging_active) return;

    // Store current pose (for wave height calculation)
    current_position.Set(
      _msg.pose().position().x(),
      _msg.pose().position().y(),
      _msg.pose().position().z()
    );

    // Store current orientation
    current_orientation.Set(
      _msg.pose().orientation().w(),
      _msg.pose().orientation().x(),
      _msg.pose().orientation().y(),
      _msg.pose().orientation().z()
    );

    // Calculate wave height (assuming z=0 is water surface)
    double wave_height = -current_position.Z();
    odom_file << current_sim_time << " " << wave_height << "\n";

    // Log pose data
    pose_file << current_sim_time << " "
              << current_position.X() << " "
              << current_position.Y() << " "
              << current_position.Z() << " "
              << current_orientation.X() << " "
              << current_orientation.Y() << " "
              << current_orientation.Z() << " "
              << current_orientation.W() << "\n";

    // Calculate heading angle (yaw)
    auto euler = current_orientation.Euler();
    double yaw = euler.Z();

    // Calculate error angle (90° - yaw for wave impact at 90°)
    double error_angle = M_PI/2 - std::abs(yaw);
    angles_file << current_sim_time << " " << error_angle * 180/M_PI << "\n";
  }

  void imuCallback(const gz::msgs::IMU &_msg) {
    if (!logging_active) return;

    // Get linear acceleration (excluding gravity)
    gz::math::Vector3d linear_accel(
      _msg.linear_acceleration().x(),
      _msg.linear_acceleration().y(),
      _msg.linear_acceleration().z() + 9.81
    );

    // Calculate drag force (F = m*a)
    gz::math::Vector3d drag_force = linear_accel * mass;

    // Log force components
    force_file << current_sim_time << " " 
               << drag_force.X() << " "
               << drag_force.Y() << " "
               << drag_force.Z() << "\n";
  }

private:
  gz::transport::Node node;
  const double mass;
  const double max_sim_time;
  std::atomic<bool> logging_active;
  std::atomic<double> current_sim_time;
  
  gz::math::Vector3d current_position;
  gz::math::Quaterniond current_orientation;
  
  std::ofstream odom_file;
  std::ofstream pose_file;
  std::ofstream angles_file;
  std::ofstream force_file;
};

int main(int argc, char **argv) {
  double log_duration = 50.0; // Default 50 seconds
  
  // Allow duration override via command line
  if (argc > 1) {
    log_duration = std::stod(argv[1]);
    std::cout << "Setting logging duration to " << log_duration << " seconds\n";
  }

  WamVDragLogger logger(log_duration);
  gz::transport::waitForShutdown();
  return 0;
}