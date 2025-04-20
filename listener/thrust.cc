#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <mutex>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

using namespace gz;

class PropellerController {
public:
  PropellerController() {
    Kp = 3.0;
    Ki = 0.0;
    Kd = .1;

    left_pub = node.Advertise<msgs::Double>("/model/wam-v/joint/left_propeller_joint/cmd_thrust");
    right_pub = node.Advertise<msgs::Double>("/model/wam-v/joint/right_propeller_joint/cmd_thrust");

    if (!node.Subscribe("/model/wam-v/odometry", &PropellerController::odomCallback, this)) {
      std::cerr << "Failed to subscribe to odometry topic" << std::endl;
    }

    std::cout << "Controller initialized. Running PID loop..." << std::endl;
  }

  void Run() {
    while (true) {
      std::lock_guard<std::mutex> lock(data_mutex);
      
      if (!new_data) {
        continue;
      }

      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - last_time).count();
      last_time = now;

      double error = target_angle - current_error_angle;
      integral += error * dt;
      double derivative = (error - prev_error) / dt;
      prev_error = error;

      double output = Kp * error + Ki * integral + Kd * derivative;

      double base_thrust = 0.0;
      double left_thrust = base_thrust + output;
      double right_thrust = base_thrust - output;

      left_thrust = std::clamp(left_thrust, -2000.0, 2000.0);
      right_thrust = std::clamp(right_thrust, -2000.0, 2000.0);

      msgs::Double left_msg, right_msg;
      left_msg.set_data(left_thrust);
      right_msg.set_data(right_thrust);

      left_pub.Publish(left_msg);
      right_pub.Publish(right_msg);

      std::cout << "Error: " << error << "° | "
                << "Left thrust: " << left_thrust << " N | "
                << "Right thrust: " << right_thrust << " N" << std::endl;

      new_data = false;
    }
  }

private:
  void odomCallback(const msgs::Odometry &_msg) {
    std::lock_guard<std::mutex> lock(data_mutex);

    math::Quaterniond orientation(
      _msg.pose().orientation().w(),
      _msg.pose().orientation().x(),
      _msg.pose().orientation().y(),
      _msg.pose().orientation().z()
    );

    double yaw = orientation.Euler().Z();

    current_error_angle = (M_PI/2 - std::abs(yaw)) * 180/M_PI;
    new_data = true;
  }

  double Kp, Ki, Kd;
  double integral = 0;
  double prev_error = 0;
  const double target_angle = 0;

  double current_error_angle = 0;
  bool new_data = false;
  std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

  transport::Node node;
  transport::Node::Publisher left_pub;
  transport::Node::Publisher right_pub;

  std::mutex data_mutex;
};

int main() {
  PropellerController controller;
  controller.Run();
  return 0;
}
