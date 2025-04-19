// #include <chrono>
// #include <iostream>
// #include <thread>

// #include <gz/msgs/vector2d.pb.h>
// #include <gz/transport/Node.hh>

// int main()
// {
//   // Create a transport node
//   gz::transport::Node node;

//   // Topic to publish to
//   std::string topic = "/thrust";

//   // Publisher for Vector2d messages
//   auto pub = node.Advertise<gz::msgs::Vector2d>(topic);

//   if (!pub)
//   {
//     std::cerr << "Failed to advertise on topic [" << topic << "]" << std::endl;
//     return -1;
//   }

//   std::cout << "Publishing dummy thrust commands on [" << topic << "]..." << std::endl;

//   while (true)
//   {
//     gz::msgs::Vector2d thrustMsg;

//     // Dummy thrust values: left = 500 N, right = 600 N
//     thrustMsg.set_x(500.0);
//     thrustMsg.set_y(600.0);

//     pub.Publish(thrustMsg);
//     std::cout << "Published thrust: (" << thrustMsg.x() << ", " << thrustMsg.y() << ")" << std::endl;

//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   }

//   return 0;
// }
#include <chrono>
#include <iostream>
#include <thread>

#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

int main()
{
  // Create a transport node
  gz::transport::Node node;

  // Topics to publish to
  std::string left_topic = "/world/waves/model/wam-v/joint/left_propeller_joint/ang_vel";
  std::string right_topic = "/world/waves/model/wam-v/joint/right_propeller_joint/ang_vel";

  // Publishers for angular velocity commands
  auto left_pub = node.Advertise<gz::msgs::Double>(left_topic);
  auto right_pub = node.Advertise<gz::msgs::Double>(right_topic);

  if (!left_pub || !right_pub)
  {
    std::cerr << "Failed to advertise on topics:" << std::endl;
    if (!left_pub) std::cerr << "  [" << left_topic << "]" << std::endl;
    if (!right_pub) std::cerr << "  [" << right_topic << "]" << std::endl;
    return -1;
  }

  std::cout << "Publishing angular velocity commands:" << std::endl;
  std::cout << "  Left:  [" << left_topic << "]" << std::endl;
  std::cout << "  Right: [" << right_topic << "]" << std::endl;

  while (true)
  {
    gz::msgs::Double left_msg;
    gz::msgs::Double right_msg;

    // Set angular velocities (rad/s)
    // Positive values for clockwise rotation when viewed from behind
    left_msg.set_data(100.0);   // 10 rad/s (~95.5 RPM)
    right_msg.set_data(-100.0);  // -10 rad/s (counter-clockwise)

    // Publish messages
    left_pub.Publish(left_msg);
    right_pub.Publish(right_msg);

    std::cout << "Published angular velocities: "
              << "Left = " << left_msg.data() << " rad/s, "
              << "Right = " << right_msg.data() << " rad/s" << std::endl;

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return 0;
}