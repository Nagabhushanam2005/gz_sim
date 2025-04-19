#ifndef GZ_WAVES_THRUST_CONTROLLER_HH_
#define GZ_WAVES_THRUST_CONTROLLER_HH_

#include <string>
#include <mutex>
#include <memory>

#include <gz/msgs/vector2d.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief A system that subscribes to a /thrust topic and applies force to
/// specified left and right propeller joints.
class ThrustController
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  /// \brief Configure the system and retrieve parameters from SDF.
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  /// \brief Apply thrust values on each simulation step.
  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  /// \brief Callback for thrust topic messages.
  void OnThrustCmd(const gz::msgs::Vector2d &_msg);

  /// \brief Communication node for topic subscription.
  gz::transport::Node node;

  /// \brief Handle to the model.
  Model model;

  /// \brief Joint names from SDF.
  std::string leftJointName;
  std::string rightJointName;

  /// \brief Joint entities.
  Entity leftJoint{kNullEntity};
  Entity rightJoint{kNullEntity};

  /// \brief Maximum allowed thrust.
  double maxThrust{2000};

  /// \brief Last received thrust commands.
  double leftThrust{0};
  double rightThrust{0};

  /// \brief Flag indicating new command received.
  bool newCmd{false};

  /// \brief Mutex for command data.
  std::mutex cmdMutex;
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_WAVES_THRUST_CONTROLLER_HH_
