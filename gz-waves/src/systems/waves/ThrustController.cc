// // #include <gz/msgs/vector2d.pb.h>
// // #include <gz/transport/Node.hh>
// // #include <gz/sim/Model.hh>
// // #include <gz/sim/Util.hh>
// // #include <gz/sim/System.hh>
// // #include <gz/sim/components/JointForceCmd.hh>
// // #include <gz/sim/components/Joint.hh>
// // #include "gz/waves/Utilities.hh"

// // namespace gz {
// // namespace sim {
// // inline namespace GZ_SIM_VERSION_NAMESPACE {
// // namespace systems {

// // class ThrustController : public System,
// //                          public ISystemConfigure,
// //                          public ISystemPreUpdate
// // {
// // public:
// //   void Configure(const Entity &_entity,
// //                  const std::shared_ptr<const sdf::Element> &_sdf,
// //                  EntityComponentManager &_ecm,
// //                  EventManager & _eventMgr) override
// //   {
// //     this->model = Model(_entity);
// //     if (!this->model.Valid(_ecm)) {
// //       gzerr << "ThrustController plugin should be attached to a model entity. "
// //             << "Failed to initialize." << std::endl;
// //       return;
// //     }

// //     // Get joint names from SDF
// //     this->leftJointName = _sdf->Get<std::string>("left_propeller_joint");
// //     this->rightJointName = _sdf->Get<std::string>("right_propeller_joint");
// //     this->maxThrust = _sdf->Get<double>("max_thrust", 2000).first;

// //     // Initialize joints
// //     this->leftJoint = this->model.JointByName(_ecm, this->leftJointName);
// //     this->rightJoint = this->model.JointByName(_ecm, this->rightJointName);

// //     if (this->leftJoint == kNullEntity || this->rightJoint == kNullEntity) {
// //       gzerr << "Failed to find joints [" << this->leftJointName 
// //             << "] and/or [" << this->rightJointName << "]" << std::endl;
// //       return;
// //     }

// //     // Create JointForceCmd components if they don't exist
// //     if (!_ecm.Component<components::JointForceCmd>(this->leftJoint)) {
// //       _ecm.CreateComponent(this->leftJoint, components::JointForceCmd({0}));
// //     }
// //     if (!_ecm.Component<components::JointForceCmd>(this->rightJoint)) {
// //       _ecm.CreateComponent(this->rightJoint, components::JointForceCmd({0}));
// //     }

// //     // Subscribe to thrust topic
// //     std::string topic = _sdf->Get<std::string>("thrust_topic", "/thrust").first;
// //     this->node.Subscribe(topic, &ThrustController::OnThrustCmd, this);

// //     gzmsg << "ThrustController initialized for joints ["
// //           << this->leftJointName << "] and ["
// //           << this->rightJointName << "] on topic ["
// //           << topic << "]" << std::endl;
// //   }

// //   void PreUpdate(const UpdateInfo &_info,
// //                  EntityComponentManager &_ecm) override
// //   {
// //     if (_info.paused)
// //       return;

// //     std::lock_guard<std::mutex> lock(this->cmdMutex);
// //     if (this->newCmd) {
// //       auto leftForce = this->leftThrust;
// //       auto rightForce = this->rightThrust;

// //       auto leftForceCmd = _ecm.Component<components::JointForceCmd>(this->leftJoint);
// //       if (leftForceCmd)
// //         leftForceCmd->Data()[0] = leftForce;

// //       auto rightForceCmd = _ecm.Component<components::JointForceCmd>(this->rightJoint);
// //       if (rightForceCmd)
// //         rightForceCmd->Data()[0] = rightForce;

// //       this->newCmd = false;
// //     }
// //   }

// // private:
// //   void OnThrustCmd(const msgs::Vector2d &_msg)
// //   {
// //     std::lock_guard<std::mutex> lock(this->cmdMutex);
// //     this->leftThrust = std::clamp(_msg.x(), -this->maxThrust, this->maxThrust);
// //     this->rightThrust = std::clamp(_msg.y(), -this->maxThrust, this->maxThrust);
// //     this->newCmd = true;
// //   }

// // private:
// //   transport::Node node;
// //   Model model;

// //   std::string leftJointName;
// //   std::string rightJointName;
// //   Entity leftJoint{kNullEntity};
// //   Entity rightJoint{kNullEntity};

// //   double maxThrust{2000};
// //   double leftThrust{0};
// //   double rightThrust{0};
// //   bool newCmd{false};
// //   std::mutex cmdMutex;
// // };

// // }  // namespace systems
// // }  // namespace GZ_SIM_VERSION_NAMESPACE
// // }  // namespace sim
// // }  // namespace gz

// // // Register plugin
// // GZ_ADD_PLUGIN(gz::sim::systems::ThrustController,
// //               gz::sim::System,
// //               gz::sim::systems::ThrustController::ISystemConfigure,
// //               gz::sim::systems::ThrustController::ISystemPreUpdate)

// // GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ThrustController,
// //                     gz_waves_ThrustController)
// #include <gz/msgs/vector2d.pb.h>
// #include <gz/transport/Node.hh>
// #include <gz/sim/Model.hh>
// #include <gz/sim/Util.hh>
// #include <gz/sim/System.hh>
// #include <gz/sim/components/JointForceCmd.hh>
// #include <gz/sim/components/Joint.hh>
// #include <mutex>

// using namespace gz;
// using namespace sim;
// using namespace systems;

// class ThrustController : public System,
//                          public ISystemConfigure,
//                          public ISystemPreUpdate
// {
// public:
//   void Configure(const Entity &_entity,
//                 const std::shared_ptr<const sdf::Element> &_sdf,
//                 EntityComponentManager &_ecm,
//                 EventManager & /*_eventMgr*/) override
//   {
//     this->model = Model(_entity);
//     if (!this->model.Valid(_ecm)) {
//       gzerr << "ThrustController plugin should be attached to a model entity. "
//             << "Failed to initialize." << std::endl;
//       return;
//     }

//     // Get joint names from SDF
//     this->leftJointName = _sdf->Get<std::string>("left_propeller_joint");
//     this->rightJointName = _sdf->Get<std::string>("right_propeller_joint");
//     this->maxThrust = _sdf->Get<double>("max_thrust", 2000).first;

//     // Initialize joints
//     this->leftJoint = this->model.JointByName(_ecm, this->leftJointName);
//     this->rightJoint = this->model.JointByName(_ecm, this->rightJointName);

//     if (this->leftJoint == kNullEntity || this->rightJoint == kNullEntity) {
//       gzerr << "Failed to find joints [" << this->leftJointName 
//             << "] and/or [" << this->rightJointName << "]" << std::endl;
//       return;
//     }

//     // Create JointForceCmd components if they don't exist
//     if (!_ecm.Component<components::JointForceCmd>(this->leftJoint)) {
//       _ecm.CreateComponent(this->leftJoint, components::JointForceCmd({0}));
//     }
//     if (!_ecm.Component<components::JointForceCmd>(this->rightJoint)) {
//       _ecm.CreateComponent(this->rightJoint, components::JointForceCmd({0}));
//     }

//     // Subscribe to thrust topic
//     std::string topic = _sdf->Get<std::string>("thrust_topic", "/thrust").first;
//     if (!this->node.Subscribe(topic, &ThrustController::OnThrustCmd, this)) {
//       gzerr << "Failed to subscribe to topic [" << topic << "]" << std::endl;
//       return;
//     }

//     gzmsg << "ThrustController initialized for joints ["
//           << this->leftJointName << "] and ["
//           << this->rightJointName << "] on topic ["
//           << topic << "]" << std::endl;
//   }

//   void PreUpdate(const UpdateInfo &_info,
//                 EntityComponentManager &_ecm) override
//   {
//     if (_info.paused) return;

//     std::lock_guard<std::mutex> lock(this->cmdMutex);
//     if (this->newCmd) {
//       auto leftForce = this->leftThrust;
//       auto rightForce = this->rightThrust;

//       auto leftForceCmd = _ecm.Component<components::JointForceCmd>(this->leftJoint);
//       auto rightForceCmd = _ecm.Component<components::JointForceCmd>(this->rightJoint);

//       if (leftForceCmd) leftForceCmd->Data()[0] = leftForce;
//       if (rightForceCmd) rightForceCmd->Data()[0] = rightForce;

//       this->newCmd = false;
//     }
//   }

// private:
//   void OnThrustCmd(const msgs::Vector2d &_msg)
//   {
//     std::lock_guard<std::mutex> lock(this->cmdMutex);
//     this->leftThrust = std::clamp(_msg.x(), -this->maxThrust, this->maxThrust);
//     this->rightThrust = std::clamp(_msg.y(), -this->maxThrust, this->maxThrust);
//     this->newCmd = true;
//   }

// private:
//   transport::Node node;
//   Model model;

//   std::string leftJointName;
//   std::string rightJointName;
//   Entity leftJoint{kNullEntity};
//   Entity rightJoint{kNullEntity};

//   double maxThrust{2000};
//   double leftThrust{0};
//   double rightThrust{0};
//   bool newCmd{false};
//   std::mutex cmdMutex;
// };

// // Register the plugin
// GZ_ADD_PLUGIN(ThrustController,
//               System,
//               ThrustController::ISystemConfigure,
//               ThrustController::ISystemPreUpdate)

// // Register plugin alias
// GZ_ADD_PLUGIN_ALIAS(ThrustController, "gz::sim::systems::ThrustController")

#include <gz/msgs/vector2d.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Joint.hh>
#include <mutex>

 namespace gz{
 namespace sim{
  inline namespace GZ_SIM_VERSION_NAMESPACE {
 namespace systems{

class ThrustController : public System,
                         public ISystemConfigure,
                         public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager & /*_eventMgr*/) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm)) {
      gzerr << "ThrustController plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
      return;
    }

    // Get joint names from SDF
    this->leftJointName = _sdf->Get<std::string>("left_propeller_joint");
    this->rightJointName = _sdf->Get<std::string>("right_propeller_joint");
    this->maxThrust = _sdf->Get<double>("max_thrust", 2000).first;

    // Initialize joints
    this->leftJoint = this->model.JointByName(_ecm, this->leftJointName);
    this->rightJoint = this->model.JointByName(_ecm, this->rightJointName);

    if (this->leftJoint == kNullEntity || this->rightJoint == kNullEntity) {
      gzerr << "Failed to find joints [" << this->leftJointName 
            << "] and/or [" << this->rightJointName << "]" << std::endl;
      return;
    }

    // Create JointForceCmd components if they don't exist
    if (!_ecm.Component<components::JointForceCmd>(this->leftJoint)) {
      _ecm.CreateComponent(this->leftJoint, components::JointForceCmd({0}));
    }
    if (!_ecm.Component<components::JointForceCmd>(this->rightJoint)) {
      _ecm.CreateComponent(this->rightJoint, components::JointForceCmd({0}));
    }

    // Subscribe to thrust topic
    std::string topic = _sdf->Get<std::string>("thrust_topic", "/thrust").first;
    if (!this->node.Subscribe(topic, &ThrustController::OnThrustCmd, this)) {
      gzerr << "Failed to -------------------------------------------------------------------------subscribe to topic [" << topic << "]" << std::endl;
      return;
    }

    gzmsg << "ThrustController initialized for joints ["
          << this->leftJointName << "] and ---------------------------------------------------------------------------------------["
          << this->rightJointName << "] on topic ["
          << topic << "]" << std::endl;
  }

  void PreUpdate(const UpdateInfo &_info,
                EntityComponentManager &_ecm) override
  {
    if (_info.paused) return;

    std::lock_guard<std::mutex> lock(this->cmdMutex);
    if (this->newCmd) {
      auto leftForce = this->leftThrust;
      auto rightForce = this->rightThrust;

      auto leftForceCmd = _ecm.Component<components::JointForceCmd>(this->leftJoint);
      auto rightForceCmd = _ecm.Component<components::JointForceCmd>(this->rightJoint);

      if (leftForceCmd) leftForceCmd->Data()[0] = leftForce;
      if (rightForceCmd) rightForceCmd->Data()[0] = rightForce;

      this->newCmd = false;
    }
  }

private:
  void OnThrustCmd(const msgs::Vector2d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    this->leftThrust = std::clamp(_msg.x(), -this->maxThrust, this->maxThrust);
    this->rightThrust = std::clamp(_msg.y(), -this->maxThrust, this->maxThrust);
    this->newCmd = true;
  }

private:
  transport::Node node;
  Model model;

  std::string leftJointName;
  std::string rightJointName;
  Entity leftJoint{kNullEntity};
  Entity rightJoint{kNullEntity};

  double maxThrust{2000};
  double leftThrust{0};
  double rightThrust{0};
  bool newCmd{false};
  std::mutex cmdMutex;
};
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

// THESE MUST BE IN GLOBAL NAMESPACE!
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(gz::sim::systems::ThrustController,
              gz::sim::System,
              gz::sim::systems::ThrustController::ISystemConfigure,
              gz::sim::systems::ThrustController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ThrustController,
                    "gz::sim::systems::ThrustController")