# Diagram

```plantuml
left to right direction


package controller {

  package pn_driver {
    class "PnDriver" as controller::pn_driver::PnDriver
    struct "Configuration" as controller::pn_driver::Configuration
    struct "Subslot" as controller::pn_driver::Subslot
    enum "cm_ar_reason_enum" as controller::pn_driver::cm_ar_reason_enum

    controller::pn_driver::Configuration *-- controller::pn_driver::PnDriver
    controller::pn_driver::Subslot *-- controller::pn_driver::PnDriver
  }

  package simulation {
    class "Simulation" as controller::simulation::Simulation
    class "Kinematics" as controller::simulation::Kinematics
    class "PowerSource" as controller::simulation::PowerSource
    class "KinematicsAxis" as controller::simulation::KinematicsAxis

    controller::simulation::Kinematics *-- controller::simulation::Simulation
    controller::simulation::PowerSource *-- controller::simulation::Simulation
    controller::simulation::KinematicsAxis *-- controller::simulation::Simulation
  }

  package systems {
      class "HeartbeatSystem" as controller::systems::HeartbeatSystem
  }

  enum "AxisId" as controller::AxisId
  enum "ControllerErrorCode" as controller::ControllerErrorCode
  class "ConfigManager" as controller::ControllerConfigManager
  class "ControllerConfiguration" as controller::ControllerConfiguration
  class "ControllerFactory" as controller::ControllerFactory
  class "ControllerMessenger" as controller::ControllerMessenger
  interface "Controller" as controller::Controller
  interface "ControllerDataCallbacks" as controller::ControllerDataCallbacks
  interface "System" as controller::System
  struct "ControllerConfigurationData" as controller::ControllerConfigurationData


  controller::ControllerConfigurationData <-- controller::ControllerConfigManager
  controller::Controller <-- controller::ControllerMessenger
  controller::pn_driver::PnDriver <-- controller::ControllerFactory
  controller::Controller <|-- controller::pn_driver::PnDriver
  controller::ControllerConfiguration <-- controller::ControllerFactory
  controller::ControllerDataCallbacks <|-- controller::ControllerMessenger
  controller::System <|-- controller::systems::HeartbeatSystem
  controller::systems::HeartbeatSystem *-- controller::pn_driver::PnDriver
  controller::simulation::Simulation <-- controller::ControllerFactory
  controller::Controller <|-- controller::simulation::Simulation
}

```
