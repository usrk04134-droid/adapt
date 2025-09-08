# Diagram

```plantuml
left to right direction

package kinematics {
  class "KinematicsClientImpl" as kinematics::KinematicsClientImpl
  interface "KinematicsClient" as kinematics::KinematicsClient
  interface "KinematicsClientObserver" as kinematics::KinematicsClientObserver


  kinematics::KinematicsClientImpl --> kinematics::KinematicsClientObserver
  kinematics::KinematicsClientImpl --|>  kinematics::KinematicsClient
}

```
