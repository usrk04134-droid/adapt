# Diagram

```plantuml
left to right direction

package common {
  package logging {
    package quest {
      class "Quest" as common::logging::quest::Quest
    }

    struct "LogData" as common::logging::LogData
    interface "DataLogger" as common::logging::DataLogger

    common::logging::DataLogger <|-- common::logging::quest::Quest
  }
}

package zevs {
  package adapter {
    circle "getGlobalContext" as zevs::adapter::getGlobalContext
  }

  abstract "Context" as zevs::Context
  circle "ExitEventLoop" as zevs::ExitEventLoop
  circle "GetCoreFactory" as zevs::GetCoreFactory
  circle "SetCoreFactoryGenerator" as zevs::SetCoreFactoryGenerator
  class "Socket" as zevs::Socket
  class "Timer" as zevs::Timer
  enum "MessageType" as zevs::MessageType
  enum "SocketType" as zevs::SocketType
  interface "CoreFactory" as zevs::CoreFactory
  interface "CoreSocket" as zevs::CoreSocket
  interface "CoreTimer" as zevs::CoreTimer
  interface "EventLoop" as zevs::EventLoop
  interface "Logging" as zevs::Logging
  interface "Message" as zevs::Message

  zevs::CoreFactory <-- zevs::GetCoreFactory
  zevs::CoreFactory <-- zevs::SetCoreFactoryGenerator
  zevs::CoreSocket <-- zevs::Socket
  zevs::CoreTimer *-- zevs::Timer
  zevs::CoreTimer <-- zevs::CoreFactory
  zevs::EventLoop <-- zevs::CoreFactory
  zevs::EventLoop <-- zevs::ExitEventLoop
  zevs::GetCoreFactory <-- zevs::Socket
  zevs::Logging *-- zevs::Timer
  zevs::Message <-- zevs::CoreFactory
  zevs::Message <-- zevs::CoreSocket
  zevs::Message <-- zevs::Socket
  zevs::MessageType <-- zevs::CoreFactory
  zevs::MessageType <-- zevs::Message
  zevs::SocketType <-- zevs::CoreFactory
  zevs::SocketType <-- zevs::Socket
}
```
