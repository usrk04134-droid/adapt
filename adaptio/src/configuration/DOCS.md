# Diagram

```plantuml
left to right direction

package configuration {
  circle "GetFactory" as components::configuration::GetFactory
  circle "SetFactoryGenerator" as components::configuration::SetFactoryGenerator
  class "CalibFixJointGeometryConverter" as components::configuration::CalibFixJointGeometryConverter
  class "CircWeldObjectCalibConverter" as components::configuration::CircWeldObjectCalibConverter
  class "ConfigManager" as components::configuration::ConfigManager
  class "ControllerConfigConverter" as components::configuration::ControllerConfigConverter
  class "FactoryImpl" as components::configuration::FactoryImpl
  class "FileHandlerImpl" as components::configuration::FileHandlerImpl
  class "ImageProviderConverter" as components::configuration::ImageProviderConverter
  class "JointGeometryConverter" as components::configuration::JointGeometryConverter
  class "LaserTorchCalibConverter" as components::configuration::LaserTorchCalibConverter
  class "ScannerCalibrationConverter" as components::configuration::ScannerCalibrationConverter
  class "ScannerConfigurationConverter" as components::configuration::ScannerConfigurationConverter
  interface "ConfigurationHandle" as components::configuration::ConfigurationHandle
  interface "Converter" as components::configuration::Converter
  interface "Factory" as components::configuration::Factory
  interface "FileHandler" as components::configuration::FileHandler

  components::configuration::ConfigurationHandle <-- components::configuration::ConfigManager
  components::configuration::ConfigurationHandle <|-- components::configuration::Converter
  components::configuration::Converter *--- components::configuration::ConfigManager
  components::configuration::Converter <-- components::configuration::FactoryImpl
  components::configuration::Converter <|-- components::configuration::CalibFixJointGeometryConverter
  components::configuration::Converter <|-- components::configuration::CircWeldObjectCalibConverter
  components::configuration::Converter <|-- components::configuration::JointGeometryConverter
  components::configuration::Converter <|-- components::configuration::CalibFixJointGeometryConverter
  components::configuration::Converter <|-- components::configuration::ControllerConfigConverter
  components::configuration::Converter <|-- components::configuration::ImageProviderConverter
  components::configuration::Converter <|-- components::configuration::JointGeometryConverter
  components::configuration::Converter <|-- components::configuration::LaserTorchCalibConverter
  components::configuration::Converter <|-- components::configuration::ScannerCalibrationConverter
  components::configuration::Converter <|-- components::configuration::ScannerConfigurationConverter
  components::configuration::FileHandler *-- components::configuration::CalibFixJointGeometryConverter
  components::configuration::FileHandler *-- components::configuration::CircWeldObjectCalibConverter
  components::configuration::FileHandler *-- components::configuration::ConfigManager
  components::configuration::FileHandler *-- components::configuration::ControllerConfigConverter
  components::configuration::FileHandler *-- components::configuration::ImageProviderConverter
  components::configuration::FileHandler *-- components::configuration::JointGeometryConverter
  components::configuration::FileHandler *-- components::configuration::LaserTorchCalibConverter
  components::configuration::FileHandler *-- components::configuration::ScannerCalibrationConverter
  components::configuration::FileHandler *-- components::configuration::ScannerConfigurationConverter
  components::configuration::FileHandler <-- components::configuration::FactoryImpl
  components::configuration::FileHandler <|-- components::configuration::FileHandlerImpl
  components::configuration::Factory <-- components::configuration::GetFactory
  components::configuration::Factory <-- components::configuration::SetFactoryGenerator
  components::configuration::Factory <|-- components::configuration::FactoryImpl
}

```
