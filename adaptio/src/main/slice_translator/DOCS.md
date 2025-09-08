# Diagram

```plantuml
left to right direction

package slice_translator {
  package calibration {
    interface "CalibrationManager" as calibration::CalibrationManager
    class "CalibrationManagerImpl" as calibration::CalibrationManagerImpl

    calibration::CalibrationManager <|-- calibration::CalibrationManagerImpl
  }

  class "SliceTranslator" as slice_translator::SliceTranslator
  struct "CircularWeldObjectCalibration" as calibration::WeldObjectCalibration
  struct "LaserTorchCalibration" as calibration::LaserTorchCalibration
}
```
