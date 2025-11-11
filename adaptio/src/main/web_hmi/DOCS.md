# Diagram

```plantuml

class "WebHmiServer"
class "OperationDataImpl"
class "WebHmiCalibration"
interface "OperationData"
interface "kinematics::KinematicsClientObserver"
interface "slice_translator::SliceObserver"
interface "kinematics::KinematicsClient"
interface "tracking::TrackingManager"
interface "scanner::ScannerClient"
interface "calibration::CalibrationManager"

WebHmiCalibration <-- WebHmiServer
OperationData <|-- OperationDataImpl
kinematics::KinematicsClientObserver <|-- WebHmiServer
slice_translator::SliceObserver <|-- WebHmiServer

calibration::CalibrationManager <-- WebHmiCalibration
```
