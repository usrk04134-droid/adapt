# Diagram

```plantuml

title Calibration and Slice Translation

skinparam backgroundColor #DCE8F7
skinparam rectangle {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
}
skinparam package {
    BackgroundColor #DCE8F7
    BorderColor #000000
    FontColor #000000
}

' Namespaces
package "weld-control\n<NS>" {
    rectangle "WeldControlImpl" as WeldControlImpl
}

package "calibration\n<NS>" {
    rectangle "CalibrationManagerImpl" as CalibrationManagerImpl
    rectangle "CWCalibrationHandler" as CWCalibrationHandler
    rectangle "LWCalibrationHandler" as LWCalibrationHandler
    rectangle "CalibrationSolver\n<IF>" as CalibrationSolver
    rectangle "CalibrationSolverImpl" as CalibrationSolverImpl
    rectangle "WOCalStorage" as WOCalStorage
    rectangle "LTCalStorage" as LTCalStorage
    rectangle "LWCalStorage" as LWCalStorage
}

package "scanner-client\n<NS>" {
    rectangle "ScannerObserver\n<IF>" as ScannerObserver
}

package "slice-translator\n<NS>" {
    rectangle "SliceObserver\n<IF>" as SliceObserver
    rectangle "CoordinatesTranslator" as CoordinatesTranslator
    rectangle "SliceTranslatorService\n<IF>" as SliceTranslatorService
    rectangle "ModelImpl" as ModelImpl
    rectangle "ModelConfig\n<IF>" as ModelConfig
    rectangle "ModelExtract\n<IF>" as ModelExtract
}

' Relationships
WeldControlImpl -down-|> SliceObserver

CalibrationManagerImpl -up-|> ScannerObserver
CalibrationManagerImpl --> CWCalibrationHandler
CalibrationManagerImpl --> LWCalibrationHandler

CWCalibrationHandler --> CalibrationSolver
CWCalibrationHandler --> WOCalStorage
CWCalibrationHandler --> LTCalStorage
CWCalibrationHandler --> ModelConfig
LWCalibrationHandler --> CalibrationSolver
LWCalibrationHandler --> LWCalStorage
LWCalibrationHandler --> ModelConfig
CalibrationSolverImpl -up-|> CalibrationSolver
CalibrationSolverImpl -up-> ModelExtract

CoordinatesTranslator -up-|> ScannerObserver
CoordinatesTranslator -up-> SliceObserver
CoordinatesTranslator --> SliceTranslatorService
ModelImpl -up-|> SliceTranslatorService
ModelImpl -up-|> ModelConfig
ModelImpl -up-|> ModelExtract

' Bottom note
note right of WeldControlImpl
Description of Interfaces

- SliceTranslatorService: Used to translate coordinates from LPCS to MCS

- ModelConfig: Interface used to set model parameters after calibration procedure
is completed or after reading parameters from database after a restart

- ModelExtract: Called by the Solver implementation class during the solve phase
when extracting the model parameters

- CalibrationSolver: Called when the calibration sequence has been completed. 
The call contains ABW points, ScannerBracketAngle, WeldObjectRadius, and returns
model parameters for storage and use in the ModelConfig interface.

- CalibrationHandlers: Handlers implement
specific calibration procedures (CW or LW) and subscribe to their own WebHMI messages.

end note


```
