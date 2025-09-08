# Diagram

```plantuml

title Module Architecture: Calibration and Slice Translation

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
    rectangle "CalibrationManagerV2Impl" as CalibrationManagerV2Impl
    rectangle "CalibrationSolver\n<IF>" as CalibrationSolver
    rectangle "CalibrationSolverImpl" as CalibrationSolverImpl
    rectangle "Storage" as Storage
}

package "slice-translator\n<NS>" {
    rectangle "SliceObserver\n<IF>" as SliceObserver
    rectangle "CoordinateTranslatorV2" as CoordinateTranslator
    rectangle "SliceTranslatorServiceV2\n<IF>" as SliceTranslatorServiceV2
    rectangle "ModelImpl" as ModelImpl
    rectangle "ModelConfig\n<IF>" as ModelConfig
    rectangle "ModelExtract\n<IF>" as ModelExtract
}

' Relationships
WeldControlImpl -down-|> SliceObserver

CalibrationManagerV2Impl --> CalibrationSolver
CalibrationManagerV2Impl --> Storage
CalibrationManagerV2Impl --> ModelConfig
CalibrationSolverImpl -up-|> CalibrationSolver
CalibrationSolverImpl -up-> ModelExtract

CoordinateTranslator -up-> SliceObserver
CoordinateTranslator --> SliceTranslatorServiceV2
ModelImpl -up-|> SliceTranslatorServiceV2
ModelImpl -up-|> ModelConfig
ModelImpl -up-|> ModelExtract

' Bottom note
note right of WeldControlImpl
Description of Interfaces

- SliceTranslatorServiceV2: similar to existing interface with LPCSToMCS method

- ModelConfig: Interface used to set model parameters after calibration procedure
is completed or after reading parameters from database after a restart

- ModelExtract: Called by the Solver implementation class during the solve phase
when extracting the model parameters

- CalibrationSolver: Called once when the automatic movement/measurement phase
has been completed. The call contains data for LeftPosition, RightPosition,
GridPositions, ScannerBracketAngle, WeldObjectRadius, ABW points. It returns
the model parameters which will be stored to database and set using the ModelConfig interface.
end note


```
