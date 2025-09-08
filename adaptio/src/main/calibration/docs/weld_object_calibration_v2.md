# Diagram

```plantuml

title Start Weld Object Calibration V2

skinparam backgroundColor #DCE8F7
skinparam sequenceParticipant {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
}

hide footbox

participant "Storage" as Storage
participant "CalibrationSolver" as CalibrationSolver
participant "ModelConfig" as ModelConfig
participant "Kinematics" as Kinematics
participant "CalibrationMgrV2" as CalibrationMgrV2
participant "WebHMI" as WebHMI
participant "Scanner" as Scanner

  == Update Frontend ==
  WebHMI -> CalibrationMgrV2: WeldObjectCalGet
  WebHMI <- CalibrationMgrV2: WeldObjectGetRsp(result=ok/fail, laser_clock_pos, object_orientation, calibration_center)

  == Phase 1 ==
  note over WebHMI : Start Weld Object \nCalibration button pressed

  WebHMI -> CalibrationMgrV2 : WeldObjectCalStart(ScannerMountAngle, WeldObjectRadius)

  note over CalibrationMgrV2 : Check that weld axis velocity = 0
  CalibrationMgrV2 -> Kinematics : GetWeldAxisData

  note over CalibrationMgrV2 : Start scanner
  CalibrationMgrV2 -> Scanner: Start
  CalibrationMgrV2 <- Scanner: OnScannerStarted
  WebHMI <- CalibrationMgrV2 : WeldObjectCalStartRsp(Result=OK)

  note over WebHMI : Left wall position \nbutton pressed
  WebHMI -> CalibrationMgrV2 : WeldObjectCalLeftPos()

  Scanner -> CalibrationMgrV2 : OnScannerDataUpdate(LPCSSlice, SliderPosition)
  note over CalibrationMgrV2 : Store slide cross position and latest ABW slice\nfor left wall
  CalibrationMgrV2 -> WebHMI : WeldObjectCalLeftPosRsp(result=ok)

  note over WebHMI : Right wall position \nbutton pressed
  WebHMI -> CalibrationMgrV2 : WeldObjectCalRightPos()
  Scanner -> CalibrationMgrV2 : OnScannerDataUpdate(LPCSSlice, SliderPosition)
  note over CalibrationMgrV2 : Store slide cross position and latest ABW slice\nfor right wall
  CalibrationMgrV2 -> WebHMI : WeldObjectCalRightPosRsp(result=ok)

  == Phase 2 ==
  note over CalibrationMgrV2 : Disable operator slide input?

  note over CalibrationMgrV2 : Calculate how far down in groove\nWith use of left/right wall position\n populate grid
  loop All positions in grid
    CalibrationMgrV2 -> Kinematics : SetSlidesPosition
    loop Until in_position
      Scanner -> CalibrationMgrV2 : OnScannerDataUpdate(LPCSSlice, SliderPosition)
      CalibrationMgrV2 -> Kinematics : GetSlidesStatus()
    end
    note over CalibrationMgrV2
      Wait ~300ms for a median slice
    end note
    note over CalibrationMgrV2 : Store slide cross position and latest ABW slice\nfor grid position
  end

  CalibrationMgrV2 -> CalibrationSolver : Calculate(LeftPosition, RightPosition, GridPositions, ScannerBracketAngle, WeldObjectRadius)

  CalibrationMgrV2 <- CalibrationSolver : Result(laser_clock_pos, object_orientation, calibration_center)

  note over CalibrationMgrV2
    Set slider to position center/above groove
  end note
  CalibrationMgrV2 -> Kinematics: SetSlidesPosition

  WebHMI <- CalibrationMgrV2: WeldObjectCalResult(result=ok/fail, laser_clock_pos, object_orientation, calibration_center)

  == Set ==
  note over CalibrationMgrV2 : Set is done as a separate step for better testability

  WebHMI -> CalibrationMgrV2: WeldObjectCalSet(laser_clock_pos, object_orientation, calibration_center)
  CalibrationMgrV2 -> Storage : Store(laser_clock_pos, object_orientation, calibration_center)
  CalibrationMgrV2 -> ModelConfig: Set(laser_clock_pos, object_orientation, calibration_center)
  WebHMI <- CalibrationMgrV2: WeldObjectCalSetRsp(result=ok/fail)

```
