# Diagram

```plantuml

title Start Weld Object Calibration

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
participant "CalibrationMgr" as CalibrationMgr
participant "WebHMI" as WebHMI
participant "Scanner" as Scanner

  == Update Frontend ==
  WebHMI -> CalibrationMgr: WeldObjectCalGet
  WebHMI <- CalibrationMgr: WeldObjectGetRsp(result=ok/fail, laser_clock_pos, object_orientation, calibration_center)

  == Phase 1 ==
  note over WebHMI : Start Weld Object \nCalibration button pressed

  WebHMI -> CalibrationMgr : WeldObjectCalStart(ScannerMountAngle, WeldObjectRadius)

  note over CalibrationMgr : Check that weld axis velocity = 0
  CalibrationMgr -> Kinematics : GetWeldAxisData

  note over CalibrationMgr : Start scanner
  CalibrationMgr -> Scanner: Start
  CalibrationMgr <- Scanner: OnScannerStarted
  WebHMI <- CalibrationMgr : WeldObjectCalStartRsp(Result=OK)

  note over WebHMI : Top Position \nbutton pressed
  WebHMI -> CalibrationMgr : WeldObjectCalTopPos()
  Scanner -> CalibrationMgr : OnScannerDataUpdate(LPCSSlice, SliderPosition)
  note over CalibrationMgr : Store slide cross position and latest ABW slice\nfor top position
  CalibrationMgr -> WebHMI : WeldObjectCalTopPosRsp(result=ok)

  note over WebHMI : Left wall position \nbutton pressed
  WebHMI -> CalibrationMgr : WeldObjectCalLeftPos()
  Scanner -> CalibrationMgr : OnScannerDataUpdate(LPCSSlice, SliderPosition)
  note over CalibrationMgr : Store slide cross position and latest ABW slice\nfor left wall
  CalibrationMgr -> WebHMI : WeldObjectCalLeftPosRsp(result=ok)

  note over WebHMI : Right wall position \nbutton pressed
  WebHMI -> CalibrationMgr : WeldObjectCalRightPos()
  Scanner -> CalibrationMgr : OnScannerDataUpdate(LPCSSlice, SliderPosition)
  note over CalibrationMgr : Store slide cross position and latest ABW slice\nfor right wall
  CalibrationMgr -> WebHMI : WeldObjectCalRightPosRsp(result=ok)

  == Phase 2 ==
  note over CalibrationMgr : Disable operator slide input?

  note over CalibrationMgr : Calculate how far down in groove\nWith use of top position, left/right wall position\n populate grid
  loop All positions in grid
    CalibrationMgr -> Kinematics : SetSlidesPosition
    loop Until in_position
      Scanner -> CalibrationMgr : OnScannerDataUpdate(LPCSSlice, SliderPosition)
      CalibrationMgr -> Kinematics : GetSlidesStatus()
    end
    note over CalibrationMgr
      Wait ~300ms for a median slice
    end note
    note over CalibrationMgr : Store slide cross position and latest ABW slice\nfor grid position
  end

  CalibrationMgr -> CalibrationSolver : Calculate(TopPosition, LeftPosition, RightPosition, GridPositions, ScannerBracketAngle, WeldObjectRadius)

  CalibrationMgr <- CalibrationSolver : Result(laser_clock_pos, object_orientation, calibration_center)

  note over CalibrationMgr
    Set slider to position center/above groove
  end note
  CalibrationMgr -> Kinematics: SetSlidesPosition

  WebHMI <- CalibrationMgr: WeldObjectCalResult(result=ok/fail, laser_clock_pos, object_orientation, calibration_center)

  == Set ==
  note over CalibrationMgr : Set is done as a separate step for better testability

  WebHMI -> CalibrationMgr: WeldObjectCalSet(laser_clock_pos, object_orientation, calibration_center)
  CalibrationMgr -> Storage : Store(laser_clock_pos, object_orientation, calibration_center)
  CalibrationMgr -> ModelConfig: Set(laser_clock_pos, object_orientation, calibration_center)
  WebHMI <- CalibrationMgr: WeldObjectCalSetRsp(result=ok/fail)

```
