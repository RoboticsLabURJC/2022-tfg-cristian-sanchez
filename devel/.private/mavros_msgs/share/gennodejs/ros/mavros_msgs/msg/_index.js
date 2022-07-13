
"use strict";

let RTCM = require('./RTCM.js');
let CommandCode = require('./CommandCode.js');
let VehicleInfo = require('./VehicleInfo.js');
let Mavlink = require('./Mavlink.js');
let WaypointList = require('./WaypointList.js');
let ESCStatus = require('./ESCStatus.js');
let LogData = require('./LogData.js');
let HilGPS = require('./HilGPS.js');
let HomePosition = require('./HomePosition.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let RCOut = require('./RCOut.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let ActuatorControl = require('./ActuatorControl.js');
let FileEntry = require('./FileEntry.js');
let ESCInfo = require('./ESCInfo.js');
let PositionTarget = require('./PositionTarget.js');
let WaypointReached = require('./WaypointReached.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let State = require('./State.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let Vibration = require('./Vibration.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let ManualControl = require('./ManualControl.js');
let Param = require('./Param.js');
let GPSINPUT = require('./GPSINPUT.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let ExtendedState = require('./ExtendedState.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let VFR_HUD = require('./VFR_HUD.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let HilControls = require('./HilControls.js');
let StatusText = require('./StatusText.js');
let Thrust = require('./Thrust.js');
let RCIn = require('./RCIn.js');
let Trajectory = require('./Trajectory.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let RTKBaseline = require('./RTKBaseline.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let BatteryStatus = require('./BatteryStatus.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let GPSRTK = require('./GPSRTK.js');
let Waypoint = require('./Waypoint.js');
let Altitude = require('./Altitude.js');
let HilSensor = require('./HilSensor.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let RadioStatus = require('./RadioStatus.js');
let DebugValue = require('./DebugValue.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let LogEntry = require('./LogEntry.js');
let GPSRAW = require('./GPSRAW.js');
let LandingTarget = require('./LandingTarget.js');
let MountControl = require('./MountControl.js');
let ParamValue = require('./ParamValue.js');

module.exports = {
  RTCM: RTCM,
  CommandCode: CommandCode,
  VehicleInfo: VehicleInfo,
  Mavlink: Mavlink,
  WaypointList: WaypointList,
  ESCStatus: ESCStatus,
  LogData: LogData,
  HilGPS: HilGPS,
  HomePosition: HomePosition,
  HilActuatorControls: HilActuatorControls,
  RCOut: RCOut,
  OpticalFlowRad: OpticalFlowRad,
  ESCTelemetry: ESCTelemetry,
  ActuatorControl: ActuatorControl,
  FileEntry: FileEntry,
  ESCInfo: ESCInfo,
  PositionTarget: PositionTarget,
  WaypointReached: WaypointReached,
  PlayTuneV2: PlayTuneV2,
  State: State,
  OnboardComputerStatus: OnboardComputerStatus,
  Vibration: Vibration,
  AttitudeTarget: AttitudeTarget,
  ManualControl: ManualControl,
  Param: Param,
  GPSINPUT: GPSINPUT,
  GlobalPositionTarget: GlobalPositionTarget,
  ExtendedState: ExtendedState,
  CamIMUStamp: CamIMUStamp,
  VFR_HUD: VFR_HUD,
  ADSBVehicle: ADSBVehicle,
  ESCInfoItem: ESCInfoItem,
  WheelOdomStamped: WheelOdomStamped,
  HilControls: HilControls,
  StatusText: StatusText,
  Thrust: Thrust,
  RCIn: RCIn,
  Trajectory: Trajectory,
  MagnetometerReporter: MagnetometerReporter,
  RTKBaseline: RTKBaseline,
  TimesyncStatus: TimesyncStatus,
  BatteryStatus: BatteryStatus,
  EstimatorStatus: EstimatorStatus,
  CompanionProcessStatus: CompanionProcessStatus,
  GPSRTK: GPSRTK,
  Waypoint: Waypoint,
  Altitude: Altitude,
  HilSensor: HilSensor,
  HilStateQuaternion: HilStateQuaternion,
  RadioStatus: RadioStatus,
  DebugValue: DebugValue,
  OverrideRCIn: OverrideRCIn,
  NavControllerOutput: NavControllerOutput,
  ESCTelemetryItem: ESCTelemetryItem,
  ESCStatusItem: ESCStatusItem,
  LogEntry: LogEntry,
  GPSRAW: GPSRAW,
  LandingTarget: LandingTarget,
  MountControl: MountControl,
  ParamValue: ParamValue,
};
