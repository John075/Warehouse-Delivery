
"use strict";

let ServoCommand = require('./ServoCommand.js');
let Compass = require('./Compass.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let ControllerState = require('./ControllerState.js');
let Altimeter = require('./Altimeter.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let ThrustCommand = require('./ThrustCommand.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let MotorPWM = require('./MotorPWM.js');
let RC = require('./RC.js');
let RuddersCommand = require('./RuddersCommand.js');
let RawRC = require('./RawRC.js');
let MotorCommand = require('./MotorCommand.js');
let RawImu = require('./RawImu.js');
let HeightCommand = require('./HeightCommand.js');
let MotorStatus = require('./MotorStatus.js');
let HeadingCommand = require('./HeadingCommand.js');
let Supply = require('./Supply.js');
let YawrateCommand = require('./YawrateCommand.js');
let RawMagnetic = require('./RawMagnetic.js');
let TakeoffGoal = require('./TakeoffGoal.js');
let PoseResult = require('./PoseResult.js');
let TakeoffFeedback = require('./TakeoffFeedback.js');
let PoseFeedback = require('./PoseFeedback.js');
let LandingActionFeedback = require('./LandingActionFeedback.js');
let LandingGoal = require('./LandingGoal.js');
let PoseActionFeedback = require('./PoseActionFeedback.js');
let LandingActionGoal = require('./LandingActionGoal.js');
let TakeoffResult = require('./TakeoffResult.js');
let PoseGoal = require('./PoseGoal.js');
let LandingAction = require('./LandingAction.js');
let TakeoffActionResult = require('./TakeoffActionResult.js');
let LandingFeedback = require('./LandingFeedback.js');
let PoseActionGoal = require('./PoseActionGoal.js');
let TakeoffActionGoal = require('./TakeoffActionGoal.js');
let LandingResult = require('./LandingResult.js');
let TakeoffAction = require('./TakeoffAction.js');
let LandingActionResult = require('./LandingActionResult.js');
let TakeoffActionFeedback = require('./TakeoffActionFeedback.js');
let PoseAction = require('./PoseAction.js');
let PoseActionResult = require('./PoseActionResult.js');

module.exports = {
  ServoCommand: ServoCommand,
  Compass: Compass,
  PositionXYCommand: PositionXYCommand,
  ControllerState: ControllerState,
  Altimeter: Altimeter,
  AttitudeCommand: AttitudeCommand,
  VelocityZCommand: VelocityZCommand,
  ThrustCommand: ThrustCommand,
  VelocityXYCommand: VelocityXYCommand,
  MotorPWM: MotorPWM,
  RC: RC,
  RuddersCommand: RuddersCommand,
  RawRC: RawRC,
  MotorCommand: MotorCommand,
  RawImu: RawImu,
  HeightCommand: HeightCommand,
  MotorStatus: MotorStatus,
  HeadingCommand: HeadingCommand,
  Supply: Supply,
  YawrateCommand: YawrateCommand,
  RawMagnetic: RawMagnetic,
  TakeoffGoal: TakeoffGoal,
  PoseResult: PoseResult,
  TakeoffFeedback: TakeoffFeedback,
  PoseFeedback: PoseFeedback,
  LandingActionFeedback: LandingActionFeedback,
  LandingGoal: LandingGoal,
  PoseActionFeedback: PoseActionFeedback,
  LandingActionGoal: LandingActionGoal,
  TakeoffResult: TakeoffResult,
  PoseGoal: PoseGoal,
  LandingAction: LandingAction,
  TakeoffActionResult: TakeoffActionResult,
  LandingFeedback: LandingFeedback,
  PoseActionGoal: PoseActionGoal,
  TakeoffActionGoal: TakeoffActionGoal,
  LandingResult: LandingResult,
  TakeoffAction: TakeoffAction,
  LandingActionResult: LandingActionResult,
  TakeoffActionFeedback: TakeoffActionFeedback,
  PoseAction: PoseAction,
  PoseActionResult: PoseActionResult,
};
