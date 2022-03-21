
"use strict";

let VehicleInfoGet = require('./VehicleInfoGet.js')
let SetMode = require('./SetMode.js')
let MountConfigure = require('./MountConfigure.js')
let ParamPush = require('./ParamPush.js')
let FileList = require('./FileList.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let ParamPull = require('./ParamPull.js')
let CommandTOL = require('./CommandTOL.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let FileRead = require('./FileRead.js')
let FileRename = require('./FileRename.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let MessageInterval = require('./MessageInterval.js')
let FileClose = require('./FileClose.js')
let FileWrite = require('./FileWrite.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let LogRequestData = require('./LogRequestData.js')
let CommandLong = require('./CommandLong.js')
let FileMakeDir = require('./FileMakeDir.js')
let LogRequestList = require('./LogRequestList.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let ParamGet = require('./ParamGet.js')
let CommandHome = require('./CommandHome.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let WaypointPull = require('./WaypointPull.js')
let FileRemove = require('./FileRemove.js')
let WaypointClear = require('./WaypointClear.js')
let FileChecksum = require('./FileChecksum.js')
let ParamSet = require('./ParamSet.js')
let CommandAck = require('./CommandAck.js')
let FileTruncate = require('./FileTruncate.js')
let StreamRate = require('./StreamRate.js')
let FileOpen = require('./FileOpen.js')
let SetMavFrame = require('./SetMavFrame.js')
let CommandInt = require('./CommandInt.js')
let WaypointPush = require('./WaypointPush.js')
let CommandBool = require('./CommandBool.js')

module.exports = {
  VehicleInfoGet: VehicleInfoGet,
  SetMode: SetMode,
  MountConfigure: MountConfigure,
  ParamPush: ParamPush,
  FileList: FileList,
  CommandTriggerInterval: CommandTriggerInterval,
  ParamPull: ParamPull,
  CommandTOL: CommandTOL,
  CommandTriggerControl: CommandTriggerControl,
  FileRead: FileRead,
  FileRename: FileRename,
  FileRemoveDir: FileRemoveDir,
  MessageInterval: MessageInterval,
  FileClose: FileClose,
  FileWrite: FileWrite,
  LogRequestEnd: LogRequestEnd,
  LogRequestData: LogRequestData,
  CommandLong: CommandLong,
  FileMakeDir: FileMakeDir,
  LogRequestList: LogRequestList,
  WaypointSetCurrent: WaypointSetCurrent,
  ParamGet: ParamGet,
  CommandHome: CommandHome,
  CommandVtolTransition: CommandVtolTransition,
  WaypointPull: WaypointPull,
  FileRemove: FileRemove,
  WaypointClear: WaypointClear,
  FileChecksum: FileChecksum,
  ParamSet: ParamSet,
  CommandAck: CommandAck,
  FileTruncate: FileTruncate,
  StreamRate: StreamRate,
  FileOpen: FileOpen,
  SetMavFrame: SetMavFrame,
  CommandInt: CommandInt,
  WaypointPush: WaypointPush,
  CommandBool: CommandBool,
};
