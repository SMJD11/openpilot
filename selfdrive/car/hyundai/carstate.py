from collections import deque
import copy
import math

from cereal import car, custom
from openpilot.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, HyundaiFrogPilotFlags, CAR, DBC, CAN_GEARS, CAMERA_SCC_CAR, \
                                                   CANFD_CAR, Buttons, CarControllerParams
from openpilot.selfdrive.car.interfaces import CarStateBase

PREV_BUTTON_SAMPLES = 8
CLUSTER_SAMPLE_RATE = 20  # frames
STANDSTILL_THRESHOLD = 12 * 0.03125 * CV.KPH_TO_MS


# Traffic signals for Speed Limit Controller - Credit goes to Multikyd!
@staticmethod
def calculate_speed_limit(CP, FPCP, cp, cp_cam):
  if CP.carFingerprint in CANFD_CAR:
    if CP.flags & HyundaiFlags.CANFD_HDA2:
      speed_limit_bus = cp
    else:
      speed_limit_bus = cp_cam

    speed_limit = speed_limit_bus.vl["CLUSTER_SPEED_LIMIT"]["SPEED_LIMIT_1"]
  else:
    if FPCP.fpFlags & HyundaiFrogPilotFlags.LKAS12:
      speed_limit = cp_cam.vl["LKAS12"]["CF_Lkas_TsrSpeed_Display_Clu"]
    else:
      speed_limit = 0

    if speed_limit in (0, 255) and FPCP.fpFlags & HyundaiFrogPilotFlags.NAV_MSG:
      speed_limit = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]

  if speed_limit not in (0, 255):
    return speed_limit
  else:
    return 0


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.cruise_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)

    self.gear_msg_canfd = "GEAR_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS else \
                          "GEAR_ALT_2" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS_2 else \
                          "GEAR_SHIFTER"
    if CP.carFingerprint in CANFD_CAR:
      self.shifter_values = can_define.dv[self.gear_msg_canfd]["GEAR"]
    elif self.CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    self.accelerator_msg_canfd = "ACCELERATOR" if CP.flags & HyundaiFlags.EV else \
                                 "ACCELERATOR_ALT" if CP.flags & HyundaiFlags.HYBRID else \
                                 "ACCELERATOR_BRAKE_ALT"
    self.cruise_btns_msg_canfd = "CRUISE_BUTTONS_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else \
                                 "CRUISE_BUTTONS"
    self.is_metric = False
    self.buttons_counter = 0

    self.cruise_info = {}

    # On some cars, CLU15->CF_Clu_VehicleSpeed can oscillate faster than the dash updates. Sample at 5 Hz
    self.cluster_speed = 0
    self.cluster_speed_counter = CLUSTER_SAMPLE_RATE

    self.params = CarControllerParams(CP)

    # FrogPilot variables
    self.main_enabled = False

    self.active_mode = 0
    self.drive_mode_prev = 0

  def update(self, cp, cp_cam, frogpilot_toggles):
    if self.CP.carFingerprint in CANFD_CAR:
      return self.update_canfd(cp, cp_cam, frogpilot_toggles)

    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()
    cp_cruise = cp_cam if self.CP.carFingerprint in CAMERA_SCC_CAR else cp
    self.is_metric = cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] == 0
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.wheelSpeeds.fl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    self.cluster_speed_counter += 1
    if self.cluster_speed_counter > CLUSTER_SAMPLE_RATE:
      self.cluster_speed = cp.vl["CLU15"]["CF_Clu_VehicleSpeed"]
      self.cluster_speed_counter = 0

      # Mimic how dash converts to imperial.
      # Sorento is the only platform where CF_Clu_VehicleSpeed is already imperial when not is_metric
      # TODO: CGW_USM1->CF_Gway_DrLockSoundRValue may describe this
      if not self.is_metric and self.CP.carFingerprint not in (CAR.KIA_SORENTO,):
        self.cluster_speed = math.floor(self.cluster_speed * CV.KPH_TO_MPH + CV.KPH_TO_MPH)

    ret.vEgoCluster = self.cluster_speed * speed_conv

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0

    # cruise state
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.available = self.main_enabled
      ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False
    else:
      ret.cruiseState.available = cp_cruise.vl["SCC11"]["MainMode_ACC"] == 1
      ret.cruiseState.enabled = cp_cruise.vl["SCC12"]["ACCMode"] != 0
      ret.cruiseState.standstill = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 4.
      ret.cruiseState.nonAdaptive = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 2.  # Shows 'Cruise Control' on dash
      ret.cruiseState.speed = cp_cruise.vl["SCC11"]["VSetDis"] * speed_conv

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverOverride"] == 2  # 2 includes regen braking by user on HEV/EV
    ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    ret.espDisabled = cp.vl["TCS11"]["TCS_PAS"] == 1
    ret.accFaulted = cp.vl["TCS13"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED

    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      if self.CP.flags & HyundaiFlags.HYBRID:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    elif self.CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if not self.CP.openpilotLongitudinalControl:
      aeb_src = "FCA11" if self.CP.flags & HyundaiFlags.USE_FCA.value else "SCC12"
      aeb_sig = "FCA_CmdAct" if self.CP.flags & HyundaiFlags.USE_FCA.value else "AEB_CmdAct"
      aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
      scc_warning = cp_cruise.vl["SCC12"]["TakeOverReq"] == 1  # sometimes only SCC system shows an FCW
      aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src][aeb_sig] != 0
      ret.stockFcw = (aeb_warning or scc_warning) and not aeb_braking
      ret.stockAeb = aeb_warning and aeb_braking

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    # save the entire LKAS11 and CLU11
    self.lkas11 = copy.copy(cp_cam.vl["LKAS11"])
    self.clu11 = copy.copy(cp.vl["CLU11"])
    self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    self.prev_main_buttons = self.main_buttons[-1]
    self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])
    if self.prev_main_buttons == 0 and self.main_buttons[-1] != 0:
      self.main_enabled = not self.main_enabled

    # FrogPilot CarState functions
    fp_ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"])

    if self.FPCP.fpFlags & HyundaiFrogPilotFlags.LKAS12 or self.FPCP.fpFlags & HyundaiFrogPilotFlags.NAV_MSG:
      fp_ret.dashboardSpeedLimit = calculate_speed_limit(self.CP, self.FPCP, cp, cp_cam) * speed_conv

    self.prev_distance_button = self.distance_button
    self.distance_button = self.cruise_buttons[-1] == Buttons.GAP_DIST

    self.lkas_previously_enabled = self.lkas_enabled
    if self.FPCP.fpFlags & HyundaiFrogPilotFlags.CAN_LFA_BTN:
      self.lkas_enabled = cp.vl["BCM_PO_11"]["LFA_Pressed"]

    return ret, fp_ret

  def update_canfd(self, cp, cp_cam, frogpilot_toggles):
    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()

    self.is_metric = cp.vl["CRUISE_BUTTONS_ALT"]["DISTANCE_UNIT"] != 1
    speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    if self.CP.flags & (HyundaiFlags.EV | HyundaiFlags.HYBRID):
      offset = 255. if self.CP.flags & HyundaiFlags.EV else 1023.
      ret.gas = cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL"] / offset
      ret.gasPressed = ret.gas > 1e-5
    else:
      ret.gasPressed = bool(cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL_PRESSED"])

    ret.brakePressed = cp.vl["TCS"]["DriverBraking"] == 1

    ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR"] == 1
    ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT"] == 0

    gear = cp.vl[self.gear_msg_canfd]["GEAR"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    # TODO: figure out positions
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_1"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_2"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_3"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_4"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.wheelSpeeds.fl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]
    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS"]["LKA_FAULT"] != 0

    # TODO: alt signal usage may be described by cp.vl['BLINKERS']['USE_ALT_LAMP']
    left_blinker_sig, right_blinker_sig = "LEFT_LAMP", "RIGHT_LAMP"
    if self.CP.carFingerprint == CAR.HYUNDAI_KONA_EV_2ND_GEN:
      left_blinker_sig, right_blinker_sig = "LEFT_LAMP_ALT", "RIGHT_LAMP_ALT"
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["BLINKERS"][left_blinker_sig],
                                                                      cp.vl["BLINKERS"][right_blinker_sig])
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FL_INDICATOR"] != 0
      ret.rightBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FR_INDICATOR"] != 0

    # cruise state
    # CAN FD cars enable on main button press, set available if no TCS faults preventing engagement
    if self.CP.openpilotLongitudinalControl:
      ret.cruiseState.available = self.main_enabled
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.enabled = cp.vl["TCS"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
    else:
      cp_cruise_info = cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else cp
      ret.cruiseState.available = cp_cruise_info.vl["SCC_CONTROL"]["MainMode_ACC"] == 1
      ret.cruiseState.enabled = cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] in (1, 2)
      ret.cruiseState.standstill = cp_cruise_info.vl["SCC_CONTROL"]["CRUISE_STANDSTILL"] == 1
      ret.cruiseState.speed = cp_cruise_info.vl["SCC_CONTROL"]["VSetDis"] * speed_factor
      self.cruise_info = copy.copy(cp_cruise_info.vl["SCC_CONTROL"])

    # Manual Speed Limit Assist is a feature that replaces non-adaptive cruise control on EV CAN FD platforms.
    # It limits the vehicle speed, overridable by pressing the accelerator past a certain point.
    # The car will brake, but does not respect positive acceleration commands in this mode
    # TODO: find this message on ICE & HYBRID cars + cruise control signals (if exists)
    if self.CP.flags & HyundaiFlags.EV:
      ret.cruiseState.nonAdaptive = cp.vl["MANUAL_SPEED_LIMIT_ASSIST"]["MSLA_ENABLED"] == 1

    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"])
    self.prev_main_buttons = self.main_buttons[-1]
    self.main_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["ADAPTIVE_CRUISE_MAIN_BTN"])
    if self.prev_main_buttons == 0 and self.main_buttons[-1] != 0:
      self.main_enabled = not self.main_enabled
    self.buttons_counter = cp.vl[self.cruise_btns_msg_canfd]["COUNTER"]
    ret.accFaulted = cp.vl["TCS"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED

    if self.CP.flags & HyundaiFlags.CANFD_HDA2:
      self.hda2_lfa_block_msg = copy.copy(cp_cam.vl["CAM_0x362"] if self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING
                                          else cp_cam.vl["CAM_0x2a4"])

    # FrogPilot CarState functions
    self.params = CarControllerParams(self.CP, ret.vEgoRaw, frogpilot_toggles)

    fp_ret.brakeLights = bool(cp.vl["TCS"]["DriverBraking"])

    if self.FPCP.fpFlags & HyundaiFrogPilotFlags.NAV_MSG:
      fp_ret.dashboardSpeedLimit = calculate_speed_limit(self.CP, self.FPCP, cp, cp_cam) * speed_factor

    self.prev_distance_button = self.distance_button
    self.distance_button = self.cruise_buttons[-1] == Buttons.GAP_DIST

    drive_mode = cp.vl["DRIVE_MODE"]["DRIVE_MODE2"]

    if drive_mode != 0 and drive_mode != self.drive_mode_prev:
      self.active_mode = drive_mode if drive_mode in (2, 3) else 1
      self.drive_mode_prev = drive_mode

    fp_ret.ecoGear = self.active_mode == 2
    fp_ret.sportGear = self.active_mode == 3

    self.lkas_previously_enabled = self.lkas_enabled
    self.lkas_enabled = cp.vl[self.cruise_btns_msg_canfd]["LFA_BTN"]

    return ret, fp_ret

  def get_can_parser(self, CP, FPCP):
    if CP.carFingerprint in CANFD_CAR:
      return self.get_can_parser_canfd(CP, FPCP)

    messages = [
      # address, frequency
      ("MDPS12", 50),
      ("TCS11", 100),
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("CLU15", 5),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
      ("SAS11", 100),
    ]

    if not CP.openpilotLongitudinalControl and CP.carFingerprint not in CAMERA_SCC_CAR:
      messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
      if CP.flags & HyundaiFlags.USE_FCA.value:
        messages.append(("FCA11", 50))

    if CP.enableBsm:
      messages.append(("LCA11", 50))

    if CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      messages.append(("E_EMS11", 50))
    else:
      messages += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      messages.append(("ELECT_GEAR", 20))
    elif CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      pass
    elif CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      messages.append(("TCU12", 100))
    else:
      messages.append(("LVR12", 100))

    if FPCP.fpFlags & HyundaiFrogPilotFlags.CAN_LFA_BTN:
      messages.append(("BCM_PO_11", 50))

    if FPCP.fpFlags & HyundaiFrogPilotFlags.NAV_MSG:
      messages.append(("Navi_HU", 5))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP, FPCP):
    if CP.carFingerprint in CANFD_CAR:
      return CarState.get_cam_can_parser_canfd(CP, FPCP)

    messages = [
      ("LKAS11", 100),
    ]

    if not CP.openpilotLongitudinalControl and CP.carFingerprint in CAMERA_SCC_CAR:
      messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.flags & HyundaiFlags.USE_FCA.value:
        messages.append(("FCA11", 50))

    if FPCP.fpFlags & HyundaiFrogPilotFlags.LKAS12:
      messages.append(("LKAS12", 10))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)

  def get_can_parser_canfd(self, CP, FPCP):
    messages = [
      (self.gear_msg_canfd, 100),
      (self.accelerator_msg_canfd, 100),
      ("WHEEL_SPEEDS", 100),
      ("STEERING_SENSORS", 100),
      ("MDPS", 100),
      ("TCS", 50),
      ("CRUISE_BUTTONS_ALT", 50),
      ("BLINKERS", 4),
      ("DOORS_SEATBELTS", 4),
      ("DRIVE_MODE", 0),
    ]

    if CP.flags & HyundaiFlags.EV:
      messages += [
        ("MANUAL_SPEED_LIMIT_ASSIST", 10),
      ]

    if not (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
      messages += [
        ("CRUISE_BUTTONS", 50)
      ]

    if CP.enableBsm:
      messages += [
        ("BLINDSPOTS_REAR_CORNERS", 20),
      ]

    if not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and not CP.openpilotLongitudinalControl:
      messages += [
        ("SCC_CONTROL", 50),
      ]

    if CP.flags & HyundaiFlags.CANFD_HDA2 and FPCP.fpFlags & HyundaiFrogPilotFlags.NAV_MSG:
      messages.append(("CLUSTER_SPEED_LIMIT", 10))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).ECAN)

  @staticmethod
  def get_cam_can_parser_canfd(CP, FPCP):
    messages = []
    if CP.flags & HyundaiFlags.CANFD_HDA2:
      block_lfa_msg = "CAM_0x362" if CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING else "CAM_0x2a4"
      messages += [(block_lfa_msg, 20)]
    elif CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
      messages += [
        ("SCC_CONTROL", 50),
      ]

    if not (CP.flags & HyundaiFlags.CANFD_HDA2) and FPCP.fpFlags & HyundaiFrogPilotFlags.NAV_MSG:
      messages.append(("CLUSTER_SPEED_LIMIT", 10))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).CAM)
