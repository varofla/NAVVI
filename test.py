import utime as time
from machine import Timer
from machine import Pin, ADC, PWM

from lib.TMC_2209_StepperDriver import Loglevel, MovementAbsRel
from lib.NAVVI_TMC import *
from lib.NAVVI_PIN import *

def map_py(x,input_min,input_max,output_min,output_max):
    return int((x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min) #map()함수 정의.


class NAVVI() :
    POSITION_HOME = {
        'yaw' : 90,
        'pitch' : 135,
        'stepper' : 0
    }

    _position_yaw = POSITION_HOME['yaw']
    _position_pitch = POSITION_HOME['pitch']
    _position_stepper = POSITION_HOME['stepper']

    def __init__(self) :
        self.VR1 = ADC(PIN_VR1) # 테스트
        self.VR2 = ADC(PIN_VR2) # 테스트
        self.VR3 = ADC(PIN_VR3) # 테스트

        # 서보모터 초기화
        self.YAW_SERVO = PWM(Pin(PIN_SERVO_YAW))
        self.YAW_SERVO.freq(50)
        self.PITCH_SERVO = PWM(Pin(PIN_SERVO_PITCH))
        self.PITCH_SERVO.freq(50)

        # 스텝모터 초기화
        self.TMC = TMC_2209_NAVVI(PIN_TMC_STEP, PIN_TMC_DIR, PIN_TMC_EN, PIN_TMC_DIAG)
        self.TMC.set_log_level(Loglevel.error)
        self.TMC.set_movement_abs_rel(MovementAbsRel.absolute)
        self.TMC.set_direction_reg(False)
        self.TMC.set_vsense(True)
        self.TMC.set_current(600)
        self.TMC.set_iscale_analog(True)
        self.TMC.set_interpolation(True)
        self.TMC.set_spread_cycle(False)
        self.TMC.set_microstepping_resolution(2)
        self.TMC.set_internal_rsense(False)
        self.TMC.set_acceleration(2000)
        self.TMC.set_speed(10)
        self.TMC._maxSpeedHoming = 500
        self.TMC.set_motor_enabled(True)

        print("Init Finished!")
        self._home()


    def _test(self) :
        while True :
            pass

    # def _move(self, rail, yaw, pitch, speed=None) :
    #     print("_move")

    def _home(self) :
        self._move_servo_yaw(self.POSITION_HOME['yaw'])
        self._move_servo_pitch(self.POSITION_HOME['pitch'])
        self._position_stepper = self.POSITION_HOME['stepper']
        self.TMC.do_homing(1, 160)

    def _move_servo_yaw(self, where) :
        duty = map_py(where, 0, 180, 1640, 8190)
        self.YAW_SERVO.duty_u16(duty)
        self._position_yaw = where

    def _move_servo_pitch(self, where) :
        duty = map_py(where, 0, 180, 1640, 8190)
        self.PITCH_SERVO.duty_u16(duty)
        self._position_pitch = where

    def _move_stepper(self, where) :
        self._position_stepper = where

tset = NAVVI()
tset.TMC.set_motor_enabled(False)
