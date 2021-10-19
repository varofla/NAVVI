from machine import Pin
import utime as time

import os

from TMC_2209_StepperDriver import *
import TMC_2209_reg as reg
from TMC_2209_uart import TMC_UART

class TMC_2209_NAVVI(TMC_2209) :
    def __init__(self, pin_step, pin_dir, pin_en, pin_diag, baudrate=115200):
        super().__init__(pin_step, pin_dir, pin_en, baudrate=baudrate)
        self._pin_diag = Pin(pin_diag, Pin.IN, pull=Pin.PULL_DOWN)

        self._stepps_per_mm = 80 # mm당 펄스수


#-----------------------------------------------------------------------
# <Function names should be lowercase, with words separated by underscores as necessary to improve readability>
# 사실 그냥 보기 싫어서 이렇게함 :)
#-----------------------------------------------------------------------
    def set_log_level(self, loglevel):
        return super().setLoglevel(loglevel)

    def set_movement_abs_rel(self, movement_abs_rel):
        return super().setMovementAbsRel(movement_abs_rel)

    def set_direction_reg(self, direction):
        return super().setDirection_reg(direction)

    def set_vsense(self, en):
        return super().setVSense(en)

    def set_current(self, run_current, hold_current_multiplier=0.5, hold_current_delay=10, Vref=1.2):
        return super().setCurrent(run_current, hold_current_multiplier=hold_current_multiplier, hold_current_delay=hold_current_delay, Vref=Vref)

    def set_iscale_analog(self, en):
        return super().setIScaleAnalog(en)

    def set_interpolation(self, en):
        return super().setInterpolation(en)

    def set_spread_cycle(self, en_spread):
        return super().setSpreadCycle(en_spread)

    def set_microstepping_resolution(self, msres):
        return super().setMicrosteppingResolution(msres)

    def set_internal_rsense(self, en):
        return super().setInternalRSense(en)

    def set_acceleration(self, acceleration):
        return super().setAcceleration(acceleration)

    def set_stallguard_threshold(self, threshold):
        return super().setStallguard_Threshold(threshold)
    
    def set_coolstep_threshold(self, threshold):
        return super().setCoolStep_Threshold(threshold)

    def set_direction_pin(self, direction):
        return super().setDirection_pin(direction)

    def set_motor_enabled(self, en):
        return super().setMotorEnabled(en)


#-----------------------------------------------------------------------
# 속도 설정 (mm/s)
#-----------------------------------------------------------------------
    def set_speed(self, speed) :
        self._speed = speed


#-----------------------------------------------------------------------
# 펄스 생성
#-----------------------------------------------------------------------
    def make_step(self) :
        self._pin_step.on()
        time.sleep_us(1)
        self._pin_step.off()
        time.sleep_us(1)

    def do_homing(self, direction, threshold=None) :
        self.set_stallguard_threshold(threshold)
        self.set_coolstep_threshold(2000)
        self.set_direction_pin(direction)
        g = int(1000000 / self._stepps_per_mm / self._speed)
        while True :
            self.make_step()
            time.sleep_us(g)
            if self._pin_diag.value() == 1 :
                break

