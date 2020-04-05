import utime
from machine import Pin

class STEPPER:

    def __init__(self, pins, speed=250):
        self._pins = pins
        self._In1 = Pin(self._pins["In1"], Pin.OUT)
        self._In2 = Pin(self._pins["In2"], Pin.OUT)
        self._In3 = Pin(self._pins["In3"], Pin.OUT)
        self._In4 = Pin(self._pins["In4"], Pin.OUT)
        self._delay = speed
        self._number_of_steps = 205
        self._step_number = 0
        self._last_step_time = 0
        self.set_speed(speed)

    def set_speed(self, speed):
        self._step_delay = 60 * 1000 * 1000 / self._number_of_steps / speed
        if self._step_delay < 1200:
            self._step_delay = 1200

    def step_motor(self, step):
        if step == 0:  #1010
            self._In1.value(1)
            self._In2.value(0)
            self._In3.value(1)
            self._In4.value(0)
        elif step == 1: #0110
            self._In1.value(0)
            self._In2.value(1)
            self._In3.value(1)
            self._In4.value(0)
        elif step == 2: #0101
            self._In1.value(0)
            self._In2.value(1)
            self._In3.value(0)
            self._In4.value(1)
        elif step == 3: #1001
            self._In1.value(1)
            self._In2.value(0)
            self._In3.value(0)
            self._In4.value(1)

    def stop(self):
        self._In1.value(0)
        self._In2.value(0)
        self._In3.value(0)
        self._In4.value(0)

    def step(self, steps_to_move, speed=None, stop=True):
        if speed is not None:
            self.set_speed(speed)

        steps_left = abs(steps_to_move)
        
        if steps_to_move > 0:
            direction = 1
        else:
            direction = 0

        while steps_left > 0:
            now = utime.ticks_us()
            if now - self._last_step_time >= self._step_delay:
                self._last_step_time = now
                if direction == 1:
                    self._step_number += 1

                if direction == 0:
                    if self._step_number == 0:
                        self._step_number == steps_left

                    self._step_number -= 1

                self.step_motor(self._step_number % 4)
                steps_left -= 1

        if self._step_number == steps_left:
            self._step_number = 0

        if steps_left == 0 and stop:
            self.stop()
