"""
Copyright 2020 LeMaRiva|Tech (Mauro Riva) info@lemariva.com
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import utime
from machine import Pin

class STEPPER:

    def __init__(self, device_config):
        self._In1 = Pin(device_config["In1"], Pin.OUT)
        self._In2 = Pin(device_config["In2"], Pin.OUT)
        self._In3 = Pin(device_config["In3"], Pin.OUT)
        self._In4 = Pin(device_config["In4"], Pin.OUT)
        self._number_of_steps = device_config["number_of_steps"] + 1
        self._max_speed = 60 * 1000 * 1000 / self._number_of_steps / device_config["max_speed"]
        self._step_number = 0
        self._last_step_time = 0
        self.set_speed(device_config["max_speed"]/2)

    def set_speed(self, speed):
        self._step_delay = 60 * 1000 * 1000 / self._number_of_steps / speed
        if self._step_delay < self._max_speed:
            self._step_delay = self._max_speed

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

    def release(self):
        self._In1.value(0)
        self._In2.value(0)
        self._In3.value(0)
        self._In4.value(0)

    def step(self, steps_to_move, speed=None, hold=True):
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

        if steps_left == 0 and not hold:
            self.release()
