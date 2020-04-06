
import machine
import utime

epoch_offset = 946684800

class PMSA003:
    def __init__(self, uart, pins):
        
        self._uart = uart
        self._pins = pins
        
        self._set = machine.Pin(self._pins["set"], machine.Pin.OUT, value=0)
        self._rst = machine.Pin(self._pins["rst"], machine.Pin.OUT, value=0)
        self._uart.init(tx=self._pins["tx"], rx=self._pins["rx"])
        self.power_off()

    def wake_up(self):
        self._set(True)
        self._rst(True)
        # sleep for 7 seconds to initialize the sensor properly
        self._set_normal()
        utime.sleep_ms(7000)
        # warning init
        for idx in range(10):
            data = self.measurements
            utime.sleep_ms(500)

    def _set_idle(self):
        idelcmd = b'\x42\x4d\xe4\x00\x00\x01\x73'
        ary = bytearray(idelcmd)
        self._uart.write(ary)

    def _set_normal(self):
        normalcmd = b'\x42\x4d\xe4\x00\x01\x01\x74'
        ary = bytearray(normalcmd)
        self._uart.write(ary)

    def power_off(self):
        self._set_idle()
        self._set(False)
        self._rst(False)

    def reset(self):
        self._rst(False)
        utime.sleep_ms(2000)
        self._rst(True)

    @property
    def measurements(self):
        # flush the buffer to read fresh data
        ret_data = None
        self._wait_for_data(32)

        while self._uart.read(1) != b'\x42':
            machine.idle()

        if self._uart.read(1) == b'\x4D':
            self._wait_for_data(30)
            try:
                self._data = self._uart.read(30)
                if self._data:
                    ret_data = self._PMdata()
            except ValueError as e:
                print('error reading frame: {}'.format(e.message))
                pass
                
        return ret_data

    def _wait_for_data(self, byte_count):
        u = self._uart.any()
        while u < byte_count:
            u = self._uart.any()
            # 32*8*1000/9600 (32 bytes @9600kbps)
            # but let's assume byte is 10 bits to skip complex math
            utime.sleep_ms(10)

    def _PMdata(self):
        d = {}
        check = False
        # check data
        control_sum = 0x42 + 0x4d
        for b in range(len(self._data)-2):
            control_sum += self._data[b]

        control_sum_data = self._data[28] * 256 + self._data[29]
        print()
        if control_sum == control_sum_data:
            check = True

        d['time'] = utime.time() + epoch_offset
        d['cpm10'] = self._data[4] * 256 + self._data[5]
        d['cpm25'] = self._data[6] * 256 + self._data[7]
        d['cpm100'] = self._data[8] * 256 + self._data[9]
        d['apm10'] = self._data[10] * 256 + self._data[11]
        d['apm25'] = self._data[12] * 256 + self._data[13]
        d['apm100'] = self._data[14] * 256 + self._data[15]

        return [check, d]