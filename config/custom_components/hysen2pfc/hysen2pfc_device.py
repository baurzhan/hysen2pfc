"""
Hysen Controller for 2 Pipe Fan Coil Interface
Hysen HY03AC-1-Wifi device and derivative
http://www.xmhysen.com/products_detail/productId=201.html
"""

from broadlink import device
import logging
from PyCRC.CRC16 import CRC16

REQUIREMENTS = ['broadlink==0.9.0']

_LOGGER = logging.getLogger(__name__)

HYSEN_2PFC_REMOTE_LOCK_OFF      = 0
HYSEN_2PFC_REMOTE_LOCK_ON       = 1

HYSEN_2PFC_KEY_ALL_UNLOCKED     = 0
HYSEN_2PFC_KEY_POWER_UNLOCKED   = 1
HYSEN_2PFC_KEY_ALL_LOCKED       = 2

HYSEN_2PFC_POWER_OFF            = 0
HYSEN_2PFC_POWER_ON             = 1

HYSEN_2PFC_VALVE_OFF            = 0
HYSEN_2PFC_VALVE_ON             = 1

HYSEN_2PFC_HYSTERESIS_HALVE     = 0
HYSEN_2PFC_HYSTERESIS_WHOLE     = 1

HYSEN_2PFC_CALIBRATION_MIN      = -5.0
HYSEN_2PFC_CALIBRATION_MAX      = 5.0

HYSEN_2PFC_FAN_LOW              = 1
HYSEN_2PFC_FAN_MEDIUM           = 2
HYSEN_2PFC_FAN_HIGH             = 3
HYSEN_2PFC_FAN_AUTO             = 4

HYSEN_2PFC_MODE_FAN             = 1
HYSEN_2PFC_MODE_COOL            = 2
HYSEN_2PFC_MODE_HEAT            = 3

HYSEN_2PFC_FAN_CONTROL_ON       = 0
HYSEN_2PFC_FAN_CONTROL_OFF      = 1

HYSEN_2PFC_FROST_PROTECTION_OFF = 0
HYSEN_2PFC_FROST_PROTECTION_ON  = 1

HYSEN_2PFC_SCHEDULE_TODAY       = 0
HYSEN_2PFC_SCHEDULE_12345_67    = 1
HYSEN_2PFC_SCHEDULE_123456_7    = 2
HYSEN_2PFC_SCHEDULE_1234567     = 3

HYSEN_2PFC_PERIOD_DISABLED      = 0
HYSEN_2PFC_PERIOD_ENABLED       = 1

HYSEN_2PFC_COOLING_MAX_TEMP     = 40
HYSEN_2PFC_COOLING_MIN_TEMP     = 10
HYSEN_2PFC_HEATING_MAX_TEMP     = 40
HYSEN_2PFC_HEATING_MIN_TEMP     = 10

HYSEN_2PFC_MAX_TEMP             = 40
HYSEN_2PFC_MIN_TEMP             = 10

HYSEN_2PFC_DEFAULT_TARGET_TEMP  = 22
HYSEN_2PFC_DEFAULT_CALIBRATION  = 0.0

class Hysen2PipeFanCoilDevice(device):
    
    def __init__ (self, host, mac, devtype, timeout):
        device.__init__(self, host, mac, devtype, timeout)
        self.type = "Hysen 2 Pipe Fan Coil Controller"
        self._host = host[0]
        
        self.remote_lock = HYSEN_2PFC_REMOTE_LOCK_OFF
        self.key_lock = HYSEN_2PFC_KEY_ALL_UNLOCKED
        self.valve_state = HYSEN_2PFC_VALVE_OFF
        self.power_state = HYSEN_2PFC_POWER_ON
        self.operation_mode = HYSEN_2PFC_MODE_FAN
        self.fan_mode = HYSEN_2PFC_FAN_LOW
        self.room_temp = 0        
        self.target_temp = HYSEN_2PFC_DEFAULT_TARGET_TEMP
        self.hysteresis = HYSEN_2PFC_HYSTERESIS_WHOLE
        self.calibration = HYSEN_2PFC_DEFAULT_CALIBRATION
        self.cooling_max_temp = HYSEN_2PFC_MAX_TEMP
        self.cooling_min_temp = HYSEN_2PFC_MIN_TEMP
        self.heating_max_temp = HYSEN_2PFC_MAX_TEMP
        self.heating_min_temp = HYSEN_2PFC_MIN_TEMP
        self.fan_control = HYSEN_2PFC_FAN_CONTROL_ON
        self.frost_protection = HYSEN_2PFC_FROST_PROTECTION_ON
        self.clock_hour = 0
        self.clock_min = 0
        self.clock_sec = 0
        self.clock_weekday = 1
        self.unknown = 0
        self.schedule = HYSEN_2PFC_SCHEDULE_TODAY
        self.period1_on_enabled = HYSEN_2PFC_PERIOD_DISABLED
        self.period1_on_hour = 0
        self.period1_on_min = 0
        self.period1_off_enabled = HYSEN_2PFC_PERIOD_DISABLED
        self.period1_off_hour = 0
        self.period1_off_min = 0
        self.period2_on_enabled = HYSEN_2PFC_PERIOD_DISABLED
        self.period2_on_hour = 0
        self.period2_on_min = 0
        self.period2_off_enabled = HYSEN_2PFC_PERIOD_DISABLED
        self.period2_off_hour = 0
        self.period2_off_min = 0
        self.time_valve_on = 0

    # Send a request
    # Returns decrypted payload
    # Device's memory data is structured in an array of bytes, word (2 bytes) aligned
    # input_payload should be a bytearray
    # There are three known different request types (commands)
    # 1. write a word (2 bytes) at a given position (position counted in words)
    #    Command example
    #      0x01, 0x06, 0x00, 0x04, 0x28, 0x0A
    #      first byte 
    #        0x01 - header
    #      a byte representing command type
    #        0x06 - write word at a given position 
    #      an unknown byte (always 0x00)
    #        0x00
    #      a byte which is memory data word's index
    #        0x04 - the fifth word in memory data 
    #      the bytes to be written
    #        0x28, 0x0A - cooling max_temp (sh1) = 40, cooling min_temp (sl1) = 10
    #    No error confirmation response 
    #      0x01, 0x06, 0x00, 0x04, 0x28, 0x0A
    #      first byte 
    #        0x01 - header
    #      a byte representing command type
    #        0x06 - write word at a given position 
    #      an unknown byte (always 0x00)
    #        0x00
    #      a byte which is memory data word's index
    #        0x04 - the fifth word in memory data 
    #      the bytes written
    #        0x28, 0x0A - cooling max_temp (sh1) = 40, cooling min_temp (sl1) = 10
    # 2. write several words (multiple of 2 bytes) at a given position (position counted in words)
    #    Command example
    #      0x01, 0x10, 0x00, 0x07, 0x00, 0x02, 0x04, 0x08, 0x14, 0x10, 0x02
    #      first byte 
    #        0x01 - header
    #      a byte representing command type
    #        0x10 - write several words at a given position 
    #      an unknown byte (always 0x00)
    #        0x00
    #      a byte representing memory data word's index
    #        0x07 - the eighth word in memory data
    #      an unknown byte (always 0x00)
    #        0x00
    #      a byte representing the number of words (2 bytes) to be written
    #        0x02 - 2 words
    #      a byte representing the number of bytes to be written (previous word multiplied by 2)
    #        0x04 - 4 bytes
    #      the bytes to be written
    #        0x08, 0x14, 0x10, 0x02 - hour = 8, min = 20, sec = 10, weekday = 2 = Tuesday
    #    No error confirmation response
    #      0x01, 0x10, 0x00, 0x08, 0x00, 0x02
    #      first byte 
    #        0x01 - header
    #      a byte representing command type
    #        0x10 - write several words at a given position 
    #      an unknown byte (always 0x00)
    #        0x00
    #      a byte representing memory data word's index
    #        0x07 - the eighth word in memory data
    #      an unknown byte (always 0x00)
    #        0x00
    #      a byte representing the number of words (2 bytes) written
    #        0x02 - 2 words
    # 3. read memory data from a given position (position counted in words)
    #    Command example
    #      0x01, 0x03, 0x00, 0x07, 0x00, 0x02
    #      first byte 
    #        0x01 - header
    #      a byte representing command type
    #        0x03 - read several words at a given position 
    #      an unknown byte (always 0x00)
    #        0x00
    #      a byte representing memory data word's index
    #        0x07 - the eighth word in memory data
    #      a byte representing the number of words to be read
    #        0x02 - 2 words
    #    No error confirmation response
    #      0x01, 0x03, 0x04, 0x08, 0x14, 0x10, 0x02
    #      first byte 
    #        0x01 - header
    #      a byte representing command type
    #        0x03 - read command 
    #      a byte representing the number of bytes read
    #        0x04 - 4 bytes 
    #      the memory data bytes
    #        0x08, 0x14, 0x10, 0x02 - hour = 8, min = 20, sec = 10, weekday = 2 = Tuesday
    # Error responses for any command type
    #      0x01, 0xXX, 0xYY where
    #      first byte 
    #        0x01 - header
    #      second byte - Most significant bit 1 (error), last significant bits is the command type
    #        e.g. 0x90 - error in command type 0x10
    #      third byte
    #        0xYY - error type
    #        0x01 - Unknown command
    #        0x02 - Length missing or too big
    #        0x03 - Wrong length
    # New behavior: raises a ValueError if the device response indicates an error or CRC check fails
    # The function prepends length (2 bytes) and appends CRC
    def send_request(self, input_payload):
        for i in range(1, 3):
            crc = CRC16(modbus_flag = True).calculate(bytes(input_payload))
            if crc == None:
                _LOGGER.error("[%s] CRC16 returned None, step %s.", self._host, i)
            else:
                break
                
        # first byte is length, +2 for CRC16
        request_payload = bytearray([len(input_payload) + 2,0x00])
        request_payload.extend(input_payload)

        # append CRC
        request_payload.append(crc & 0xFF)
        request_payload.append((crc >> 8) & 0xFF)

        # send to device
        response = self.send_packet(0x6a, request_payload)

        # check for error
        err = response[0x22] | (response[0x23] << 8)
        if err:
            raise ValueError('broadlink_response_error',err)
      
        response_payload = bytearray(self.decrypt(bytes(response[0x38:])))
        
        # experimental check on CRC in response (first 2 bytes are len, and trailing bytes are crc)
        response_payload_len = response_payload[0]
        if response_payload_len + 2 > len(response_payload):
            raise ValueError('hysen_response_error','first byte of response is not length')
        crc = CRC16(modbus_flag=True).calculate(bytes(response_payload[2:response_payload_len]))
        if (response_payload[response_payload_len] == crc & 0xFF) and \
           (response_payload[response_payload_len+1] == (crc >> 8) & 0xFF):
            return_payload = response_payload[2:response_payload_len]
        else:
            raise ValueError('hysen_response_error','CRC check on response failed')
            
        # check if return response is right
        if (input_payload[0:2] == bytearray([0x01, 0x06])) and \
           (input_payload != return_payload):
            _LOGGER.debug("[%s] request %s response %s",
                self._host,
                ' '.join(format(x, '02x') for x in bytearray(input_payload)),
                ' '.join(format(x, '02x') for x in bytearray(return_payload)))
            raise ValueError('hysen_response_error','response is wrong')
        elif (input_payload[0:2] == bytearray([0x01, 0x10])) and \
             (input_payload[0:6] != return_payload):
            _LOGGER.debug("[%s] request %s response %s",
                self._host,
                ' '.join(format(x, '02x') for x in bytearray(input_payload)),
                ' '.join(format(x, '02x') for x in bytearray(return_payload)))
            raise ValueError('hysen_response_error','response is wrong')
        elif (input_payload[0:2] == bytearray([0x01, 0x03])) and \
             ((input_payload[0:2] != return_payload[0:2]) or \
             ((2 * input_payload[5]) != return_payload[2]) or \
             ((2 * input_payload[5]) != len(return_payload[3:]))):
            _LOGGER.debug("[%s] request %s response %s",
                self._host,
                ' '.join(format(x, '02x') for x in bytearray(input_payload)),
                ' '.join(format(x, '02x') for x in bytearray(return_payload)))
            raise ValueError('hysen_response_error','response is wrong')
        else:
            return return_payload


    # set lock and power
    # 0x01, 0x06, 0x00, 0x00, 0xrk, 0x0p
    # r = Remote lock, 0 = Off, 1 = On
    # k = Key lock (Loc), 0 = Unlocked, 1 = All buttons locked except Power, 2 = All buttons locked
    # p = Power State, 0 = Power off, 1 = Power on
    # If remote lock is Off then key lock has to be unlocked otherwise after any subsequent command we will get remote lock on
    def set_lock_power(self, remote_lock, key_lock, power_state):
        _request = bytearray([0x01, 0x06, 0x00, 0x00])
        _request.append((remote_lock << 4) + key_lock)
        _request.append(power_state)
        self.send_request(_request)

    def set_remote_lock(self, key_lock):
        if key_lock not in [
            HYSEN_2PFC_KEY_ALL_UNLOCKED,
            HYSEN_2PFC_KEY_POWER_UNLOCKED,
            HYSEN_2PFC_KEY_ALL_LOCKED]:
            raise ValueError(
                'Can\'t set key lock (%s) outside device\'s admitted values (%s), (%s), (%s).' % ( \
                key_lock,
                HYSEN_2PFC_KEY_ALL_UNLOCKED,
                HYSEN_2PFC_KEY_POWER_UNLOCKED,
                HYSEN_2PFC_KEY_ALL_LOCKED))
        self.get_device_status()
        if key_lock == HYSEN_2PFC_KEY_ALL_UNLOCKED:
            self.remote_lock = HYSEN_2PFC_REMOTE_LOCK_OFF;
        else:
            self.remote_lock = HYSEN_2PFC_REMOTE_LOCK_ON;
        self.set_lock_power(
            self.remote_lock,
            key_lock,
            self.power_state)

    def set_power(self, power):
        if power not in [
            HYSEN_2PFC_POWER_OFF,
            HYSEN_2PFC_POWER_ON]:
            raise ValueError(
                'Can\'t set power (%s) outside device\'s admitted values (%s), (%s).' % ( \
                power,
                HYSEN_2PFC_POWER_OFF,
                HYSEN_2PFC_POWER_ON))
        self.get_device_status()
        self.set_lock_power(
            self.remote_lock,
            self.key_lock,
            power)

    # set mode and fan
    # 0x01, 0x06, 0x00, 0x01, Mod, Fs
    # Mod = Operation mode, 0x01 = Ventilation, 0x02 = Cooling, 0x03 = Heating
    # Fs = Fan speed, 0x01 = Low, 0x02 = Medium, 0x03 = High, 0x04 = Auto
    # Note: Ventilation and fan auto are mutual exclusive (e.g. Mod = 0x01 and Fs = 0x04 is not allowed)
    #       The calling method should deal with that 
    def set_mode_fan(self, operation_mode, fan_mode):
        _request = bytearray([0x01, 0x06, 0x00, 0x01])
        _request.append(operation_mode)
        _request.append(fan_mode)
        self.send_request(_request)

    def set_fan_mode(self, fan_mode):
        if fan_mode not in [
            HYSEN_2PFC_FAN_LOW,
            HYSEN_2PFC_FAN_MEDIUM,
            HYSEN_2PFC_FAN_HIGH,
            HYSEN_2PFC_FAN_AUTO]:
            raise ValueError(
                'Can\'t set fan_mode (%s) outside device\'s admitted values (%s), (%s), (%s), (%s).' % ( \
                fan_mode,
                HYSEN_2PFC_FAN_LOW,
                HYSEN_2PFC_FAN_MEDIUM,
                HYSEN_2PFC_FAN_HIGH,
                HYSEN_2PFC_FAN_AUTO))
        self.get_device_status()
        if (fan_mode == HYSEN_2PFC_FAN_AUTO) and \
           (self.operation_mode == HYSEN_2PFC_MODE_FAN):
            raise ValueError(
                'Can\'t have fan_mode \'auto\' and operation_mode \'fan_only\'.')
        self.set_mode_fan(
            self.operation_mode,
            fan_mode)
    
    def set_operation_mode(self, operation_mode):
        if operation_mode not in [
            HYSEN_2PFC_MODE_FAN,
            HYSEN_2PFC_MODE_COOL,
            HYSEN_2PFC_MODE_HEAT]:
            raise ValueError(
                'Can\'t set operation_mode (%s) outside device\'s admitted values (%s), (%s), (%s).' % ( \
                operation_mode,
                HYSEN_2PFC_MODE_FAN,
                HYSEN_2PFC_MODE_COOL,
                HYSEN_2PFC_MODE_HEAT))
        self.get_device_status()
        if (operation_mode == HYSEN_2PFC_MODE_FAN) and \
           (self.fan_mode == HYSEN_2PFC_FAN_AUTO):
            raise ValueError(
                'Can\'t have operation_mode \'fan_only\' and fan_mode \'auto\'.')
        self.set_mode_fan(
            operation_mode,
            self.fan_mode)
 
    # set target temperature
    # 0x01,0x06,0x00,0x02,0x00, Tt
    # Tt = Target temperature in degrees Celsius
    # confirmation response:
    # response 0x01,0x06,0x00,0x02,0x00,Tt
    # Note: The calling method should not do anything if in ventilation mode
    #       Check temp against Sh1, Sl1 for cooling and against Sh2, Sl2 for heating
    def set_target_temp(self, temp):
        self.get_device_status()
        if self.operation_mode == HYSEN_2PFC_MODE_FAN:
            raise ValueError(
                'Can\'t set a target temperature when operation_mode is \'fan_only\'.') 
        elif self.operation_mode == HYSEN_2PFC_MODE_HEAT:
            if (temp > self.heating_max_temp):
                raise ValueError(
                    'Can\'t set a heating target temperature (%s°) higher than maximum set (%s°).' % ( \
                    temp,
                    self.heating_max_temp))
            if temp < self.heating_min_temp:
                raise ValueError(
                    'Can\'t set a heating target temperature (%s°) lower than minimum set (%s°).' % ( \
                    temp,
                    self.heating_min_temp))
        else:
            if temp > self.cooling_max_temp:
                raise ValueError(
                    'Can\'t set a cooling target temperature (%s°) higher than maximum set (%s°).' % ( \
                    temp,
                    self.cooling_max_temp))
            if temp < self.cooling_min_temp:
                raise ValueError(
                    'Can\'t set a cooling target temperature (%s°) lower than minimum set (%s°).' % ( \
                    temp,
                    self.cooling_min_temp))
        _request = bytearray([0x01, 0x06, 0x00, 0x02])
        _request.append(0)
        _request.append(temp)
        self.send_request(_request)

    # set options
    # 0x01, 0x10, 0x00, 0x03, 0x00, 0x04, 0x08, Dif, Adj, Sh1, Sl1, Sh2, Sl2, Fan, Fre
    # Dif = Hysteresis, 0x00 = 0.5 degree Celsius, 0x01 = 1 degree Celsius
    # Adj = Temperature calibration -5~+5, 0.1 degree Celsius step 
    #       (e.g. -1 = 0xF6, -1.4 = 0xF2, 0 = 0x00, +1 = 0x0A, +1.2 = 0x0C, +2 = 0x14, etc.)
    # Sh1 = Cooling max. temperature
    # Sl1 = Cooling min. temperature
    # Sh2 = Heating max. temperature
    # Sl2 = Heating min. temperature
    # Fan = Fan coil control mode, 0x00 = Fan coil in control, 0x01 = Fan coil out of control
    # Fre = Frost Protection, 0x00 = On, 0x01 = Off
    # confirmation response:
    # payload 0x01,0x10,0x00,0x03,0x00,0x04
    def set_options(self, hysteresis, calibration, cooling_max_temp, cooling_min_temp, heating_max_temp, heating_min_temp, fan_control, frost_protection):
        # Truncate the fractional part to 1 digit 
        calibration = int(calibration * 10 // 1)
        # Convert to signed byte
        calibration = (0x100 + calibration) & 0xFF
        _request = bytearray([0x01, 0x10, 0x00, 0x03, 0x00, 0x04, 0x08])
        _request.append(hysteresis)
        _request.append(calibration)
        _request.append(cooling_max_temp)
        _request.append(cooling_min_temp)
        _request.append(heating_max_temp)
        _request.append(heating_min_temp)
        _request.append(fan_control)
        _request.append(frost_protection)
        self.send_request(_request)

    def set_hysteresis(self, hysteresis):
        if hysteresis not in [
            HYSEN_2PFC_HYSTERESIS_HALVE,
            HYSEN_2PFC_HYSTERESIS_WHOLE]:
            raise ValueError(
                'Can\'t set hysteresis (%s) outside device\'s admitted values (%s), (%s).' % ( \
                hysteresis,
                HYSEN_2PFC_HYSTERESIS_HALVE,
                HYSEN_2PFC_HYSTERESIS_WHOLE))
        self.get_device_status()
        self.set_options(
            hysteresis,
            self.calibration,
            self.cooling_max_temp,
            self.cooling_min_temp,
            self.heating_max_temp,
            self.heating_min_temp,
            self.fan_control,
            self.frost_protection)

    def set_calibration(self, calibration):
        if calibration < HYSEN_2PFC_CALIBRATION_MIN:
            raise ValueError(
                'Can\'t set calibration (%s°) lower than device\'s minimum (%s°).' % ( \
                calibration,
                HYSEN_2PFC_CALIBRATION_MIN))
        if calibration > HYSEN_2PFC_CALIBRATION_MAX:
            raise ValueError(
                'Can\'t set calibration (%s°) higher than device\'s maximum (%s°).' % ( \
                calibration,
                HYSEN_2PFC_CALIBRATION_MAX))
        self.get_device_status()
        self.set_options(
            self.hysteresis,
            calibration,
            self.cooling_max_temp,
            self.cooling_min_temp,
            self.heating_max_temp,
            self.heating_min_temp,
            self.fan_control,
            self.frost_protection)

    def set_cooling_max_temp(self, cooling_max_temp):
        self.get_device_status()
        if cooling_max_temp > HYSEN_2PFC_COOLING_MAX_TEMP:
            raise ValueError(
                'Can\'t set cooling maximum temperature (%s°) higher than device\'s maximum (%s°).' % ( \
                cooling_max_temp,
                HYSEN_2PFC_COOLING_MAX_TEMP))
        if cooling_max_temp < self.cooling_min_temp:
            raise ValueError(
                'Can\'t set cooling maximum temperature (%s°) lower than minimum set (%s°).' % ( \
                cooling_max_temp,
                self.cooling_min_temp))
        if cooling_max_temp < self.target_temp:
            raise ValueError(
                'Can\'t set cooling maximum temperature (%s°) lower than target temperature (%s°).' % ( \
                cooling_max_temp,
                self.target_temp))
        self.set_options(
            self.hysteresis,
            self.calibration,
            cooling_max_temp,
            self.cooling_min_temp,
            self.heating_max_temp,
            self.heating_min_temp,
            self.fan_control,
            self.frost_protection)

    def set_cooling_min_temp(self, cooling_min_temp):
        self.get_device_status()
        if cooling_min_temp < HYSEN_2PFC_COOLING_MIN_TEMP:
            raise ValueError(
                'Can\'t set cooling minimum temperature (%s°) lower than device\'s minimum (%s°).' % ( \
                cooling_min_temp,
                HYSEN_2PFC_COOLING_MIN_TEMP))
        if cooling_min_temp > self.cooling_max_temp:
            raise ValueError(
                'Can\'t set cooling minimum temperature (%s°) higher than maximum set (%s°).' % ( \
                cooling_min_temp,
                self.cooling_max_temp))
        if cooling_min_temp > self.target_temp:
            raise ValueError(
                'Can\'t set cooling minimum temperature (%s°) higher than target temperature (%s°).' % ( \
                cooling_min_temp,
                self.target_temp))
        self.set_options(
            self.hysteresis,
            self.calibration,
            self.cooling_max_temp,
            cooling_min_temp,
            self.heating_max_temp,
            self.heating_min_temp,
            self.fan_control,
            self.frost_protection)

    def set_heating_max_temp(self, heating_max_temp):
        self.get_device_status()
        if heating_max_temp > HYSEN_2PFC_HEATING_MAX_TEMP:
            raise ValueError(
                'Can\'t set heating maximum temperature (%s°) higher than device\'s maximum (%s°).' % ( \
                 heating_max_temp,
                HYSEN_2PFC_HEATING_MAX_TEMP))
        if  heating_max_temp < self. heating_min_temp:
            raise ValueError(
                'Can\'t set heating maximum temperature (%s°) lower than minimum set (%s°).' % ( \
                 heating_max_temp,
                self. heating_min_temp))
        if  heating_max_temp < self.target_temp:
            raise ValueError(
                'Can\'t set heating maximum temperature (%s°) lower than target temperature (%s°).' % ( \
                 heating_max_temp,
                self.target_temp))
        self.set_options(
            self.hysteresis,
            self.calibration,
            self.cooling_max_temp,
            self.cooling_min_temp,
            heating_max_temp,
            self.heating_min_temp,
            self.fan_control,
            self.frost_protection)

    def set_heating_min_temp(self, heating_min_temp):
        self.get_device_status()
        if heating_min_temp < HYSEN_2PFC_HEATING_MIN_TEMP:
            raise ValueError(
                'Can\'t set heating minimum temperature (%s°) lower than device\'s minimum (%s°).' % ( \
                heating_min_temp,
                HYSEN_2PFC_HEATING_MIN_TEMP))
        if heating_min_temp > self.heating_max_temp:
            raise ValueError(
                'Can\'t set heating minimum temperature (%s°) higher than maximum set (%s°).' % ( \
                heating_min_temp,
                self.heating_max_temp))
        if heating_min_temp > self.target_temp:
            raise ValueError(
                'Can\'t set heating minimum temperature (%s°) higher than target temperature (%s°).' % ( \
                heating_min_temp,
                self.target_temp))
        self.set_options(
            self.hysteresis,
            self.calibration,
            self.cooling_max_temp,
            self.cooling_min_temp,
            self.heating_max_temp,
            heating_min_temp,
            self.fan_control,
            self.frost_protection)

    def set_fan_control(self, fan_control):
        if fan_control not in [
            HYSEN_2PFC_FAN_CONTROL_ON,
            HYSEN_2PFC_FAN_CONTROL_OFF]:
            raise ValueError(
                'Can\'t set fan control (%s) outside device\'s admitted values (%s), (%s).' % ( \
                fan_control,
                HYSEN_2PFC_FAN_CONTROL_ON,
                HYSEN_2PFC_FAN_CONTROL_OFF))
        self.get_device_status()
        self.set_options(
            self.hysteresis,
            self.calibration,
            self.cooling_max_temp,
            self.cooling_min_temp,
            self.heating_max_temp,
            self.heating_min_temp,
            fan_control,
            self.frost_protection)

    def set_frost_protection(self, frost_protection):
        if frost_protection not in [
            HYSEN_2PFC_FROST_PROTECTION_OFF,
            HYSEN_2PFC_FROST_PROTECTION_ON]:
            raise ValueError(
                'Can\'t set frost protection (%s) outside device\'s admitted values (%s), (%s).' % ( \
                frost_protection,
                HYSEN_2PFC_FROST_PROTECTION_OFF,
                HYSEN_2PFC_FROST_PROTECTION_ON))
        self.get_device_status()
        self.set_options(
            self.hysteresis,
            self.calibration,
            self.cooling_max_temp,
            self.cooling_min_temp,
            self.heating_max_temp,
            self.heating_min_temp,
            self.fan_control,
            frost_protection)

    # set time
    # 0x01, 0x10, 0x00, 0x07, 0x00, 0x02, 0x04, hh, mm, ss, wd
    # hh = Time hour past midnight
    # mm = Time minute past hour
    # ss = Time second past minute
    # wd = Weekday 0x01 = Monday, 0x02 = Tuesday, ..., 0x06 = Saturday, 0x07 = Sunday
    # confirmation response:
    # payload 0x01, 0x10, 0x00, 0x07, 0x00, 0x02
    def set_time(self, clock_hour, clock_minute, clock_second, clock_weekday):
        if (clock_weekday < 1) or (clock_weekday > 7):
            raise ValueError(
                'Weekday (%s) has to be between 1 (Monday) and 7 (Saturday).' % ( \
                clock_weekday))
        if (clock_hour < 0) or (clock_hour > 23):
            raise ValueError(
                'Hour (%s) has to be between 0 and 23.' % ( \
                clock_hour))
        if (clock_minute < 0) or (clock_minute > 59):
            raise ValueError(
                'Minute (%s) has to be between 0 and 59.' % ( \
                clock_minute))
        if (clock_second < 0) or (clock_second > 59):
            raise ValueError(
                'Second (%s) has to be between 0 and 59.' % ( \
                clock_second))
        _request = bytearray([0x01, 0x10, 0x00, 0x07, 0x00, 0x02, 0x04])
        _request.append(clock_hour)
        _request.append(clock_minute)
        _request.append(clock_second)
        _request.append(clock_weekday)
        self.send_request(_request)

    # set weekly schedule
    # 0x01, 0x10, 0x00, 0x09, 0x00, 0x01, 0x02, 0x00, Lm
    # Unknown = 0x00
    # Lm = Weekly schedule, 0x00 = Today, 0x01 = 12345_67, 0x02 = 123456_7, 0x03 = 1234567
    # confirmation response:
    # payload 0x01, 0x10, 0x00, 0x09, 0x00, 0x01
    def set_weekly_schedule(self, schedule):
        if schedule not in [
            HYSEN_2PFC_SCHEDULE_TODAY,
            HYSEN_2PFC_SCHEDULE_12345_67,
            HYSEN_2PFC_SCHEDULE_123456_7,
            HYSEN_2PFC_SCHEDULE_1234567]:
            raise ValueError(
                'Can\'t set schedule (%s) outside device\'s admitted values (%s), (%s), (%s), (%s).' % ( \
                schedule,
                HYSEN_2PFC_SCHEDULE_TODAY,
                HYSEN_2PFC_SCHEDULE_12345_67,
                HYSEN_2PFC_SCHEDULE_123456_7,
                HYSEN_2PFC_SCHEDULE_1234567))
        _request = bytearray([0x01, 0x10, 0x00, 0x09, 0x00, 0x01, 0x02])
        _request.append(0)
        _request.append(schedule)
        self.send_request(_request)

    # set daily schedule
    # 0x01, 0x10, 0x00, 0x0A, 0x00, 0x04, 0x08, P1OnH, P1OnM, P1OffH, P1OffM, P2OnH, P2OnM, P2OffH, P2OffM
    # P1OnH = Period1 On Hour, Note: The most significant bit, 0 = Disabled, 1 = Enabled
    # P1OnM = Period1 On Minute past hour
    # P1OffH = Period1 Off Hour, Note: The most significant bit, 0 = Disabled, 1 = Enabled
    # P1OffM = Period1 Off Minute past hour
    # P2OnH = Period2 On Hour, Note: The most significant bit, 0 = Disabled, 1 = Enabled
    # P2OnM = Period2 On Minute past hour
    # P2OffH = Period2 Off Hour, Note: The most significant bit, 0 = Disabled, 1 = Enabled
    # P2OffM = Period2 Off Minute past hour
    # confirmation response:
    # payload 0x01, 0x10, 0x00, 0x0A, 0x00, 0x04
    def set_daily_schedule(self, period1_on_enabled, period1_on_hour, period1_on_min, period1_off_enabled, period1_off_hour, period1_off_min, period2_on_enabled, period2_on_hour, period2_on_min, period2_off_enabled, period2_off_hour, period2_off_min):
        _request = bytearray([0x01, 0x10, 0x00, 0x0A, 0x00, 0x04, 0x08])
        _request.append((period1_on_enabled << 7) + period1_on_hour)
        _request.append(period1_on_min)
        _request.append((period1_off_enabled << 7) + period1_off_hour)
        _request.append(period1_off_min)
        _request.append((period2_on_enabled << 7) + period2_on_hour)
        _request.append(period2_on_min)
        _request.append((period2_off_enabled << 7) + period2_off_hour)
        _request.append(period2_off_min)
        self.send_request(_request)

    def set_period1_on(self, period1_on_enabled = None, period1_on_hour = None, period1_on_min = None):
        self.get_device_status()
        if period1_on_enabled is None:
            period1_on_enabled = self.period1_on_enabled
        if period1_on_hour is None:
            period1_on_hour = self.period1_on_hour
        if period1_on_min is None:
            period1_on_min = self.period1_on_min
        if period1_on_enabled not in [
            HYSEN_2PFC_PERIOD_DISABLED,
            HYSEN_2PFC_PERIOD_ENABLED]:
            raise ValueError(
                'Can\'t set period1_on_enabled (%s) outside device\'s admitted values (%s), (%s).' % ( \
                period1_on_enabled,
                HYSEN_2PFC_PERIOD_DISABLED,
                HYSEN_2PFC_PERIOD_ENABLED))
        if (period1_on_hour < 0) or (period1_on_hour > 23):
            raise ValueError(
                'period1_on_hour (%s) has to be between 0 and 23.' % ( \
                period1_on_hour))
        if (period1_on_min < 0) or (period1_on_min > 59):
            raise ValueError(
                'period1_on_min (%s) has to be between 0 and 59.' % ( \
                period1_on_min))
        if (period1_on_hour > self.period1_off_hour) or \
            ((period1_on_hour == self.period1_off_hour) and (period1_on_min >= self.period1_off_min)):
            raise ValueError(
                'period1 on (%s:%s) has to be before period1 off (%s:%s).' % ( \
                period1_on_hour,
                period1_on_min,
                self.period1_off_hour,
                self.period1_off_min))
        self.set_daily_schedule(
            period1_on_enabled,
            period1_on_hour,
            period1_on_min,
            self.period1_off_enabled,
            self.period1_off_hour,
            self.period1_off_min,
            self.period2_on_enabled,
            self.period2_on_hour,
            self.period2_on_min,
            self.period2_off_enabled,
            self.period2_off_hour,
            self.period2_off_min)

    def set_period1_off(self, period1_off_enabled = None, period1_off_hour = None, period1_off_min = None):
        if period1_off_enabled is None:
            period1_off_enabled = self.period1_off_enabled
        if period1_off_hour is None:
            period1_off_hour = self.period1_off_hour
        if period1_off_min is None:
            period1_off_min = self.period1_off_min
        if period1_off_enabled not in [
            HYSEN_2PFC_PERIOD_DISABLED,
            HYSEN_2PFC_PERIOD_ENABLED]:
            raise ValueError(
                'Can\'t set period1_off_enabled (%s) outside device\'s admitted values (%s), (%s).' % ( \
                period1_off_enabled,
                HYSEN_2PFC_PERIOD_DISABLED,
                HYSEN_2PFC_PERIOD_ENABLED))
        if (period1_off_hour < 0) or (period1_off_hour > 23):
            raise ValueError(
                'period1_off_hour (%s) has to be between 0 and 23.' % ( \
                period1_off_hour))
        if (period1_off_min < 0) or (period1_off_min > 59):
            raise ValueError(
                'period1_off_min (%s) has to be between 0 and 59.' % ( \
                period1_off_min))
        if (period1_off_hour < self.period1_on_hour) or \
            ((period1_off_hour == self.period1_on_hour) and (period1_off_min <= self.period1_on_min)):
            raise ValueError(
                'period1 off (%s:%s) has to be after period1 on (%s:%s).' % ( \
                period1_off_hour,
                period1_off_min,
                self.period1_on_hour,
                self.period1_on_min))
        if (period1_off_hour > self.period2_on_hour) or \
            ((period1_off_hour == self.period2_on_hour) and (period1_off_min >= self.period2_on_min)):
            raise ValueError(
                'period1 off (%s:%s) has to be before period2 on (%s:%s).' % ( \
                period1_off_hour,
                period1_off_min,
                self.period2_on_hour,
                self.period2_on_min))
        self.get_device_status()
        self.set_daily_schedule(
            self.period1_on_enabled,
            self.period1_on_hour,
            self.period1_on_min,
            period1_off_enabled,
            period1_off_hour,
            period1_off_min,
            self.period2_on_enabled,
            self.period2_on_hour,
            self.period2_on_min,
            self.period2_off_enabled,
            self.period2_off_hour,
            self.period2_off_min)

    def set_period2_on(self, period2_on_enabled = None, period2_on_hour = None, period2_on_min = None):
        if period2_on_enabled is None:
            period2_on_enabled = self.period2_on_enabled
        if period2_on_hour is None:
            period2_on_hour = self.period2_on_hour
        if period2_on_min is None:
            period2_on_min = self.period2_on_min
        if period2_on_enabled not in [
            HYSEN_2PFC_PERIOD_DISABLED,
            HYSEN_2PFC_PERIOD_ENABLED]:
            raise ValueError(
                'Can\'t set period2_on_enabled (%s) outside device\'s admitted values (%s), (%s).' % ( \
                period2_on_enabled,
                HYSEN_2PFC_PERIOD_DISABLED,
                HYSEN_2PFC_PERIOD_ENABLED))
        if (period2_on_hour < 0) or (period2_on_hour > 23):
            raise ValueError(
                'period2_on_hour (%s) has to be between 0 and 23.' % ( \
                period2_on_hour))
        if (period2_on_min < 0) or (period2_on_min > 59):
            raise ValueError(
                'period2_on_min (%s) has to be between 0 and 59.' % ( \
                period2_on_min))
        if (period2_on_hour < self.period1_off_hour) or \
            ((period2_on_hour == self.period1_off_hour) and (period2_on_min <= self.period1_off_min)):
            raise ValueError(
                'period2 on (%s:%s) has to be after period1 off (%s:%s).' % ( \
                period2_on_hour,
                period2_on_min,
                self.period1_off_hour,
                self.period1_off_min))
        if (period2_on_hour > self.period2_off_hour) or \
            ((period2_on_hour == self.period2_off_hour) and (period2_on_min >= self.period2_off_min)):
            raise ValueError(
                'period2 on (%s:%s) has to be before period2 off (%s:%s).' % ( \
                period2_on_hour,
                period2_on_min,
                self.period2_off_hour,
                self.period2_off_min))
        self.get_device_status()
        self.set_daily_schedule(
            self.period1_on_enabled,
            self.period1_on_hour,
            self.period1_on_min,
            self.period1_off_enabled,
            self.period1_off_hour,
            self.period1_off_min,
            period2_on_enabled,
            period2_on_hour,
            period2_on_min,
            self.period2_off_enabled,
            self.period2_off_hour,
            self.period2_off_min)

    def set_period2_off(self, period2_off_enabled = None, period2_off_hour = None, period2_off_min = None):
        if period2_off_enabled is None:
            period2_off_enabled = self.period2_off_enabled
        if period2_off_hour is None:
            period2_off_hour = self.period2_off_hour
        if period2_off_min is None:
            period2_off_min = self.period2_off_min
        if period2_off_enabled not in [
            HYSEN_2PFC_PERIOD_DISABLED,
            HYSEN_2PFC_PERIOD_ENABLED]:
            raise ValueError(
                'Can\'t set period2_off_enabled (%s) outside device\'s admitted values (%s), (%s).' % ( \
                period2_off_enabled,
                HYSEN_2PFC_PERIOD_DISABLED,
                HYSEN_2PFC_PERIOD_ENABLED))
        if (period2_off_hour < 0) or (period2_off_hour > 23):
            raise ValueError(
                'period2_off_hour (%s) has to be between 0 and 23.' % ( \
                period2_off_hour))
        if (period2_off_min < 0) or (period2_off_min > 59):
            raise ValueError(
                'period2_off_min (%s) has to be between 0 and 59.' % ( \
                period2_off_min))
        if (period2_off_hour < self.period2_on_hour) or \
            ((period2_off_hour == self.period2_on_hour) and (period2_off_min <= self.period2_on_min)):
            raise ValueError(
                'period2 off (%s:%s) has to be after period2 on (%s:%s).' % ( \
                period2_off_hour,
                period2_off_min,
                self.period2_on_hour,
                self.period2_on_min))
        self.get_device_status()
        self.set_daily_schedule(
            self.period1_on_enabled,
            self.period1_on_hour,
            self.period1_on_min,
            self.period1_off_enabled,
            self.period1_off_hour,
            self.period1_off_min,
            self.period2_on_enabled,
            self.period2_on_hour,
            self.period2_on_min,
            period2_off_enabled,
            period2_off_hour,
            period2_off_min)

    # get device status
    # 0x01, 0x03, 0x00, 0x00, 0x00, 0x10
    # response:
    # 0x01, 0x03, 0x20, 0xrk, 0xvp, Mod, Fs, Rt, Tt, Dif, Adj, Sh1, Sl1, Sh2, Sl2, Fan, Fre, hh, mm, ss, wd, Unk, Lm, P1OnH, P1OnMin, P1OffH, P1OffM, P2OnH, P2OnMin, P2OffH, P2OffM, Tv1, Tv2, Tv3, Tv4
    # r = Remote lock, 0 = Off, 1 = On
    # k = Key lock (Loc), 0 = Unlocked, 1 = All buttons locked except Power, 2 = All buttons locked
    # v = Valve, 0 = Valve off, 1 = Valve on
    # p = Power, 0 = Power off, 1 = Power on
    # Mod = Operation mode, 0x01 = Ventilation, 0x02 = Cooling, 0x03 = Heating
    # Fs = Fan speed, 0x01 = Low, 0x02 = Medium, 0x03 = High, 0x04 = Auto
    # Rt = Room temperature
    # Tt = Target temperature
    # Dif = Hysteresis, 0x00 = 0.5 degree Celsius, 0x01 = 1 degree Celsius
    # Adj = Temperature calibration -5~+5, 0.1 degree Celsius step
    #       (e.g. -1 = 0xF6, -1.4 = 0xF2, 0 = 0x00, +1 = 0x0A, +1.2 = 0x0C, +2 = 0x14)
    # Sh1 = Cooling max. temperature
    # Sl1 = Cooling min. temperature
    # Sh2 = Heating max. temperature
    # Sl2 = Heating min. temperature
    # Fan = Fan coil control mode, 0x00 = Fan coil in control, 0x01 = Fan coil out of control
    # Fre = Frost Protection, 0 = On, 1 = Off
    # hh = Time hour past midnight
    # mm = Time minute past hour
    # ss = Time second past minute
    # wd = Weekday 0x01 = Monday, 0x01 = Tuesday, ..., 0x06 = Saturday, 0x07 = Sunday
    # Unk = Unknown, 0x00
    # Lm = Weekly schedule, 0x00 = Today, 0x01 = 12345_67, 0x02 = 123456_7, 0x03 = 1234567
    # P1OnH = Period1 On Hour, Note: The most significant bit, 0 = Disabled, 1 = Enabled
    # P1OnM = Period1 On Minute past hour
    # P1OffH = Period1 Off Hour, Note: The most significant bit, 0 = Disabled, 1 = Enabled
    # P1OffM = Period1 Off Minute past hour
    # P2OnH = Period2 On Hour, Note: The most significant bit, 0 = Disabled, 1 = Enabled
    # P2OnM = Period2 On Minute past hour
    # P2OffH = Period2 Off Hour, Note: The most significant bit, 0 = Disabled, 1 = Enabled
    # P2OffM = Period2 Off Minute past hour
    # Tv1 = Total time valve on in seconds MSByte
    # Tv3 = Total time valve on in seconds
    # Tv3 = Total time valve on in seconds
    # Tv4 = Total time valve on in seconds LSByte
    def get_device_status(self):
        _request = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x10])
        _response = self.send_request(_request)
#        _LOGGER.debug("[%s] get_device_status : %s", 
#            self._host, 
#            ' '.join(format(x, '02x') for x in bytearray(_response)))
        self.remote_lock =  (_response[3]>>4) & 1
        self.key_lock = _response[3] & 3
        self.valve_state =  (_response[4]>>4) & 1
        self.power_state =  _response[4] & 1
        self.operation_mode = _response[5]
        self.fan_mode = _response[6]
        self.room_temp = _response[7]
        self.target_temp = _response[8]
        self.hysteresis = _response[9]
        self.calibration = _response[10]
        if self.calibration > 0x7F:
            self.calibration = self.calibration - 0x100
        self.calibration = float(self.calibration/10.0)
        self.cooling_max_temp = _response[11]
        self.cooling_min_temp = _response[12]
        self.heating_max_temp = _response[13]
        self.heating_min_temp = _response[14]
        self.fan_control = _response[15]
        self.frost_protection = _response[16]
        self.clock_hour = _response[17]
        self.clock_min = _response[18]
        self.clock_sec = _response[19]
        self.clock_weekday = _response[20]
        self.unknown = _response[21]
        self.schedule = _response[22]
        self.period1_on_enabled = (_response[23]>>7) & 1
        self.period1_on_hour = _response[23] & 0x1F
        self.period1_on_min = _response[24] & 0x3F
        self.period1_off_enabled = (_response[25]>>7) & 1
        self.period1_off_hour = _response[25] & 0x1F
        self.period1_off_min = _response[26] & 0x3F
        self.period2_on_enabled = (_response[27]>>7) & 1
        self.period2_on_hour = _response[27] & 0x1F
        self.period2_on_min = _response[28] & 0x3F
        self.period2_off_enabled = (_response[29]>>7) & 1
        self.period2_off_hour = _response[29] & 0x1F
        self.period2_off_min = _response[30] & 0x3F
        self.time_valve_on = (_response[31] << 24) + (_response[32] << 16) + (_response[33] << 8) + _response[34]
