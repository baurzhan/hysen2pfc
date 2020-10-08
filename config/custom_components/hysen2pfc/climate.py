"""
Support for Hysen Controller for 2 Pipe Fan Coil units.
Hysen HY03AC-1-Wifi device and derivative
http://www.xmhysen.com/products_detail/productId=201.html
"""
import asyncio
from functools import partial
import binascii
import socket
import logging

import voluptuous as vol

from homeassistant.components.climate import ClimateDevice, PLATFORM_SCHEMA
from typing import List, Optional

from homeassistant.components.climate.const import (
    DOMAIN,
    SUPPORT_TARGET_TEMPERATURE,
    SUPPORT_FAN_MODE,
    SUPPORT_PRESET_MODE,
    SUPPORT_AUX_HEAT,
    HVAC_MODE_HEAT,
    HVAC_MODE_COOL,
    HVAC_MODE_FAN_ONLY,
    CURRENT_HVAC_HEAT,
    CURRENT_HVAC_COOL,
    CURRENT_HVAC_FAN,
    PRESET_NONE,
    HVAC_MODE_AUTO,
    FAN_AUTO,
    FAN_LOW,
    FAN_MEDIUM,
    FAN_HIGH,
)

from homeassistant.components.fan import SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH
from homeassistant.const import (
    CONF_NAME,
    CONF_HOST,
    CONF_MAC,
    CONF_TIMEOUT,
    TEMP_CELSIUS,
    STATE_ON,
    STATE_OFF,
    STATE_LOCKED,
    STATE_UNLOCKED,
    STATE_IDLE,
    STATE_OPEN,
    STATE_CLOSED,
    ATTR_TEMPERATURE,
    PRECISION_WHOLE,
    ATTR_ENTITY_ID,
)
import homeassistant.helpers.config_validation as cv
import homeassistant.util.dt as dt_util

from .hysen2pfc_device import (
    Hysen2PipeFanCoilDevice,
    HYSEN_2PFC_REMOTE_LOCK_OFF,
    HYSEN_2PFC_REMOTE_LOCK_ON,
    HYSEN_2PFC_KEY_ALL_UNLOCKED,
    HYSEN_2PFC_KEY_POWER_UNLOCKED,
    HYSEN_2PFC_KEY_ALL_LOCKED,
    HYSEN_2PFC_POWER_OFF,
    HYSEN_2PFC_POWER_ON,
    HYSEN_2PFC_VALVE_OFF,
    HYSEN_2PFC_VALVE_ON,
    HYSEN_2PFC_HYSTERESIS_HALVE,
    HYSEN_2PFC_HYSTERESIS_WHOLE,
    HYSEN_2PFC_CALIBRATION_MIN,
    HYSEN_2PFC_CALIBRATION_MAX,
    HYSEN_2PFC_FAN_LOW,
    HYSEN_2PFC_FAN_MEDIUM,
    HYSEN_2PFC_FAN_HIGH,
    HYSEN_2PFC_FAN_AUTO,
    HYSEN_2PFC_MODE_FAN,
    HYSEN_2PFC_MODE_COOL,
    HYSEN_2PFC_MODE_HEAT,
    HYSEN_2PFC_FAN_CONTROL_ON,
    HYSEN_2PFC_FAN_CONTROL_OFF,
    HYSEN_2PFC_FROST_PROTECTION_OFF,
    HYSEN_2PFC_FROST_PROTECTION_ON,
    HYSEN_2PFC_SCHEDULE_TODAY,
    HYSEN_2PFC_SCHEDULE_12345_67,
    HYSEN_2PFC_SCHEDULE_123456_7,
    HYSEN_2PFC_SCHEDULE_1234567,
    HYSEN_2PFC_PERIOD_DISABLED,
    HYSEN_2PFC_PERIOD_ENABLED,
    HYSEN_2PFC_COOLING_MAX_TEMP,
    HYSEN_2PFC_COOLING_MIN_TEMP,
    HYSEN_2PFC_HEATING_MAX_TEMP,
    HYSEN_2PFC_HEATING_MIN_TEMP,
    HYSEN_2PFC_MAX_TEMP,
    HYSEN_2PFC_MIN_TEMP,
)

_LOGGER = logging.getLogger(__name__)

STATE_ALL_UNLOCKED = "unlocked"
STATE_POWER_UNLOCKED = "power_unlocked"
STATE_ALL_LOCKED = "locked"

STATE_HYSTERESIS_HALVE = "0.5"
STATE_HYSTERESIS_WHOLE = "1"

STATE_SCHEDULE_TODAY = "today"
STATE_SCHEDULE_12345_67 = "12345"
STATE_SCHEDULE_123456_7 = "123456"
STATE_SCHEDULE_1234567 = "1234567"

KEY_LOCK_MODES = [STATE_ALL_UNLOCKED, STATE_POWER_UNLOCKED, STATE_ALL_LOCKED]

HYSTERESIS_MODES = [STATE_HYSTERESIS_HALVE, STATE_HYSTERESIS_WHOLE]

SCHEDULE_MODES = [
    STATE_SCHEDULE_TODAY,
    STATE_SCHEDULE_12345_67,
    STATE_SCHEDULE_123456_7,
    STATE_SCHEDULE_1234567,
]

HVAC_MODES = [HVAC_MODE_HEAT, HVAC_MODE_COOL, HVAC_MODE_FAN_ONLY]

FAN_MODES = [SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH]

FAN_MODES_MANUAL = [SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH]

HYSEN_KEY_LOCK_TO_HASS = {
    HYSEN_2PFC_KEY_ALL_UNLOCKED: STATE_ALL_UNLOCKED,
    HYSEN_2PFC_KEY_POWER_UNLOCKED: STATE_POWER_UNLOCKED,
    HYSEN_2PFC_KEY_ALL_LOCKED: STATE_ALL_LOCKED,
}

HASS_KEY_LOCK_TO_HYSEN = {
    STATE_ALL_UNLOCKED: HYSEN_2PFC_KEY_ALL_UNLOCKED,
    STATE_POWER_UNLOCKED: HYSEN_2PFC_KEY_POWER_UNLOCKED,
    STATE_ALL_LOCKED: HYSEN_2PFC_KEY_ALL_LOCKED,
}

HYSEN_POWER_STATE_TO_HASS = {
    HYSEN_2PFC_POWER_ON: STATE_ON,
    HYSEN_2PFC_POWER_OFF: STATE_OFF,
}

HASS_POWER_STATE_TO_HYSEN = {
    True: HYSEN_2PFC_POWER_ON,
    False: HYSEN_2PFC_POWER_OFF,
}

HYSEN_VALVE_STATE_TO_HASS = {
    HYSEN_2PFC_VALVE_ON: STATE_OPEN,
    HYSEN_2PFC_VALVE_OFF: STATE_CLOSED,
}

HYSEN_HYSTERESIS_TO_HASS = {
    HYSEN_2PFC_HYSTERESIS_HALVE: STATE_HYSTERESIS_HALVE,
    HYSEN_2PFC_HYSTERESIS_WHOLE: STATE_HYSTERESIS_WHOLE,
}

HASS_HYSTERESIS_TO_HYSEN = {
    STATE_HYSTERESIS_HALVE: HYSEN_2PFC_HYSTERESIS_HALVE,
    STATE_HYSTERESIS_WHOLE: HYSEN_2PFC_HYSTERESIS_WHOLE,
}

HYSEN_FAN_CONTROL_TO_HASS = {
    HYSEN_2PFC_FAN_CONTROL_ON: STATE_ON,
    HYSEN_2PFC_FAN_CONTROL_OFF: STATE_OFF,
}

HASS_FAN_CONTROL_TO_HYSEN = {
    True: HYSEN_2PFC_FAN_CONTROL_ON,
    False: HYSEN_2PFC_FAN_CONTROL_OFF,
}

HYSEN_FROST_PROTECTION_TO_HASS = {
    HYSEN_2PFC_FROST_PROTECTION_ON: STATE_ON,
    HYSEN_2PFC_FROST_PROTECTION_OFF: STATE_OFF,
}

HASS_FROST_PROTECTION_TO_HYSEN = {
    True: HYSEN_2PFC_FROST_PROTECTION_ON,
    False: HYSEN_2PFC_FROST_PROTECTION_OFF,
}

HYSEN_SCHEDULE_TO_HASS = {
    HYSEN_2PFC_SCHEDULE_TODAY: STATE_SCHEDULE_TODAY,
    HYSEN_2PFC_SCHEDULE_12345_67: STATE_SCHEDULE_12345_67,
    HYSEN_2PFC_SCHEDULE_123456_7: STATE_SCHEDULE_123456_7,
    HYSEN_2PFC_SCHEDULE_1234567: STATE_SCHEDULE_1234567,
}

HASS_SCHEDULE_TO_HYSEN = {
    STATE_SCHEDULE_TODAY: HYSEN_2PFC_SCHEDULE_TODAY,
    STATE_SCHEDULE_12345_67: HYSEN_2PFC_SCHEDULE_12345_67,
    STATE_SCHEDULE_123456_7: HYSEN_2PFC_SCHEDULE_123456_7,
    STATE_SCHEDULE_1234567: HYSEN_2PFC_SCHEDULE_1234567,
}

HYSEN_MODE_TO_HASS = {
    HYSEN_2PFC_MODE_FAN: CURRENT_HVAC_FAN,
    HYSEN_2PFC_MODE_COOL: CURRENT_HVAC_COOL,
    HYSEN_2PFC_MODE_HEAT: CURRENT_HVAC_HEAT,
}

HYSEN_MODE_TO_ACTION = {
    HYSEN_2PFC_MODE_FAN: HVAC_MODE_FAN_ONLY,
    HYSEN_2PFC_MODE_COOL: HVAC_MODE_COOL,
    HYSEN_2PFC_MODE_HEAT: HVAC_MODE_HEAT,
}

HASS_MODE_TO_HYSEN = {
    HVAC_MODE_FAN_ONLY: HYSEN_2PFC_MODE_FAN,
    HVAC_MODE_COOL: HYSEN_2PFC_MODE_COOL,
    HVAC_MODE_HEAT: HYSEN_2PFC_MODE_HEAT,
}

HYSEN_FAN_TO_HASS = {
    HYSEN_2PFC_FAN_LOW: FAN_LOW,
    HYSEN_2PFC_FAN_MEDIUM: FAN_MEDIUM,
    HYSEN_2PFC_FAN_HIGH: FAN_HIGH,
    HYSEN_2PFC_FAN_AUTO: FAN_AUTO,
}

HASS_FAN_TO_HYSEN = {
    FAN_LOW: HYSEN_2PFC_FAN_LOW,
    FAN_MEDIUM: HYSEN_2PFC_FAN_MEDIUM,
    FAN_HIGH: HYSEN_2PFC_FAN_HIGH,
    FAN_AUTO: HYSEN_2PFC_FAN_AUTO,
}

HYSEN_PERIOD_ENABLED_TO_HASS = {
    HYSEN_2PFC_PERIOD_ENABLED: STATE_ON,
    HYSEN_2PFC_PERIOD_DISABLED: STATE_OFF,
}

HASS_PERIOD_ENABLED_TO_HYSEN = {
    None: None,
    True: HYSEN_2PFC_PERIOD_ENABLED,
    False: HYSEN_2PFC_PERIOD_DISABLED,
}

HYSEN_2PFC_DEV_TYPE = 0x4F5B
HYSEN_2PFC_DEFAULT_NAME = "Hysen 2 Pipe Fan Coil Thermostat"
HYSEN_2PFC_DEFAULT_TIMEOUT = 10

DATA_KEY = "climate.hysen_2pfc"

PLATFORM_SCHEMA = PLATFORM_SCHEMA.extend(
    {
        vol.Optional(CONF_NAME, default=HYSEN_2PFC_DEFAULT_NAME): cv.string,
        vol.Required(CONF_HOST): cv.string,
        vol.Required(CONF_MAC): cv.string,
        vol.Optional(CONF_TIMEOUT, default=HYSEN_2PFC_DEFAULT_TIMEOUT): cv.positive_int,
    }
)

ATTR_KEY_LOCK = "key_lock"
ATTR_POWER_STATE = "power_state"
ATTR_VALVE_STATE = "valve_state"
ATTR_HYSTERESIS = "hysteresis"
ATTR_CALIBRATION = "calibration"
ATTR_LIMIT_TEMP = "temp"
ATTR_COOLING_MAX_TEMP = "cooling_max_temp"
ATTR_COOLING_MIN_TEMP = "cooling_min_temp"
ATTR_HEATING_MAX_TEMP = "heating_max_temp"
ATTR_HEATING_MIN_TEMP = "heating_min_temp"
ATTR_FAN_CONTROL = "fan_control"
ATTR_FROST_PROTECTION = "frost_protection"
ATTR_CLOCK_HOUR = "clock_hour"
ATTR_CLOCK_MIN = "clock_min"
ATTR_CLOCK_SEC = "clock_sec"
ATTR_CLOCK_WEEKDAY = "clock_weekday"
ATTR_SCHEDULE = "schedule"
ATTR_PERIOD_ENABLE = "enable"
ATTR_PERIOD_HOUR = "hour"
ATTR_PERIOD_MIN = "min"
ATTR_PERIOD1_ON_ENABLED = "period1_on_enabled"
ATTR_PERIOD1_ON_HOUR = "period1_on_hour"
ATTR_PERIOD1_ON_MIN = "period1_on_min"
ATTR_PERIOD1_OFF_ENABLED = "period1_off_enabled"
ATTR_PERIOD1_OFF_HOUR = "period1_off_hour"
ATTR_PERIOD1_OFF_MIN = "period1_off_min"
ATTR_PERIOD2_ON_ENABLED = "period2_on_enabled"
ATTR_PERIOD2_ON_HOUR = "period2_on_hour"
ATTR_PERIOD2_ON_MIN = "period2_on_min"
ATTR_PERIOD2_OFF_ENABLED = "period2_off_enabled"
ATTR_PERIOD2_OFF_HOUR = "period2_off_hour"
ATTR_PERIOD2_OFF_MIN = "period2_off_min"
ATTR_TIME_VALVE_ON = "time_valve_on"

SERVICE_SET_KEY_LOCK = "hysen2pfc_set_key_lock"
SERVICE_SET_HYSTERESIS = "hysen2pfc_set_hysteresis"
SERVICE_SET_CALIBRATION = "hysen2pfc_set_calibration"
SERVICE_SET_COOLING_MAX_TEMP = "hysen2pfc_set_cooling_max_temp"
SERVICE_SET_COOLING_MIN_TEMP = "hysen2pfc_set_cooling_min_temp"
SERVICE_SET_HEATING_MAX_TEMP = "hysen2pfc_set_heating_max_temp"
SERVICE_SET_HEATING_MIN_TEMP = "hysen2pfc_set_heating_min_temp"
SERVICE_SET_FAN_CONTROL = "hysen2pfc_set_fan_control"
SERVICE_SET_FROST_PROTECTION = "hysen2pfc_set_frost_protection"
SERVICE_SET_TIME_NOW = "hysen2pfc_set_time_now"
SERVICE_SET_SCHEDULE = "hysen2pfc_set_schedule"
SERVICE_SET_PERIOD1_ON = "hysen2pfc_set_period1_on"
SERVICE_SET_PERIOD1_OFF = "hysen2pfc_set_period1_off"
SERVICE_SET_PERIOD2_ON = "hysen2pfc_set_period2_on"
SERVICE_SET_PERIOD2_OFF = "hysen2pfc_set_period2_off"

CLIMATE_SERVICE_SCHEMA = vol.Schema(
    {
        vol.Optional(ATTR_ENTITY_ID): cv.entity_ids,
    }
)

SERVICE_SCHEMA_KEY_LOCK = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_KEY_LOCK): cv.string,
    }
)

SERVICE_SCHEMA_HYSTERESIS = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_HYSTERESIS): cv.string,
    }
)

SERVICE_SCHEMA_CALIBRATION = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_CALIBRATION): vol.Coerce(float),
    }
)

SERVICE_SCHEMA_COOLING_MAX_TEMP = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_LIMIT_TEMP): vol.Coerce(int),
    }
)

SERVICE_SCHEMA_COOLING_MIN_TEMP = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_LIMIT_TEMP): vol.Coerce(int),
    }
)

SERVICE_SCHEMA_HEATING_MAX_TEMP = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_LIMIT_TEMP): vol.Coerce(int),
    }
)

SERVICE_SCHEMA_HEATING_MIN_TEMP = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_LIMIT_TEMP): vol.Coerce(int),
    }
)

SERVICE_SCHEMA_FAN_CONTROL = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_FAN_CONTROL): vol.Boolean,
    }
)

SERVICE_SCHEMA_FROST_PROTECTION = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_FROST_PROTECTION): vol.Boolean,
    }
)

SERVICE_SCHEMA_TIME_NOW = CLIMATE_SERVICE_SCHEMA

SERVICE_SCHEMA_SCHEDULE = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Required(ATTR_SCHEDULE): cv.string,
    }
)

SERVICE_SCHEMA_PERIOD1_ON = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Optional(ATTR_PERIOD_ENABLE): vol.Boolean(),
        vol.Optional(ATTR_PERIOD_HOUR): vol.Coerce(int),
        vol.Optional(ATTR_PERIOD_MIN): vol.Coerce(int),
    }
)

SERVICE_SCHEMA_PERIOD1_OFF = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Optional(ATTR_PERIOD_ENABLE): vol.Boolean,
        vol.Optional(ATTR_PERIOD_HOUR): vol.Coerce(int),
        vol.Optional(ATTR_PERIOD_MIN): vol.Coerce(int),
    }
)

SERVICE_SCHEMA_PERIOD2_ON = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Optional(ATTR_PERIOD_ENABLE): vol.Boolean(),
        vol.Optional(ATTR_PERIOD_HOUR): vol.Coerce(int),
        vol.Optional(ATTR_PERIOD_MIN): vol.Coerce(int),
    }
)

SERVICE_SCHEMA_PERIOD2_OFF = CLIMATE_SERVICE_SCHEMA.extend(
    {
        vol.Optional(ATTR_PERIOD_ENABLE): vol.Boolean(),
        vol.Optional(ATTR_PERIOD_HOUR): vol.Coerce(int),
        vol.Optional(ATTR_PERIOD_MIN): vol.Coerce(int),
    }
)

SERVICE_TO_METHOD = {
    SERVICE_SET_KEY_LOCK: {
        "method": "async_set_key_lock",
        "schema": SERVICE_SCHEMA_KEY_LOCK,
    },
    SERVICE_SET_HYSTERESIS: {
        "method": "async_set_hysteresis",
        "schema": SERVICE_SCHEMA_HYSTERESIS,
    },
    SERVICE_SET_CALIBRATION: {
        "method": "async_set_calibration",
        "schema": SERVICE_SCHEMA_CALIBRATION,
    },
    SERVICE_SET_COOLING_MAX_TEMP: {
        "method": "async_set_cooling_max_temp",
        "schema": SERVICE_SCHEMA_COOLING_MAX_TEMP,
    },
    SERVICE_SET_COOLING_MIN_TEMP: {
        "method": "async_set_cooling_min_temp",
        "schema": SERVICE_SCHEMA_COOLING_MIN_TEMP,
    },
    SERVICE_SET_HEATING_MAX_TEMP: {
        "method": "async_set_heating_max_temp",
        "schema": SERVICE_SCHEMA_HEATING_MAX_TEMP,
    },
    SERVICE_SET_HEATING_MIN_TEMP: {
        "method": "async_set_heating_min_temp",
        "schema": SERVICE_SCHEMA_HEATING_MIN_TEMP,
    },
    SERVICE_SET_FAN_CONTROL: {
        "method": "async_set_fan_control",
        "schema": SERVICE_SCHEMA_FAN_CONTROL,
    },
    SERVICE_SET_FROST_PROTECTION: {
        "method": "async_set_frost_protection",
        "schema": SERVICE_SCHEMA_FROST_PROTECTION,
    },
    SERVICE_SET_TIME_NOW: {
        "method": "async_set_time_now",
        "schema": SERVICE_SCHEMA_TIME_NOW,
    },
    SERVICE_SET_SCHEDULE: {
        "method": "async_set_schedule",
        "schema": SERVICE_SCHEMA_SCHEDULE,
    },
    SERVICE_SET_PERIOD1_ON: {
        "method": "async_set_period1_on",
        "schema": SERVICE_SCHEMA_PERIOD1_ON,
    },
    SERVICE_SET_PERIOD1_OFF: {
        "method": "async_set_period1_off",
        "schema": SERVICE_SCHEMA_PERIOD1_OFF,
    },
    SERVICE_SET_PERIOD2_ON: {
        "method": "async_set_period2_on",
        "schema": SERVICE_SCHEMA_PERIOD2_ON,
    },
    SERVICE_SET_PERIOD2_OFF: {
        "method": "async_set_period2_off",
        "schema": SERVICE_SCHEMA_PERIOD2_OFF,
    },
}


async def async_setup_platform(hass, config, async_add_entities, discovery_info=None):
    """Set up the Hysen HVACR thermostat platform."""
    if DATA_KEY not in hass.data:
        hass.data[DATA_KEY] = {}

    host = config.get(CONF_HOST)
    name = config.get(CONF_NAME)
    mac_addr = binascii.unhexlify(config.get(CONF_MAC).encode().replace(b":", b""))
    timeout = config.get(CONF_TIMEOUT)

    hysen_device = Hysen2PipeFanCoilDevice(
        (host, 80), mac_addr, HYSEN_2PFC_DEV_TYPE, timeout
    )

    device = Hysen2PipeFanCoil(name, hysen_device, host)
    hass.data[DATA_KEY][host] = device

    async_add_entities([device], update_before_add=True)

    async def async_service_handler(service):
        """Map services to methods on target thermostat."""
        method = SERVICE_TO_METHOD.get(service.service)
        params = {
            key: value for key, value in service.data.items() if key != ATTR_ENTITY_ID
        }
        entity_ids = service.data.get(ATTR_ENTITY_ID)
        if entity_ids:
            target_hvacrs = [
                dev
                for dev in hass.data[DATA_KEY].values()
                if dev.entity_id in entity_ids
            ]
        else:
            target_hvacrs = hass.data[DATA_KEY].values()

        update_tasks = []
        for hvacr in target_hvacrs:
            await getattr(hvacr, method["method"])(**params)

        for hvacr in target_hvacrs:
            update_tasks.append(hvacr.async_update_ha_state(True))

        if update_tasks:
            await asyncio.wait(update_tasks, loop=hass.loop)

    for hvacr_service in SERVICE_TO_METHOD:
        schema = SERVICE_TO_METHOD[hvacr_service].get("schema", CLIMATE_SERVICE_SCHEMA)
        hass.services.async_register(
            DOMAIN, hvacr_service, async_service_handler, schema=schema
        )


class Hysen2PipeFanCoil(ClimateDevice):
    """Representation of a Hysen HVACR device."""

    def __init__(self, name, hysen_device, host):
        """Initialize the Hysen HVACR device."""
        self._name = name
        self._host = host
        self._hysen_device = hysen_device
        self._preset_mode = PRESET_NONE
        self._device_available = False
        self._device_authenticated = False

    @property
    def should_poll(self):
        """Return the polling state."""
        return True

    @property
    def name(self):
        """Returns the name of the device."""
        return self._name

    @property
    def available(self) -> bool:
        """Return True if entity is available."""
        return self._device_available

    @property
    def precision(self):
        """Return the precision of the system."""
        return PRECISION_WHOLE

    @property
    def device_state_attributes(self):
        """Return the specific state attributes of the device."""
        attr = {}
        if self._device_available:
            attr.update(
                {
                    ATTR_KEY_LOCK: str(
                        HYSEN_KEY_LOCK_TO_HASS[self._hysen_device.key_lock]
                    ),
                    ATTR_VALVE_STATE: str(
                        HYSEN_VALVE_STATE_TO_HASS[self._hysen_device.valve_state]
                    ),
                    ATTR_POWER_STATE: str(
                        HYSEN_POWER_STATE_TO_HASS[self._hysen_device.power_state]
                    ),
                    ATTR_HYSTERESIS: str(
                        HYSEN_HYSTERESIS_TO_HASS[self._hysen_device.hysteresis]
                    ),
                    ATTR_CALIBRATION: float(self._hysen_device.calibration),
                    ATTR_COOLING_MAX_TEMP: int(self._hysen_device.cooling_max_temp),
                    ATTR_COOLING_MIN_TEMP: int(self._hysen_device.cooling_min_temp),
                    ATTR_HEATING_MAX_TEMP: int(self._hysen_device.heating_max_temp),
                    ATTR_HEATING_MIN_TEMP: int(self._hysen_device.heating_min_temp),
                    ATTR_FAN_CONTROL: str(
                        HYSEN_FAN_CONTROL_TO_HASS[self._hysen_device.fan_control]
                    ),
                    ATTR_FROST_PROTECTION: str(
                        HYSEN_FROST_PROTECTION_TO_HASS[
                            self._hysen_device.frost_protection
                        ]
                    ),
                    ATTR_CLOCK_HOUR: int(self._hysen_device.clock_hour),
                    ATTR_CLOCK_MIN: int(self._hysen_device.clock_min),
                    ATTR_CLOCK_SEC: int(self._hysen_device.clock_sec),
                    ATTR_CLOCK_WEEKDAY: int(self._hysen_device.clock_weekday),
                    ATTR_SCHEDULE: str(
                        HYSEN_SCHEDULE_TO_HASS[self._hysen_device.schedule]
                    ),
                    ATTR_PERIOD1_ON_ENABLED: str(
                        HYSEN_PERIOD_ENABLED_TO_HASS[
                            self._hysen_device.period1_on_enabled
                        ]
                    ),
                    ATTR_PERIOD1_ON_HOUR: int(self._hysen_device.period1_on_hour),
                    ATTR_PERIOD1_ON_MIN: int(self._hysen_device.period1_on_min),
                    ATTR_PERIOD1_OFF_ENABLED: str(
                        HYSEN_PERIOD_ENABLED_TO_HASS[
                            self._hysen_device.period1_off_enabled
                        ]
                    ),
                    ATTR_PERIOD1_OFF_HOUR: int(self._hysen_device.period1_off_hour),
                    ATTR_PERIOD1_OFF_MIN: int(self._hysen_device.period1_off_min),
                    ATTR_PERIOD2_ON_ENABLED: str(
                        HYSEN_PERIOD_ENABLED_TO_HASS[
                            self._hysen_device.period2_on_enabled
                        ]
                    ),
                    ATTR_PERIOD2_ON_HOUR: int(self._hysen_device.period2_on_hour),
                    ATTR_PERIOD2_ON_MIN: int(self._hysen_device.period2_on_min),
                    ATTR_PERIOD2_OFF_ENABLED: str(
                        HYSEN_PERIOD_ENABLED_TO_HASS[
                            self._hysen_device.period2_off_enabled
                        ]
                    ),
                    ATTR_PERIOD2_OFF_HOUR: int(self._hysen_device.period2_off_hour),
                    ATTR_PERIOD2_OFF_MIN: int(self._hysen_device.period2_off_min),
                    ATTR_TIME_VALVE_ON: int(self._hysen_device.time_valve_on),
                }
            )
        return attr

    @property
    def temperature_unit(self):
        """Returns the unit of measurement which this thermostat uses."""
        return TEMP_CELSIUS

    @property
    def hvac_modes(self) -> List[str]:
        """Return the list of available hvac operation modes.
        Need to be a subset of HVAC_MODES.
        """
        return HVAC_MODES

    @property
    def hvac_mode(self) -> str:
        """Return hvac operation ie. heat, cool mode.
        Need to be one of HVAC_MODE_*.
        """
        return HYSEN_MODE_TO_HASS[self._hysen_device.operation_mode]

    @property
    def hvac_action(self) -> Optional[str]:
        """Return the current running hvac operation if supported.
        Need to be one of CURRENT_HVAC_*.
        """
        return HYSEN_MODE_TO_ACTION[self._hysen_device.operation_mode]

    @property
    def preset_mode(self) -> str:
        """Return the current preset mode, e.g., home, away, temp.
        Requires SUPPORT_PRESET_MODE.
        """
        return self._preset_mode

    @property
    def preset_modes(self) -> Optional[List[str]]:
        """Return a list of available preset modes.
        Requires SUPPORT_PRESET_MODE.
        """
        return [PRESET_NONE]

    @property
    def current_temperature(self):
        """Returns the sensor temperature."""
        return self._hysen_device.room_temp

    @property
    def target_temperature(self):
        """Returns the target temperature."""
        if self._hysen_device.power_state:
            if self._hysen_device.operation_mode == HYSEN_2PFC_MODE_FAN:
                return None
            return self._hysen_device.target_temp
        else:
            return None

    @property
    def target_temperature_step(self):
        """Returns the supported step of target temperature."""
        if self._hysen_device.power_state:
            if self._hysen_device.operation_mode == HYSEN_2PFC_MODE_FAN:
                return None
            return PRECISION_WHOLE
        else:
            return None

    @property
    def is_on(self):
        """Return true if device is on."""
        return self._hysen_device.power_state

    @property
    def fan_mode(self):
        """Return the current fan speed."""
        return HYSEN_FAN_TO_HASS[self._hysen_device.fan_mode]

    @property
    def fan_modes(self):
        """Returns the list of available fan modes."""
        if self._hysen_device.operation_mode == HYSEN_2PFC_MODE_FAN:
            return FAN_MODES_MANUAL
        else:
            return FAN_MODES

    @property
    def supported_features(self):
        """Returns the list of supported features."""
        if self._hysen_device.operation_mode == HYSEN_2PFC_MODE_FAN:
            return SUPPORT_FAN_MODE
        else:
            return SUPPORT_FAN_MODE | SUPPORT_TARGET_TEMPERATURE
    @property
    def min_temp(self):
        """Returns the minimum supported temperature."""
        if self._hysen_device.power_state:
            if self._hysen_device.operation_mode == HYSEN_2PFC_MODE_FAN:
                return None
            elif self._hysen_device.operation_mode == HYSEN_2PFC_MODE_HEAT:
                return self._hysen_device.heating_min_temp
            else:
                return self._hysen_device.cooling_min_temp
        else:
            return None

    @property
    def max_temp(self):
        """Returns the maximum supported temperature."""
        if self._hysen_device.power_state:
            if self._hysen_device.operation_mode == HYSEN_2PFC_MODE_FAN:
                return None
            elif self._hysen_device.operation_mode == HYSEN_2PFC_MODE_HEAT:
                return self._hysen_device.heating_max_temp
            else:
                return self._hysen_device.cooling_max_temp
        else:
            return None

    async def async_set_temperature(self, **kwargs):
        """Set new target temperature."""
        temp = int(kwargs.get(ATTR_TEMPERATURE))
        await self._try_command(
            "Error in set_temperature", self._hysen_device.set_target_temp, temp
        )

    async def async_set_fan_mode(self, fan_mode):
        """Set fan speed."""
        if fan_mode.lower() not in FAN_MODES:
            _LOGGER.error(
                "[%s] Error in async_set_fan_mode. Unknown fan mode '%s'.",
                self._host,
                fan_mode,
            )
            return
        await self._try_command(
            "Error in set_fan_mode",
            self._hysen_device.set_fan_mode,
            HASS_FAN_TO_HYSEN[fan_mode.lower()],
        )

    async def async_set_preset_mode(self, preset_mode) -> None:
        """Set new preset mode."""
        self._preset_mode = preset_mode

    async def async_set_hvac_mode(self, hvac_mode) -> None:
        """Set operation mode."""
        if hvac_mode.lower() not in HVAC_MODES:
            _LOGGER.error(
                "[%s] Error in async_set_operation_mode. Unknown operation mode '%s'.",
                self._host,
                hvac_mode,
            )
            return
        await self._try_command(
            "Error in set_operation_mode",
            self._hysen_device.set_operation_mode,
            HASS_MODE_TO_HYSEN[hvac_mode.lower()],
        )

    async def async_turn_on(self):
        """Turn device on."""
        await self._try_command(
            "Error in turn_on",
            self._hysen_device.set_power,
            HASS_POWER_STATE_TO_HYSEN[True],
        )

    async def async_turn_off(self):
        """Turn device off."""
        await self._try_command(
            "Error in turn_off",
            self._hysen_device.set_power,
            HASS_POWER_STATE_TO_HYSEN[False],
        )

    async def async_set_key_lock(self, key_lock):
        """Set key lock 0 = Unlocked, 1 = All buttons locked except Power, 2 = All buttons locked"""
        if key_lock.lower() not in KEY_LOCK_MODES:
            _LOGGER.error(
                "[%s] Error in async_set_key_lock. Unknown key_lock '%s'.",
                self._host,
                key_lock,
            )
            return
        await self._try_command(
            "Error in set_remote_lock",
            self._hysen_device.set_remote_lock,
            HASS_KEY_LOCK_TO_HYSEN[key_lock.lower()],
        )

    async def async_set_hysteresis(self, hysteresis):
        """Set hysteresis. 0 = 0.5 degree Celsius, 1 = 1 degree Celsius"""
        if hysteresis.lower() not in HYSTERESIS_MODES:
            _LOGGER.error(
                "[%s] Error in async_set_hysteresis. Unknown hysteresis '%s'.",
                self._host,
                hysteresis,
            )
            return
        await self._try_command(
            "Error in set_hysteresis",
            self._hysen_device.set_hysteresis,
            HASS_HYSTERESIS_TO_HYSEN[hysteresis.lower()],
        )

    async def async_set_calibration(self, calibration):
        """Set temperature calibration. Range -5~+5 degree Celsius in 0.1 degree Celsius step."""
        await self._try_command(
            "Error in set_calibration", self._hysen_device.set_calibration, calibration
        )

    async def async_set_cooling_max_temp(self, temp):
        """Set cooling upper limit."""
        await self._try_command(
            "Error in set_cooling_max_temp",
            self._hysen_device.set_cooling_max_temp,
            temp,
        )

    async def async_set_cooling_min_temp(self, temp):
        """Set cooling lower limit."""
        await self._try_command(
            "Error in set_cooling_min_temp",
            self._hysen_device.set_cooling_min_temp,
            temp,
        )

    async def async_set_heating_max_temp(self, temp):
        """Set heating upper limit."""
        await self._try_command(
            "Error in set_heating_max_temp",
            self._hysen_device.set_heating_max_temp,
            temp,
        )

    async def async_set_heating_min_temp(self, temp):
        """Set heating lower limit."""
        await self._try_command(
            "Error in set_heating_min_temp",
            self._hysen_device.set_heating_min_temp,
            temp,
        )

    async def async_set_fan_control(self, fan_control):
        """Set fan coil control mode, 0 = Fan is stopped when target temp reached, 1 = Fan is spinning when target temp reached."""
        await self._try_command(
            "Error in set_fan_control",
            self._hysen_device.set_fan_control,
            HASS_FAN_CONTROL_TO_HYSEN[fan_control],
        )

    async def async_set_frost_protection(self, frost_protection):
        """Set frost_protection 0 = Off, 1 = When power off keeps the room temp between 5 to 7 degree."""
        await self._try_command(
            "Error in set_frost_protection",
            self._hysen_device.set_frost_protection,
            HASS_FROST_PROTECTION_TO_HYSEN[frost_protection],
        )

    async def async_set_time_now(self):
        """Set device time to system time."""
        clock_weekday = int(dt_util.as_local(dt_util.now()).strftime("%w"))
        if clock_weekday == 0:
            clock_weekday = 7
        clock_hour = int(dt_util.as_local(dt_util.now()).strftime("%H"))
        clock_min = int(dt_util.as_local(dt_util.now()).strftime("%M"))
        clock_sec = int(dt_util.as_local(dt_util.now()).strftime("%S"))
        await self._try_command(
            "Error in set_time",
            self._hysen_device.set_time,
            clock_hour,
            clock_min,
            clock_sec,
            clock_weekday,
        )

    async def async_set_schedule(self, schedule):
        """Set schedule mode 0 = Today 1 = 12345,67 2 = 123456,7 3 = 1234567."""
        if schedule.lower() not in SCHEDULE_MODES:
            _LOGGER.error(
                "[%s] Error in async_set_schedule. Unknown schedule mode '%s'.",
                self._host,
                schedule,
            )
            return
        await self._try_command(
            "Error in set_weekly_schedule",
            self._hysen_device.set_weekly_schedule,
            HASS_SCHEDULE_TO_HYSEN[schedule.lower()],
        )

    async def async_set_period1_on(self, enable=None, hour=None, min=None):
        """Set period 1 start."""
        await self._try_command(
            "Error in set_period1_on",
            self._hysen_device.set_period1_on,
            HASS_PERIOD_ENABLED_TO_HYSEN[enable],
            hour,
            min,
        )

    async def async_set_period1_off(self, enable=None, hour=None, min=None):
        """Set period 1 end."""
        await self._try_command(
            "Error in set_period1_off",
            self._hysen_device.set_period1_off,
            HASS_PERIOD_ENABLED_TO_HYSEN[enable],
            hour,
            min,
        )

    async def async_set_period2_on(self, enable=None, hour=None, min=None):
        """Set period 2 start."""
        await self._try_command(
            "Error in set_period2_on",
            self._hysen_device.set_period2_on,
            HASS_PERIOD_ENABLED_TO_HYSEN[enable],
            hour,
            min,
        )

    async def async_set_period2_off(self, enable=None, hour=None, min=None):
        """Set period 2 end."""
        await self._try_command(
            "Error in set_period2_off",
            self._hysen_device.set_period2_off,
            HASS_PERIOD_ENABLED_TO_HYSEN[enable],
            hour,
            min,
        )

    async def async_authenticate_device(self):
        """Connect to device ."""
        try:
            _authenticated = await self.hass.async_add_executor_job(
                self._hysen_device.auth
            )
            if _authenticated:
                _LOGGER.debug("[%s] Device authenticated.", self._host)
            else:
                _LOGGER.debug("[%s] Device not authenticated.", self._host)
        except Exception as exc:
            _LOGGER.error("[%s] Device authentication error: %s", self._host, exc)
            _authenticated = False
        return _authenticated

    async def async_get_device_status(self):
        """Get device status."""
        await self._try_command(
            "Error in get_device_status", self._hysen_device.get_device_status
        )

    async def _try_command(self, mask_error, func, *args, **kwargs):
        """Calls a device command and handle error messages."""
        self._device_available = True
        try:
            await self.hass.async_add_executor_job(partial(func, *args, **kwargs))
        except socket.timeout as timeout_error:
            _LOGGER.error("[%s] %s: %s", self._host, mask_error, timeout_error)
            self._device_available = False
        except Exception as exc:
            _LOGGER.error("[%s] %s: %s", self._host, mask_error, exc)
            self._device_available = False

    async def async_update(self):
        """Get the latest state from the device."""
        if self._device_authenticated is False:
            self._device_authenticated = await self.async_authenticate_device()
            if self._device_authenticated is False:
                await self.async_get_device_status()
            if self._device_available:
                self._device_authenticated = True
        if self._device_authenticated:
            await self.async_get_device_status()
            _weekday = int(dt_util.as_local(dt_util.now()).strftime("%w"))
            if self._device_available:
                if _weekday == 0:
                    _weekday = 7
                if (
                    (self._hysen_device.clock_weekday != _weekday)
                    or (
                        self._hysen_device.clock_hour
                        != int(dt_util.as_local(dt_util.now()).strftime("%H"))
                    )
                    or (
                        self._hysen_device.clock_min
                        != int(dt_util.as_local(dt_util.now()).strftime("%M"))
                    )
                ):
                    await self.async_set_time_now()
