import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_DEVICE_CLASS,
    DEVICE_CLASS_CONNECTIVITY,
    CONF_PORT,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "async_tcp"]

status_ns = cg.esphome_ns.namespace("gcodestreamer")
PrinterBinarySensor = status_ns.class_(
    "GCodeStreamer", binary_sensor.BinarySensor, cg.Component, uart.UARTDevice
)

CONFIG_SCHEMA = (
    binary_sensor.BINARY_SENSOR_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(PrinterBinarySensor),
            cv.Optional(
                CONF_DEVICE_CLASS, default=DEVICE_CLASS_CONNECTIVITY
            ): binary_sensor.device_class,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(
        {
            cv.Optional(CONF_PORT, default=1212): cv.port,
        }
    )
)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_port(config[CONF_PORT]))
    yield cg.register_component(var, config)
    yield binary_sensor.register_binary_sensor(var, config)
    yield uart.register_uart_device(var, config)
