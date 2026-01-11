"""
Docstring for bridge
"""

from signals import *
from pydantic import BaseModel, Field

class Sensor(BaseModel):
    data: list[float]

    def read(self):
        return self.data

class Magnatometer(Sensor):
    data = [0.0] * 3

    def update(self, states: STATES):
        pass

class Accelerometer(Sensor):
    pass

class Gyro(Sensor):
    pass

class Barometer(Sensor):
    pass

class Thermometer(Sensor):
    pass

class Converter:
    """
    Docstring for Converter
    """
    def _fill_sensor(self, )

    def from_vehicle(self, data: HIL_VEHICLE_STATE) -> HIL_SEND:
        """
        Docstring for from_plant
        
        :param self: Description
        :param data: Description
        :type data: tuple[STATES, DCM]
        """
        send: HIL_SEND = HIL_SEND()

        sensor = send.sensor
        gps = send.gps
        rc_inputs = send.rc_inputs
        quat = send.quat
        flag = send.flag
        heartbeat = send.heartbeat
        time = send.time

        return send

    def to_vehicle(self, data: HIL_REC) -> HIL_ACTUATOR_CTL:
        """
        Docstring for to_plant
        
        :param self: Description
        :param data: Description
        :type data: HIL_REC
        :return: Description
        :rtype: HIL_ACTUATOR_CTL
        """
        return data.actuator_controls
