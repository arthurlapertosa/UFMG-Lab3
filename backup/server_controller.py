from connection.fbsocket import Socket
from coppelia.simulation import Simulation
from datetime import datetime, timedelta
import pytz
import time


class Controller:
    def __init__(self, update_frequency=0.8):
        self._socket = Socket({"name": "server"})
        self._simulation = Simulation()
        self._simulation_status = {"arm_position": 0, "cable_position": 0,
                                   "detection": 0, "distance": 0}
        self._socket.bind_command_event_listener(self.on_snapshot)
        self._utc = pytz.UTC
        self.update_frequency = update_frequency

    def _get_sim_status(self):
        return {"arm_position": self._simulation.get_arm_position(),
                "cable_position": self._simulation.get_cable_position(),
                "detection": self._simulation.get_sensor_detection(),
                "distance": self._simulation.get_sensor_distance(),
                "coin": self._simulation.get_coin_connection()}

    def on_snapshot(self, col_snapshot, changes, read_time):
        for doc in col_snapshot:
            event = doc.to_dict()
            ts_server = event["timestamp"].replace(tzinfo=self._utc)
            ts_now = (datetime.now() + timedelta(seconds=10798)).replace(
                tzinfo=self._utc)
            if ts_server > ts_now:
                self._simulation.load_event(event)

    def publish_status_event(self):
        new_simulation_status = self._get_sim_status()
        if new_simulation_status != self._simulation_status:
            self._simulation_status = new_simulation_status
            self._socket.publish_status_event(*tuple(self._simulation_status.values()))

    def auto_update_gui(self):
        while True:
            self.publish_status_event()
            time.sleep(self.update_frequency)


if __name__ == '__main__':
    controller = Controller()
    controller.auto_update_gui()
