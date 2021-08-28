from flask import Flask, render_template
from flask_classful import FlaskView, route
from firebaseSocket import Socket
import pytz
from werkzeug.routing import IntegerConverter

app = Flask(__name__)


class Gui(FlaskView):
    def __init__(self):
        self._utc = pytz.UTC
        self._socket = Socket({"name": "client"}, credentialsFile="firebaseCredentials.json")
        self._socket.bind_db_listener(self.on_snapshot)
        self._simulation_data = {
            'arm_actuator': 0,
            'crab_actuator': 0,
            'hoist_actuator': 0,
            'crab_rotation_actuator': 0,
            'suction_pad': False,
            'suction_pad_force_sensor': False
        }
        # Resetting firebase db data
        self._socket.set_db_data(self._simulation_data)

    def on_snapshot(self, _data):
        db_data = self._socket.get_db_data()
        self._simulation_data = db_data

    def update_simulation_data(self, data):
        self._socket.update_db_data(data)

    @route('/simulation_data')
    def simulation_data(self):
        return self._simulation_data

    @route('/gui')
    def gui(self):
        return render_template('index.html')

    @route('/update_arm_actuator/<signed_int:value>', methods=['GET', 'POST'])
    def update_arm_actuator(self, value):
        self.update_simulation_data({
            'arm_actuator': value
        })
        return self._simulation_data

    @route('/update_crab_actuator/<signed_int:value>', methods=['GET', 'POST'])
    def update_crab_actuator(self, value):
        self.update_simulation_data({
            'crab_actuator': value
        })
        return self._simulation_data

    @route('/update_hoist_actuator/<signed_int:value>', methods=['GET', 'POST'])
    def update_hoist_actuator(self, value):
        self.update_simulation_data({
            'hoist_actuator': value
        })
        return self._simulation_data

    @route('/update_crab_rotation_actuator/<signed_int:value>', methods=['GET', 'POST'])
    def update_crab_rotation_actuator(self, value):
        self.update_simulation_data({
            'crab_rotation_actuator': value
        })
        return self._simulation_data

    @route('/update_suction_pad/<signed_int:value>', methods=['GET', 'POST'])
    def update_suction_pad(self, value):
        self.update_simulation_data({
            'suction_pad': bool(value)
        })
        return self._simulation_data

    @route('/')
    def hello_world(self):
        return 'Lab 3 - Grupo 1'


class SignedIntConverter(IntegerConverter):
    regex = r'-?\d+'


app.url_map.converters['signed_int'] = SignedIntConverter

Gui().register(app, route_base="/")

if __name__ == '__main__':
    app.run(debug=True)
