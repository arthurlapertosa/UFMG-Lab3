from flask_classful import FlaskView, route
from connection.fbsocket import Socket
from datetime import datetime, timedelta
import pytz
from flask import *

app = Flask(__name__)


class GUI(FlaskView):
    def __init__(self):
        self._utc = pytz.UTC
        self._socket = Socket({"name": "client"}, certificate="credentials.json")
        self._socket.bind_status_event_listener(self.on_snapshot)
        self._simulation_status = {"arm_position": 0, "cable_position": 0,
                                   "detection": False, "distance": 0,
                                   "coin": False}

    def _update_sim_status(self, event):
        self._simulation_status["arm_position"] = event["arm_position"]
        self._simulation_status["cable_position"] = event["cable_position"]
        self._simulation_status["detection"] = event["detection"]
        self._simulation_status["distance"] = event["distance"]
        self._simulation_status["coin"] = event["coin"]

    def on_snapshot(self, col_snapshot, changes, read_time):
        for doc in col_snapshot:
            event = doc.to_dict()
            ts_server = event["timestamp"].replace(tzinfo=self._utc)
            ts_now = (datetime.now() + timedelta(seconds=10799)).replace(
                tzinfo=self._utc)
            if ts_server > ts_now:
                # FIXME: se tiver algo lento é aqui
                self._update_sim_status(event)

    def index(self):
        return render_template('index.html')

    @route('/rotate_anti_hour', methods=['POST'])
    def rotate_anti_hour(self):
        rotation_dt = request.form['rotation_dt'] if request.form['rotation_dt'] else 10
        self._socket.publish_command_event('turn_arm_relative', float(rotation_dt))
        return json.dumps({'status': 'OK', 'method': 'rotate_anti_hour',
                           'rotation_dt': rotation_dt})

    @route('/rotate_hour', methods=['POST'])
    def rotate_hour(self):
        rotation_dt = request.form['rotation_dt'] if request.form['rotation_dt'] else 10
        self._socket.publish_command_event('turn_arm_relative', -float(rotation_dt))
        return json.dumps({'status': 'OK', 'method': 'rotate_hour',
                           'rotation_dt': rotation_dt})

    @route('/cable_up', methods=['POST'])
    def cable_up(self):
        cable_dt = request.form['cable_dt'] if request.form['cable_dt'] else 10
        self._socket.publish_command_event('move_cable_relative', float(cable_dt))
        return json.dumps({'status': 'OK', 'method': 'cable_up',
                           'cable_dt': cable_dt})

    @route('/cable_down', methods=['POST'])
    def cable_down(self):
        cable_dt = request.form['cable_dt'] if request.form['cable_dt'] else 10
        self._socket.publish_command_event('move_cable_relative', -float(cable_dt))
        return json.dumps({'status': 'OK', 'method': 'cable_down',
                           'cable_dt': cable_dt})

    @route('/turn_magnet', methods=['POST'])
    def turn_magnet(self):
        self._socket.publish_command_event('turn_magnet')
        return json.dumps({'status': 'OK', 'method': 'turn_magnet'})

    @route('/update_values', methods=['GET'])
    def update_values(self):
        return jsonify(arm_position=self._simulation_status["arm_position"],
                       cable_position=self._simulation_status["cable_position"],
                       detection=self._simulation_status["detection"],
                       distance=self._simulation_status["distance"],
                       coin=self._simulation_status["coin"])


GUI().register(app, route_base="/")
app.run(debug=True, port=6969)
