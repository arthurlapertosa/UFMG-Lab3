from flask import Flask, render_template
from flask_classful import FlaskView, route
from firebaseSocket import Socket
import pytz

app = Flask(__name__)


class Gui(FlaskView):
    def __init__(self):
        self._utc = pytz.UTC
        self._socket = Socket({"name": "client"}, credentialsFile="firebaseCredentials.json")
        self._socket.bind_db_listener(self.on_snapshot)
        self._simulation_data = {
            "test": 0,
            "test2": 0
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

    @route('/increment_test', methods=['GET', 'POST'])
    def increment_test(self):
        self.update_simulation_data({
            "test": self._simulation_data["test"] + 10
        })
        return self._simulation_data

    @route('/update_test/<int:value>', methods=['GET', 'POST'])
    def update_test(self, value):
        self.update_simulation_data({
            "test": value
        })
        return self._simulation_data

    @route('/increment_test2', methods=['GET', 'POST'])
    def increment_test2(self):
        self.update_simulation_data({
            "test2": self._simulation_data["test2"] + 30
        })
        return self._simulation_data

    @route('/gui')
    def gui(self):
        return render_template('index.html')

@app.route('/')
def hello_world():
    return 'Lab 3 - Grupo 1'


Gui().register(app, route_base="/")

if __name__ == '__main__':
    app.run(debug=True)
