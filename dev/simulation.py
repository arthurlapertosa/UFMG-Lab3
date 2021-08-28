# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
from base64 import b64encode
import numpy as np
import io
from PIL import Image
import sim
from firebaseSocket import Socket
import time
import eventlet
import socketio


class Simulation:
    def __init__(self):
        print('Program started')
        sim.simxFinish(-1)  # just in case, close all opened connections
        self._clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

        self._socket = Socket({"name": "client"}, credentialsFile="firebaseCredentials.json")
        self._socket.bind_db_listener(self.on_snapshot)

        if self._clientID != -1:
            print('Connected to remote API server')
            sim.simxAddStatusbarMessage(self._clientID, 'Hello CoppeliaSim!', sim.simx_opmode_oneshot)
            self._arm_actuator = self.start_handle("Arm_actuator")
            self._crab_actuator = self.start_handle("Crab_actuator")
            self._hoist_actuator = self.start_handle("Hoist_actuator")
            self._crab_rotation_actuator = self.start_handle("Crab_Rotation_actuator")
            self._suction_pad_status = False
            self._proximity_sensor = self.start_handle("Proximity_sensor")
            self._vision_sensor = self.start_handle("Vision_sensor")
            self._vision_sensor_0 = self.start_handle("Vision_sensor1")
            self._container1 = self.start_handle("Container1")
            self._container2 = self.start_handle("Container2")
            self._container3 = self.start_handle("Container3")
            self._container4 = self.start_handle("Container4")

            self.sio = socketio.Server(cors_allowed_origins="*")
            self.app = socketio.WSGIApp(self.sio)

            @self.sio.event
            def connect(sid, env):
                print('connect ', sid)
                self.sio.emit('connected', {'data': sid})

            @self.sio.event
            def disconnect(sid):
                print('disconnect ', sid)
                self.sio.emit('disconnect', {'data': sid})

            @self.sio.event
            def telemetry(sid):
                camera_image = self.get_vision_sensor_base_64_image(self._vision_sensor)
                camera_image_0 = self.get_vision_sensor_base_64_image(self._vision_sensor_0)
                x, arm_data = self.get_joint_position(self._arm_actuator)
                x, crab_data = self.get_joint_position(self._crab_actuator)
                x, hoist_data = self.get_joint_position(self._hoist_actuator)
                x, crab_rotation_data = self.get_joint_position(self._crab_rotation_actuator)
                proximity_detection, proximity_data = self.get_proximity_sensor_data()
                containers_data = [
                    self.is_connected(self._container1),
                    self.is_connected(self._container2),
                    self.is_connected(self._container3),
                    self.is_connected(self._container4)
                ]
                # print(proximity_data)

                self.sio.emit('message', {
                    'camera_image': camera_image,
                    'camera_image_0': camera_image_0,
                    'arm_data': arm_data,
                    'crab_data': crab_data,
                    'hoist_data': hoist_data,
                    'crab_rotation_data': crab_rotation_data,
                    'proximity_detection': proximity_detection,
                    'proximity_data': proximity_data,
                    'containers_data': containers_data
                })

        else:
            print('Failed connecting to remote API server')

    def start(self):
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(self._clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print('Number of objects in the scene: ', len(objs))
            self.sio.emit('message', {'data': len(objs)})
        else:
            print('Remote API function call returned with error code: ', res)

    def on_snapshot(self, _data):
        db_data = self._socket.get_db_data()
        sim.simxAddStatusbarMessage(self._clientID, 'Atualizando dados...', sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self._clientID, self._arm_actuator, db_data["arm_actuator"]/1000, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self._clientID, self._crab_actuator, db_data["crab_actuator"]/100, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self._clientID, self._hoist_actuator, db_data["hoist_actuator"]/100, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self._clientID, self._crab_rotation_actuator, db_data["crab_rotation_actuator"]/1000, sim.simx_opmode_streaming)
        self.handle_suction_pad(db_data["suction_pad"])

    def stop(self):
        sim.simxAddStatusbarMessage(self._clientID, 'Bye CoppeliaSim!', sim.simx_opmode_oneshot)
        sim.simxGetPingTime(self._clientID)
        sim.simxFinish(self._clientID)

    def start_handle(self, handle):
        return_code, handle = sim.simxGetObjectHandle(self._clientID, handle, sim.simx_opmode_blocking)
        return handle if return_code == 0 else print(f'Failed to start handle: {handle}')

    def handle_suction_pad(self, status):
        if status != self._suction_pad_status:
            self._suction_pad_status = status
            sim.simxCallScriptFunction(self._clientID, "Base",
                                       sim.sim_scripttype_childscript,
                                       "actuateMagnet",
                                       [], [], [], "0", sim.simx_opmode_blocking)

    def get_proximity_sensor_data(self):
        proximity_data = sim.simxReadProximitySensor(self._clientID, self._proximity_sensor, sim.simx_opmode_streaming)
        return proximity_data[1], proximity_data[2][2]

    def get_vision_sensor_base_64_image(self, vision_sensor):
        error_code, resolution, image = sim.simxGetVisionSensorImage(self._clientID, vision_sensor, 0, sim.simx_opmode_streaming)
        # print('vision_image',  error_code, resolution, image)

        if resolution:
            # Process the image to the format (64,64,3)
            sensor_image = []
            sensor_image = np.array(image, dtype=np.uint8)
            sensor_image.resize([resolution[0], resolution[1], 3])

            file_object = io.BytesIO()
            img = Image.fromarray(sensor_image.astype('uint8'))
            img.save(file_object, 'PNG')
            base64img = "data:image/png;base64," + b64encode(file_object.getvalue()).decode('ascii')

            return base64img

    def get_joint_position(self, joint):
        return sim.simxGetJointPosition(self._clientID, joint, sim.simx_opmode_streaming)

    def is_connected(self, handle):
        erro, child = sim.simxGetObjectChild(self._clientID, handle, 0, sim.simx_opmode_streaming)
        if erro == 0:
            return False if child == -1 else True
        else:
            return False


if __name__ == '__main__':
    simulation = Simulation()

    simulation.start()
    eventlet.wsgi.server(eventlet.listen(('', 8765)), simulation.app)

    # time.sleep(30)
    # simulation.stop()
