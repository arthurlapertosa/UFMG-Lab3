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

import sim
from firebaseSocket import Socket
import time


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
            self._suction_pad_status = False
        else:
            print('Failed connecting to remote API server')

    def start(self):
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(self._clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print('Number of objects in the scene: ', len(objs))
        else:
            print('Remote API function call returned with error code: ', res)

    def on_snapshot(self, _data):
        db_data = self._socket.get_db_data()
        sim.simxAddStatusbarMessage(self._clientID, 'Atualizando dados...', sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self._clientID, self._arm_actuator, db_data["arm_actuator"]/1000, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self._clientID, self._crab_actuator, db_data["crab_actuator"]/100, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self._clientID, self._hoist_actuator, db_data["hoist_actuator"]/100, sim.simx_opmode_streaming)
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

if __name__ == '__main__':
    simulation = Simulation()

    simulation.start()
    # time.sleep(30)
    # simulation.stop()
