import coppelia.sim as sim
import sys
from coppelia.calculus import Calculus


class Simulation:
    def __init__(self, addr="127.0.0.1", port=19999,
                 wait_connected=True, no_reconnect=True,
                 timeout=5000, comm_thread_cycle=5,
                 close_connections=True):
        """
        Main coppelia class
        @param addr: Connection address
        @param port: Connection port
        @param wait_connected: Freeze execution until loading is complete
        @param no_reconnect: Disables auto reconnect
        @param timeout: Timeout on freezing in ms
        @param comm_thread_cycle: Communication thread cycle in ms
        @param close_connections: Close previous connections if existent
        """
        if close_connections:
            sim.simxFinish(-1)
        self.clientID = sim.simxStart(connectionAddress=addr,
                                      connectionPort=port,
                                      waitUntilConnected=wait_connected,
                                      doNotReconnectOnceDisconnected=no_reconnect,
                                      timeOutInMs=timeout,
                                      commThreadCycleInMs=comm_thread_cycle)
        self.check_connection()
        self.arm_handle = self.__start_handle("Arm_control")
        self.cable_handle = self.__start_handle("cabo")
        self.sensor = self.__start_handle("Proximity_sensor")
        self.coin = self.__start_handle("50cents")


    @staticmethod
    def __throw_error(message):
        print(message)
        sim.simxFinish(-1)
        sys.exit()

    def __start_handle(self, handle):
        return_code, handle = sim.simxGetObjectHandle(self.clientID, handle,
                                                      sim.simx_opmode_blocking)
        return handle if return_code == 0 else \
            self.__throw_error(f"Failed to start handle: {handle}")

    def __get_joint_position(self, handle):
        return_code, p = sim.simxGetJointPosition(self.clientID, handle,
                                                  sim.simx_opmode_streaming)
        # FIXME: Error on the first run of each handle get position,
        #  when fixed reactivate error throwing
        return p if return_code == 0 else \
            print("Failed to get handle position")
            # self.__throw_error("Failed to get handle position")

    def __set_joint_position(self, handle, target):
        sim.simxSetJointTargetPosition(self.clientID, handle, target,
                                       sim.simx_opmode_streaming)

    def __get_proximity_sensor_data(self, handle):
        return sim.simxReadProximitySensor(self.clientID, handle,
                                           sim.simx_opmode_streaming)

    def __has_child(self, handle):
        erro, child = sim.simxGetObjectChild(self.clientID, handle,
                                             0, sim.simx_opmode_streaming)
        if erro == 0:
            return False if child == -1 else True
        else:
            print("Failed to get handle child")
            return False

    def check_connection(self, **kwargs):
        """
        Checks if connection is alive
        """
        if self.clientID == -1:
            self.__throw_error("Disconnected from remote API")
        else:
            print(f"Connected to remote API server. Client ID: {self.clientID}")

    def get_arm_position(self, **kwargs):
        """
        Get arm handle position in degrees
        @return: Arm position
        """
        return Calculus.rad_to_degree(self.__get_joint_position(self.arm_handle))

    def get_cable_position(self, **kwargs):
        """
        Get cable handle position
        @return: Cable position
        """
        return Calculus.round_cable_position(self.__get_joint_position(self.cable_handle))

    def get_sensor_detection(self, **kwargs):
        """
        Get boolean 0of detection with the proximity sensor
        @return: bool of the detection
        """
        return self.__get_proximity_sensor_data(self.sensor)[1]

    def get_sensor_distance(self, **kwargs):
        return Calculus.round_sensor_distance(self.__get_proximity_sensor_data(self.sensor)[2][2])

    def get_coin_connection(self, **kwargs):
        """
        Get boolen of the coin's child
        :return: bool if the coin is connected
        """
        return self.__has_child(self.coin)

    def set_arm_position(self, **kwargs):
        """
        Sets arm position in degrees
        @param target: Target position in degrees
        @return: 0 if success; -1 if failed
        """
        return self.__set_joint_position(self.arm_handle,
                                         Calculus.degree_to_rad(kwargs["target"]))

    def set_cable_position(self, **kwargs):
        """
        Sets cable vertical position
        @param target: Target position
        @return: 0 if success; -1 if failed
        """
        return self.__set_joint_position(self.cable_handle,
                                         Calculus.format_vertical(kwargs["target"]))

    def turn_magnet(self, **kwargs):
        """
        Turn the magnet on and off
        """
        sim.simxCallScriptFunction(self.clientID, "Base",
                                   sim.sim_scripttype_childscript,
                                   "turn_suction",
                                   [], [], [], "0", sim.simx_opmode_blocking)

    def turn_arm_relative(self, **kwargs):
        pos = self.get_arm_position()
        self.set_arm_position(target=pos + kwargs["target"])

    def move_cable_relative(self, **kwargs):
        pos = self.get_cable_position()
        self.set_cable_position(target=min(0, pos + kwargs["target"]))

    def load_event(self, event):
        """
        Load any method call received remotely
        """
        getattr(self, event["method"])(target=event["target"],
                                       handle=event["handle"])

