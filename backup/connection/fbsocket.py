import firebase_admin
from firebase_admin import credentials, firestore
from datetime import datetime, timedelta


class Socket:
    def __init__(self, app_name="app", certificate='credentials.json'):
        cred = credentials.Certificate(certificate)
        if not firebase_admin._apps:
            firebase_admin.initialize_app(cred, app_name)
        self.db = firestore.client()

    @staticmethod
    def __get_clock():
        return datetime.now() + timedelta(seconds=10798)

    def __publish_event(self, data, msg_type):
        self.db.collection(msg_type).document().set(data)

    def bind_command_event_listener(self, method):
        events_query = self.db.collection(u'commands')\
            .where(u'timestamp', '>', self.__get_clock())
        events_query.on_snapshot(method)

    def bind_status_event_listener(self, method):
        events_query = self.db.collection(u'status')\
            .where(u'timestamp', '>', self.__get_clock())
        events_query.on_snapshot(method)

    def publish_command_event(self, method, target=None, handle=None):
        data = {
            u'method': method,
            u'target': target,
            u'handle': handle,
            u'timestamp': firestore.SERVER_TIMESTAMP
        }
        self.__publish_event(data, u'commands')

    def publish_status_event(self, arm_position, cable_position, detection,
                             distance, coin):
        data = {
            u'arm_position': arm_position,
            u'cable_position': cable_position,
            u'detection': detection,
            u'distance': distance,
            u'coin': coin,
            u'timestamp': firestore.SERVER_TIMESTAMP
        }
        self.__publish_event(data, u'status')
