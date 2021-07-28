import firebase_admin
from firebase_admin import credentials, db

from datetime import datetime, timedelta


class Socket:
    def __init__(self, app_name="app", credentialsFile='firebaseCredentials.json'):
        cred = credentials.Certificate(credentialsFile)
        if not firebase_admin._apps:
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://lab3-cd914-default-rtdb.firebaseio.com/'
            })
        self.ref = db.reference('/')

    @staticmethod
    def __get_clock():
        return datetime.now() + timedelta(seconds=10798)

    def get_db_data(self):
        return self.ref.get()

    def set_db_data(self, data):
        self.ref.set(data)

    def update_db_data(self, data):
        self.ref.update(data)

    def bind_db_listener(self, callback):
        self.ref.listen(callback)

    def publish_command_event(self, method, target=None, handle=None):
        data = {
            'target': target,
            'handle': handle,
            # 'timestamp': firestore.SERVER_TIMESTAMP
        }

        self.db.reference('/' + method).update(data)

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
