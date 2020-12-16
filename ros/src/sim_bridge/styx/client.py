import socketio

sio = socketio.Client()

@sio.event
def connect():
    print("Connected")

@sio.event
def connect_error():
    print("The connection failed!")

@sio.event
def disconnect():
    print("I'm disconnected!")

sio.connect('http://127.0.0.1:4567')
print("my sid is", sio.sid)
