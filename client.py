import socket

SERVICE_CENTER = ("127.0.0.1", 8000)

eyecar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
eyecar.connect(SERVICE_CENTER)  #connect to the server port
print("Connected to", SERVICE_CENTER)

#1
eyecar.send('240:150:300:0:0#'.encode())