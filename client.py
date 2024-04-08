import socket

SERVICE_CENTER = ("192.168.30.51", 8000)

eyecar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
eyecar.connect(SERVICE_CENTER)  #connect to the server port
print("Connected to", SERVICE_CENTER)

while True:
    com  =input()
    eyecar.send(com.encode())#'240:150:200:0:0#'
                               # 240:150:280:0:0#
