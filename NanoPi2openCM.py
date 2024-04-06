# import serial
# import serial.tools.list_ports

# port = "/dev/ttyS1"
# baudrate = 9600


# def send_data(data):
#     try:
#         ser = serial.Serial(port, baudrate=baudrate, timeout=1)
#         print("Serial connection established.")

#         print(data)
#         ser.write(data.encode())


#     except ValueError as ve:
#         print("Error:", str(ve))

#     except serial.SerialException as se:
#         print("Serial port error:", str(se))

#     except Exception as e:
#         print("An error occurred:", str(e))

        
#     finally:
#         if ser.is_open:
#             ser.close()
#             print("Serial connection closed.")


import serial
import serial.tools.list_ports

class SerialConnection:

    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None


    def open(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=1)
            print("Serial connection established.")
            return self.ser
        except ValueError as ve:
            print("Error:", str(ve))
            return None
        except serial.SerialException as se:
            print("Serial port error:", str(se))
            return None
        except Exception as e:
            print("An error occurred:", str(e))
            return None


    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")


    def send_data(self, data):
        if self.ser is not None:
            print("Data to transmission", data)
            self.ser.write(data.encode())






