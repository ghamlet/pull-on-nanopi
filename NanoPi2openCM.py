import serial
import serial.tools.list_ports

port = "/dev/ttyS1"
baudrate = 9600


def send_data(data):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        print("Serial connection established.")

        print(data)
        ser.write(data.encode())


    except ValueError as ve:
        print("Error:", str(ve))

    except serial.SerialException as se:
        print("Serial port error:", str(se))

    except Exception as e:
        print("An error occurred:", str(e))

        
    finally:
        if ser.is_open:
            ser.close()
            print("Serial connection closed.")


