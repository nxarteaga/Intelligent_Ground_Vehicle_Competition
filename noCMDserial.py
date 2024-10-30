import serial
import threading

def read_serial(ser):
    while True:
        try:
            if ser.in_waiting > 0:  # Check if there's data in the input buffer
                data = ser.readline().decode('utf-8').rstrip()
                if data:
                    print(f"Received: {data}")
        except serial.SerialException as e:
            print(f"Serial Exception: {e}")
            break  # Exit the loop if there's an issue
        except Exception as e:
            print(f"Unexpected error: {e}")
            break  # Exit on unexpected error

def main():
    port = '/dev/serial0'  # Specify the port directly
    
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=1)  # Set timeout
        ser.setRTS(False)
        ser.setDTR(False)

        # Start a thread to read from the serial port
        serial_recv_thread = threading.Thread(target=read_serial, args=(ser,))
        serial_recv_thread.daemon = True  # Daemonize thread
        serial_recv_thread.start()

        try:
            while True:
                # Accept command input from the user
                command = input("Enter command: ")
                ser.write(command.encode() + b'\n')
        except KeyboardInterrupt:
            pass
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
