import serial
import time
import sys # For exiting script

# --- Configuration ---
SERIAL_PORT = 'COM6' # Updated based on your screenshot
BAUD_RATE = 230400
TIMEOUT = 0.5 # Shorter timeout for continuous reading is often better

# --- Command ---
# Assuming this command requests a data packet (or needs to be sent before reading one)
# !!! This might need to be changed to a 'start scan' command if polling doesn't work !!!
COMMAND_REQUEST_DATA = b'\x5A\x04\x01\x5F' # Using the command from your script

# --- Expected Response Lengths (Adjust if needed) ---
EXPECTED_LEN_RESP1 = 7
EXPECTED_LEN_RESP2 = 9 # The one containing distance/power

ser = serial.Serial()
ser.port = SERIAL_PORT
ser.baudrate = BAUD_RATE
ser.timeout = TIMEOUT

try:
    ser.open()
    print(f"Portul serial {ser.name} este deschis.")

    # --- Main Loop for Continuous Reading ---
    while True:
        try:
            # 1. Send Command (if required for each reading)
            # print(f"Trimit comanda: {COMMAND_REQUEST_DATA.hex()}") # Uncomment for debug
            ser.write(COMMAND_REQUEST_DATA)

            # 2. Read First Response (if needed, e.g., acknowledgment or status)
            #    You might be able to remove this read if response1 isn't needed every time.
            response1 = ser.read(EXPECTED_LEN_RESP1)
            if len(response1) != EXPECTED_LEN_RESP1:
                print(f"Avertisment: Timeout/eroare la citirea raspuns 1 (primit {len(response1)} bytes)")
                # Optionally clear buffer if reads seem out of sync
                ser.reset_input_buffer()
                time.sleep(0.1) # Small delay before retrying
                continue # Skip to next loop iteration

            # print(f"Raspuns 1 ({len(response1)} bytes): {response1.hex()}") # Uncomment for debug

            # 3. Read Second Response (containing the data)
            response2 = ser.read(EXPECTED_LEN_RESP2)
            if len(response2) != EXPECTED_LEN_RESP2:
                print(f"Avertisment: Timeout/eroare la citirea raspuns 2 (primit {len(response2)} bytes)")
                ser.reset_input_buffer() # Clear buffer on error
                time.sleep(0.1)
                continue # Skip to next loop iteration

            # print(f"Raspuns 2 ({len(response2)} bytes): {response2.hex()}") # Uncomment for debug

            # 4. Parse Data (Distance and Power)
            try:
                # Distance (Assuming bytes 2 & 3, little-endian, unsigned)
                # !!! Check documentation for correct indices & byteorder !!!
                dist_bytes = response2[2:4]
                dist = int.from_bytes(dist_bytes, byteorder='little', signed=False)

                # Power (Assuming bytes 4 & 5, little-endian, unsigned)
                # !!! Check documentation for correct indices & byteorder !!!
                putere_bytes = response2[4:6]
                putere = int.from_bytes(putere_bytes, byteorder='little', signed=False)

                # 5. Print Results
                print(f'Distanta: {dist:5d} mm, Putere: {putere:5d}') # Formatted print

            except (IndexError, ValueError) as e:
                print(f"Eroare la parsarea datelor: {e} - Packet: {response2.hex()}")
                ser.reset_input_buffer() # Clear buffer if parsing failed

            # 6. Control Loop Speed
            time.sleep(0.05) # Adjust for desired frequency (e.g., 0.05 -> ~20 Hz)

        except KeyboardInterrupt:
            # Allow stopping the loop with Ctrl+C
            print("\nOprire solicitata de utilizator...")
            break # Exit the while loop

except serial.SerialException as e:
    print(f"Eroare Seriala Critica: {e}")
except Exception as e:
    print(f"O eroare neasteptata a aparut: {e}")
finally:
    # Ensure the port is closed when exiting
    if ser.is_open:
        ser.close()
        print(f"Portul serial {ser.name} este inchis.")
    else:
        print("Portul serial nu a fost deschis sau este deja inchis.")
    print("Script terminat.")