import struct
import sys
import time

import cv2
import numpy as np
import serial

# --- CONFIGURATION ---
# Change this to match your OpenMV serial port on macOS
SERIAL_PORT = "/dev/cu.usbmodem3175345F31302"
BAUD_RATE = 115200


def main():
    try:
        # Open the serial port
        print(f"Connecting to {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Connected successfully.")
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
        print("Please check the port name and ensure the OpenMV is plugged in.")
        sys.exit(1)

    print(
        "Listening for high-speed binary video stream... Press 'q' in the video window to quit."
    )

    try:
        while True:
            # Sync with the stream by looking for our magic start sequence "SNAP"
            # ser.read_until() reads bytes until the sequence is found or it times out
            sync_data = ser.read_until(b"SNAP")

            if not sync_data.endswith(b"SNAP"):
                # Timeout occurred or garbage data, keep trying
                continue

            # Read the 12-byte header: <ffI (2 floats, 1 unsigned int, little-endian)
            header_data = ser.read(12)
            if len(header_data) != 12:
                continue

            # Unpack the binary header directly
            nx, ny, img_size = struct.unpack("<ffI", header_data)

            # Basic sanity check to prevent huge memory allocations on serial corruption
            if img_size == 0 or img_size > 500000:
                continue

            # Read the exact number of bytes specified in the header for the image
            img_data = ser.read(img_size)

            if len(img_data) == img_size:
                # Decode the JPEG data using OpenCV
                np_arr = np.frombuffer(img_data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    height, width = frame.shape[:2]

                    # If tracking is active (nx and ny are not -1.0)
                    if nx >= 0.0 and ny >= 0.0:
                        # Convert normalized coordinates back to pixel coordinates
                        cx = int(nx * width)
                        cy = int(ny * height)

                        # Draw a filled red circle at the tracked position
                        cv2.circle(
                            frame,
                            (cx, cy),
                            radius=6,
                            color=(0, 0, 255),
                            thickness=-1,
                        )

                    # Display the image
                    cv2.imshow("Machine Vision - Fast Binary Tracking", frame)

                    # Check for 'q' key to quit (1 ms delay allows GUI to update)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("Quitting...")
                        break

    except KeyboardInterrupt:
        print("Interrupted by user.")
    except Exception as e:
        print(f"Unexpected error: {e}")
        time.sleep(0.1)
    finally:
        # Cleanup
        print("Disconnecting and cleaning up...")
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
