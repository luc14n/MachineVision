import queue
import struct
import threading
import time

import cv2
import numpy as np
import serial
from Kalman import TrackingKalmanFilter
from SVD import SVDKinematicFitter

# --- CONFIGURATION ---
SERIAL_PORT = "/dev/cu.usbmodem3175345F31302"
BAUD_RATE = 115200
BUFFER_SIZE = 6  # Number of frames for SVD cold start

# Thread-safe queues for inter-thread communication
frame_queue = queue.Queue(maxsize=10)
display_queue = queue.Queue(maxsize=10)
shutdown_flag = threading.Event()


def serial_reader_thread():
    """Reads binary data from the serial port and pushes to the frame queue."""
    try:
        print(f"[Serial] Connecting to {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("[Serial] Connected successfully.")
    except serial.SerialException as e:
        print(f"[Serial] Error: Could not open serial port {SERIAL_PORT}.")
        print(f"[Serial] Details: {e}")
        shutdown_flag.set()
        return

    while not shutdown_flag.is_set():
        try:
            sync_data = ser.read_until(b"SNAP")
            if not sync_data.endswith(b"SNAP"):
                continue

            header_data = ser.read(12)
            if len(header_data) != 12:
                continue

            nx, ny, img_size = struct.unpack("<ffI", header_data)

            if img_size == 0 or img_size > 500000:
                continue

            img_data = ser.read(img_size)
            if len(img_data) == img_size:
                np_arr = np.frombuffer(img_data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    # Drop old frames if the queue is falling behind
                    if frame_queue.full():
                        try:
                            frame_queue.get_nowait()
                        except queue.Empty:
                            pass

                    frame_queue.put((time.time(), nx, ny, frame))
        except Exception as e:
            print(f"[Serial] Unexpected error: {e}")
            break

    ser.close()
    print("[Serial] Thread exiting.")


def tracking_engine_thread():
    """Processes frames, manages the SVD cold start, and runs the Kalman filter."""
    fitter = SVDKinematicFitter(min_points=BUFFER_SIZE)
    kalman_filter = None

    # State tracking
    is_tracking = False
    buffer_times = []
    buffer_x = []
    buffer_y = []

    last_time = None
    lost_frames = 0
    MAX_LOST_FRAMES = 10

    while not shutdown_flag.is_set():
        try:
            timestamp, nx, ny, frame = frame_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        height, width = frame.shape[:2]
        cx, cy = -1, -1
        pred_vx, pred_vy = 0.0, 0.0
        trajectory = []

        # Valid measurement received
        if nx >= 0.0 and ny >= 0.0:
            cx = nx * width
            cy = ny * height

            if not is_tracking:
                # Accumulate buffer for SVD Cold Start
                buffer_times.append(timestamp)
                buffer_x.append(cx)
                buffer_y.append(cy)

                if len(buffer_times) >= BUFFER_SIZE:
                    try:
                        # Initialize SVD
                        initial_state = fitter.fit_2d_trajectory(
                            buffer_times, buffer_x, buffer_y
                        )

                        # Initialize Kalman
                        dt = (
                            buffer_times[-1] - buffer_times[-2]
                            if len(buffer_times) > 1
                            else 0.033
                        )
                        kalman_filter = TrackingKalmanFilter(
                            dt=max(dt, 0.001), initial_state=initial_state
                        )
                        is_tracking = True
                        last_time = timestamp
                        lost_frames = 0

                        print("[Tracker] Kalman Filter initialized via SVD!")
                    except Exception as e:
                        print(f"[Tracker] SVD Fit failed: {e}")
                        # Reset buffer on failure
                        buffer_times.clear()
                        buffer_x.clear()
                        buffer_y.clear()

                pred_x, pred_y = cx, cy  # Fallback for display while buffering

            else:
                # Tracking Mode: Predict and Update
                dt = timestamp - last_time
                last_time = timestamp

                # 1. Predict
                kalman_filter.predict(dt=max(dt, 0.001))

                # 2. Check Gate and Update
                measurement = (cx, cy)
                if kalman_filter.check_gate(measurement, threshold=1e5):
                    lost_frames = 0
                    state = kalman_filter.update(measurement)
                    pred_x, pred_y = state["x"], state["y"]
                    pred_vx, pred_vy = state["vx"], state["vy"]
                else:
                    # Measurement rejected by Mahalanobis gate, trust prediction
                    lost_frames += 1
                    if lost_frames > MAX_LOST_FRAMES:
                        is_tracking = False
                        buffer_times.clear()
                        buffer_x.clear()
                        buffer_y.clear()
                        pred_x, pred_y = -1, -1
                    else:
                        state = kalman_filter.get_state()
                        pred_x, pred_y = state["x"], state["y"]
                        pred_vx, pred_vy = state["vx"], state["vy"]

        else:
            # Measurement lost
            if is_tracking:
                # We lost the marker, but we can still predict its location using physics!
                lost_frames += 1
                if lost_frames > MAX_LOST_FRAMES:
                    is_tracking = False
                    buffer_times.clear()
                    buffer_x.clear()
                    buffer_y.clear()
                    pred_x, pred_y = -1, -1
                else:
                    dt = timestamp - last_time if last_time else 0.033
                    last_time = timestamp
                    kalman_filter.predict(dt=max(dt, 0.001))
                    state = kalman_filter.get_state()
                    pred_x, pred_y = state["x"], state["y"]
                    pred_vx, pred_vy = state["vx"], state["vy"]
            else:
                pred_x, pred_y = -1, -1
                buffer_times.clear()
                buffer_x.clear()
                buffer_y.clear()

        if is_tracking and kalman_filter is not None:
            trajectory = kalman_filter.predict_trajectory(steps=10)

        # Send result to GUI
        if display_queue.full():
            try:
                display_queue.get_nowait()
            except queue.Empty:
                pass

        display_queue.put(
            (
                frame,
                pred_x,
                pred_y,
                is_tracking,
                nx >= 0.0 and ny >= 0.0,
                cx,
                cy,
                pred_vx,
                pred_vy,
                trajectory,
            )
        )

    print("[Tracker] Thread exiting.")


def main():
    print("Starting Machine Vision - Fast Binary Tracking Engine")

    # Start worker threads
    serial_thread = threading.Thread(target=serial_reader_thread, daemon=True)
    tracker_thread = threading.Thread(target=tracking_engine_thread, daemon=True)

    serial_thread.start()
    tracker_thread.start()

    print(
        "Listening for high-speed binary video stream... Press 'q' in the video window to quit."
    )

    try:
        while not shutdown_flag.is_set():
            try:
                (
                    frame,
                    pred_x,
                    pred_y,
                    is_tracking,
                    has_measurement,
                    meas_x,
                    meas_y,
                    pred_vx,
                    pred_vy,
                    trajectory,
                ) = display_queue.get(timeout=0.1)

                # Draw measurement if present
                if has_measurement:
                    cv2.circle(
                        frame,
                        (int(meas_x), int(meas_y)),
                        radius=4,
                        color=(0, 255, 0),
                        thickness=-1,
                    )

                # Draw Kalman prediction
                if is_tracking and pred_x >= 0 and pred_y >= 0:
                    cv2.circle(
                        frame,
                        (int(pred_x), int(pred_y)),
                        radius=8,
                        color=(0, 0, 255),
                        thickness=2,
                    )
                    # Draw velocity vector line (where it thinks it will be in ~0.2 seconds)
                    end_x = int(pred_x + pred_vx * 0.2)
                    end_y = int(pred_y + pred_vy * 0.2)
                    cv2.line(
                        frame,
                        (int(pred_x), int(pred_y)),
                        (end_x, end_y),
                        (0, 165, 255),  # Orange line
                        2,
                    )
                    # Draw predicted trajectory and covariance ellipses
                    if trajectory:
                        pts = np.array(
                            [[int(p["x"]), int(p["y"])] for p in trajectory],
                            np.int32,
                        )
                        pts = pts.reshape((-1, 1, 2))
                        cv2.polylines(frame, [pts], False, (0, 255, 255), 2)

                    for point in trajectory:
                        px = int(point["x"])
                        py = int(point["y"])

                        # Draw path dot
                        cv2.circle(frame, (px, py), 2, (0, 255, 255), -1)

                    cv2.putText(
                        frame,
                        "TRACKING",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        2,
                    )
                elif not is_tracking and has_measurement:
                    cv2.putText(
                        frame,
                        "BUFFERING (SVD)",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 255),
                        2,
                    )
                else:
                    cv2.putText(
                        frame,
                        "NO SIGNAL",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (100, 100, 100),
                        2,
                    )

                cv2.imshow("Machine Vision - Fast Binary Tracking", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("Quitting...")
                    shutdown_flag.set()
                    break

            except queue.Empty:
                continue

    except KeyboardInterrupt:
        print("Interrupted by user.")
        shutdown_flag.set()
    finally:
        print("Cleaning up...")
        shutdown_flag.set()
        serial_thread.join(timeout=2.0)
        tracker_thread.join(timeout=2.0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
