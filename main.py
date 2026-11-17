
import cv2
import mediapipe as mp
import serial
import time
from pathlib import Path

# ==================================================
# GESTURE COPYING ROBOT
# Webcam + MediaPipe + Arduino UNO + 2 SG90 servos
# ==================================================

MODEL_PATH = (
    Path(__file__).parent
    / "models"
    / "pose_landmarker_lite.task"
)

ARDUINO_PORT = "COM5"
BAUD_RATE = 9600
CAMERA_INDEX = 0

# MediaPipe landmark IDs
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_ELBOW = 13
RIGHT_ELBOW = 14
LEFT_WRIST = 15
RIGHT_WRIST = 16

# Detection sensitivity
RAISE_THRESHOLD = 0.06
LOWER_THRESHOLD = 0.02
VISIBILITY_THRESHOLD = 0.50

# Detected anatomical arm states
arm_states = {
    "LEFT": False,
    "RIGHT": False
}

CONNECTIONS = [
    (11, 12),
    (11, 13), (13, 15),
    (12, 14), (14, 16),
    (11, 23), (12, 24),
    (23, 24)
]

arduino = None
arduino_connected = False


# ==================================================
# ARM DETECTION
# ==================================================

def detect_arm(landmarks, side):
    """Detect whether the specified anatomical arm is raised."""

    if side == "LEFT":
        shoulder_id = LEFT_SHOULDER
        elbow_id = LEFT_ELBOW
        wrist_id = LEFT_WRIST
    else:
        shoulder_id = RIGHT_SHOULDER
        elbow_id = RIGHT_ELBOW
        wrist_id = RIGHT_WRIST

    shoulder = landmarks[shoulder_id]
    elbow = landmarks[elbow_id]
    wrist = landmarks[wrist_id]

    visibility = [
        getattr(shoulder, "visibility", 0.0),
        getattr(elbow, "visibility", 0.0),
        getattr(wrist, "visibility", 0.0)
    ]

    if min(visibility) < VISIBILITY_THRESHOLD:
        return None

    # Smaller image y means a higher position.
    difference = shoulder.y - wrist.y

    if arm_states[side]:
        if difference < LOWER_THRESHOLD:
            arm_states[side] = False
    else:
        if difference > RAISE_THRESHOLD:
            arm_states[side] = True

    return arm_states[side]


# ==================================================
# ARDUINO SERIAL COMMUNICATION
# ==================================================

def send_command(command):
    """Send a command without crashing the camera loop."""

    global arduino_connected

    if arduino is None or not arduino_connected:
        return False

    try:
        if not arduino.is_open:
            arduino_connected = False
            return False

        arduino.write(command.encode("ascii"))
        return True

    except (serial.SerialException, OSError) as error:
        print(f"\nArduino communication lost: {error}")
        arduino_connected = False
        return False


def connect_arduino():
    """Connect to Arduino once at startup."""

    global arduino, arduino_connected

    try:
        arduino = serial.Serial(
            ARDUINO_PORT,
            BAUD_RATE,
            timeout=1,
            write_timeout=1
        )

        # Arduino UNO may reset when serial opens.
        time.sleep(2)
        arduino.reset_input_buffer()

        arduino_connected = True
        print(f"Arduino connected on {ARDUINO_PORT}")
        return True

    except (serial.SerialException, OSError) as error:
        arduino_connected = False
        print(f"Arduino connection error: {error}")
        print("Check COM port and close Arduino Serial Monitor.")
        return False


# ==================================================
# DRAW POSE
# ==================================================

def draw_upper_body(frame, landmarks):
    """Draw upper-body landmarks and connections."""

    height, width = frame.shape[:2]

    visible_ids = [
        LEFT_SHOULDER, RIGHT_SHOULDER,
        LEFT_ELBOW, RIGHT_ELBOW,
        LEFT_WRIST, RIGHT_WRIST,
        23, 24
    ]

    points = {}

    for idx in visible_ids:
        lm = landmarks[idx]

        if getattr(lm, "visibility", 0.0) >= 0.4:
            points[idx] = (
                int(lm.x * width),
                int(lm.y * height)
            )

    for start, end in CONNECTIONS:
        if start in points and end in points:
            cv2.line(
                frame,
                points[start],
                points[end],
                (255, 180, 0),
                3
            )

    for point in points.values():
        cv2.circle(
            frame,
            point,
            6,
            (0, 255, 0),
            -1
        )


# ==================================================
# MAIN CAMERA APPLICATION
# ==================================================

def main():
    global arduino, arduino_connected

    if not MODEL_PATH.exists():
        print("ERROR: Pose model file not found:")
        print(MODEL_PATH)
        return

    camera = cv2.VideoCapture(CAMERA_INDEX)

    if not camera.isOpened():
        print("ERROR: Could not open webcam.")
        return

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    try:
        connect_arduino()

        vision = mp.tasks.vision
        BaseOptions = mp.tasks.BaseOptions

        options = vision.PoseLandmarkerOptions(
            base_options=BaseOptions(
                model_asset_path=str(MODEL_PATH)
            ),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )

        print("\nGesture Copy Robot started.")
        print("Raise and lower your arms.")
        print("Press Q to quit.\n")

        start_time = time.monotonic()
        previous_timestamp = -1

        with vision.PoseLandmarker.create_from_options(
            options
        ) as landmarker:

            while True:
                success, frame = camera.read()

                if not success:
                    print("Unable to read webcam frame.")
                    break

                # Mirror the preview.
                frame = cv2.flip(frame, 1)

                rgb = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGR2RGB
                )

                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=rgb
                )

                timestamp_ms = int(
                    (time.monotonic() - start_time) * 1000
                )

                timestamp_ms = max(
                    timestamp_ms,
                    previous_timestamp + 1
                )
                previous_timestamp = timestamp_ms

                result = landmarker.detect_for_video(
                    mp_image,
                    timestamp_ms
                )

                # Detection labels before display-only swapping.
                detected_left = None
                detected_right = None

                if result.pose_landmarks:
                    landmarks = result.pose_landmarks[0]
                    draw_upper_body(frame, landmarks)

                    old_left = arm_states["LEFT"]
                    old_right = arm_states["RIGHT"]

                    detected_left = detect_arm(
                        landmarks, "LEFT"
                    )
                    detected_right = detect_arm(
                        landmarks, "RIGHT"
                    )

                    # SERVO CONTROL REMAINS UNCHANGED:
                    # Physical anatomical LEFT -> left servo, D9.
                    # Physical anatomical RIGHT -> right servo, D10.

                    if detected_left is not None:
                        if detected_left != old_left:
                            send_command(
                                "L" if detected_left else "l"
                            )

                    if detected_right is not None:
                        if detected_right != old_right:
                            send_command(
                                "R" if detected_right else "r"
                            )

                    # DISPLAY-ONLY SWAP:
                    # Swap these two values if camera labels are reversed.
                    display_left = detected_right
                    display_right = detected_left

                else:
                    display_left = None
                    display_right = None

                # Status text
                left_status = (
                    "NOT DETECTED"
                    if display_left is None
                    else "RAISED" if display_left else "LOWERED"
                )

                right_status = (
                    "NOT DETECTED"
                    if display_right is None
                    else "RAISED" if display_right else "LOWERED"
                )

                left_color = (
                    (0, 255, 0)
                    if left_status == "RAISED"
                    else (255, 255, 255)
                )

                right_color = (
                    (0, 255, 0)
                    if right_status == "RAISED"
                    else (255, 255, 255)
                )

                cv2.putText(
                    frame,
                    f"YOUR LEFT ARM: {left_status}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    left_color,
                    2
                )

                cv2.putText(
                    frame,
                    f"YOUR RIGHT ARM: {right_status}",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    right_color,
                    2
                )

                connection_status = (
                    "ARDUINO CONNECTED"
                    if arduino_connected
                    else "ARDUINO DISCONNECTED"
                )

                connection_color = (
                    (0, 255, 0)
                    if arduino_connected
                    else (0, 0, 255)
                )

                cv2.putText(
                    frame,
                    connection_status,
                    (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    connection_color,
                    2
                )

                cv2.putText(
                    frame,
                    "Press Q to quit",
                    (20, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

                cv2.imshow("Gesture Copy Robot", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except (serial.SerialException, OSError) as error:
        print("Serial communication error:", error)

    except Exception as error:
        print("Application error:", repr(error))

    finally:
        camera.release()
        cv2.destroyAllWindows()

        if arduino is not None:
            try:
                if arduino.is_open and arduino_connected:
                    arduino.write(b"C")
                    arduino.flush()
            except (serial.SerialException, OSError):
                pass

            try:
                arduino.close()
            except Exception:
                pass

        print("Gesture tracking stopped.")


if __name__ == "__main__":
    main()
