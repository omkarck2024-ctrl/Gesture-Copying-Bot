
import cv2
import mediapipe as mp
import serial
import time
from pathlib import Path

# ---------------- SETTINGS ----------------
PORT = "COM5"
BAUD_RATE = 9600
CAMERA_INDEX = 0

MODEL_PATH = (
    Path(__file__).parent
    / "models"
    / "pose_landmarker_lite.task"
)

RAISE_THRESHOLD = 0.08
LOWER_THRESHOLD = 0.02
VISIBILITY_THRESHOLD = 0.50

# MediaPipe landmark indices
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_WRIST = 15
RIGHT_WRIST = 16

# ------------------------------------------

mp_vision = mp.tasks.vision
mp_base = mp.tasks.BaseOptions

if not MODEL_PATH.exists():
    raise FileNotFoundError(
        f"Pose model not found: {MODEL_PATH}"
    )

arduino = None
camera = None

left_raised = False
right_raised = False


def arm_is_raised(landmarks, shoulder_id, wrist_id, previous):
    shoulder = landmarks[shoulder_id]
    wrist = landmarks[wrist_id]

    if (
        shoulder.visibility < VISIBILITY_THRESHOLD
        or wrist.visibility < VISIBILITY_THRESHOLD
    ):
        return previous

    # Smaller y means higher on the image.
    height_difference = shoulder.y - wrist.y

    # Hysteresis prevents rapid repeated switching.
    if not previous and height_difference > RAISE_THRESHOLD:
        return True

    if previous and height_difference < LOWER_THRESHOLD:
        return False

    return previous


def send_command(connection, command):
    connection.write(command.encode("ascii"))
    connection.flush()

    # Arduino prints a response for recognized commands.
    response = connection.readline().decode(
        "utf-8", errors="replace"
    ).strip()

    print(f"Sent {command}: {response or '(no reply)'}")


try:
    # Open serial connection first.
    arduino = serial.Serial(
        PORT, BAUD_RATE, timeout=0.25
    )

    # Opening the port can reset the Arduino.
    time.sleep(2)
    arduino.reset_input_buffer()

    print("Arduino connected on", PORT)
    print("Keep both servo connectors unplugged for this test.")

    # Start webcam.
    camera = cv2.VideoCapture(CAMERA_INDEX)

    if not camera.isOpened():
        raise RuntimeError(
            "Cannot open webcam. Check camera permissions."
        )

    options = mp_vision.PoseLandmarkerOptions(
        base_options=mp_base(
            model_asset_path=str(MODEL_PATH)
        ),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_poses=1
    )

    start_time = time.monotonic()
    last_timestamp = -1

    with mp_vision.PoseLandmarker.create_from_options(
        options
    ) as landmarker:

        print("Gesture tracking started.")
        print("Raise your arms; press Q to quit.")

        while True:
            success, frame = camera.read()

            if not success:
                print("Could not read webcam frame.")
                break

            # Use the original camera frame for detection.
            rgb_frame = cv2.cvtColor(
                frame, cv2.COLOR_BGR2RGB
            )

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb_frame
            )

            timestamp = int(
                (time.monotonic() - start_time) * 1000
            )

            # Video timestamps must increase.
            timestamp = max(timestamp, last_timestamp + 1)
            last_timestamp = timestamp

            result = landmarker.detect_for_video(
                mp_image, timestamp
            )

            if result.pose_landmarks:
                landmarks = result.pose_landmarks[0]

                new_left = arm_is_raised(
                    landmarks,
                    LEFT_SHOULDER,
                    LEFT_WRIST,
                    left_raised
                )

                new_right = arm_is_raised(
                    landmarks,
                    RIGHT_SHOULDER,
                    RIGHT_WRIST,
                    right_raised
                )

                if new_left != left_raised:
                    send_command(
                        arduino, "L" if new_left else "l"
                    )
                    left_raised = new_left

                if new_right != right_raised:
                    send_command(
                        arduino, "R" if new_right else "r"
                    )
                    right_raised = new_right

            # Mirror only the preview for a selfie-like view.
            display = cv2.flip(frame, 1)

            cv2.putText(
                display,
                f"LEFT ARM: {'UP' if left_raised else 'DOWN'}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display,
                f"RIGHT ARM: {'UP' if right_raised else 'DOWN'}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display,
                "Press Q to quit",
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.imshow(
                "Gesture Copy Robot", display
            )

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

except Exception as error:
    print("ERROR:", error)

finally:
    if camera is not None:
        camera.release()

    cv2.destroyAllWindows()

    if arduino is not None and arduino.is_open:
        # Center both servo targets before closing.
        # No motors should be connected during software-only testing.
        try:
            send_command(arduino, "C")
        except Exception:
            pass

        arduino.close()

    print("Gesture tracking stopped.")
