import cv2
import numpy as np
import time
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from comtypes import CLSCTX_ALL
from ctypes import cast, POINTER
import HandTrackingModule as htm

# Initialize video capture
cap = cv2.VideoCapture(0)
detector = htm.HandDetector(detectionCon=0.75)
count = 0
dir = 0
pTime = 0

# Initialize audio settings
devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
volRange = volume.GetVolumeRange()
minVol = volRange[0]
maxVol = volRange[1]

# Initialize volume control state
volume_control_active = False

def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points."""
    return np.linalg.norm(np.array(point1) - np.array(point2))

while True:
    success, img = cap.read()
    img = cv2.resize(img, (1280, 720))
    img = detector.findHands(img)
    lmList, bboxInfo = detector.findPosition(img, draw=False)

    if lmList:
        # Index tip (8) and Ring tip (16)
        index_tip = lmList[8]
        ring_tip = lmList[16]

        # Check if the distance between index tip and ring tip is small (fingers crossed)
        distance = calculate_distance(index_tip, ring_tip)
        if distance < 10:  # Threshold for detecting crossed fingers
            volume_control_active = not volume_control_active
            time.sleep(1)  # Add a delay to avoid multiple toggles in quick succession

        if volume_control_active:
            # Wrist (0), Thumb tip (4), and Index tip (8)
            wrist = lmList[0]
            thumb_tip = lmList[4]
            index_tip = lmList[8]

            # Calculate angle between wrist, thumb tip, and index tip
            angle = detector.findAngle(wrist, thumb_tip, index_tip)

            # Map angle to volume level
            vol = np.interp(angle, (30, 150), [minVol, maxVol])
            volume.SetMasterVolumeLevel(vol, None)

            # Map angle to percentage and bar position
            volPer = np.interp(angle, (30, 150), [0, 100])
            bar = np.interp(angle, (30, 150), [650, 100])

            color = (255, 0, 255)
            if volPer == 100:
                color = (0, 255, 0)
                if dir == 0:
                    count += 0.5
                    dir = 1
            if volPer == 0:
                color = (0, 255, 0)
                if dir == 1:
                    count += 0.5
                    dir = 0
            print(f"Volume: {int(volPer)}%")

            # Draw Bar
            cv2.rectangle(img, (1100, 100), (1175, 650), color, 3)
            cv2.rectangle(img, (1100, int(bar)), (1175, 650), color, cv2.FILLED)
            cv2.putText(img, f'{int(volPer)} %', (1100, 75), cv2.FONT_HERSHEY_PLAIN, 4, color, 4)

            # Draw Curl Count
            cv2.rectangle(img, (0, 450), (250, 720), (0, 255, 0), cv2.FILLED)
            cv2.putText(img, str(int(count)), (45, 670), cv2.FONT_HERSHEY_PLAIN, 15, (255, 0, 0), 25)

        # Draw toggle state on the image
        state_text =         "Volume Control: ON" if volume_control_active else "Volume Control: OFF"
        cv2.putText(img, state_text, (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(img, str(int(fps)), (50, 100), cv2.FONT_HERSHEY_PLAIN, 5, (255, 0, 0), 5)

    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

