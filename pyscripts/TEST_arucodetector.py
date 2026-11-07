import cv2
import numpy as np

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
param = cv2.aruco.DetectorParameters_create()

def detectAruco(image):
    gs = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(
        gs,
        aruco_dict, 
        parameters = param
    )

    output = image.copy()

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(
            output,
            corners,
            ids
        )

        for marker_id in ids.flatten():
            marker_corners = corners[i][0]

            center_x = np.mean(marker_corners[:, 0])
            center_y = np.mean(marker_corners[:, 1])
            print(
                f"ID: {marker_id} | "
                f"Center: ({center_x:.2f}, {center_y:.2f})"
            )
    return output, ids, corners

def main() -> int:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return 1

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Cannot receive frame")
            break

        output, ids, corners = detectAruco(frame)
        cv2.imshow("Aruco Detection", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
