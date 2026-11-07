import cv2
import numpy as np

aruco_dict = cv2.aruco.Dictionary_get(
    cv2.aruco.DICT_4X4_50
)

parameters = cv2.aruco.DetectorParameters_create()


def detect_aruco(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray,
        aruco_dict,
        parameters=parameters
    )

    output = image.copy()

    print("Rejected markers:", len(rejected))

    if ids is None:
        print("No markers detected!")

    else:
        cv2.aruco.drawDetectedMarkers(
            output,
            corners,
            ids
        )

        for marker_id in ids.flatten():
            print(f"Detected marker ID: {marker_id}")

    return output, ids, corners

imagename = "image.png"
img = "/pyscripts/" + imagename

def main():

    image = cv2.imread('image2.png')

    if image is None:
        print("Failed to load image!")
        return

    print("Image shape:", image.shape)

    output, ids, corners = detect_aruco(image)

    cv2.imshow("ArUco Detection", output)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()