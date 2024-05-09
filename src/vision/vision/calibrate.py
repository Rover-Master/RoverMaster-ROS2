import sys, cv2
import numpy as np
from .lib.env import video


def init_undistort(frame_size: tuple[int, int]):
    # Check and load previous calibration data
    try:
        mtx = np.load("res/camera_mtx.npy")
        dist = np.load("res/distortion.npy")
    except FileNotFoundError:
        print("Please run calibrate.py first.")
        sys.exit(1)
    # Find optimal camera matrix
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, frame_size, 1, frame_size)
    # Return undistort function
    return mtx, dist, new_mtx


if __name__ == "__main__":
    CHECKER = W, H = (6, 6)

    last_found_img = None
    last_found_corners = None

    corners = []

    obj = np.zeros((W * H, 3), np.float32)
    obj[:, :2] = np.mgrid[0:W, 0:H].T.reshape(-1, 2)

    frame_size = None

    while True:
        ret, frame = video.read()
        if not ret:
            break

        # Try to find corners
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, c = cv2.findChessboardCorners(gray, CHECKER, None)
        if found:
            last_found_img = gray
            last_found_corners = c
            h, w = gray.shape
            frame_size = (w, h)
            cv2.drawChessboardCorners(frame, CHECKER, c, found)

        h, w, _ = frame.shape

        cv2.putText(
            frame,
            f"{len(corners)} snapshots taken.",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (255, 255, 0),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            "Press space to take snapshot, 'q' to exit.",
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 0, 255),
            3,
            cv2.LINE_AA,
        )
        cv2.imshow("raw frame", frame)

        key = cv2.waitKey(1)
        if key < 0:
            continue
        if key == ord(" ") and last_found_corners is not None:
            c = cv2.cornerSubPix(
                last_found_img,
                last_found_corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            corners.append(c)
            last_found_corners = None
        elif key == 13:  # Enter key
            break
        elif key == ord("q"):
            sys.exit(0)
        else:
            print("Key:", key)

    cv2.destroyAllWindows()

    if len(corners) > 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            [obj] * len(corners), corners, frame_size, None, None
        )
        # Get optimal camera matrix
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, frame_size, 1, frame_size
        )
        # Display undistorted image
        while True:
            ret, frame = video.read()
            if not ret:
                break
            undistorted = cv2.undistort(frame, mtx, dist, None, new_mtx)
            h, w, _ = undistorted.shape
            cv2.putText(
                undistorted,
                "Press 's' to save, 'q' to discard.",
                (20, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                2,
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )
            cv2.imshow("undistorted preview", undistorted)
            key = cv2.waitKey(1)
            if key < 0:
                continue
            elif key == ord("s") or key == 13:
                np.save("res/camera_mtx.npy", mtx)
                np.save("res/distortion.npy", dist)
                break
            elif key == ord("q") or key == 27:
                break

    cv2.destroyAllWindows()
    video.release()
