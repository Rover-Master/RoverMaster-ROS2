import cv2, numpy as np
from .ArUco import aruco_bits, find_internal_corners
from .transforms import get_transform


def estimate(img: np.ndarray, id: int, loc: np.ndarray, mtx, dist, ITER=4):
    # Get bit matrix
    bit_map = aruco_bits(id, pad=0)
    # Prepare points
    pts_src = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32)
    pts_obs = loc.reshape(-1, 2)
    # Initial estimation of homography
    H, _ = cv2.findHomography(pts_src, pts_obs)
    # Prepare src points for homography projection
    pts_src = np.concatenate([pts_src, np.ones((4, 1))], axis=1)
    # Iterative refinement
    iter = 0
    while True:
        loc_proj = H @ pts_src.T
        loc_proj = loc_proj[:2] / loc_proj[2]
        r = np.average([np.linalg.norm(H[:2, 0]), np.linalg.norm(H[:2, 1])]) / 6
        r = min(img.shape[0] / 4, img.shape[1] / 4, r / 4)
        r = int(r)
        # Find internal corners
        corners_pred, obj_pts = find_internal_corners(bit_map, H)
        # Subpixel accuracy
        pts_itn = corners_pred.reshape(-1, 1, 2).astype(np.float32)
        pts_itn = np.ascontiguousarray(pts_itn)
        pts_itn = cv2.cornerSubPix(
            img,
            pts_itn,
            (r, r),
            (-1, -1),
            (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            ),
        )
        pts_itn = np.squeeze(pts_itn)
        iter += 1
        if iter >= ITER:
            break
        # Reiterate homography based on additional corners
        pts_itn_src = obj_pts[:, :2] / 6 + 0.5
        H, _ = cv2.findHomography(pts_itn_src, pts_itn)
    # Compute 3D location of this marker
    GRID_SIZE = 10  # millimeters
    obj_pts = obj_pts * GRID_SIZE
    # Flip X-axis, face up
    obj_pts[:, 0] *= -1
    return solve(obj_pts, pts_itn, mtx, dist)


def solve(
    obj_pts: np.ndarray,
    img_pts: np.ndarray,
    cam_mtx: np.ndarray,
    dist: np.ndarray,
) -> np.ndarray:
    """
    Using a cluster of 2D points from a known geometry (i.e. obj_pts),
    estimate its 3D transformation matrix from the camera.
    @param   obj_pts: 3D points from the object's geometry (intrinsic)
    @param   img_pts: 2D points on the image plane (observations)
    @param   cam_mtx: Intrinsic matrix of the camera
    @param   dist: distortion coefficients of the camera
    @returns 3D transformation matrix T = [R | t], such that:
             cam_mtx @ T @ obj_pts = img_pts
    """
    # Estimate transformation matrix
    obj_pts = obj_pts.reshape(-1, 1, 3)
    img_pts = img_pts.reshape(-1, 1, 2)
    _, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, cam_mtx, dist, flags=cv2.SOLVEPNP_ITERATIVE
    )
    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    # Return transformation matrix
    return get_transform(R, tvec.reshape((-1)))
