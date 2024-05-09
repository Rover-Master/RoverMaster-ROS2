import numpy as np
import cv2
import cv2.aruco as aruco

VER = float(".".join(cv2.__version__.split(".")[:1]))
# Default dictionary
dictionary = None
parameters = None
detector = None


def use_dict(dict_id: int) -> None:
    global dictionary, parameters, detector
    if VER <= 4.6:
        # Old API, 4.6 and below
        dictionary = aruco.Dictionary_get(dict_id)
        parameters = aruco.DetectorParameters_create()
    else:
        # New API, 4.7 and above
        dictionary = aruco.getPredefinedDictionary(dict_id)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)


def aruco_bits(id: int, pad=None) -> np.ndarray:
    arr = dictionary.getBitsFromByteList(
        dictionary.bytesList[id : id + 1], dictionary.markerSize
    )
    if pad is not None:
        arr = np.pad(arr, (1, 1), mode="constant", constant_values=pad)
    return arr


def find_internal_corners(
    grid: np.ndarray, mapping: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """
    Find internal corners in a given checker grid.
    For binary checker matrix of size (h, w), grid size a = 1/h and b = 1/w.
    The result will first be normalized to float with an range of (p, 1.0 - p)
    for both X and Y axis, p is the padding size.
    """
    # Index by [x, y] instead of [y, x]
    grid = grid.T
    # tl, tr, br, bl
    corners = np.stack(
        [grid[:-1, :-1], grid[1:, :-1], grid[1:, 1:], grid[:-1, 1:]], axis=2
    )
    # XOR results
    corners = np.logical_xor(corners, np.roll(corners, 1, axis=2))
    # Valid corners:
    # different color in 2 consecutive edges
    corners: np.ndarray = np.logical_and(corners, np.roll(corners, 1, axis=2))
    corners = np.any(corners, axis=2)
    h, w = corners.shape
    # Convert from boolean array to indices
    pts = np.array(np.where(corners))
    # Center around 0
    obj = pts - np.array([[(h - 1) / 2], [(w - 1) / 2]])
    # [x, y, 0].T
    obj = obj.T
    N = obj.shape[0]
    obj = np.concatenate([obj, np.zeros((N, 1))], axis=1)
    k = np.array([h + 1, w + 1]).reshape(2, 1)
    pts = (pts + 1) / k
    if mapping is not None:
        i = np.ones((1, pts.shape[1]), dtype=pts.dtype)
        pts = np.concatenate([pts, i], axis=0)
        pts = mapping @ pts
        pts = pts[:2] / pts[2]
    return pts.T, obj


def detect(img: np.ndarray):
    """
    Detect ArUco markers in a given image.
    Returns a tuple of (corners, ids).
    """
    if VER <= 4.6:
        # Old API, 4.6 and below
        return aruco.detectMarkers(img, dictionary, parameters=parameters)
    else:
        # New API, 4.7 and above
        return detector.detectMarkers(img)


# Default dictionary
use_dict(aruco.DICT_4X4_100)
