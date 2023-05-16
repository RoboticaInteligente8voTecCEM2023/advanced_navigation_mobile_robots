# Aruco generator with opencv in-built functions
# https://pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/
# https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/aruco_basics.html

import cv2

# aruco dictionary implies NxN grid with _### available ids
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
# aruco properties
aruco_id = 5
aruco_size = 450

# generate
aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_5X5_50"])
aruco_img = cv2.aruco.drawMarker(aruco_dict,aruco_id,aruco_size)  # (dict, id, size[mm])
# extended: drawMarker(dict, id, size, destination=None, borderBits=1)
# save
cv2.imwrite('aruco_id_'+str(aruco_id)+'.jpg',aruco_img)

# show
cv2.imshow('ARUCO',aruco_img)
cv2.waitKey(0)
