import time
import cv2
import cv2.aruco as A
import numpy as np

def testUse(*args, **kwargs):
    actvicateTest = True
    if actvicateTest == True:
        for singleValue in args:
            print(str(singleValue))
        for singleArgument, singleInfo in kwargs:
            print("%s = %s" % (singleArgument, singleInfo))

def cameraCalibration(allCorners,allIds,board,imsize):
    #Calibration fails for lots of reasons. Release the video if we do
    try:
        cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
        retval, cameraMatrix, distCoeffs, rvecs, tvecs = cal
        np.savez('calib.npz',mtx=cameraMatrix,dist=distCoeffs,rvecs=rvecs,tvecs=tvecs)
    except:
        cap.release()
    return (cal)

# create a dictionary of 250 tiles with Marker of a size AxA (5x5_50)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

# instance a calibration board of size X,Y with square length of A (normally in meters) and marker length of B (normally in meters)
# cv2.aruco.CharucoBoard_create(X ,Y ,A，B，Dictionary) A = 4*B
board = cv2.aruco.CharucoBoard_create(3,3,.025,.020,dictionary)

# create the calibration board
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)

#Start capturing images for calibration
cap = cv2.VideoCapture(0)

allCorners = []
allIds = []
decimator = 0

while True:

    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # res = cv2.aruco.detectMarkers(gray,dictionary)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray,dictionary)
    cv2.aruco.drawDetectedMarkers(gray,corners,ids)

    if len(corners)>0:
        # cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, board)
        # interpolatin is looking for a board and not a simple image
        charucoCorners, charucoIds, rejected = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
#        print(str(charucoCorners))
#        print(str(charucoIds))
#        print(str(rejected))
#        print(str(cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)))

        if charucoIds is not None and rejected is not None and len(charucoIds) > 3 and decimator%3==0:
            allCorners.append(charucoIds)
            allIds.append(rejected)

#    print(str(allCorners))
#    print(str(allIds))
#    retval, cameraMatrix, distCoeffs, rvecs, tvecs = cameraCalibration(allCorners,allIds,board,gray.shape)
    testUse(corners=corners,ids=ids)
#    print(str(retval))
#    print(str(cameraMatrix))
#    print(str(distCoeffs))
#    print(str(rvecs))
#    print(str(tvecs))
#    vecs = np.load("calib.npz")  # I already calibrated the camera
#    cameraMatrix, distCoeffs, _, _ = [vecs[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
#    ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs)
#    if ret is not None and ret is True:
#        cv2.aruco.drawAxis(gray,cameraMatrix,distCoeffs,np.array(rvecs),np.array(tvecs),np.array(0.1))
#    if ids is not None and corners is not None and len(ids) > 0 and len(ids) == len(corners):
#        diamond_corners, diamond_ids = cv2.aruco.detectCharucoDiamond(img, corners, ids, 0.05 / 0.03, cameraMatrix=mtx, distCoeffs=dist)

#    if len(res[0])>0:
#        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
#        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
#            allCorners.append(res2[1])
#            allIds.append(res2[2])

#        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
    cv2.aruco.drawDetectedMarkers(gray,corners,ids)

    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    decimator+=1

imsize = gray.shape

#Calibration fails for lots of reasons. Release the video if we do
try:
    cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
    retval, cameraMatrix, distCoeffs, rvecs, tvecs = cal
    np.savez('calib.npz',mtx=cameraMatrix,dist=distCoeffs,rvecs=rvecs,tvecs=tvecs)
except:
    cap.release()

cap.release()
cv2.destroyAllWindows()
