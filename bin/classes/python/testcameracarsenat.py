import numpy as np
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
MirePoints = np.float32([[-0.040,-0.04,0],[-0.04,0.04,0],[0.04,0.04,0],[0.04,-0.04,0]])

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    B = frame.copy()
    B[:,:] = (0,0,0)
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 11, 17, 17)
    edged = cv2.Canny(gray, 130, 200)
    (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    screenCnt = None
    #RotationMatrix = np.float32([[0,0,0],[0,0,0],[0,0,0]])
    camera_rotation_vector = np.float32([0,0,0]);
    for c in cnts:
    # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
             
        # if our approximated contour has four points, then
        # we can assume that we have found our screen
        if len(approx) == 4:
            screenCnt = approx
            cv2.fillConvexPoly(B,np.array(screenCnt,'int32'),(1,1,1),8,0)
            B = B*frame
            imagepoints = np.float32(screenCnt)
            fx = 1
            h,w = frame.shape[:2]
            dist_coefs = np.zeros(4)
            K = np.float64([[fx*w,0,0.5*(w-1)],[0,fx*w,0.5*(h-1)],[0.0,0.0,1]])
            ret,rvec,tvec = cv2.solvePnP(MirePoints,imagepoints,K,dist_coefs)
            RotationMatrix, Jac = cv2.Rodrigues(rvec)
            
            
            camera_translation_vector = np.matrix(np.linalg.inv(RotationMatrix))*np.matrix(tvec)
            print "tvec"
            print camera_translation_vector
            break
        
    cv2.drawContours(gray, [screenCnt], -1, (0, 255, 0), 3)
    cv2.imshow("gray", gray)
    cv2.imshow("frame", frame)
    #cv2.waitKey(0)


    # Display the resulting frame
    cv2.imshow('frame',B)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
