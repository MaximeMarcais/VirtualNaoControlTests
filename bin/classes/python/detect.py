import numpy as np
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
MirePoints = np.float32([[0.02210916206240654, 0.0, 0.44615206122398376],[0.1174665167927742, 0.11092513799667358, 0.27571398019790649],[0.1174665167927742, -0.11092513799667358, 0.27571398019790649],[0.020062578842043877, 0.08999999612569809, 0.42575448751449585]])

imagepoints = np.float32([[1673, 461],[1977, 1161],[1353, 1161],[1897, 609]])
fx = 1
h=1920
w=2560
dist_coefs = np.zeros(4)
K = np.float64([[fx*w,0,0.5*(w-1)],[0,fx*w,0.5*(h-1)],[0.0,0.0,1]])
ret,rvec,tvec = cv2.solvePnP(MirePoints,imagepoints,K,dist_coefs)
RotationMatrix, Jac = cv2.Rodrigues(rvec)
RotationMatrixInv = np.matrix(np.linalg.inv(RotationMatrix))
rtvec = np.float32([
[RotationMatrix[0][0],RotationMatrix[0][1],RotationMatrix[0][2],tvec[0]],
[RotationMatrix[1][0],RotationMatrix[1][1],RotationMatrix[1][2],tvec[1]],
[RotationMatrix[2][0],RotationMatrix[2][1],RotationMatrix[2][2],tvec[2]]])
#rtvec = np.float32([[RotationMatrixInv.item(0,0),RotationMatrixInv.item(0,1),RotationMatrixInv.item(0,2),tvec[0]],[RotationMatrixInv.item(1,0),RotationMatrixInv.item(1,1),RotationMatrixInv.item(1,2),tvec[1]],[RotationMatrixInv.item(2,0),RotationMatrixInv.item(2,1),RotationMatrixInv.item(2,2),tvec[2]]])
A = np.matrix(K)*np.matrix(rtvec)
temp = A*np.matrix([[0], [0], [0], [1]])
C = np.float32(
[[[A.item(0,0)], [A.item(0,1)], [A.item(0,3)]],
[[A.item(1,0)], [A.item(1,1)], [A.item(1,3)]],
[[A.item(2,0)], [A.item(2,1)], [A.item(2,3)]]])

pos_sol = np.matrix(np.linalg.inv(np.matrix(C)))*np.matrix([[749], [921], [1]])             

camera_translation_vector = np.matrix(np.linalg.inv(RotationMatrix))*np.matrix(tvec)
print "pos_sol"
#print temp/temp.item(2)

print "RES attendu : X=-0.9, Y=-0.6"
print pos_sol*temp.item(2)
