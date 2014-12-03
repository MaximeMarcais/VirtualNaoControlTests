import cv2
import numpy as np
print cv2.__version__

img = cv2.imread('20120114_060107.jpg',1)
MirePoints = np.float32([[-0.01538732647895813, -0.0183858722448349, 0.4551582932472229],
                         [0.032784223556518555, 0.10014607757329941, 0.2254450023174286],
                         [0.0391731858253479, -0.11687855422496796, 0.21903225779533386],
                         [-0.016333427280187607, 0.07193655520677567, 0.43833619356155396],[0,0,0]])
#MirePoints = np.float32([[0, 0.0, 0],[0.1174665167927742, 0.11092513799667358, 0.27571398019790649],[0.1174665167927742, -0.11092513799667358, 0.27571398019790649],[0.020062578842043877, 0.08999999612569809, 0.42575448751449585]])

imagepoints = np.float32([[1673, 461],
                          [1977, 1161],
                          [1353, 1161],
                          [1897, 609],[1641,1641]])

##
##MirePoints = np.float32([[-0.01538732647895813, -0.0183858722448349, 0.4551582932472229],
##                         [0.032784223556518555, 0.10014607757329941, 0.2254450023174286],
##                         [0.0391731858253479, -0.11687855422496796, 0.21903225779533386],
##                         [-0.016333427280187607, 0.07193655520677567, 0.43833619356155396]])
##
##imagepoints = np.float32([[1673, 461],
##                          [1977, 1161],
##                          [1353, 1161],
##                          [1897, 609]])

def calculProjection(event, x,y,flags,param):
    #print 'test'
    global ix,iy
    global btn
    if event == cv2.EVENT_LBUTTONDOWN:
        print 'test2'
        btn = 1
        ix,iy = x,y
        print ix,iy

cv2.namedWindow('test',cv2.WINDOW_NORMAL)        
cv2.setMouseCallback('test',calculProjection)

btn = 0
while (True):
    fx = 1
    h=1920
    w=2560
    dist_coefs = np.zeros(4)
    K = np.float64([[fx*w,0,0.5*(w-1)],[0,fx*w,0.5*(h-1)],[0.0,0.0,1]])
    ret,rvec,tvec = cv2.solvePnP(MirePoints,imagepoints,K,dist_coefs )
    RotationMatrix, Jac = cv2.Rodrigues(rvec)
    RotationMatrixInv = np.matrix(np.linalg.inv(RotationMatrix))
    rtvec = np.float32([
    [RotationMatrix[0][0],RotationMatrix[0][1],RotationMatrix[0][2],tvec[0]],
    [RotationMatrix[1][0],RotationMatrix[1][1],RotationMatrix[1][2],tvec[1]],
    [RotationMatrix[2][0],RotationMatrix[2][1],RotationMatrix[2][2],tvec[2]]])
    A = np.matrix(K)*np.matrix(rtvec)
    temp = A*np.matrix([[0], [0], [0], [1]])
    C = np.float32(
    [[[A.item(0,0)], [A.item(0,1)], [A.item(0,3)]],
    [[A.item(1,0)], [A.item(1,1)], [A.item(1,3)]],
    [[A.item(2,0)], [A.item(2,1)], [A.item(2,3)]]])


    camera_translation_vector = np.matrix(np.linalg.inv(RotationMatrix))*np.matrix(tvec)

    #on trace les axes
    VX = cv2.projectPoints(np.float32([[0,0,0],[0.1,0,0]]), rvec, tvec, K, dist_coefs)[0]
    VY = cv2.projectPoints(np.float32([[0,0,0],[0,0.1,0]]), rvec, tvec, K, dist_coefs)[0]
    VZ = cv2.projectPoints(np.float32([[0,0,0],[0,0,0.1]]), rvec, tvec, K, dist_coefs)[0]

##    cv2.line(img, (int(VX.item(0)),int(VX.item(1))), (int(VX.item(2)),int(VX.item(3))), (255, 0, 0), 2)
##    cv2.line(img, (int(VY.item(0)),int(VY.item(1))), (int(VY.item(2)),int(VY.item(3))), (0, 255, 0), 2)
##    cv2.line(img, (int(VZ.item(0)),int(VZ.item(1))), (int(VZ.item(2)),int(VZ.item(3))), (0, 0, 255), 2)





    #On considere que la camera a subi une rotation de 110 deg sur x
    rcam = ([[ 110*3.14159/180.0],[ 0.0],[0]])
    #matrice de rotation correspondante
    MrcamX = np.matrix([[1,0,0],[0,np.cos(rcam[0]),-np.sin(rcam[0])],[0,np.sin(rcam[0]),np.cos(rcam[0])]])
    Rcam,Jac = cv2.Rodrigues(MrcamX)

    #coordonnees de X robot dans le repere donne par la camera 
    Xrob = (MrcamX*RotationMatrix)*np.matrix([[1],[0],[0]])
    #on fait tourner le repere jusqu'a ce que le produit scalaire soit le plus gran possible
    pi = np.pi
    rotationRepereCamera = 0
    resMax = 0
    for az in range(0,360,5):
        MRotation = np.matrix([[np.cos(az*pi/180),-np.sin(az*pi/180),0],[np.sin(az*pi/180),np.cos(az*pi/180),0],[0,0,1]])
        Rrotation,Jac = cv2.Rodrigues(MRotation)
        Xrob = (MRotation*(MrcamX*RotationMatrix))*np.matrix([[1],[0],[0]])
        res = Xrob[0]*1
        if res>resMax:
            resMax = res
            rotationRepereCamera = az
            MRotationFinale = MRotation
    #print resMax,rotationRepereCamera
    MrcamX =  MrcamX*MRotationFinale
    Rcam,Jac = cv2.Rodrigues(MrcamX)
    #ici je trace le sol retrouve
    delta = 0.01;
    ampl = 0.1
    for i in range(int(-(ampl/delta)),int((ampl/delta))):

        P1 = cv2.projectPoints(np.float32([[i*delta,-ampl,0],[i*delta,ampl,0]]), Rcam, tvec, K, dist_coefs)[0]
        cv2.line(img, (int(P1.item(0)),int(P1.item(1))), (int(P1.item(2)),int(P1.item(3))), (0, 0, 255), 2)
         
        P1 = cv2.projectPoints(np.float32([[-ampl,i*delta,0],[ampl,i*delta,0]]), Rcam, tvec, K, dist_coefs)[0]
        cv2.line(img, (int(P1.item(0)),int(P1.item(1))), (int(P1.item(2)),int(P1.item(3))), (0, 0, 255), 2)
 #       print P1

    #On calcul un nouveau vecteur de translation
    Centre =cv2.projectPoints(np.float32([[0,0,0]]), Rcam, tvec, K, dist_coefs)[0] 
    Ntvec = np.linalg.inv(np.matrix(K))*np.matrix([[Centre.item(0)],[Centre.item(1)],[1]])
    Ntvec = Ntvec*np.sqrt(Ntvec[0]*Ntvec[0]+Ntvec[2]*Ntvec[2]+Ntvec[1]*Ntvec[1])

    
    #on trace les nouveaux axes
    VX = cv2.projectPoints(np.float32([[0,0,0],[0.1,0,0]]), Rcam, Ntvec, K, dist_coefs)[0]
    VY = cv2.projectPoints(np.float32([[0,0,0],[0,0.1,0]]), Rcam, Ntvec, K, dist_coefs)[0]
    VZ = cv2.projectPoints(np.float32([[0,0,0],[0,0,0.1]]), Rcam, Ntvec, K, dist_coefs)[0]

    cv2.line(img, (int(VX.item(0)),int(VX.item(1))), (int(VX.item(2)),int(VX.item(3))), (255, 0, 0), 2)
    cv2.line(img, (int(VY.item(0)),int(VY.item(1))), (int(VY.item(2)),int(VY.item(3))), (0, 255, 0), 2)
    cv2.line(img, (int(VZ.item(0)),int(VZ.item(1))), (int(VZ.item(2)),int(VZ.item(3))), (0, 0, 255), 2)
    #ici, je verifie la reprojection des point trouvés (On ne retombe pas sur les points exact car
    # la methode utilise les moidres carres

    PointsCaptes = cv2.projectPoints(MirePoints, rvec, tvec, K, dist_coefs)[0]

    for p in PointsCaptes :
        cv2.circle(img,(int(p.item(0)),int(p.item(1))),20,(255, 0, 0),2)


    
    #on affiche un point au centre du

    CentreDansEcran = cv2.projectPoints(np.float32([[tvec[0],tvec[1],tvec[2]]]), Rcam*0, tvec*0, K, dist_coefs)[0]
    


    camera_translation_vector = np.matrix(np.linalg.inv(MrcamX))*np.matrix(Ntvec)

    
    #si on appui sur le bouton
    if btn == 1:
        #"coordonee du clic dans repere camera (écran situe à 1m de focale)"
        CoordonneeClic = np.linalg.inv(np.matrix(K))*np.matrix([[ix],[iy],[1]])
        #coordonee du clic dans repere monde (toujours dans le plan focal)
        CoordonneeClic = np.linalg.inv(MrcamX)*CoordonneeClic
        print 
        #Ici on résous l'equation d'intersection d'une droite et d'un plan
        Vclic = CoordonneeClic
        t = -camera_translation_vector[2]/Vclic[2]
        X = -Vclic[0]*t-camera_translation_vector[0]
        Y = -Vclic[1]*t-camera_translation_vector[1]

        #on vérifie les projections
        PointsClic = cv2.projectPoints(np.float32([[X,Y,0]]), Rcam, Ntvec, K, dist_coefs)[0]
        for p in PointsClic:
            #print p
            cv2.circle(img,(int(p.item(0)),int(p.item(1))),20,(255, 0, 0),2)
        print "RES attendu : X=-0.9, Y=-0.6"
        print X
        print Y

    btn = 0        
    cv2.imshow('test',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


    
#print "camera_translation_vector"
#print tvec

#cv2.waitKey(0)
cv2.destroyAllWindows()


