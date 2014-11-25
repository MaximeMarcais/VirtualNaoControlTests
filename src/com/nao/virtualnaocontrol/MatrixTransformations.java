package com.nao.virtualnaocontrol;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;

import android.util.DisplayMetrics;
import android.util.TypedValue;

public class MatrixTransformations {

	private final static float CAMERA_FOCAL_LENGTH = 3.43f; // Exprim�e en mm

	public static Mat detection(DisplayMetrics displayMetrics, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat touchedPointMatrix) {

		// R�cup�ration des param�tres intrins�ques � la cam�ra
		Mat cameraMatrix = buildIntrinsicParametersMatrix(displayMetrics);

		// Construction d'une matrices repr�sentant les coefficients de distorsion de la cam�ra (aucune distorsion : matrice de z�ros)
		MatOfDouble distCoeffs = new MatOfDouble(0, 0, 0, 0);

		// R�cup�ration de la matrice de correspondance 2D/3D
		Mat rvec = new Mat(); // Output
		Mat tvec = new Mat(); // Output
		Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

		// Conversion vecteur de rotation vers matrice de rotation
		Mat rotationMatrix = new Mat(); // Output
		Calib3d.Rodrigues(rvec, rotationMatrix);

		// Construction de la matrice de passage
		Mat rtvec = new Mat(3, 4, CvType.CV_32F);
		rtvec.put(0, 0, rotationMatrix.get(0, 0)[0]);
		rtvec.put(0, 1, rotationMatrix.get(0, 1)[0]);
		rtvec.put(0, 2, rotationMatrix.get(0, 2)[0]);
		rtvec.put(0, 3, tvec.get(0, 0)[0]);
		rtvec.put(1, 0, rotationMatrix.get(1, 0)[0]);
		rtvec.put(1, 1, rotationMatrix.get(1, 1)[0]);
		rtvec.put(1, 2, rotationMatrix.get(1, 2)[0]);
		rtvec.put(1, 3, tvec.get(1, 0)[0]);
		rtvec.put(2, 0, rotationMatrix.get(2, 0)[0]);
		rtvec.put(2, 1, rotationMatrix.get(2, 1)[0]);
		rtvec.put(2, 2, rotationMatrix.get(2, 2)[0]);
		rtvec.put(2, 3, tvec.get(2, 0)[0]);

		Mat a = new Mat(3, 4, CvType.CV_32F); // Output
		Core.gemm(cameraMatrix, rtvec, 1, new Mat(), 0, a, 0); // Equivalent � : cameraMatrix * rtvec;

		Mat c = new Mat(3, 3, CvType.CV_32F);
		c.put(0, 0, a.get(0, 0)[0]);
		c.put(0, 1, a.get(0, 1)[0]);
		c.put(0, 2, a.get(0, 3)[0]);
		c.put(1, 0, a.get(1, 0)[0]);
		c.put(1, 1, a.get(1, 1)[0]);
		c.put(1, 2, a.get(1, 3)[0]);
		c.put(2, 0, a.get(2, 0)[0]);
		c.put(2, 1, a.get(2, 1)[0]);
		c.put(2, 2, a.get(2, 3)[0]);

		// R�solution du syst�me matriciel, on r�cup�re les coordonn�es du point touch� (� l'�cran) dans le rep�re du robot
		Mat invC = c.inv(); // Output
		Mat touchPointInRobotReference = new Mat(3, 1, CvType.CV_32F);
		touchedPointMatrix.put(2, 0, 1);
		Core.gemm(invC, touchedPointMatrix, 1, new Mat(), 0, touchPointInRobotReference, 0);

		return touchPointInRobotReference;

	}

	protected static Mat buildIntrinsicParametersMatrix(DisplayMetrics displayMetrics) {

		// R�cup�ration de la focale en pixels
		float focalLengthInPixel = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM, CAMERA_FOCAL_LENGTH, displayMetrics); // TODO : float focalLength = cameraParameters.getFocalLength();

		// Coordonn�es du centre de l'�cran dans notre rep�re
		float centreX = (float) (displayMetrics.widthPixels / 2);
		float centreY = (float) (displayMetrics.heightPixels / 2);

		// Initialisation de la matrice des param�tres intrins�ques � la cam�ra et ajout des composants � la matrice
		Mat intrinsicParametersMatrix = new Mat(3, 3, CvType.CV_32F);
		intrinsicParametersMatrix.put(0, 0, focalLengthInPixel);
		intrinsicParametersMatrix.put(0, 1, 0);
		intrinsicParametersMatrix.put(0, 2, centreX);
		intrinsicParametersMatrix.put(1, 0, 0);
		intrinsicParametersMatrix.put(1, 1, focalLengthInPixel);
		intrinsicParametersMatrix.put(1, 2, centreY);
		intrinsicParametersMatrix.put(2, 0, 0);
		intrinsicParametersMatrix.put(2, 1, 0);
		intrinsicParametersMatrix.put(2, 2, 1);

		return intrinsicParametersMatrix;

	}
}
