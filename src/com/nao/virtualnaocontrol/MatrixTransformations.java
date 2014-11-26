package com.nao.virtualnaocontrol;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;

import android.util.DisplayMetrics;

public class MatrixTransformations {

	private final static float CAMERA_FOCAL_LENGTH = 0.00343f; // Exprimée en m

	public static Mat detection(DisplayMetrics displayMetrics, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat touchedPointMatrix) throws Exception {

		if (displayMetrics == null || objectPoints == null || imagePoints == null || touchedPointMatrix == null)
			throw new Exception("VNC ERROR : Null parameter in detection.");
		if (objectPoints.height() != imagePoints.height())
			throw new Exception("VNC ERROR : number of object points and image points are different.");

		// Transformation des données en pixel vers des mètres
		imagePoints = (MatOfPoint2f) pixelToMeter(displayMetrics, imagePoints);
		touchedPointMatrix = pixelToMeter(displayMetrics, touchedPointMatrix);

		// Récupération des paramètres intrinsèques à la caméra
		Mat cameraMatrix = buildIntrinsicParametersMatrix(displayMetrics);

		// Construction d'une matrices représentant les coefficients de distorsion de la caméra (aucune distorsion : matrice de zéros)
		MatOfDouble distCoeffs = new MatOfDouble(0, 0, 0, 0);

		// Récupération de la matrice de correspondance 2D/3D
		Mat rvec = new Mat(); // Output de solvePnP
		Mat tvec = new Mat(); // Output de solvePnP
		Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

		// Conversion vecteur de rotation vers matrice de rotation
		Mat rotationMatrix = new Mat(); // Output de Rodrigues
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

		Mat a = new Mat(3, 4, CvType.CV_32F); // Output de gemm
		Core.gemm(cameraMatrix, rtvec, 1, new Mat(), 0, a, 0); // Equivalent à : cameraMatrix * rtvec;

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

		// Résolution du système matriciel : on récupère les coordonnées du point touché (à l'écran) dans le repère du robot
		Mat invC = c.inv();
		touchedPointMatrix.put(2, 0, 1); // Ajout d'une coordonnée en Z pour le point touché (à l'écran), utile seulement pour le calcul
		Mat touchPointInRobotReference = new Mat(3, 1, CvType.CV_32F); // Output de gemm
		Core.gemm(invC, touchedPointMatrix, 1, new Mat(), 0, touchPointInRobotReference, 0);

		return touchPointInRobotReference;

	}

	protected static Mat pixelToMeter(DisplayMetrics displayMetrics, Mat imagePoints) {

		double value = 0;

		for (int i = 0; i < imagePoints.rows(); i++) {
			for (int j = 0; j < imagePoints.cols(); j++) {
				value = (imagePoints.get(i, j)[0] / displayMetrics.xdpi * 25.4) / 1000;
				imagePoints.put(i, j, value);
			}

		}

		return imagePoints;
	}

	protected static Mat buildIntrinsicParametersMatrix(DisplayMetrics displayMetrics) {

		// Récupération de la focale en pixels
		// TODO : float focalLength = cameraParameters.getFocalLength();
		// float focalLengthInPixel = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM, CAMERA_FOCAL_LENGTH*1000, displayMetrics);

		// Coordonnées du centre de l'écran dans notre repère
		float centreX = (float) (displayMetrics.widthPixels / 2);
		float centreY = (float) (displayMetrics.heightPixels / 2);

		// Initialisation de la matrice des paramètres intrinsèques à la caméra et ajout des composants à la matrice
		Mat intrinsicParametersMatrix = new Mat(3, 3, CvType.CV_32F);
		intrinsicParametersMatrix.put(0, 0, CAMERA_FOCAL_LENGTH);
		intrinsicParametersMatrix.put(0, 1, 0);
		intrinsicParametersMatrix.put(0, 2, centreX);
		intrinsicParametersMatrix.put(1, 0, 0);
		intrinsicParametersMatrix.put(1, 1, CAMERA_FOCAL_LENGTH);
		intrinsicParametersMatrix.put(1, 2, centreY);
		intrinsicParametersMatrix.put(2, 0, 0);
		intrinsicParametersMatrix.put(2, 1, 0);
		intrinsicParametersMatrix.put(2, 2, 1);

		return intrinsicParametersMatrix;

	}
}
