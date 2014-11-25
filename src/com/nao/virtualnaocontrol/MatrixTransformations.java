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
	
	private final static float CAMERA_FOCAL_LENGTH = 3.43f; // Exprimée en mm

	public Mat detection(DisplayMetrics displayMetrics, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat touchedPointMatrix) {

		Mat cameraMatrix = buildIntrinsicParametersMatrix(displayMetrics); // paramètres intrinsèques à la caméra
		MatOfDouble distCoeffs = new MatOfDouble(0.0, 0.0, 0.0, 0.0); // Matrices de zéros

		Mat rvec = new Mat(); // Output
		Mat tvec = new Mat(); // Output
		Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

		Mat rotationMatrix = new Mat(); // Output
		Calib3d.Rodrigues(rvec, rotationMatrix);

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
		Core.gemm(cameraMatrix, rtvec, 1, new Mat(), 0, a, 0);// Equivalent à : cameraMatrix * rtvec;

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

		Mat invC = c.inv(); // Output
		Mat touchPointInRobotReference = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(invC, touchedPointMatrix, 1, new Mat(), 0, touchPointInRobotReference, 0);

		System.out.println("VNCTests : touchPointInRobotReference" + touchPointInRobotReference.size().toString());
		System.out.println("VNCTests : touchPointInRobotReference" + touchPointInRobotReference);

		return touchPointInRobotReference;

	}

	protected Mat buildIntrinsicParametersMatrix(DisplayMetrics displayMetrics) {

		// Récupération de la focale en pixels
		float focalLengthInPixel = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM, CAMERA_FOCAL_LENGTH, displayMetrics); // TODO : float focalLength = cameraParameters.getFocalLength();

		// Coordonnées du centre de l'écran dans notre repère
		float centreX = (float) (displayMetrics.widthPixels / 2);
		float centreY = (float) (displayMetrics.heightPixels / 2);

		// Initialisation de la matrice des paramètres intrinsèques à la caméra et ajout des composants à la matrice
		Mat intrinsicParametersMatrix = new Mat(3, 3, CvType.CV_32F);
		intrinsicParametersMatrix.put(0, 0, focalLengthInPixel);
		intrinsicParametersMatrix.put(0, 1, 0.0f);
		intrinsicParametersMatrix.put(0, 2, centreX);
		intrinsicParametersMatrix.put(1, 0, 0.0f);
		intrinsicParametersMatrix.put(1, 1, focalLengthInPixel);
		intrinsicParametersMatrix.put(1, 2, centreY);
		intrinsicParametersMatrix.put(2, 0, 0.0f);
		intrinsicParametersMatrix.put(2, 1, 0.0f);
		intrinsicParametersMatrix.put(2, 2, 1.0f);

		return intrinsicParametersMatrix;

	}
}
