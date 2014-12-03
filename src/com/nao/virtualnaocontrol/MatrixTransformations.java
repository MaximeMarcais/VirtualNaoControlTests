package com.nao.virtualnaocontrol;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;

import android.util.DisplayMetrics;

public class MatrixTransformations {

	private final static float CAMERA_FOCAL_LENGTH = 1;// 3.43f; // Exprimée en px

	public static Point3 detection(DisplayMetrics displayMetrics, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat touchedPointMatrix) throws Exception {

		if (displayMetrics == null || objectPoints == null || imagePoints == null || touchedPointMatrix == null)
			throw new Exception("VNC ERROR : Null parameter in detection.");
		if (objectPoints.rows() != imagePoints.rows())
			throw new Exception("VNC ERROR : number of object points and image points are different.");

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

		// TODO : commentaire
		Mat a = new Mat(3, 4, CvType.CV_32F); // Output de gemm
		Core.gemm(cameraMatrix, rtvec, 1, new Mat(), 0, a, 0); // Equivalent à : cameraMatrix * rtvec;

		// TODO : commentaire
		Mat b = new Mat(4, 1, CvType.CV_32F);
		b.put(0, 0, 0);
		b.put(1, 0, 0);
		b.put(2, 0, 0);
		b.put(3, 0, 1);

		// TODO : commentaire
		Mat d = new Mat(3, 1, CvType.CV_32F); // Output de gemm
		Core.gemm(a, b, 1, new Mat(), 0, d, 0);
		double u = d.get(2, 0)[0];

		// TODO : commentaire
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

		// Application du coeff u
		touchPointInRobotReference.put(0, 0, u * touchPointInRobotReference.get(0, 0)[0]);
		touchPointInRobotReference.put(1, 0, u * touchPointInRobotReference.get(1, 0)[0]);
		touchPointInRobotReference.put(2, 0, u * touchPointInRobotReference.get(2, 0)[0]);

		// ///////////////////////
		// APPLICATION DU CARSENAT
		// ///////////////////////

		double rcam = 110 * Math.PI / 180.0;
		Mat MrcamX = new Mat(3, 3, CvType.CV_32F);
		MrcamX.put(0, 0, 1);
		MrcamX.put(0, 1, 0);
		MrcamX.put(0, 2, 0);
		MrcamX.put(1, 0, 0);
		MrcamX.put(1, 1, Math.cos(rcam));
		MrcamX.put(1, 2, -Math.sin(rcam));
		MrcamX.put(2, 0, 0);
		MrcamX.put(2, 1, Math.sin(rcam));
		MrcamX.put(2, 2, Math.cos(rcam));

		Mat Rcam = new Mat(); // Output de Rodrigues
		Calib3d.Rodrigues(MrcamX, Rcam);

		Mat rotationMatrix32F = new Mat(3, 3, CvType.CV_32F);
		rotationMatrix32F.put(0, 0, rotationMatrix.get(0, 0)[0]);
		rotationMatrix32F.put(0, 1, rotationMatrix.get(0, 1)[0]);
		rotationMatrix32F.put(0, 2, rotationMatrix.get(0, 2)[0]);
		rotationMatrix32F.put(1, 0, rotationMatrix.get(1, 0)[0]);
		rotationMatrix32F.put(1, 1, rotationMatrix.get(1, 1)[0]);
		rotationMatrix32F.put(1, 2, rotationMatrix.get(1, 2)[0]);
		rotationMatrix32F.put(2, 0, rotationMatrix.get(2, 0)[0]);
		rotationMatrix32F.put(2, 1, rotationMatrix.get(2, 1)[0]);
		rotationMatrix32F.put(2, 2, rotationMatrix.get(2, 2)[0]);

		Mat MrcamRoattionX = new Mat(3, 3, CvType.CV_32F);
		Core.gemm(MrcamX, rotationMatrix32F, 1, new Mat(), 0, MrcamRoattionX, 0);

		Mat Xrob = new Mat(3, 1, CvType.CV_32F);
		Mat unZeroZero = new Mat(3, 1, CvType.CV_32F);
		unZeroZero.put(0, 0, 1);
		unZeroZero.put(1, 0, 0);
		unZeroZero.put(2, 0, 0);
		Core.gemm(MrcamRoattionX, unZeroZero, 1, new Mat(), 0, Xrob, 0);

		Mat MRotation = new Mat(3, 3, CvType.CV_32F);

		double resMax = 0;
		int rotationRepereCamera = 0;
		Mat MRotationFinale = new Mat();

		for (int i = 0; i < 360; i += 5) {
			MRotation.put(0, 0, Math.cos(i * Math.PI / 180));
			MRotation.put(0, 1, -Math.sin(i * Math.PI / 180));
			MRotation.put(0, 2, 0);
			MRotation.put(1, 0, Math.sin(i * Math.PI / 180));
			MRotation.put(1, 1, Math.cos(i * Math.PI / 180));
			MRotation.put(1, 2, 0);
			MRotation.put(2, 0, 0);
			MRotation.put(2, 1, 0);
			MRotation.put(2, 2, 1);

			Mat Rrotation = new Mat(); // Output de Rodrigues
			Calib3d.Rodrigues(MRotation, Rrotation);

			Mat MrcamXrotationMatrix = new Mat(3, 3, CvType.CV_32F);
			Core.gemm(MrcamX, rotationMatrix32F, 1, new Mat(), 0, MrcamXrotationMatrix, 0);
			Mat MRotationMrcamXrotationMatrix = new Mat(3, 3, CvType.CV_32F);
			Core.gemm(MRotation, MrcamXrotationMatrix, 1, new Mat(), 0, MRotationMrcamXrotationMatrix, 0);
			Core.gemm(MRotationMrcamXrotationMatrix, unZeroZero, 1, new Mat(), 0, Xrob, 0);

			double res = Xrob.get(0, 0)[0];

			if (res > resMax) {
				resMax = res;
				rotationRepereCamera = i;
				MRotationFinale = MRotation;
			}
		}

		Core.gemm(MrcamX, MRotationFinale, 1, new Mat(), 0, MrcamX, 0);

		Calib3d.Rodrigues(MrcamX, Rcam);
		MatOfPoint3f zeroZeroZero = new MatOfPoint3f(new Point3(0, 0, 0));
		MatOfPoint2f centre = new MatOfPoint2f();
		Calib3d.projectPoints(zeroZeroZero, Rcam, tvec, cameraMatrix, distCoeffs, centre);

		Mat centerMatrix = new Mat(3, 1, CvType.CV_32F);
		centerMatrix.put(0, 0, centre.get(0, 0)[0]);
		centerMatrix.put(1, 0, centre.get(0, 0)[1]);
		centerMatrix.put(2, 0, 1);

		Mat Ntvec = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(cameraMatrix.inv(), centerMatrix, 1, new Mat(), 0, Ntvec, 0);
		double indice = Math.sqrt(Ntvec.get(0, 0)[0] * Ntvec.get(0, 0)[0] + Ntvec.get(1, 0)[0] * Ntvec.get(1, 0)[0] + Ntvec.get(2, 0)[0] * Ntvec.get(2, 0)[0]);

		Core.multiply(Ntvec, new Scalar(indice), Ntvec);

		Mat cameraTranslationVector = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(MrcamX.inv(), Ntvec, 1, new Mat(), 0, cameraTranslationVector, 0);

		Mat coordonneesClique = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(cameraMatrix.inv(), touchedPointMatrix, 1, new Mat(), 0, coordonneesClique, 0);
		Core.gemm(MrcamX.inv(), coordonneesClique, 1, new Mat(), 0, coordonneesClique, 0);

		double t = -cameraTranslationVector.get(2, 0)[0] / coordonneesClique.get(2, 0)[0];
		double X = -coordonneesClique.get(0, 0)[0] * (t - cameraTranslationVector.get(0, 0)[0]);
		double Y = -coordonneesClique.get(1, 0)[0] * (t - cameraTranslationVector.get(1, 0)[0]);

		return new Point3(X, Y, 0);

	}

	public static double pixelToMeter(DisplayMetrics displayMetrics, double pixelValue) {

		return (pixelValue / displayMetrics.xdpi * 25.4) / 1000;
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
		intrinsicParametersMatrix.put(0, 0, CAMERA_FOCAL_LENGTH * displayMetrics.widthPixels);
		intrinsicParametersMatrix.put(0, 1, 0);
		intrinsicParametersMatrix.put(0, 2, centreX);
		intrinsicParametersMatrix.put(1, 0, 0);
		intrinsicParametersMatrix.put(1, 1, CAMERA_FOCAL_LENGTH * displayMetrics.widthPixels);
		intrinsicParametersMatrix.put(1, 2, centreY);
		intrinsicParametersMatrix.put(2, 0, 0);
		intrinsicParametersMatrix.put(2, 1, 0);
		intrinsicParametersMatrix.put(2, 2, 1);

		return intrinsicParametersMatrix;

	}
}
