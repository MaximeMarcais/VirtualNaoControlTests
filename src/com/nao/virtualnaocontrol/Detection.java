package com.nao.virtualnaocontrol;

import org.jscience.mathematics.vector.Float64Matrix;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.TypedValue;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.Toast;

public class Detection extends Activity implements CvCameraViewListener2 {

	private final static float CAMERA_FOCAL_LENGTH = 3.43f; // Exprimée en mm

	private CameraBridgeViewBase mOpenCvCameraView;
	private boolean mIsJavaCamera = true;
	private MenuItem mItemSwitchCamera = null;

	private static Sensor accelerometer;
	private static Sensor magnetometer;
	private static float[] mGravity;
	private static float[] mGeomagnetic;

	private static float touchScreenPositionX;
	private static float touchScreenPositionY;

	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {

		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS: {
				mOpenCvCameraView.enableView();
			}
				break;
			default: {
				super.onManagerConnected(status);
			}
				break;
			}
		}
	};

	public Detection() {}

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		System.out.println("VNCTests : onCreate");
		super.onCreate(savedInstanceState);

		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		setContentView(R.layout.activity_main);

		if (mIsJavaCamera)
			mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);
		else
			mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_native_surface_view);

		mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);

		mOpenCvCameraView.setCvCameraViewListener(this);
	}

	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	public void onResume() {
		super.onResume();
		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
	}

	public void onDestroy() {
		super.onDestroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		mItemSwitchCamera = menu.add("Toggle Native/Java camera");
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		String toastMesage = new String();

		if (item == mItemSwitchCamera) {
			mOpenCvCameraView.setVisibility(SurfaceView.GONE);
			mIsJavaCamera = !mIsJavaCamera;

			if (mIsJavaCamera) {
				mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);
				toastMesage = "Java Camera";
			} else {
				mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_native_surface_view);
				toastMesage = "Native Camera";
			}

			mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
			mOpenCvCameraView.setCvCameraViewListener(this);
			mOpenCvCameraView.enableView();
			Toast toast = Toast.makeText(this, toastMesage, Toast.LENGTH_LONG);
			toast.show();
		}

		return true;
	}

	public void onCameraViewStarted(int width, int height) {
		System.out.println("VNCTests : onCameraViewStarted");
	}

	public void onCameraViewStopped() {}

	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

		detection(getResources().getDisplayMetrics(), null, null, null);

		// Affichage
		return inputFrame.rgba();
	}

	@Override
	public boolean onTouchEvent(MotionEvent event) {

		System.out.println("VNCTests : onTouchEvent");

		// Récupération de la position du clique sur l'écran
		if (event.getAction() == MotionEvent.ACTION_DOWN) {
			touchScreenPositionX = event.getX();
			touchScreenPositionY = event.getY();
		}

		return super.onTouchEvent(event);
	}

	public Mat detection(DisplayMetrics displayMetrics, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat touchedPointMatrix) {

		// Input
		// Point3 point3D1 = new Point3(objectPointsMatrix.get(0, 0)[0], objectPointsMatrix.get(0, 1)[0], objectPointsMatrix.get(0, 2)[0]);
		// Point3 point3D2 = new Point3(objectPointsMatrix.get(1, 0)[0], objectPointsMatrix.get(1, 1)[0], objectPointsMatrix.get(1, 2)[0]);
		// Point3 point3D3 = new Point3(objectPointsMatrix.get(2, 0)[0], objectPointsMatrix.get(2, 1)[0], objectPointsMatrix.get(2, 2)[0]);
		// Point3 point3D4 = new Point3(objectPointsMatrix.get(3, 0)[0], objectPointsMatrix.get(3, 1)[0], objectPointsMatrix.get(3, 2)[0]);
		// MatOfPoint3f objectPoints = new MatOfPoint3f(point3D1, point3D2, point3D3, point3D4); // Points du robot dans son repère

		// Point point2D1 = new Point(imagePointsMatrix.get(0, 0)[0], imagePointsMatrix.get(0, 1)[0]);
		// Point point2D2 = new Point(imagePointsMatrix.get(1, 0)[0], imagePointsMatrix.get(1, 1)[0]);
		// Point point2D3 = new Point(imagePointsMatrix.get(2, 0)[0], imagePointsMatrix.get(2, 1)[0]);
		// Point point2D4 = new Point(imagePointsMatrix.get(3, 0)[0], imagePointsMatrix.get(3, 1)[0]);
		// MatOfPoint2f imagePoints = new MatOfPoint2f(point2D1, point2D2, point2D3, point2D4); // Points du robot dans le repère de la caméra

		// Récupération des paramètres intrinsèques à la caméra
		Mat cameraMatrix = buildIntrinsicParametersMatrix(displayMetrics);

		// Construction d'une matrices représentant les coefficients de distorsion de la caméra (aucune distorsion : matrice de zéros)
		MatOfDouble distCoeffs = new MatOfDouble(0, 0, 0, 0);

		// Récupération de la matrice de correspondance 2D/3D
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

		// Résolution du système matriciel, on récupère les coordonnées du point touché (à l'écran) dans le repère du robot
		Mat invC = c.inv(); // Output
		Mat touchPointInRobotReference = new Mat(3, 1, CvType.CV_32F);
		touchedPointMatrix.put(2, 0, 1);
		Core.gemm(invC, touchedPointMatrix, 1, new Mat(), 0, touchPointInRobotReference, 0);

		return touchPointInRobotReference;

	}

	protected static Mat buildIntrinsicParametersMatrix(DisplayMetrics displayMetrics) {

		// Récupération de la focale en pixels
		float focalLengthInPixel = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM, CAMERA_FOCAL_LENGTH, displayMetrics); // TODO : float focalLength = cameraParameters.getFocalLength();

		// Coordonnées du centre de l'écran dans notre repère
		float centreX = (float) (displayMetrics.widthPixels / 2);
		float centreY = (float) (displayMetrics.heightPixels / 2);

		// Initialisation de la matrice des paramètres intrinsèques à la caméra et ajout des composants à la matrice
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

	protected Mat buildExtrinsicRotationParametersMatrix(SensorManager mSensorManager) {

		// Initialisation du sensor manager
		init(mSensorManager);

		// Initialisation des angles d'orientation de l'appareil
		float pitch = 0.0f;
		float roll = 0.0f;
		float azimuth = 0.0f;

		// Récupération des paramètres d'orientation de l'appareil
		if (mGravity != null && mGeomagnetic != null) {
			float R[] = new float[9];
			float I[] = new float[9];

			boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
			if (success) {
				float orientation[] = new float[3];
				SensorManager.getOrientation(R, orientation);

				// Pitch : angle selon x
				pitch = (float) Math.toDegrees(orientation[1]);
				// Roll : angle selon y
				roll = (float) Math.toDegrees(orientation[2]);
				// Azimuth : angle selon z
				azimuth = (float) Math.toDegrees(orientation[0]);
			}
		}

		// Initialisation de la matrice des paramètres intrinsèques à la caméra et ajout des composants à la matrice
		Mat extrinsicRotationParametersMatrix = new Mat(3, 3, CvType.CV_32F);
		extrinsicRotationParametersMatrix.put(0, 0, Math.cos(azimuth) * Math.cos(roll));
		extrinsicRotationParametersMatrix.put(0, 1, Math.cos(azimuth) * Math.sin(roll) * Math.sin(pitch) - Math.sin(azimuth) * Math.cos(pitch));
		extrinsicRotationParametersMatrix.put(0, 2, Math.cos(azimuth) * Math.sin(roll) * Math.cos(pitch) + Math.sin(azimuth) * Math.sin(pitch));
		extrinsicRotationParametersMatrix.put(1, 0, Math.sin(azimuth) * Math.cos(roll));
		extrinsicRotationParametersMatrix.put(1, 1, Math.sin(azimuth) * Math.sin(roll) * Math.sin(pitch) + Math.cos(azimuth) * Math.cos(pitch));
		extrinsicRotationParametersMatrix.put(1, 2, Math.sin(azimuth) * Math.sin(roll) * Math.cos(pitch) - Math.cos(azimuth) * Math.sin(pitch));
		extrinsicRotationParametersMatrix.put(2, 0, -Math.sin(roll));
		extrinsicRotationParametersMatrix.put(2, 1, Math.cos(roll) * Math.sin(pitch));
		extrinsicRotationParametersMatrix.put(2, 2, Math.cos(roll) * Math.cos(pitch));

		return extrinsicRotationParametersMatrix;
	}

	private Float64Matrix buildProjectionPointCoordinates() {
		double[][] tempMatrix = { { touchScreenPositionX, touchScreenPositionY } };

		return Float64Matrix.valueOf(tempMatrix);
	}

	public void init(SensorManager mSensorManager) {
		setAccelerometer(mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER));
		setMagnetometer(mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD));

		// Listen
		mSensorManager.registerListener(sel, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(sel, magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
	}

	static SensorEventListener sel = new SensorEventListener() {

		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {
			// System.out.println("VNCTests : onAccuracyChanged");
		}

		@Override
		public void onSensorChanged(SensorEvent event) {
			// System.out.println("VNCTests : onSensorChanged");
			if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
				mGravity = event.values;
				// System.out.println("VNCTests : Orientation mGravity = " + mGravity);
			}
			if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
				mGeomagnetic = event.values;
				// System.out.println("VNCTests : Orientation mGeomagnetic = " + mGeomagnetic);
			}
		}
	};

	public Sensor getAccelerometer() {
		return accelerometer;
	}

	public Sensor getMagnetometer() {
		return magnetometer;
	}

	private static void setMagnetometer(Sensor defaultSensor) {
		magnetometer = defaultSensor;

	}

	private static void setAccelerometer(Sensor defaultSensor) {
		accelerometer = defaultSensor;
	}
}