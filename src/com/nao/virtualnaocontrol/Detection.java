package com.nao.virtualnaocontrol;

import org.jscience.mathematics.vector.Float64Matrix;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

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

	private final static float ROBOT_HEIGHT = 580; // Exprimé en mm
	private final static float CAMERA_FOCAL_LENGTH = 3.43f; // Exprimée en mm
	private final static float CAMERA_HEIGHT_FROM_GROUND = 1500; // Exprimée en mm // TODO : le calculer
	private final static float ROBOT_HEIGHT_ON_SCREEN = 50;
	private final static float DISTANCE_ROBOT_IMAGE_FROM_CENTER = 10;

	private CameraBridgeViewBase mOpenCvCameraView;
	private boolean mIsJavaCamera = true;
	private MenuItem mItemSwitchCamera = null;

	private static Sensor accelerometer;
	private static Sensor magnetometer;
	private static float[] mGravity;
	private static float[] mGeomagnetic;

	// private static Camera.Parameters cameraParameters;
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

		// System.out.println("VNCTests : onCameraFrame");
		detection(getResources().getDisplayMetrics());

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
		 System.out.println("VNCTests : touchScreenPositionX=" + touchScreenPositionX);
		 System.out.println("VNCTests : touchScreenPositionY=" + touchScreenPositionY);
		 }

		return super.onTouchEvent(event);
	}

	public void detection(DisplayMetrics displayMetrics) {

		Float64Matrix intrinsicParametersMatrix = buildIntrinsicParametersMatrix(displayMetrics);
		Float64Matrix extrinsicParametersMatrix = buildExtrinsicParametersMatrix((SensorManager) getSystemService(SENSOR_SERVICE));

	}

	protected static Float64Matrix buildIntrinsicParametersMatrix(DisplayMetrics displayMetrics) {

		// System.out.println("VNCTests : buildIntrinsicParametersMatrix");

		// // Récupération de la focale en pixels
		// float focalLength = cameraParameters.getFocalLength();
		float focalLengthInPixel = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM, CAMERA_FOCAL_LENGTH, displayMetrics);
		// Coordonnées du centre de l'écran dans notre repère
		float centreX = (float) (displayMetrics.widthPixels / 2);
		float centreY = (float) (displayMetrics.heightPixels / 2);

		// Initialisation de la matrice des paramètres intrinsèques à la caméra et ajout des composants à la matrice
		double[][] tempMatrix = { { focalLengthInPixel, 0.0f, centreX }, { 0.0f, focalLengthInPixel, centreY }, { 0.0f, 0.0f, 1.0f } };
		Float64Matrix intrinsicParametersMatrix = Float64Matrix.valueOf(tempMatrix);

		return intrinsicParametersMatrix;

	}

	protected static Float64Matrix buildExtrinsicParametersMatrix(SensorManager mSensorManager) {

		// System.out.println("VNCTests : buildExtrinsicParametersMatrix");

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
			// System.out.println("VNCTests : Orientation success = " + success);
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

		// Initialisation des valeurs de position de l'appareil
		float thalesProportion = ROBOT_HEIGHT_ON_SCREEN / ROBOT_HEIGHT;
		float distanceRobotImageFromFocal = (float) Math.sqrt(CAMERA_FOCAL_LENGTH * CAMERA_FOCAL_LENGTH + DISTANCE_ROBOT_IMAGE_FROM_CENTER * DISTANCE_ROBOT_IMAGE_FROM_CENTER);
		float distanceRobotFromFocal = distanceRobotImageFromFocal * ROBOT_HEIGHT / ROBOT_HEIGHT_ON_SCREEN;

		float tx = (float) Math.sqrt(distanceRobotFromFocal * distanceRobotFromFocal - CAMERA_HEIGHT_FROM_GROUND * CAMERA_HEIGHT_FROM_GROUND);
		float ty = DISTANCE_ROBOT_IMAGE_FROM_CENTER * ROBOT_HEIGHT / ROBOT_HEIGHT_ON_SCREEN;
		float tz = CAMERA_HEIGHT_FROM_GROUND;

		// Initialisation de la matrice de paramètres extrinsèques et ajout des composants de rotation et de translation à la matrice
		double[][] tempMatrix = { { Math.cos(azimuth) * Math.cos(roll), Math.cos(azimuth) * Math.sin(roll) * Math.sin(pitch) - Math.sin(azimuth) * Math.cos(pitch), Math.cos(azimuth) * Math.sin(roll) * Math.cos(pitch) + Math.sin(azimuth) * Math.sin(pitch), tx }, { Math.sin(azimuth) * Math.cos(roll), Math.sin(azimuth) * Math.sin(roll) * Math.sin(pitch) + Math.cos(azimuth) * Math.cos(pitch), Math.sin(azimuth) * Math.sin(roll) * Math.cos(pitch) - Math.cos(azimuth) * Math.sin(pitch), ty }, { -Math.sin(roll), Math.cos(roll) * Math.sin(pitch), Math.cos(roll) * Math.cos(pitch), tz } };
		Float64Matrix extrinsicParametersMatrix = Float64Matrix.valueOf(tempMatrix);

		return extrinsicParametersMatrix;
	}

	public static void init(SensorManager mSensorManager) {
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