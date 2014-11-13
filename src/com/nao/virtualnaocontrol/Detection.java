package com.nao.virtualnaocontrol;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

import android.app.Activity;
import android.hardware.Camera;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.TypedValue;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.Toast;

public class Detection extends Activity implements CvCameraViewListener2 {

	private final float ROBOT_HEIGHT = 580; // Exprimé en mm
	
	private CameraBridgeViewBase mOpenCvCameraView;
	private boolean mIsJavaCamera = true;
	private MenuItem mItemSwitchCamera = null;

	private static Sensor accelerometer;
	private static Sensor magnetometer;
	private static float[] mGravity;
	private static float[] mGeomagnetic;

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

		System.out.println("VNCTests : onCameraFrame");
		detection(getResources().getDisplayMetrics());

		// Affichage
		return inputFrame.rgba();
	}

	public void detection(DisplayMetrics displayMetrics) {

		MatOfFloat intrinsicParametersMatrix = buildIntrinsicParametersMatrix(displayMetrics);
		MatOfFloat extrinsicParametersMatrix = buildExtrinsicParametersMatrix((SensorManager) getSystemService(SENSOR_SERVICE));
	}

	protected static MatOfFloat buildIntrinsicParametersMatrix(DisplayMetrics displayMetrics) {

		// Initialisation de la matrice des paramètres intrinsèques à la caméra
		MatOfFloat cameraMatrix = (MatOfFloat) MatOfFloat.zeros(3, 3, CvType.CV_32F);

		// Récupération de la focale en pixels
		Camera camera = Camera.open();
		float focalLength = camera.getParameters().getFocalLength();
		float focalLengthInPixel = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM, focalLength, displayMetrics);

		// Coordonnées du centre de l'écran dans notre repère
		float centreX = 0;
		float centreY = 0;

		// Ajout des composants à la matrice
		cameraMatrix.put(0, 0, focalLengthInPixel);
		cameraMatrix.put(1, 1, focalLengthInPixel);
		cameraMatrix.put(0, 2, centreX);
		cameraMatrix.put(1, 2, centreY);
		cameraMatrix.put(2, 2, 1);

		return cameraMatrix;

	}

	protected static MatOfFloat buildExtrinsicParametersMatrix(SensorManager mSensorManager) {

		// Initialisation de la matrice de paramètres extrinsèques
		MatOfFloat extrinsicParametersMatrix = (MatOfFloat) MatOfFloat.zeros(3, 4, CvType.CV_32F);

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
			System.out.println("VNCTests : Orientation success = " + success);
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

		// Ajout des composants de rotation de la matrice
		extrinsicParametersMatrix.put(0, 0, Math.cos(azimuth) * Math.cos(roll));
		extrinsicParametersMatrix.put(0, 1, Math.cos(azimuth) * Math.sin(roll) * Math.sin(pitch) - Math.sin(azimuth) * Math.cos(pitch));
		extrinsicParametersMatrix.put(0, 2, Math.cos(azimuth) * Math.sin(roll) * Math.cos(pitch) + Math.sin(azimuth) * Math.sin(pitch));
		extrinsicParametersMatrix.put(1, 0, Math.sin(azimuth) * Math.cos(roll));
		extrinsicParametersMatrix.put(1, 1, Math.sin(azimuth) * Math.sin(roll) * Math.sin(pitch) + Math.cos(azimuth) * Math.cos(pitch));
		extrinsicParametersMatrix.put(1, 2, Math.sin(azimuth) * Math.sin(roll) * Math.cos(pitch) - Math.cos(azimuth) * Math.sin(pitch));
		extrinsicParametersMatrix.put(2, 0, -Math.sin(roll));
		extrinsicParametersMatrix.put(2, 1, Math.cos(roll) * Math.sin(pitch));
		extrinsicParametersMatrix.put(2, 2, Math.cos(roll) * Math.cos(pitch));

		// Initialisation des valeurs de position de l'appareil
		float tx = 0.0f;
		float ty = 0.0f;
		float tz = 0.0f;

		// Récupération des paramètres de position du robot par rapprt à l'appareil

		// Ajout des composants de translation de la matrice
		extrinsicParametersMatrix.put(2, 0, tx);
		extrinsicParametersMatrix.put(2, 1, ty);
		extrinsicParametersMatrix.put(2, 2, tz);

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