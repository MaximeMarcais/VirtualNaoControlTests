package com.nao.virtualnaocontrol;

import org.jscience.mathematics.vector.Float64Matrix;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.Toast;

public class Detection extends Activity implements CvCameraViewListener2 {

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
		// System.out.println("VNCTests : onCameraViewStarted");
	}

	public void onCameraViewStopped() {}

	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

		// System.out.println("VNCTests : onCameraFrame");

		try {
			detection(getResources().getDisplayMetrics(), null, null, null);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Affichage
		return inputFrame.rgba();
	}

	@Override
	public boolean onTouchEvent(MotionEvent event) {

		// System.out.println("VNCTests : onTouchEvent");

		// Récupération de la position du clique sur l'écran
		if (event.getAction() == MotionEvent.ACTION_DOWN) {
			touchScreenPositionX = event.getX();
			touchScreenPositionY = event.getY();
		}

		return super.onTouchEvent(event);
	}

	public Mat detection(DisplayMetrics displayMetrics, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat touchedPointMatrix) throws Exception {

		System.out.println("VNCTests : detection");

		// Input
		touchedPointMatrix = new Mat(3, 1, CvType.CV_32F);
		touchedPointMatrix.put(0, 0, 749); // X ?
		touchedPointMatrix.put(1, 0, 921); // Y ?

		Point3 point3D1 = new Point3(0.02210916206240654, 0.0, 0.44615206122398376); // Tête
		Point3 point3D2 = new Point3(0.1174665167927742, 0.11092513799667358, 0.27571398019790649); // Bras gauche
		Point3 point3D3 = new Point3(0.1174665167927742, -0.11092513799667358, 0.27571398019790649); // Bras droit
		Point3 point3D4 = new Point3(0.020062578842043877, 0.08999999612569809, 0.42575448751449585); // Epaule gauche
		objectPoints = new MatOfPoint3f(point3D1, point3D2, point3D3, point3D4); // Points du robot dans son repère

		Point point2D1 = new Point(1673, 461); // Tête
		Point point2D2 = new Point(1977, 1161); // Bras gauche
		Point point2D3 = new Point(1353, 1161); // Bras droit
		Point point2D4 = new Point(1897, 609); // Epaule gauche
		imagePoints = new MatOfPoint2f(point2D1, point2D2, point2D3, point2D4); // Points du robot dans le repère de la caméra

		Mat touchPointInRobotReference = MatrixTransformations.detection(displayMetrics, objectPoints, imagePoints, touchedPointMatrix);

		System.out.println("VNCTests TOUCH : X=" + touchPointInRobotReference.get(0, 0)[0]); // -0.9 m
		System.out.println("VNCTests TOUCH : Y=" + touchPointInRobotReference.get(1, 0)[0]); // -0.6 m
		System.out.println("VNCTests TOUCH : quelquechose=" + touchPointInRobotReference.get(2, 0)[0]); // Ceci n'es tpas le Z

		return touchPointInRobotReference;

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