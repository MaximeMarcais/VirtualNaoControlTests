package com.nao.virtualnaocontrol;

import org.opencv.core.MatOfFloat4;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.v7.app.ActionBarActivity;
import android.view.Menu;
import android.view.MenuItem;

public class Orientation extends ActionBarActivity {

	// private SensorManager mSensorManager;
	private static Sensor accelerometer;
	private static Sensor magnetometer;
	private static float[] mGravity;
	private static float[] mGeomagnetic;

	// private final float D = (float) 3.3; // TODO : DELETE ON PROD

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		System.out.println("VNCTests : onCreate");
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		init((SensorManager) getSystemService(SENSOR_SERVICE));
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		System.out.println("VNCTests : onCreateOptionsMenu");
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		System.out.println("VNCTests : onOptionsItemSelected");
		// Handle action bar item clicks here. The action bar will
		// automatically handle clicks on the Home/Up button, so long
		// as you specify a parent activity in AndroidManifest.xml.

		// TEST

		captureRotation();

		// END TEST

		int id = item.getItemId();
		if (id == R.id.action_settings) {
			return true;
		}
		return super.onOptionsItemSelected(item);
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

	public static void init(SensorManager mSensorManager) {
		setAccelerometer(mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER));
		setMagnetometer(mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD));

		// Listen
		mSensorManager.registerListener(sel, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(sel, magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
	}

	public static void captureRotation() {
		if (mGravity != null && mGeomagnetic != null) {
			float R[] = new float[9];
			float I[] = new float[9];

			boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
			System.out.println("VNCTests : Orientation success = " + success);
			if (success) {
				float orientation[] = new float[3];
				SensorManager.getOrientation(R, orientation);

				// Orientation data
				float pitch = (float) Math.toDegrees(orientation[1]);
				System.out.println("VNCTests : Orientation data pitch (X) = " + pitch);
				float roll = (float) Math.toDegrees(orientation[2]);
				System.out.println("VNCTests : Orientation data roll (Y) = " + roll);
				float azimuth = (float) Math.toDegrees(orientation[0]);
				System.out.println("VNCTests : Orientation data azimuth (Z) = " + azimuth);

				// Angle
				float alpha = 90 - Math.abs(pitch);
				System.out.println("VNCTests : Angle (alpha) en degré = " + alpha);

				// Distance
				// float distance = (float) Math.cos(alpha) * D;
				// System.out.println("VNCTests : distance au robot = " + distance);

				// Vecteur
				MatOfFloat4 vector = new MatOfFloat4();
			}
		}
	}

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
