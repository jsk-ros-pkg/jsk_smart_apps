package org.ros.android.androidSensorMessage;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import android.app.IntentService;
import android.app.Notification;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;
import android.widget.TextView;

public class AndroidSensorMessageService extends IntentService {

	private SensorManager sensorManager;
	private List<Sensor> sensors;
	private AndroidSensorMessageNode androidSensorMessageNode;
	private Intent mintent;
	private int[] availableSensor;
	private int[] publishedSensor;
	private String[] sensorName;

	final IBinder binder = new MyBinder();

	public class MyBinder extends Binder {
		AndroidSensorMessageService getService() {
			return AndroidSensorMessageService.this;
		}
	}

	public AndroidSensorMessageService() {
		super("AndroidSensorMessageService");
	}

	@Override
	public IBinder onBind(Intent intent) {
		mintent = intent;
		sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		sensors = sensorManager.getSensorList(Sensor.TYPE_ALL);

		availableSensor = new int[13];
		publishedSensor = new int[13];
		for (int i = 0; i < 13; i++) {
			availableSensor[i] = -1;
			publishedSensor[i] = -1;
		}

		sensorName = new String[13];

		for (Sensor sensor : sensors) {
			int sensorType = sensor.getType();
			switch (sensorType) {
			/*
			 * case Sensor.TYPE_AMBIENT_TEMPERATURE: sensorName[sensorType-1]
			 * =new String("Temperature"); availableSensor[sensorType-1] = 1;
			 * break;
			 */
			case Sensor.TYPE_ACCELEROMETER:
				sensorName[sensorType - 1] = new String("Accelerometer");
				availableSensor[sensorType - 1] = 1;
				break;
			case Sensor.TYPE_TEMPERATURE:
				sensorName[sensorType - 1] = new String("Temperature");
				availableSensor[sensorType - 1] = 1;
				break;
			case Sensor.TYPE_MAGNETIC_FIELD:
				sensorName[sensorType - 1] = new String("MagneticField");
				availableSensor[sensorType - 1] = 1;
				break;
			case Sensor.TYPE_LIGHT:
				sensorName[sensorType - 1] = new String("Light");
				availableSensor[sensorType - 1] = 1;
				break;
			case Sensor.TYPE_PROXIMITY:
				sensorName[sensorType - 1] = new String("Proximity");
				availableSensor[sensorType - 1] = 1;
				break;
			case Sensor.TYPE_GYROSCOPE:
				sensorName[sensorType - 1] = new String("Gyroscope");
				availableSensor[sensorType - 1] = 1;
				break;
			case Sensor.TYPE_PRESSURE:
				sensorName[sensorType - 1] = new String("Pressure");
				availableSensor[sensorType - 1] = 1;
				break;
			case Sensor.TYPE_GRAVITY:
				sensorName[sensorType - 1] = new String("Gravity");
				availableSensor[sensorType - 1] = 1;
				break;
			case Sensor.TYPE_ROTATION_VECTOR:
				sensorName[sensorType - 1] = new String("RotationVector");
				availableSensor[sensorType - 1] = 1;
				break;
			/*
			 * case Sensor.TYPE_RELATIVE_HUMIDITY: sensorName[sensorType-1] =new
			 * String("Humidity"); availableSensor[sensorType-1] = 1; break;
			 */

			}

		}

		return binder;
	}

	public String getSensorName(int i) {
		return sensorName[i];
	}

	public int getSensor(int i) {
		return availableSensor[i];
	}

	public void changeSensor(int num) {
		publishedSensor[num] *= -1;
		androidSensorMessageNode.changeSensor(num);

	}

	public void changeView(int num) {
		androidSensorMessageNode.changeView(num);
	}

	public void setNode(int[] flag, AndroidSensorMessageView view,
			TextView textViewX, TextView textViewY, TextView textViewZ) {
		showNotification();

		for (int i = 0; i < 13; i++) {
			publishedSensor[i] = flag[i];
		}

		// We use this bundle
		Bundle b = mintent.getExtras();

		if (this.androidSensorMessageNode == null) {
			try {
				URI masterUri = new URI(b.getString("masterUri"));
				String hostLocal = InetAddressFactory.newNonLoopback()
						.getHostAddress();
				// At this point, the user has already been prompted to either
				// enter the URI
				// of a master to use or to start a master locally.
				NodeMainExecutor e = DefaultNodeMainExecutor.newDefault();

				NodeConfiguration nodeConfiguration = NodeConfiguration
						.newPublic(hostLocal, masterUri);
				nodeConfiguration.setMasterUri(masterUri);
				nodeConfiguration.setNodeName("android_sensor_message");

				androidSensorMessageNode = new AndroidSensorMessageNode(
						sensorManager, sensors, view, textViewX, textViewY,
						textViewZ, publishedSensor);

				e.execute(androidSensorMessageNode, nodeConfiguration);

			} catch (URISyntaxException e) {
				e.printStackTrace();
			}
		}
	}

	@Override
	protected void onHandleIntent(Intent intent) {
		try {
			Thread.sleep(10000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

	}

	/**
	 * Show a notification while this service is running.
	 */
	private void showNotification() {

		Intent i = new Intent(this, AndroidSensorMessage.class);

		i.setFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP
				| Intent.FLAG_ACTIVITY_SINGLE_TOP);

		PendingIntent pi = PendingIntent.getActivity(this, 0, i, 0);

	}

	@Override
	public void onDestroy() {
		this.androidSensorMessageNode.onShutdown(null);
	}

}