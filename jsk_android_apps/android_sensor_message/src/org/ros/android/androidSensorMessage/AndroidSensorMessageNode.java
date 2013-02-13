package org.ros.android.androidSensorMessage;


import java.util.List;

import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;

import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import jsk_gui_msgs.DeviceSensor;
import jsk_gui_msgs.Gravity;
import jsk_gui_msgs.MagneticField;
import sensor_msgs.Imu;
import android.view.View;

import org.ros.node.*;

public class AndroidSensorMessageNode implements NodeMain {

	// add
    private SensorManager sensorManager; // センサーマネージャ
    private SensorListener sensorListener;
    private AndroidSensorMessageView androidSensorMessageView;
    private List<Sensor> sensors;
    private int[] publishedSensor;
    private int currentView = 0;
    private TextView textViewX;
    private TextView textViewY;
    private TextView textViewZ;

	private Publisher<sensor_msgs.Imu> imuPublisher;
	private Publisher<jsk_gui_msgs.DeviceSensor> devPublisher;
	private Publisher<jsk_gui_msgs.Gravity> gravPublisher;
	private Publisher<jsk_gui_msgs.MagneticField> magPublisher;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("sensor_msgs/Imu");
	}

	public AndroidSensorMessageNode(SensorManager manager,
					List<Sensor> sensors, AndroidSensorMessageView androidSensorMessageView,TextView textViewX,TextView textViewY,TextView textViewZ,int[] publishedSensor) {
		sensorManager = manager;
		this.sensors = sensors;
		this.androidSensorMessageView = androidSensorMessageView;
		this.textViewX = textViewX;
		this.textViewY = textViewY;
		this.textViewZ = textViewZ;
		this.publishedSensor = new int[13];
		for (int i = 0; i < 13; i++) {

			this.publishedSensor[i] = publishedSensor[i];
		}
	}

	public void changeSensor(int num) {
		publishedSensor[num] *= -1;
		setSensors();
	}
	
	public void changeView(int num){
		currentView = num;
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		try {
			imuPublisher = connectedNode.newPublisher("imu",
					"sensor_msgs/Imu");
			devPublisher = connectedNode.newPublisher("device",
					"jsk_gui_msgs/DeviceSensor");
			gravPublisher = connectedNode.newPublisher("gravity",
					"jsk_gui_msgs/Gravity");
			magPublisher = connectedNode.newPublisher("magneticfield",
					"jsk_gui_msgs/MagneticField");


			this.sensorListener = new SensorListener(imuPublisher, devPublisher,
					gravPublisher, magPublisher);
  
		} catch (Exception e) {

		}

	}

	public void setSensors() {
		for (Sensor sensor : sensors) {
			int sensorType = sensor.getType();
			switch (sensorType) {
			/*
			 * case Sensor.TYPE_AMBIENT_TEMPERATURE:
			 * sensorManager.registerListener(this.sensorListener, sensor,
			 * SensorManager.SENSOR_DELAY_FASTEST); break;
			 */
			case Sensor.TYPE_ACCELEROMETER:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_TEMPERATURE:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_MAGNETIC_FIELD:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_LIGHT:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_PROXIMITY:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_GYROSCOPE:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_PRESSURE:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);
				break;
			case Sensor.TYPE_GRAVITY:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_ROTATION_VECTOR:
				if (publishedSensor[sensorType - 1] == 1)
					sensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					sensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			/*
			 * case Sensor.TYPE_RELATIVE_HUMIDITY:
			 * sensorManager.registerListener(this.sensorListener, sensor,
			 * SensorManager.SENSOR_DELAY_FASTEST); break;
			 */
			}

		}
	}

	@Override
	public void onShutdown(Node node) {
		sensorManager.unregisterListener(this.sensorListener);
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public void onError(Node node, Throwable throwable) {
	}

	private class SensorListener implements SensorEventListener {
		Publisher<Imu> imuPublisher;
		Publisher<DeviceSensor> devPublisher;
		Publisher<Gravity> gravPublisher;
		Publisher<MagneticField> magPublisher;

		sensor_msgs.Imu imuMessage;
		jsk_gui_msgs.DeviceSensor devMessage;
		jsk_gui_msgs.Gravity gravMessage;
		jsk_gui_msgs.MagneticField magMessage;

		private SensorListener(Publisher<sensor_msgs.Imu> imuPublisher,
				Publisher<jsk_gui_msgs.DeviceSensor> devPublisher,
				Publisher<jsk_gui_msgs.Gravity> gravPublisher,
				Publisher<jsk_gui_msgs.MagneticField> magPublisher) {

			this.imuPublisher = imuPublisher;
			this.devPublisher = devPublisher;
			this.gravPublisher = gravPublisher;
			this.magPublisher = magPublisher;

			imuMessage = this.imuPublisher.newMessage();
			devMessage = this.devPublisher.newMessage();
			magMessage = this.magPublisher.newMessage();
			gravMessage = this.gravPublisher.newMessage();
		}

		public void onSensorChanged(SensorEvent event) {
			long time_delta_millis;
			switch (event.sensor.getType()) {
			case Sensor.TYPE_ACCELEROMETER:
				if(currentView == Sensor.TYPE_ACCELEROMETER - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);
					androidSensorMessageView.addValues(event.values);
				}
				imuMessage.getLinearAcceleration().setX(event.values[0]);
				imuMessage.getLinearAcceleration().setY(event.values[1]);
				imuMessage.getLinearAcceleration().setZ(event.values[2]);
				time_delta_millis = System.currentTimeMillis()
						- SystemClock.uptimeMillis();
				imuMessage.getHeader().setStamp(
						Time.fromMillis(time_delta_millis + event.timestamp
								/ 1000000));
				imuMessage.getHeader().setFrameId("/imu");
				Log.v("type", "ok");
				imuPublisher.publish(imuMessage);
				break;
			case Sensor.TYPE_TEMPERATURE:
				if(currentView == Sensor.TYPE_TEMPERATURE - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);
					androidSensorMessageView.addValues(event.values);
				}
				devMessage.setTemperature(event.values[0]);
				devPublisher.publish(devMessage);
				break;
			case Sensor.TYPE_MAGNETIC_FIELD:
				if(currentView == Sensor.TYPE_MAGNETIC_FIELD - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					androidSensorMessageView.addValues(event.values);
				}
				magMessage.getMagneticfield().setX(event.values[0]);
				magMessage.getMagneticfield().setY(event.values[1]);
				magMessage.getMagneticfield().setZ(event.values[2]);

				time_delta_millis = System.currentTimeMillis()
						- SystemClock.uptimeMillis();
				magPublisher.publish(magMessage);
				break;
			case Sensor.TYPE_LIGHT:
				if(currentView == Sensor.TYPE_LIGHT - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					androidSensorMessageView.addValues(event.values);
				}
				devMessage.setLight(event.values[0]);
				devPublisher.publish(devMessage);
				break;
			case Sensor.TYPE_PROXIMITY:
				if(currentView == Sensor.TYPE_PROXIMITY - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					androidSensorMessageView.addValues(event.values);
				}
				devMessage.setProximity(event.values[0]);
				devPublisher.publish(devMessage);
				break;
			case Sensor.TYPE_GYROSCOPE:
				if(currentView == Sensor.TYPE_GYROSCOPE - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					androidSensorMessageView.addValues(event.values);
				}
				imuMessage.getAngularVelocity().setX(event.values[0]);
				imuMessage.getAngularVelocity().setY(event.values[1]);
				imuMessage.getAngularVelocity().setZ(event.values[2]);
				time_delta_millis = System.currentTimeMillis()
						- SystemClock.uptimeMillis();
				imuMessage.getHeader().setStamp(
						Time.fromMillis(time_delta_millis + event.timestamp
								/ 1000000));
				imuMessage.getHeader().setFrameId("/imu");
				imuPublisher.publish(imuMessage);
				break;
			case Sensor.TYPE_PRESSURE:
				if(currentView == Sensor.TYPE_PRESSURE - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					androidSensorMessageView.addValues(event.values);
				}
				devMessage.setPressure(event.values[0]);
				devPublisher.publish(devMessage);
				break;
			case Sensor.TYPE_GRAVITY:
				if(currentView == Sensor.TYPE_GRAVITY - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					androidSensorMessageView.addValues(event.values);
				}
				gravMessage.getGravity().setX(event.values[0]);
				gravMessage.getGravity().setY(event.values[1]);
				gravMessage.getGravity().setZ(event.values[2]);
				gravPublisher.publish(gravMessage);
				break;
			case Sensor.TYPE_ROTATION_VECTOR:
				if(currentView == Sensor.TYPE_ROTATION_VECTOR - 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					androidSensorMessageView.addValues(event.values);
				}
				float[] quaternion = new float[4];
				SensorManager.getQuaternionFromVector(quaternion, event.values);
				imuMessage.getOrientation().setX(quaternion[0]);
				imuMessage.getOrientation().setY(quaternion[1]);
				imuMessage.getOrientation().setZ(quaternion[2]);
				imuMessage.getOrientation().setW(quaternion[3]);
				imuMessage.getHeader().setFrameId("/imu");
				imuPublisher.publish(imuMessage);
				break;
			/*
			 * case Sensor.TYPE_RELATIVE_HUMIDITY:
			 * devMessage.setRelativeHumidity(event.values[0]);
			 * devPublisher.publish(devMessage); break;
			 */

			}

		}

		public void onAccuracyChanged(Sensor sensor, int accuracy) {
		}

	}

}
