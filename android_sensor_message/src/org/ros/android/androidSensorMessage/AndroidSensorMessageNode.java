package org.ros.android.androidSensorMessage;

import geometry_msgs.Vector3;

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
    private SensorManager mSensorManager; // センサーマネージャ
    private SensorListener sensorListener;
    private AndroidSensorMessageView view;
    private List<Sensor> sensors;
    private int[] flag;
    private int currentView = 0;
    private TextView textViewX;
    private TextView textViewY;
    private TextView textViewZ;

	private Publisher<sensor_msgs.Imu> imu_pub;
	private Publisher<jsk_gui_msgs.DeviceSensor> dev_pub;
	private Publisher<jsk_gui_msgs.Gravity> grav_pub;
	private Publisher<jsk_gui_msgs.MagneticField> mag_pub;
	// private Publisher<geometry_msgs.Vector3Stamped> mag_pub;
	private Publisher<geometry_msgs.Vector3> vec_pub;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("sensor_msgs/Imu");
	}

	public AndroidSensorMessageNode(SensorManager manager,
					List<Sensor> sensors, AndroidSensorMessageView view,TextView textViewX,TextView textViewY,TextView textViewZ,int[] usedflag) {
		mSensorManager = manager;
		this.sensors = sensors;
		this.view = view;
		this.textViewX = textViewX;
		this.textViewY = textViewY;
		this.textViewZ = textViewZ;
		flag = new int[9];
		for (int i = 0; i < 9; i++) {

			flag[i] = usedflag[i];
			Log.v("test", i + ":" + flag[i]);
		}
	}

	public void changeSensor(int num) {
		Log.v("test", "change" + flag[num]);
		flag[num] *= -1;
		setSensors();
	}
	
	public void changeView(int num){
		currentView = num;
		view.clearValues();
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		try {
			imu_pub = connectedNode.newPublisher("imu/data_raw",
					"sensor_msgs/Imu"); // std_name:"imu"
			dev_pub = connectedNode.newPublisher("device",
					"jsk_gui_msgs/DeviceSensor");
			grav_pub = connectedNode.newPublisher("gravity",
					"jsk_gui_msgs/Gravity");
			mag_pub = connectedNode.newPublisher("magneticfield",
					"jsk_gui_msgs/MagneticField");
			// mag_pub =
			// connectedNode.newPublisher("imu/mag","geometry_msgs/Vector3Stamped");
			vec_pub = connectedNode.newPublisher("vector3",
					"geometry_msgs/Vector3");

			this.sensorListener = new SensorListener(imu_pub, dev_pub,
					grav_pub, mag_pub, vec_pub);

		} catch (Exception e) {

		}

	}

	public void setSensors() {
		for (Sensor sensor : sensors) {
			int sensorType = sensor.getType();
			// Log.v("type","type="+sensor.getType());
			switch (sensorType) {
			/*
			 * case Sensor.TYPE_AMBIENT_TEMPERATURE:
			 * mSensorManager.registerListener(this.sensorListener, sensor,
			 * SensorManager.SENSOR_DELAY_FASTEST); break;
			 */
			case Sensor.TYPE_ACCELEROMETER:
				if (flag[0] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_TEMPERATURE:
				if (flag[1] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_MAGNETIC_FIELD:
				if (flag[2] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_LIGHT:
				if (flag[3] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_PROXIMITY:
				if (flag[4] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_GYROSCOPE:
				if (flag[5] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_PRESSURE:
				if (flag[6] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);
				break;
			case Sensor.TYPE_GRAVITY:
				if (flag[7] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			case Sensor.TYPE_ROTATION_VECTOR:
				if (flag[8] == 1)
					mSensorManager.registerListener(this.sensorListener,
							sensor, SensorManager.SENSOR_DELAY_FASTEST);
				else
					mSensorManager.unregisterListener(this.sensorListener,
							sensor);

				break;
			/*
			 * case Sensor.TYPE_RELATIVE_HUMIDITY:
			 * mSensorManager.registerListener(this.sensorListener, sensor,
			 * SensorManager.SENSOR_DELAY_FASTEST); break;
			 */
			}

		}
	}

	public void shutdown() {
		mSensorManager.unregisterListener(this.sensorListener);

	}

	@Override
	public void onShutdown(Node node) {
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public void onError(Node node, Throwable throwable) {
	}

	private class SensorListener implements SensorEventListener {
		Publisher<Imu> imu_pub;
		Publisher<DeviceSensor> dev_pub;
		Publisher<Gravity> grav_pub;
		Publisher<MagneticField> mag_pub;
		// Publisher<geometry_msgs.Vector3Stamped> mag_pub;
		Publisher<Vector3> vec_pub;

		sensor_msgs.Imu imu_msg;
		jsk_gui_msgs.DeviceSensor dev_msg;
		jsk_gui_msgs.Gravity grav_msg;
		// geometry_msgs.Vector3Stamped mag_msg;
		jsk_gui_msgs.MagneticField mag_msg;
		geometry_msgs.Vector3 vec_msg;

		private SensorListener(Publisher<sensor_msgs.Imu> imu_pub,
				Publisher<jsk_gui_msgs.DeviceSensor> dev_pub,
				Publisher<jsk_gui_msgs.Gravity> grav_pub,
				Publisher<jsk_gui_msgs.MagneticField> mag_pub,
				Publisher<geometry_msgs.Vector3> vec_pub) {
			// private SensorListener(Publisher<sensor_msgs.Imu>
			// imu_pub,Publisher<jsk_gui_msgs.DeviceSensor>
			// dev_pub,Publisher<jsk_gui_msgs.Gravity>
			// grav_pub,Publisher<geometry_msgs.Vector3Stamped>
			// mag_pub,Publisher<geometry_msgs.Vector3> vec_pub){

			this.imu_pub = imu_pub;
			this.dev_pub = dev_pub;
			this.grav_pub = grav_pub;
			this.mag_pub = mag_pub;
			this.vec_pub = vec_pub;

			imu_msg = this.imu_pub.newMessage();
			dev_msg = this.dev_pub.newMessage();
			mag_msg = this.mag_pub.newMessage();
			grav_msg = this.grav_pub.newMessage();
			vec_msg = this.vec_pub.newMessage();
		}

		public void onSensorChanged(SensorEvent event) {
			long time_delta_millis;
			switch (event.sensor.getType()) {
			case Sensor.TYPE_ACCELEROMETER:
				if(currentView == 0){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);
					view.addValues(event.values);
				}
				imu_msg.getLinearAcceleration().setX(event.values[0]);
				imu_msg.getLinearAcceleration().setY(event.values[1]);
				imu_msg.getLinearAcceleration().setZ(event.values[2]);
				time_delta_millis = System.currentTimeMillis()
						- SystemClock.uptimeMillis();
				imu_msg.getHeader().setStamp(
						Time.fromMillis(time_delta_millis + event.timestamp
								/ 1000000));
				imu_msg.getHeader().setFrameId("/imu");
				Log.v("type", "ok");
				imu_pub.publish(imu_msg);
				break;
			case Sensor.TYPE_TEMPERATURE:
				if(currentView == 1){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);
					view.addValues(event.values);
				}
				// case Sensor.TYPE_AMBIENT_TEMPERATURE:
				dev_msg.setTemperature(event.values[0]);
				dev_pub.publish(dev_msg);
				break;
			case Sensor.TYPE_MAGNETIC_FIELD:
				if(currentView == 2){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					view.addValues(event.values);
				}
				mag_msg.getMagneticfield().setX(event.values[0]);
				mag_msg.getMagneticfield().setY(event.values[1]);
				mag_msg.getMagneticfield().setZ(event.values[2]);
				/*
				 * mag_msg.getVector().setX(event.values[0]);
				 * mag_msg.getVector().setY(event.values[1]);
				 * mag_msg.getVector().setZ(event.values[2]);
				 */
				time_delta_millis = System.currentTimeMillis()
						- SystemClock.uptimeMillis();
				// mag_msg.getHeader().setStamp(Time.fromMillis(time_delta_millis
				// + event.timestamp/1000000));
				mag_pub.publish(mag_msg);
				break;
			case Sensor.TYPE_LIGHT:
				if(currentView == 3){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					view.addValues(event.values);
				}
				dev_msg.setLight(event.values[0]);
				dev_pub.publish(dev_msg);
				break;
			case Sensor.TYPE_PROXIMITY:
				if(currentView == 4){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					view.addValues(event.values);
				}
				dev_msg.setProximity(event.values[0]);
				dev_pub.publish(dev_msg);
				break;
			case Sensor.TYPE_GYROSCOPE:
				if(currentView == 5){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					view.addValues(event.values);
				}
				imu_msg.getAngularVelocity().setX(event.values[0]);
				imu_msg.getAngularVelocity().setY(event.values[1]);
				imu_msg.getAngularVelocity().setZ(event.values[2]);
				time_delta_millis = System.currentTimeMillis()
						- SystemClock.uptimeMillis();
				imu_msg.getHeader().setStamp(
						Time.fromMillis(time_delta_millis + event.timestamp
								/ 1000000));
				imu_msg.getHeader().setFrameId("/imu");
				imu_pub.publish(imu_msg);
				break;
			case Sensor.TYPE_PRESSURE:
				if(currentView == 6){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					view.addValues(event.values);
				}
				dev_msg.setPressure(event.values[0]);
				dev_pub.publish(dev_msg);
				break;
			case Sensor.TYPE_GRAVITY:
				if(currentView == 7){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					view.addValues(event.values);
				}
				grav_msg.getGravity().setX(event.values[0]);
				grav_msg.getGravity().setY(event.values[1]);
				grav_msg.getGravity().setZ(event.values[2]);
				grav_pub.publish(grav_msg);
				break;
			case Sensor.TYPE_ROTATION_VECTOR:
				if(currentView == 8){
				    textViewX.setText("x:"+event.values[0]);
				    textViewY.setText("y:"+event.values[1]);
				    textViewZ.setText("z:"+event.values[2]);

					view.addValues(event.values);
				}
				float[] quaternion = new float[4];
				SensorManager.getQuaternionFromVector(quaternion, event.values);
				imu_msg.getOrientation().setX(quaternion[0]);
				imu_msg.getOrientation().setY(quaternion[1]);
				imu_msg.getOrientation().setZ(quaternion[2]);
				imu_msg.getOrientation().setW(quaternion[3]);
				imu_msg.getHeader().setFrameId("/imu");
				imu_pub.publish(imu_msg);
				break;
			/*
			 * case Sensor.TYPE_RELATIVE_HUMIDITY:
			 * dev_msg.setRelativeHumidity(event.values[0]);
			 * dev_pub.publish(dev_msg); break;
			 */

			}

		}

		public void onAccuracyChanged(Sensor sensor, int accuracy) {
		}

	}

}
