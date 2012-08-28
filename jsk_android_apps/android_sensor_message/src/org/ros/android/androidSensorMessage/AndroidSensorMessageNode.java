package org.ros.android.androidSensorMessage;

import geometry_msgs.Vector3;

import java.util.List;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;


import android.os.Looper;
import android.util.Log;
import android.widget.Button;
import jsk_gui_msgs.DeviceSensor;
import jsk_gui_msgs.Gravity;
import jsk_gui_msgs.MagneticField;
import sensor_msgs.Imu;
import android.view.View;


import org.ros.node.*;

public class AndroidSensorMessageNode implements NodeMain {


	  //add
    private SensorManager mSensorManager; //センサーマネージャ
    private SensorListener sensorListener;
    private SensorThread sensorThread;
    private List<Sensor> sensors;
    private int[] flag;

   
    private Publisher<sensor_msgs.Imu> imu_pub;
    private Publisher<jsk_gui_msgs.DeviceSensor> dev_pub;
    private Publisher<jsk_gui_msgs.Gravity> grav_pub;
    private Publisher<jsk_gui_msgs.MagneticField> mag_pub;
    private Publisher<geometry_msgs.Vector3> vec_pub;
    
    
 
	
    @Override
	  public GraphName getDefaultNodeName() {
	return GraphName.of("sensor_msgs/Imu");
		    }

public AndroidSensorMessageNode(SensorManager manager,List<Sensor> sensors,int[] usedflag){
	mSensorManager = manager;
	this.sensors = sensors;
	flag = new int[9];
	for(int i=0;i<9;i++){
		
	flag[i]=usedflag[i];
	Log.v("test",i+":"+flag[i]);
	}
}
	
	  @Override
	  public void onStart(final ConnectedNode connectedNode) {
				  
		  try{
			imu_pub = connectedNode.newPublisher("imu","sensor_msgs/Imu");	
			dev_pub = connectedNode.newPublisher("device","jsk_gui_msgs/DeviceSensor");
			grav_pub = connectedNode.newPublisher("gravity","jsk_gui_msgs/Gravity");
			mag_pub = connectedNode.newPublisher("magneticfield","jsk_gui_msgs/MagneticField");
			vec_pub = connectedNode.newPublisher("vector3","geometry_msgs/Vector3");
						
			this.sensorListener = new SensorListener(imu_pub,dev_pub,grav_pub,mag_pub,vec_pub);
			this.sensorThread = new SensorThread(mSensorManager,sensorListener,sensors);
			this.sensorThread.start();

		  }catch (Exception e){
			  
		  }
		  
	  }
	

	  @Override
	  public void onShutdown(Node node) {
		 this.sensorThread.shutdown();
		  try{
			  this.sensorThread.join();
		  }catch(InterruptedException e){
			  e.printStackTrace();
		  }
	  }

	  @Override
	  public void onShutdownComplete(Node node) {
	  }

	  @Override
	  public void onError(Node node, Throwable throwable) {
	  }
	    

	    private class SensorThread extends Thread{
	    private final SensorManager sensorManager;

	    private SensorListener sensorListener;
	    private Looper threadLooper;
	     List<Sensor> sensors;
	    
	    private SensorThread(SensorManager sensorManager,SensorListener sensorListener,List<Sensor> sensors){
	    		this.sensorListener = sensorListener;
	    		this.sensorManager = sensorManager;
	    		this.sensors = sensors;
		     }
	    
	    public void run(){
	    	Looper.prepare();
	    	this.threadLooper = Looper.myLooper();
	    	
			for(Sensor sensor: sensors){
			    int sensorType = sensor.getType();
			    //Log.v("type","type="+sensor.getType());
			    switch(sensorType){
				/*				case Sensor.TYPE_AMBIENT_TEMPERATURE:
				 this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				 break;*/
			    case Sensor.TYPE_ACCELEROMETER:
			    if(flag[0]==1)this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				break;
			    case Sensor.TYPE_TEMPERATURE:
			    	if(flag[1]==1)this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				break;
			    case Sensor.TYPE_MAGNETIC_FIELD:
			    	if(flag[2]==1)this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				break;
			    case Sensor.TYPE_LIGHT:
			    	if(flag[3]==1)this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				break;
			    case Sensor.TYPE_PROXIMITY:
			    	if(flag[4]==1) this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				break;
			    case Sensor.TYPE_GYROSCOPE:
			    	if(flag[5]==1)this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				break;
			    case Sensor.TYPE_PRESSURE:
			    	if(flag[6]==1)this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
		    		break;
			    case Sensor.TYPE_GRAVITY:
			    	if(flag[7]==1)this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
		        	break;
			    case Sensor.TYPE_ROTATION_VECTOR:
			    	if(flag[8]==1)this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				break;
				/*				 case Sensor.TYPE_RELATIVE_HUMIDITY:
				 this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
				 break;*/
			    }

    		}
    
	    	Looper.loop();
	    }
	    	
	    public void shutdown(){
	    	this.sensorManager.unregisterListener(this.sensorListener);
	    	if(threadLooper != null){
	    		this.threadLooper.quit();
	    	}
	    }
	    
	    }
	    
	    private class SensorListener implements SensorEventListener{
		Publisher<Imu> imu_pub;
		Publisher<DeviceSensor> dev_pub;
		Publisher<Gravity> grav_pub;
		Publisher<MagneticField> mag_pub;
		Publisher<Vector3> vec_pub;

		sensor_msgs.Imu imu_msg;
		jsk_gui_msgs.DeviceSensor dev_msg;
		jsk_gui_msgs.Gravity grav_msg;
		jsk_gui_msgs.MagneticField mag_msg;
		geometry_msgs.Vector3 vec_msg;

	    private SensorListener(Publisher<sensor_msgs.Imu> imu_pub,Publisher<jsk_gui_msgs.DeviceSensor> dev_pub,Publisher<jsk_gui_msgs.Gravity> grav_pub,Publisher<jsk_gui_msgs.MagneticField> mag_pub,Publisher<geometry_msgs.Vector3> vec_pub){
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
	    
	    public void onSensorChanged(SensorEvent event){

		    Log.v("type","type="+event.values[0]);
	    	switch(event.sensor.getType()){
		    case Sensor.TYPE_ACCELEROMETER:
			imu_msg.getLinearAcceleration().setX(event.values[0]);
			imu_msg.getLinearAcceleration().setY(event.values[1]);
			imu_msg.getLinearAcceleration().setZ(event.values[2]);
			imu_msg.getHeader().setFrameId("/imu");
		    Log.v("type","ok");
			imu_pub.publish(imu_msg);
			break;
		   case Sensor.TYPE_TEMPERATURE:
		       //case Sensor.TYPE_AMBIENT_TEMPERATURE:
			dev_msg.setTemperature(event.values[0]);
			dev_pub.publish(dev_msg);
			break;
		    case Sensor.TYPE_MAGNETIC_FIELD:
		    	mag_msg.getMagneticfield().setX(event.values[0]);
		    	mag_msg.getMagneticfield().setY(event.values[1]);
		    	mag_msg.getMagneticfield().setZ(event.values[2]);
			mag_pub.publish(mag_msg);
			break;
		    case Sensor.TYPE_LIGHT:
			dev_msg.setLight(event.values[0]);
			dev_pub.publish(dev_msg);
			break;
		    case Sensor.TYPE_PROXIMITY:
			dev_msg.setProximity(event.values[0]);
			dev_pub.publish(dev_msg);
			break;
		   case Sensor.TYPE_GYROSCOPE:			   
			imu_msg.getAngularVelocity().setX(event.values[0]);
			imu_msg.getAngularVelocity().setY(event.values[1]);
			imu_msg.getAngularVelocity().setZ(event.values[2]);
			imu_msg.getHeader().setFrameId("/imu");
			imu_pub.publish(imu_msg);
			break;
		case Sensor.TYPE_PRESSURE:
			dev_msg.setPressure(event.values[0]);
			dev_pub.publish(dev_msg);
			break;
		case Sensor.TYPE_GRAVITY:	
		    grav_msg.getGravity().setX(event.values[0]);
		    grav_msg.getGravity().setY(event.values[1]);
		    grav_msg.getGravity().setZ(event.values[2]);
		    grav_pub.publish(grav_msg);
		    break;
		case Sensor.TYPE_ROTATION_VECTOR:
		    float[] quaternion = new float[4];
		    SensorManager.getQuaternionFromVector(quaternion, event.values);
		    imu_msg.getOrientation().setX(quaternion[0]);
		    imu_msg.getOrientation().setY(quaternion[1]);
		    imu_msg.getOrientation().setZ(quaternion[2]);
		    imu_msg.getOrientation().setW(quaternion[3]);
		    imu_msg.getHeader().setFrameId("/imu");
		    imu_pub.publish(imu_msg);
		    break;
		    /*			  case Sensor.TYPE_RELATIVE_HUMIDITY:
				dev_msg.setRelativeHumidity(event.values[0]);
				dev_pub.publish(dev_msg);
				break;*/
	    	
	    	}

	    }


	    public void onAccuracyChanged(Sensor sensor,int accuracy){
	    }
	    
	    	
	    	
	    }

	
}
