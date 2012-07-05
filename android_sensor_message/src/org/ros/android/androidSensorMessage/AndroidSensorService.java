package org.ros.android.androidSensorMessage;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import android.app.Notification;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.app.IntentService;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

public class AndroidSensorService extends IntentService{
	SensorManager mSensorManager;
	List<Sensor> sensors;
	AndroidSensorMessageNode main_node;
	NodeMain talker;
	Intent mintent;
	int[] usedFlag;
	int[] usedFlag2;
	final IBinder binder = new MyBinder();
	
	public class MyBinder extends Binder{
		AndroidSensorService getService(){
			return AndroidSensorService.this;
		}
	}
	
	
	public AndroidSensorService(){
		super("Service");
	}
	
	@Override
	public IBinder onBind(Intent intent){
		mintent = intent;
		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
		sensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);
		
		usedFlag = new int[9];
		usedFlag2 = new int[9];
		for(int i=0;i<9;i++){
			usedFlag[i]=0;
			usedFlag2[i]=0;
		}
		
		
		for(Sensor sensor: sensors){
		    int sensorType = sensor.getType();
		    switch(sensorType){
			/*case Sensor.TYPE_AMBIENT_TEMPERATURE:
			 this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
			   break;*/
		    case Sensor.TYPE_ACCELEROMETER:
		    	usedFlag[0]=1;
			break;
		    case Sensor.TYPE_TEMPERATURE:
		    	usedFlag[1]=1;
			break;
		    case Sensor.TYPE_MAGNETIC_FIELD:
		    	usedFlag[2]=1;
			break;
		    case Sensor.TYPE_LIGHT:
		    	usedFlag[3]=1;
			break;
		    case Sensor.TYPE_PROXIMITY:
		    	usedFlag[4]=1;
			break;
		    case Sensor.TYPE_GYROSCOPE:
		    	usedFlag[5]=1;
			break;
		    case Sensor.TYPE_PRESSURE:
		    	usedFlag[6]=1;
	    		break;
		    case Sensor.TYPE_GRAVITY:
		    	usedFlag[7]=1;
	        	break;
		    case Sensor.TYPE_ROTATION_VECTOR:
		    	usedFlag[8]=1;
			break;
			 /*   case Sensor.TYPE_RELATIVE_HUMIDITY:
			 this.sensorManager.registerListener(this.sensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
			    break;*/
		   }

		}

		return binder;
	}
		
	public int getSensor(int i){
		return usedFlag[i];
	}
	
	
	
		public void setNode(int[] flag){
		  showNotification();
		  
			for(int i=0;i<9;i++){
				usedFlag2[i]=flag[i];
				Log.v("test",i+":"+usedFlag2[i]);
			}
		    
		    // We use this bundle
		    Bundle b = mintent.getExtras();
		    
		    
		    if(this.main_node == null){
		    	try{
		    	URI masterUri = new URI(b.getString("masterUri"));
			  String hostLocal = InetAddressFactory.newNonLoopback().getHostAddress();
		  // At this point, the user has already been prompted to either enter the URI
		  // of a master to use or to start a master locally.
		  NodeMainExecutor e = DefaultNodeMainExecutor.newDefault();

		  NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostLocal,masterUri);
		  nodeConfiguration.setMasterUri(masterUri);
		  nodeConfiguration.setNodeName("android_sensor_message");

		  talker = new AndroidSensorMessageNode(mSensorManager,sensors,usedFlag2);

		  e.execute(talker, nodeConfiguration);


		    	}catch(URISyntaxException e){
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
	  
		Intent i=new Intent(this, AndroidSensorMessage.class);
		
		i.setFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP|
		Intent.FLAG_ACTIVITY_SINGLE_TOP);
		
		PendingIntent pi=PendingIntent.getActivity(this, 0, i, 0);
		
  }
  
  @Override
  public void onDestroy(){
  	this.talker.onShutdown(null);
  }

	
}