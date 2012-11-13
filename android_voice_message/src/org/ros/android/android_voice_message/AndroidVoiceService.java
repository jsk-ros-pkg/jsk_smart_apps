package org.ros.android.android_voice_message;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import android.app.IntentService;
import android.app.PendingIntent;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Binder;
import android.os.Bundle;
import android.os.IBinder;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.widget.TextView;




public class AndroidVoiceService extends IntentService{
	SensorManager mSensorManager;
	List<Sensor> sensors;
	AndroidVoiceMessageNode main_node;
	Intent mintent;
	NodeMainExecutor e;
    private String package_name;
   SpeechRecognizer sr;
   
	final IBinder binder = new MyBinder();
	
	public class MyBinder extends Binder{
		AndroidVoiceService getService(){
			return AndroidVoiceService.this;
		}
	}
	
	public AndroidVoiceService(){
		super("Service");
	}
	
	@Override
	public IBinder onBind(Intent intent){
		mintent = intent;
		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
		sensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);
		Log.v("test","test");
		return binder;
	}
		
	
	
	
		public void setNode(SpeechRecognizer sr,TextView textView,String package_name){
		  showNotification();
		  this.sr = sr;
		  this.package_name = package_name;
		  
		    
		    // We use this bundle
		    Bundle b = mintent.getExtras();
		    
		    
		    if(this.main_node == null){
		    	try{
		    	URI masterUri = new URI(b.getString("masterUri"));
			  String hostLocal = InetAddressFactory.newNonLoopback().getHostAddress();
		  // At this point, the user has already been prompted to either enter the URI
		  // of a master to use or to start a master locally.
		  e = DefaultNodeMainExecutor.newDefault();

		  NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostLocal,masterUri);
		  nodeConfiguration.setMasterUri(masterUri);
		  nodeConfiguration.setNodeName("android_voice_message");

		  main_node = new AndroidVoiceMessageNode(mSensorManager,sr,textView,package_name);
		  e.execute(main_node, nodeConfiguration);	  



		    	}catch(URISyntaxException e){
		    		e.printStackTrace();
		    	}
		    }
		}
		    
		public void setStart(){
			main_node.setFlag();
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
	  
		Intent i=new Intent(this, MainActivity.class);
		
		i.setFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP|
		Intent.FLAG_ACTIVITY_SINGLE_TOP);
		
		PendingIntent pi=PendingIntent.getActivity(this, 0, i, 0);
		
  }
  
  @Override
  public void onDestroy(){
	  e.shutdownNodeMain(main_node);
  }
}
