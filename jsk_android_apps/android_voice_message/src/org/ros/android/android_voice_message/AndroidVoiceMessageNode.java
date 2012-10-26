package org.ros.android.android_voice_message;




import java.util.ArrayList;
import java.util.List;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import android.content.Intent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;


import android.os.Bundle;
import android.os.Looper;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.widget.Button;
import jsk_gui_msgs.VoiceMessage;
import android.view.View;


import org.ros.node.*;

public class AndroidVoiceMessageNode implements NodeMain,SensorEventListener {


	  //add
    private SensorManager mSensorManager; //センサーマネージャ
    private List<Sensor> sensors;
    private int[] flag;
    private Publisher<jsk_gui_msgs.VoiceMessage> voice_pub;
    private jsk_gui_msgs.VoiceMessage voice_msg;
    private int startFlag = 0;
     private SpeechRecognizer sr;
     private String package_name;
     
	@Override
	  public GraphName getDefaultNodeName() {
	    return GraphName.of("jsk_gui_msgs/VoiceMessage");
	  }

public AndroidVoiceMessageNode(SensorManager manager,SpeechRecognizer sr,String package_name){
		mSensorManager = manager;
    	this.sr = sr;
    	this.package_name = package_name;
//		sr = SpeechRecognizer.createSpeechRecognizer(getApplicationContext());
}



	  @Override
	  public void onStart(final ConnectedNode connectedNode) {
		  
		  //センサー管理
		  List<Sensor>	sensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);
		  
		  for(Sensor sensor: sensors){
			  int sensorType = sensor.getType();
			  switch(sensorType){
			  case Sensor.TYPE_MAGNETIC_FIELD:
				  mSensorManager.registerListener(this,sensor,SensorManager.SENSOR_DELAY_NORMAL);
			  }
			  
		  }
		  
		  
		  try{
			voice_pub = connectedNode.newPublisher("Voice","jsk_gui_msgs/VoiceMessage");
			Log.v("test","publish start");
			startFlag=1;
		  }catch (Exception e){
			  
		  }
		  
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
	    

	   
	    public void onSensorChanged(SensorEvent event){

	    	sr.setRecognitionListener(new SpeechListener());
	    	
	    	Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
	    	intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
	    	intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, package_name);
	    	sr.startListening(intent);
	    	
	    	/*switch(event.sensor.getType()){
		    case Sensor.TYPE_ACCELEROMETER:
			imu_msg.getLinearAcceleration().setX(event.values[0]);
			imu_msg.getLinearAcceleration().setY(event.values[1]);
			imu_msg.getLinearAcceleration().setZ(event.values[2]);
			imu_msg.getHeader().setFrameId("/imu");
			imu_pub.publish(imu_msg);
			Log.v("ok","ok");
			break;
	    	}*/

	    }


	    public void onAccuracyChanged(Sensor sensor,int accuracy){
	    }
	    
	    	
	    /* inner class*/
	    public class SpeechListener implements RecognitionListener{
	    	
	    	@Override
	    	public void onBeginningOfSpeech(){
	    		
	    	}

			@Override
			public void onBufferReceived(byte[] buffer) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onEndOfSpeech() {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onError(int error) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onEvent(int eventType, Bundle params) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onPartialResults(Bundle partialResults) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onReadyForSpeech(Bundle params) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onResults(Bundle results) {
				voice_msg = voice_pub.newMessage();

				ArrayList<String> candidates = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
				  voice_msg.setTexts(candidates);
					Log.v("test","publish ready");
				  if(startFlag==1){
					  voice_pub.publish(voice_msg);
						Log.v("test","publish ok");
				  }
			}

			@Override
			public void onRmsChanged(float rmsdB) {
				// TODO Auto-generated method stub
				
			}
	    	
	    	
	    }
	    
	    
	    
	    }
