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
import android.widget.TextView;
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
    private int busyFlag = 0;
    private int receiveFlag = 0;
    private SpeechRecognizer sr;
     private String package_name;
     private TextView textView;
 
     
	@Override
	  public GraphName getDefaultNodeName() {
	    return GraphName.of("jsk_gui_msgs/VoiceMessage");
	  }

public AndroidVoiceMessageNode(SensorManager manager,SpeechRecognizer sr,TextView textView,String package_name){
    mSensorManager = manager;
    this.sr = sr;
    this.textView = textView;
    sr.setRecognitionListener(new SpeechListener());
    this.package_name = package_name;
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
			voice_pub = connectedNode.newPublisher("Tablet/voice","jsk_gui_msgs/VoiceMessage");
			Log.v("voice","publish start");
			startFlag=1;

		  }catch (Exception e){
			  
		  }

	  }
	

	  @Override
	  public void onShutdown(Node node) {
		  mSensorManager.unregisterListener(this);
		  //sr.destroy();
		  //voice_pub.shutdown();
	  }

	  @Override
	  public void onShutdownComplete(Node node) {
	  }

	  @Override
	  public void onError(Node node, Throwable throwable) {
	  }
	    

	   
	    public void onSensorChanged(SensorEvent event){

	    	
	    	//if you want eternal working
	    	
	    	/*if(busyFlag == 0){
	    		busyFlag--;
	    		receiveFlag = 0;
				Log.v("voice","sensor change");
	    	Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
	    	intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
	    	intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, package_name);
		sr.startListening(intent);
	    	}
	    	
	    	else if(busyFlag < 0){
	    		busyFlag--;	
	    		if(busyFlag<-10 && receiveFlag ==0) busyFlag = 0; 
	    	}
	    	else if(busyFlag > 0){
	    		if(busyFlag == 2) sr.stopListening();
	    		Log.v("voice","stoped");
	    		busyFlag--;
	    	}
	    	*/
	    	
	    	
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
	    
	    public void setFlag(){
	    	Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
	    	intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
	    	intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, package_name);
		Log.v("voice","button pushed!");
		sr.startListening(intent);
		
		
		
	    	
	    }
	    
	    /* inner class*/
	    public class SpeechListener implements RecognitionListener{
	    	
	    	@Override
	    	public void onBeginningOfSpeech(){
		    Log.v("voice","ready!");
	    	}

			@Override
			public void onBufferReceived(byte[] buffer) {
				// TODO Auto-generated method stub
			    Log.v("voice","buffer!");
			    receiveFlag++;
			}

			@Override
			public void onEndOfSpeech() {
			    Log.v("voice","end!");
			    busyFlag = 2;
			    				// TODO Auto-generated method stub
				
			}

			@Override
			public void onError(int error) {

			    switch (error) {
			    case SpeechRecognizer.ERROR_AUDIO:
				// 音声データ保存失敗
				Log.e("voice","save error");
				break;
			    case SpeechRecognizer.ERROR_CLIENT:
				// Android端末内のエラー(その他)
				Log.e("voice","device error");
				break;
			    case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
				// 権限無し
				Log.e("voice","nothing of right");
				break;
			    case SpeechRecognizer.ERROR_NETWORK:
				// ネットワークエラー(その他)
				Log.e("voice", "network error");
				break;
			    case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
				// ネットワークタイムアウトエラー
				Log.e("voice", "network timeout");
				break;
			    case SpeechRecognizer.ERROR_NO_MATCH:
				// 音声認識結果無し
				Log.e("voice","nothing recognition");
				break;
			    case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
				// RecognitionServiceへ要求出せず
				Log.e("voice","not request");
				break;
			    case SpeechRecognizer.ERROR_SERVER:
				// Server側からエラー通知
				Log.v("voice","from server");
				break;
			    case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
				// 音声入力無し
				Log.v("voice","no input");
				break;
			    default:
			    }

			    //sr.stopListening();
			    
			    /* try{
				Thread.sleep(1000);

			    }
			    catch(Exception e){
			    }*/


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
			    Log.v("voice","ready!");

				// TODO Auto-generated method stub
				
			}

			@Override
			public void onResults(Bundle results) {
				voice_msg = voice_pub.newMessage();

				ArrayList<String> candidates = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
<<<<<<< .mine
				  voice_msg.setTexts(candidates);
				  textView.setText("recognized text candidates:\n");
				  for(int i=0;i<candidates.size();i++){
					  textView.setText(textView.getText()+""+(i+1)+") "+candidates.get(i)+"\n");
				  }
					Log.v("voice","publish ready");
=======
				  voice_msg.setTexts(candidates);
					Log.v("test","publish ready");
>>>>>>> .r3586
				  if(startFlag==1){
					  voice_pub.publish(voice_msg);
						Log.v("voice","publish ok");
				  }
				  busyFlag = 0;
				 
			}

			@Override
			public void onRmsChanged(float rmsdB) {
				// TODO Auto-generated method stub
				
			}
	    	
	    	
	    }
	    
	    
	    
	    }
