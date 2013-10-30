package org.ros.android.androidVoiceMessage;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

import org.ros.address.InetAddressFactory;
import org.ros.android.androidVoiceMessage.AndroidVoiceMessageNode;
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
import android.widget.ImageView;


public class AndroidVoiceMessageService extends IntentService {

	SensorManager sensorManager;
	List<Sensor> sensors;
	AndroidVoiceMessageNode androidVoiceMessageNode;
	Intent mintent;
	NodeMainExecutor nodeMainExecutor;
	private String packageName;
	SpeechRecognizer speechRecognizer;

	final IBinder binder = new MyBinder();

	public class MyBinder extends Binder {
		AndroidVoiceMessageService getService() {
			return AndroidVoiceMessageService.this;
		}
	}

	public AndroidVoiceMessageService() {
		super("AndroidVoiceMessageService");
	}

	@Override
	public IBinder onBind(Intent intent) {
		mintent = intent;
		sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		sensors = sensorManager.getSensorList(Sensor.TYPE_ALL);
		return binder;
	}

	public void setNode(SpeechRecognizer speechRecognizer, TextView textView, ImageView imageView, String packageName) {
		showNotification();
		this.speechRecognizer = speechRecognizer;
		this.packageName = packageName;

		// We use this bundle
		Bundle b = mintent.getExtras();

		if (this.androidVoiceMessageNode == null) {
			try {
				URI masterUri = new URI(b.getString("masterUri"));
				String hostLocal = InetAddressFactory.newNonLoopback()
						.getHostAddress();
				nodeMainExecutor = DefaultNodeMainExecutor.newDefault();

				NodeConfiguration nodeConfiguration = NodeConfiguration
						.newPublic(hostLocal, masterUri);
				nodeConfiguration.setMasterUri(masterUri);
				nodeConfiguration.setNodeName("android_voice_message");

				androidVoiceMessageNode = new AndroidVoiceMessageNode(
						sensorManager, speechRecognizer, packageName);
				
				setView(textView,imageView);
				
				nodeMainExecutor.execute(androidVoiceMessageNode,
						nodeConfiguration);
			

			} catch (URISyntaxException e) {
				e.printStackTrace();
			}
		}
	}

	public void setStart() {
		androidVoiceMessageNode.setFlag(1);
	}
	
	public void setRawModeStart(AndroidVoiceMessageView androidVoiceMessageView) {
		androidVoiceMessageNode.publishRawSound(androidVoiceMessageView);
	}
	
	public void setRawModeStop(){
		androidVoiceMessageNode.stopRawSound();
	}
	
	public void setView(TextView textView,ImageView imageView){
		androidVoiceMessageNode.setView(textView,imageView);
	}
	
	public void setMode(int mode) {
		androidVoiceMessageNode.setMode(mode);
	}

	@Override
	protected void onHandleIntent(Intent intent) {
		try {
			Thread.sleep(10000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

	}

	private void showNotification() {

		Intent intent = new Intent(this, AndroidVoiceMessage.class);

		intent.setFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP
				| Intent.FLAG_ACTIVITY_SINGLE_TOP);

		PendingIntent pendingIntent = PendingIntent.getActivity(this, 0,
				intent, 0);

	}

	@Override
	public void onDestroy() {
		nodeMainExecutor.shutdownNodeMain(androidVoiceMessageNode);
	}
}
