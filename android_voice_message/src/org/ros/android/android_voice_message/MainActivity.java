/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.android_voice_message;

import android.os.Bundle;
import android.os.IBinder;

import org.ros.android.android_voice_message.*;
import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.rosjava_tutorial_pubsub.Talker;

//add
import android.app.AlertDialog;
import android.content.ComponentName;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.ServiceConnection;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.TextView;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import org.ros.node.topic.Publisher;
import java.util.List;
import jsk_gui_msgs.VoiceMessage;


/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MainActivity extends RosActivity {

	  private URI masterUri;
    private RosTextView<std_msgs.String> rosTextView;
    private Talker talker;
     private String package_name;

    //add new member variables
    SpeechRecognizer sr;
    private SensorManager mSensorManager;
    private int startFlag = 0;
	private Intent intent;  
	private AndroidVoiceService myService;
    
   ServiceConnection serviceConnection = new ServiceConnection(){


	    
	    @Override
	    public void onServiceConnected(ComponentName name, IBinder service) {
	      myService = ((AndroidVoiceService.MyBinder)service).getService();
	      startService(intent);
	      myService.setNode(sr,package_name);

	    }

	    @Override
	    public void onServiceDisconnected(ComponentName name){
	    	myService = null;
	    }
	    
    };

 



  public MainActivity() {
    super("AndroidVoiceMessage", "AndroidVoiceMessage");
  }

 // @SuppressWarnings("unchecked")
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);

    //add
	mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
	sr = SpeechRecognizer.createSpeechRecognizer(getApplicationContext());
   package_name = getPackageName();
   	


  }
  
  @Override
	public void onResume(){
	super.onResume();
	if (masterUri != null){
		
		intent = new Intent(getBaseContext(),AndroidVoiceService.class);
	    intent.putExtra("masterUri", masterUri.toString());
	    bindService(intent,serviceConnection,Context.BIND_AUTO_CREATE);
	    

	    
	      /*intent = new Intent(this, AndroidSensorService.class);
	      intent.putExtra("masterUri", masterUri.toString());
	      startService(intent);*/
	}
	
   }
  
  
  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    if (requestCode == 0 && resultCode == RESULT_OK) {
      try {
        masterUri = new URI(data.getStringExtra("ROS_MASTER_URI"));
      } catch (URISyntaxException e) {
        throw new RuntimeException(e);
      }
    }
  }

  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
  	menu.add(0, 0, 0, R.string.app_about);
  	menu.add(0, 1, 1, R.string.str_exit);
  	return super.onCreateOptionsMenu(menu);
  }

  @Override
  public boolean onOptionsItemSelected(MenuItem item) {
  	super.onOptionsItemSelected(item);
  	switch(item.getItemId())
  	{
  	case 0:
  	openOptionsDialog();
  	break;
  	case 1:
  	exitOptionsDialog();
  	break;
  	}
  	return true;
  }

  private void openOptionsDialog(){
  	new AlertDialog.Builder(this).setTitle(R.string.app_about).setMessage(R.string.app_about_message).setPositiveButton(R.string.str_ok,new DialogInterface.OnClickListener(){public void onClick(DialogInterface dialoginterface, int i){}}).show();
  }

  private void exitOptionsDialog()
  {
  	stopService(new Intent(this, AndroidVoiceService.class));
  	finish();
  }
  
  
  
  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
  
  }
}
