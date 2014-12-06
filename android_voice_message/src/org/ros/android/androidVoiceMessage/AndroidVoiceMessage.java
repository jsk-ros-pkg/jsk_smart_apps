package org.ros.android.androidVoiceMessage;

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

import android.os.Bundle;
import android.os.IBinder;

import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

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
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.ImageView;
import android.widget.TextView;


import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import org.ros.node.topic.Publisher;

import java.util.List;
import speech_recognition_msgs.SpeechRecognitionCandidates;


public class AndroidVoiceMessage extends RosActivity {

	private URI masterUri;
	private TextView textView;
	private ImageView imageView;
	private Button button;
	private String packageName;
	private int mode;
	private boolean eternal = false;

	private SensorManager sensorManager;
	private Intent intent;
	private AndroidVoiceMessageService androidVoiceMessageService;
	private AndroidVoiceMessageView androidVoiceMessageView;

	ServiceConnection serviceConnection = new ServiceConnection() {

		@Override
		public void onServiceConnected(ComponentName name, IBinder service) {
			androidVoiceMessageService = ((AndroidVoiceMessageService.MyBinder) service)
					.getService();
			startService(intent);
			imageView = (ImageView)findViewById(R.id.ImageView);
			androidVoiceMessageService.setNode(AndroidVoiceMessage.this, textView,imageView, packageName);
		}

		@Override
		public void onServiceDisconnected(ComponentName name) {
			androidVoiceMessageService = null;
		}

	};

	public AndroidVoiceMessage() {
		super("AndroidVoiceMessage", "AndroidVoiceMessage");
	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		
		setContentView(R.layout.main);
		mode = 1;
		textView = (TextView) findViewById(R.id.text);
		imageView = (ImageView) findViewById(R.id.ImageView);
		imageView.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				androidVoiceMessageService.setStart();
			}
		});
		button = (Button)findViewById(R.id.button);
		button.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if(eternal) {
					eternal = false;
					androidVoiceMessageService.setMode(1);
					button.setText("Once");	
				}
				else {
					eternal = true;
					androidVoiceMessageService.setMode(-1);
					button.setText("Eternal");
				}
			}
		});

		// add
		sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		packageName = getPackageName();
		androidVoiceMessageView = new AndroidVoiceMessageView(this);
	}

	@Override
	public void onResume() {
		super.onResume();
		if (masterUri != null) {

			intent = new Intent(getBaseContext(),
					AndroidVoiceMessageService.class);
			intent.putExtra("masterUri", masterUri.toString());
			bindService(intent, serviceConnection, Context.BIND_AUTO_CREATE);

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
		menu.add(0,0,0,R.string.change_mode);
		menu.add(0, 1, 1, R.string.app_about);
		menu.add(0, 2, 1, R.string.str_exit);
		return super.onCreateOptionsMenu(menu);
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		super.onOptionsItemSelected(item);
		switch (item.getItemId()) {
		case 0:
			mode *= -1;
			if(mode == 1){
				setContentView(R.layout.main);
				eternal = false;
				textView = (TextView) findViewById(R.id.text);
				imageView = (ImageView) findViewById(R.id.ImageView);
				imageView.setOnClickListener(new View.OnClickListener() {
					@Override
					public void onClick(View v) {
						androidVoiceMessageService.setStart();
					}
				});

				button = (Button)findViewById(R.id.button);
				button.setOnClickListener(new View.OnClickListener() {
					@Override
					public void onClick(View v) {
						if(eternal) {
							eternal = false;
							androidVoiceMessageService.setMode(1);
							button.setText("Once");	
						}
						else {
							eternal = true;
							androidVoiceMessageService.setMode(-1);
							button.setText("Eternal");
						}
					}
				});
				
				androidVoiceMessageService.setView(textView,imageView);
				androidVoiceMessageService.setRawModeStop();
			}
			else{
				androidVoiceMessageService.setMode(1);
				setContentView(R.layout.main2);
				SurfaceView surfaceView = (SurfaceView) findViewById(R.id.SV);
				androidVoiceMessageView.setView(surfaceView);
				androidVoiceMessageService.setRawModeStart(androidVoiceMessageView);

			}
			break;
		case 1:
			openOptionsDialog();
			break;
		case 2:
			exitOptionsDialog();
			break;
		}
		return true;
	}

	private void openOptionsDialog() {
		new AlertDialog.Builder(this)
				.setTitle(R.string.app_about)
				.setMessage(R.string.app_about_message)
				.setPositiveButton(R.string.str_ok,
						new DialogInterface.OnClickListener() {
							public void onClick(
									DialogInterface dialoginterface, int i) {
							}
						}).show();
	}

	private void exitOptionsDialog() {
		stopService(new Intent(this, AndroidVoiceMessageService.class));
		finish();
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {

	}
}
