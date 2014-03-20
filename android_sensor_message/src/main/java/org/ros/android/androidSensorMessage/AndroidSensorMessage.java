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

package org.ros.android.androidSensorMessage;

import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
//add
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;

import org.ros.android.MasterChooser;
import org.ros.android.RosActivity;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeMainExecutor;
import android.content.ServiceConnection;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;

import android.app.AlertDialog;
import android.content.ClipData.Item;
import android.content.DialogInterface;
import android.content.Intent;
import android.widget.LinearLayout;
import android.widget.Button;
import android.util.Log;
import android.content.ComponentName;
import android.os.IBinder;
import android.content.Context;
import android.view.View;
import android.widget.TextView;
import android.widget.CheckBox;

//add ribbon_menu
import com.darvds.ribbonmenu.RibbonMenuView;
import com.darvds.ribbonmenu.iRibbonMenuCallback;

public class AndroidSensorMessage extends RosActivity implements
		iRibbonMenuCallback, OnTouchListener {

	private URI masterUri;
	private Intent intent;
	private AndroidSensorMessageService androidSensorMessageService;
	private int currentSensor = 0;
	private int multiTouchAction = 0;
	private float previousY;
	private float previousDistanceY;
	private int[] availableSensor;
	private int[] publishedSensor;
	private int[] selectedSensor;

	private Button menuButton;
	private Button setButton;

	private RibbonMenuView ribbonMenuView;
	private AndroidSensorMessageView androidSensorMessageView;
	private SurfaceView surfaceView;

	private LinearLayout linearLayout;

	// Button
	Button baseButton;
	Button xButton;
	Button yButton;
	Button zButton;

	// Label
	TextView textViewX;
	TextView textViewY;
	TextView textViewZ;
	TextView textViewSensorName;

	ServiceConnection serviceConnection = new ServiceConnection() {

		public void onServiceConnected(ComponentName name, IBinder service) {
			androidSensorMessageService = ((AndroidSensorMessageService.MyBinder) service)
					.getService();
			startService(intent);
			availableSensor = new int[13];
			publishedSensor = new int[13];
			selectedSensor = new int[13];

			menuButton = (Button) findViewById(R.id.menuButton);
			setButton = (Button) findViewById(R.id.setButton);
			xButton = (Button) findViewById(R.id.viewXButton);
			yButton = (Button) findViewById(R.id.viewYButton);
			zButton = (Button) findViewById(R.id.viewZButton);
			baseButton = (Button) findViewById(R.id.viewBaseButton);

			textViewX = (TextView) findViewById(R.id.sensorX);
			textViewY = (TextView) findViewById(R.id.sensorY);
			textViewZ = (TextView) findViewById(R.id.sensorZ);
			textViewSensorName = (TextView) findViewById(R.id.sensorName);

			for (int i = 0; i < 13; i++) {
				availableSensor[i] = androidSensorMessageService.getSensor(i);
				publishedSensor[i] = -1;
				selectedSensor[i] = -1;
			}

			androidSensorMessageService.setNode(publishedSensor,
					androidSensorMessageView, textViewX, textViewY, textViewZ);
                        //			ribbonMenuView.setItemIcon(R.drawable.noncheck, R.drawable.check);
			for (int i = 0; i < 13; i++) {
				if (availableSensor[i] != -1) {
                                    /*					selectedSensor[i] = ribbonMenuView.addItem(
							androidSensorMessageService.getSensorName(i),
							publishedSensor[i]);*/
				}
			}

			menuButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					ribbonMenuView.toggleMenu();
				}
			});

			setButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					androidSensorMessageService.changeSensor(currentSensor);
					ribbonMenuView.setMenuItems(R.menu.ribbon_menu);
                                        //					ribbonMenuView.clearButtonId();
					publishedSensor[currentSensor] *= -1;
					if (publishedSensor[currentSensor] == -1) {
						setButton.setText("ON");
					} else {
						setButton.setText("OFF");
					}
					for (int i = 0; i < 13; i++) {
						if (availableSensor[i] != -1) {
                                                    /*							ribbonMenuView.addItem(androidSensorMessageService
                                                                                                        .getSensorName(i), publishedSensor[i]);*/
						}
					}
				}
			});

			xButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					androidSensorMessageView.viewChange(1);
				}
			});

			yButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					androidSensorMessageView.viewChange(2);
				}
			});

			zButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					androidSensorMessageView.viewChange(3);
				}
			});

			baseButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					androidSensorMessageView.viewChange(0);
				}
			});

			textViewSensorName.setText(androidSensorMessageService
					.getSensorName(0));

		}

		@Override
		public void onServiceDisconnected(ComponentName name) {
			androidSensorMessageService = null;
		}

	};

	public AndroidSensorMessage() {
		super("AndroidSensorMessage", "AndroidSensorMessage");

	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		linearLayout = new LinearLayout(this);
		setContentView(R.layout.main);

		ribbonMenuView = (RibbonMenuView) findViewById(R.id.ribbonMenuView1);
		ribbonMenuView.setMenuClickCallback(this);
		ribbonMenuView.setMenuItems(R.menu.ribbon_menu);

		surfaceView = (SurfaceView) findViewById(R.id.SV);
		androidSensorMessageView = new AndroidSensorMessageView(this,
				surfaceView);
		surfaceView.setOnTouchListener(this);
	}

	private LinearLayout.LayoutParams createParam(int width, int height) {
		return new LinearLayout.LayoutParams(width, height);
	}

	@Override
	protected void onResume() {
		super.onResume();
		if (masterUri != null) {

			intent = new Intent(getBaseContext(),
					AndroidSensorMessageService.class);
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
		menu.add(0, 0, 0, R.string.app_about);
		menu.add(0, 1, 1, R.string.str_exit);
		return super.onCreateOptionsMenu(menu);
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		super.onOptionsItemSelected(item);
		switch (item.getItemId()) {
		case 0:
			openOptionsDialog();
			break;
		case 1:
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
		stopService(new Intent(this, AndroidSensorMessageService.class));
		finish();
	}

	@Override
	protected void onPause() {
		super.onPause();
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {

	}

	@Override
	public void RibbonMenuItemClick(int itemId) {
		for (int i = 0; i < 13; i++) {
			if (itemId == selectedSensor[i]) {
				currentSensor = i;
				textViewSensorName.setText(androidSensorMessageService
						.getSensorName(i));
				textViewX.setText("x:");
				textViewY.setText("y:");
				textViewZ.setText("z:");
				if (publishedSensor[i] == -1) {
					setButton.setText("ON");
				} else {
					setButton.setText("OFF");
				}
				androidSensorMessageService.changeView(currentSensor);
				break;
			}
		}
	}

	public boolean onTouch(View v, MotionEvent event) {
		if (event.getPointerCount() == 1) {
			if (multiTouchAction > 0) {
				if (event.getAction() == MotionEvent.ACTION_UP) {
					multiTouchAction--;
				}
			} else {
				if (event.getAction() == MotionEvent.ACTION_MOVE) {
					androidSensorMessageView.updateOriginalY(event.getY()
							- previousY);
				}
				previousY = event.getY();
			}
		} else {
			int pointerId1 = event.getPointerId(0);
			int pointerId2 = event.getPointerId(1);

			int pointerIndex1 = event.findPointerIndex(pointerId1);
			int pointerIndex2 = event.findPointerIndex(pointerId2);

			float distanceY = event.getY(pointerIndex2) - event.getY(pointerIndex1);
			distanceY = (float)Math.sqrt(distanceY * distanceY);
			if (event.getAction() == MotionEvent.ACTION_MOVE) {
				androidSensorMessageView.updateScale(distanceY - previousDistanceY);
			}
			Log.v("distance","pre:"+previousDistanceY+" now:"+distanceY);
			previousDistanceY = distanceY;
			
			multiTouchAction = 1;

		}

		return true;
	}

}
