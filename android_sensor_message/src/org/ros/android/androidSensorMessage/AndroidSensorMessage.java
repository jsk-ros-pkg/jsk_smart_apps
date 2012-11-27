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

import org.ros.android.MasterChooser;
import org.ros.android.RosActivity;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeMainExecutor;
import android.content.ServiceConnection;

//add libraries
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
//import com.darvds.ribbonmenu.R;
import com.darvds.ribbonmenu.RibbonMenuView;
import com.darvds.ribbonmenu.iRibbonMenuCallback;

public class AndroidSensorMessage extends RosActivity implements
		iRibbonMenuCallback {

	private URI masterUri;
	private Intent intent;
	private AndroidSensorService myService;
	private int[] useFlag;
	private int[] useFlag2;
	private Button menuBtn;
	private Button setBtn;

	private RibbonMenuView rbmView;
	private AndroidSensorMessageView view;
	private SurfaceView sv;

	private LinearLayout linearLayout;

	private int currentSensor = 0;
	private int[] currentSensorNum;

	final private static String[] sensor_name = { "加速度", "温度", "磁界", "光", "近接",
			"ジャイロ", "圧力", "重力", "回転ベクトル" };

	// ボタン
	Button baseBtn;
	Button xBtn;
	Button yBtn;
	Button zBtn;

	// ラベル
	TextView textViewX;
	TextView textViewY;
	TextView textViewZ;
	TextView textViewName;

	ServiceConnection serviceConnection = new ServiceConnection() {

		public void onServiceConnected(ComponentName name, IBinder service) {
			myService = ((AndroidSensorService.MyBinder) service).getService();
			startService(intent);
			useFlag = new int[9];
			useFlag2 = new int[9];
			currentSensorNum = new int[9];

			menuBtn = (Button) findViewById(R.id.menuButton);
			setBtn = (Button) findViewById(R.id.setButton);
			xBtn = (Button) findViewById(R.id.viewXButton);
			yBtn = (Button) findViewById(R.id.viewYButton);
			zBtn = (Button) findViewById(R.id.viewZButton);
			baseBtn = (Button) findViewById(R.id.viewBaseButton);

			textViewX = (TextView) findViewById(R.id.sensorX);
			textViewY = (TextView) findViewById(R.id.sensorY);
			textViewZ = (TextView) findViewById(R.id.sensorZ);
			textViewName = (TextView) findViewById(R.id.sensorName);

			for (int i = 0; i < 9; i++) {
				useFlag[i] = myService.getSensor(i);
				useFlag2[i] = -1;
				currentSensorNum[i] = -1;
			}

			myService.setNode(useFlag2, view, textViewX, textViewY, textViewZ);
			rbmView.setItemIcon(R.drawable.noncheck, R.drawable.check);
			for (int i = 0; i < 8; i++) {
				if (useFlag[i] != 0) {
					currentSensorNum[i] = rbmView.addItem(sensor_name[i], useFlag2[i]);
				}
			}

			menuBtn.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					rbmView.toggleMenu();
				}
			});

			setBtn.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					myService.changeSensor(currentSensor);
					rbmView.setMenuItems(R.menu.ribbon_menu);
					rbmView.clearButtonId();
					useFlag2[currentSensor] *= -1;
					if(useFlag2[currentSensor]==-1){
						setBtn.setText("ON");
					}
					else{
						setBtn.setText("OFF");
					}
					for (int i = 0; i < 8; i++) {
						if (useFlag[i] != 0) {
							rbmView.addItem(sensor_name[i], useFlag2[i]);
						}
					}
				}
			});

			xBtn.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					view.viewChange(1);
				}
			});

			yBtn.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					view.viewChange(2);
				}
			});

			zBtn.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					view.viewChange(3);
				}
			});

			baseBtn.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					view.viewChange(0);
				}
			});

			textViewName.setText(sensor_name[0]);

		}

		@Override
		public void onServiceDisconnected(ComponentName name) {
			myService = null;
		}

	};

	public AndroidSensorMessage() {
		// The RosActivity constructor configures the notification title and
		// ticker
		// messages.
		super("AndroidSensorMessage", "AndroidSensorMessage");

	}

	// @SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		linearLayout = new LinearLayout(this);
		setContentView(R.layout.main);

		// rbmMenu.findItem(0).setVisible(false);
		rbmView = (RibbonMenuView) findViewById(R.id.ribbonMenuView1);
		rbmView.setMenuClickCallback(this);
		rbmView.setMenuItems(R.menu.ribbon_menu);

		sv = (SurfaceView) findViewById(R.id.SV);
		view = new AndroidSensorMessageView(this, sv);

	}

	private LinearLayout.LayoutParams createParam(int w, int h) {
		return new LinearLayout.LayoutParams(w, h);
	}

	@Override
	protected void onResume() {
		super.onResume();
		if (masterUri != null) {

			intent = new Intent(getBaseContext(), AndroidSensorService.class);
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
		stopService(new Intent(this, AndroidSensorService.class));
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
		Log.v("test", "" + itemId);
		for (int i = 0; i < 9; i++) {
			if (itemId == currentSensorNum[i]) {
				currentSensor = i;
				textViewName.setText(sensor_name[i]);
				textViewX.setText("x:");
				textViewY.setText("y:");
				textViewZ.setText("z:");
				if(useFlag2[i]==-1){
					setBtn.setText("ON");
				}
				else{
					setBtn.setText("OFF");
				}
				myService.changeView(currentSensor);
				break;
			}
		}
	}
}
