package org.ros.android.jskAndroidGui;

import java.util.ArrayList;

import jsk_gui_msgs.Action;
import jsk_gui_msgs.QueryRequest;
import jsk_gui_msgs.QueryResponse;

import org.ros.address.InetAddressFactory;
import com.github.rosjava.android_apps.application_management.RosAppActivity;
import org.ros.android.view.VirtualJoystickView;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceServer;

import android.content.res.Configuration;
import android.view.KeyEvent;
import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.ContextMenu;
import android.view.ContextMenu.ContextMenuInfo;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ViewFlipper;

//import java.util.*;

/**
 * @author chen@jsk.t.u-tokyo.ac.jp (Haseru Azuma)
 */

public class JskAndroidGui extends RosAppActivity {

	public JskAndroidGui() {
		super("jsk android gui", "jsk android gui");
	}

	private String robotAppName, cameraTopic, cameraInfoTopic,
			moveItWrt = "local";
	private SensorImageViewInfo cameraView;
	private VirtualJoystickView joystickView;
	private TextView postv, rottv;
	private EditText urltv;
	private JskAndroidGuiNode jskAndroidGuiNode;

	private ParameterTree params;
	private Node public_node;
	private ServiceServer<QueryRequest, QueryResponse> server;
        private Button yes_button, no_button, x_minus_button, x_plus_button, menu_button,
			y_minus_button, y_plus_button, z_minus_button, z_plus_button,
			start_button, stop_button, movearm_button, url_button,
			circle_button, linear_button, cancel_button;
	private ImageButton tweet_button;
	private RadioGroup radioGroup;
	private RadioGroup radioGroup_moveIt;
	private RadioGroup radioGroup_wrt;
	private boolean moveit_pos;
	private SeekBar posbar, rotbar;
	private int posval, rotval;
	private Spinner spots_spinner, tasks_spinner, image_spinner,
			points_spinner;
	private ViewFlipper viewFlipper, webViewFlipper;
	private WebView webView;
	private BoundingBoxView boundingBoxView;
	private FrameLayout frameLayout;

	private ArrayList<String> spots_list = new ArrayList(),
			tasks_list = new ArrayList(), image_list = new ArrayList(),
			camera_info_list = new ArrayList(), points_list = new ArrayList();
	private String defaultImage = "/openni/rgb/image_color",
			defaultCameraInfo = "/openni/rgb/camera_info",
			defaultPoints = "/openni/depth_registered/points_throttle";
	// will be renamed when parameter updated

	private Object[] found_task, query_input;
	private boolean isDrawLine = false, isAdapterSet_spots = false,
			isAdapterSet_tasks = false, isNotParamInit = true,
			isAdapterSet_camera = false, isAdapterSet_points = false,
			isParamSet = false, LongTouchFlag = true;

	private Handler mHandler;

	static final int CONTEXT_MENU1_ID = 0, CONTEXT_MENU2_ID = 1,
			CONTEXT_MENU3_ID = 2, CONTEXT_MENU4_ID = 4;

	@SuppressLint("SetJavaScriptEnabled")
	@Override
	public void onCreate(Bundle savedInstanceState) {

		setDefaultRobotName("pr1040");
		setDefaultAppName("jsk_android_gui/jsk_android_gui");
		setDashboardResource(R.id.top_bar);
		setMainWindowResource(R.layout.main);
		super.onCreate(savedInstanceState);

		jskAndroidGuiNode = new JskAndroidGuiNode();

		viewFlipper = (ViewFlipper) findViewById(R.id.flipper);
		webViewFlipper = (ViewFlipper) findViewById(R.id.webviewflipper);
		webView = (WebView) findViewById(R.id.WebView);
		webView.setWebViewClient(new WebViewClient() {
			@Override
			public boolean shouldOverrideUrlLoading(WebView view, String url) {
				return false;
			}
		});
		webView.getSettings().setJavaScriptEnabled(true);
		webView.loadUrl("http://www.google.co.jp/");
		urltv = (EditText) findViewById(R.id.webURL);
		urltv.setText("http://www.google.co.jp/");
		url_button = (Button) findViewById(R.id.URLbutton);

		tweet_button = (ImageButton) findViewById(R.id.tweet);
		menu_button = (Button) findViewById(R.id.menu_button);
		yes_button = (Button) findViewById(R.id.resultyes);
		no_button = (Button) findViewById(R.id.resultno);
		x_minus_button = (Button) findViewById(R.id.move_x_minus);
		x_plus_button = (Button) findViewById(R.id.move_x_plus);
		y_minus_button = (Button) findViewById(R.id.move_y_minus);
		y_plus_button = (Button) findViewById(R.id.move_y_plus);
		z_minus_button = (Button) findViewById(R.id.move_z_minus);
		z_plus_button = (Button) findViewById(R.id.move_z_plus);
		start_button = (Button) findViewById(R.id.manipulation_start);
		stop_button = (Button) findViewById(R.id.manipulation_stop);
		movearm_button = (Button) findViewById(R.id.movearm);
		circle_button = (Button) findViewById(R.id.circular_manipulation);
		linear_button = (Button) findViewById(R.id.linear_manipulation);
		cancel_button = (Button) findViewById(R.id.cancel_manipulation);

		posbar = (SeekBar) findViewById(R.id.seekbarpos);
		rotbar = (SeekBar) findViewById(R.id.seekbarrot);
		postv = (TextView) findViewById(R.id.posvaluetv);
		postv.setText("pos: " + posbar.getProgress() + " mm");
		posval = posbar.getProgress();
		rottv = (TextView) findViewById(R.id.rotvaluetv);
		rottv.setText("rot " + rotbar.getProgress() + " degree");
		rotval = rotbar.getProgress();

		posbar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				postv.setText("pos: " + progress + " mm");
				posval = progress;
			}

			public void onStartTrackingTouch(SeekBar seekBar) {

			}

			public void onStopTrackingTouch(SeekBar seekBar) {

			}
		});

		rotbar.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
			public void onProgressChanged(SeekBar seekBar, int progress,
					boolean fromUser) {
				rottv.setText("rot: " + progress + " degree");
				rotval = progress;
			}

			public void onStartTrackingTouch(SeekBar seekBar) {

			}

			public void onStopTrackingTouch(SeekBar seekBar) {

			}
		});

		radioGroup = (RadioGroup) findViewById(R.id.radiogroup);
		radioGroup.check(R.id.radiobutton_L);
		radioGroup
				.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
					public void onCheckedChanged(RadioGroup group, int checkedId) {
						RadioButton radioButton = (RadioButton) findViewById(checkedId);
						if (radioButton.getText().equals("left")) {
							cameraView.SetRobotArm(Action.LARMID);
							Toast.makeText(JskAndroidGui.this,
									"robot arm set to :larm",
									Toast.LENGTH_SHORT).show();
							Log.i("JskAndroidGui:ItemSeleted",
									"Set arm to :larm");
						} else {
							cameraView.SetRobotArm(Action.RARMID);
							Toast.makeText(JskAndroidGui.this,
									"robot arm set to :rarm",
									Toast.LENGTH_SHORT).show();
							Log.i("JskAndroidGui:ItemSeleted",
									"Set arm to :rarm");
						}
					}
				});
		radioGroup_moveIt = (RadioGroup) findViewById(R.id.radiogroup_moveit);
		radioGroup_moveIt.check(R.id.radiobutton_pos);
		moveit_pos = true;
		radioGroup_moveIt
				.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
					public void onCheckedChanged(RadioGroup group, int checkedId) {
						RadioButton radioButton = (RadioButton) findViewById(checkedId);
						if (radioButton.getText().equals("pos")) {
							moveit_pos = true;
							x_minus_button.setText("x-");
							x_plus_button.setText("x+");
							y_minus_button.setText("y-");
							y_plus_button.setText("y+");
							z_minus_button.setText("z-");
							z_plus_button.setText("z+");

						} else {
							moveit_pos = false;
							x_minus_button.setText("roll-");
							x_plus_button.setText("roll+");
							y_minus_button.setText("pitch-");
							y_plus_button.setText("pitch+");
							z_minus_button.setText("yaw-");
							z_plus_button.setText("yaw+");
						}
					}
				});

		radioGroup_wrt = (RadioGroup) findViewById(R.id.radiogroup_wrt);
		radioGroup_wrt.check(R.id.radiobutton_local);
		radioGroup_wrt
				.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
					public void onCheckedChanged(RadioGroup group, int checkedId) {
						RadioButton radioButton = (RadioButton) findViewById(checkedId);
						if (radioButton.getText().equals("local")) {
							moveItWrt = "local";
						} else {
							moveItWrt = "world";
						}
					}
				});

		spots_spinner = (Spinner) findViewById(R.id.spinner_spots);
		ArrayAdapter<String> adapter_spots = new ArrayAdapter<String>(this,
				R.layout.list);
		spots_spinner.setAdapter(adapter_spots);
		adapter_spots
				.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		spots_spinner.setPromptId(R.string.SpinnerPrompt_spots);

		tasks_spinner = (Spinner) findViewById(R.id.spinner_tasks);
		ArrayAdapter<String> adapter_tasks = new ArrayAdapter<String>(this,
				R.layout.list);
		tasks_spinner.setAdapter(adapter_tasks);
		adapter_tasks
				.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		tasks_spinner.setPromptId(R.string.SpinnerPrompt_tasks);

		image_spinner = (Spinner) findViewById(R.id.spinner_image);

		ArrayAdapter<String> adapter_image = new ArrayAdapter<String>(this,
				R.layout.list);
		image_spinner.setAdapter(adapter_image);
		adapter_image
				.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

		points_spinner = (Spinner) findViewById(R.id.spinner_points);

		ArrayAdapter<String> adapter_points = new ArrayAdapter<String>(this,
				R.layout.list);
		points_spinner.setAdapter(adapter_points);
		adapter_points
				.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

		if (getIntent().hasExtra("camera_topic")) {
			cameraTopic = getIntent().getStringExtra("camera_topic");
		} else {
			cameraTopic = "/tablet/marked/image_rect_color/compressed_throttle";
		}
		if (getIntent().hasExtra("camera_info_topic")) {
			cameraInfoTopic = getIntent().getStringExtra("camera_info_topic");
		} else {
			cameraInfoTopic = "/tablet/marked/camera_info";
		}
		joystickView = (VirtualJoystickView) findViewById(R.id.joystick);
		joystickView.setTopicName("android/cmd_vel");
		frameLayout = (FrameLayout) findViewById(R.id.image);
		cameraView = new SensorImageViewInfo(this);
		cameraView.setClickable(true);
		cameraView.SetRobotArm(Action.LARMID);
		cameraView.setCameraTopic(cameraTopic);
		cameraView.setCameraInfoTopic(cameraInfoTopic);
		boundingBoxView = new BoundingBoxView(this);
		cameraView.setView(boundingBoxView);
		frameLayout.addView(boundingBoxView);
		frameLayout.addView(cameraView);

		mHandler = new Handler();
		// ImageView ivInContext = (ImageView) findViewById(R.id.image);
		// ivInContext.setFocusable(true);
		// ivInContext.setClickable(true);
		boundingBoxView.setFocusable(true);
		boundingBoxView.setClickable(true);
		registerForContextMenu(boundingBoxView);
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {

		super.init(nodeMainExecutor);

		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
				InetAddressFactory.newNonLoopback().getHostAddress(),
				getMasterUri());

		NameResolver appNamespace = getAppNameSpace();
		nodeMainExecutor.execute(cameraView,
				nodeConfiguration.setNodeName("android/camera_view"));

		// cameraView.start(node, appNamespace.resolve(cameraTopic).toString());
		cameraView.post(new Runnable() {
			@Override
			public void run() {
				Log.i("JskAndroidGui:debug", "cameraView run");
				cameraView.setSelected(true);
			}
		});
		nodeMainExecutor.execute(jskAndroidGuiNode,
				nodeConfiguration.setNodeName("android/jsk_android_gui"));
		nodeMainExecutor.execute(joystickView,
				nodeConfiguration.setNodeName("android/joystick"));

		waitForSetupNode();
	}

	private void waitForSetupNode() {
		while (!jskAndroidGuiNode.setupEnd) {

			try {
				Thread.sleep(1000);
			} catch (Exception e) {

			}
		}

		setListener();
		setupListener();
	}

	private void setListener() {
		// Tweet function
		tweet_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				LayoutInflater inflater = LayoutInflater
						.from(JskAndroidGui.this);
				final View view = inflater.inflate(R.layout.dialog, null);

				final EditText editText = (EditText) view
						.findViewById(R.id.editText1);
				mHandler.post(new Runnable() {
					public void run() {
						Log.i("JskAndroidGui:debug", "dialog handler");
						new AlertDialog.Builder(JskAndroidGui.this)
								.setTitle("tweet with image")
								.setView(view)
								.setPositiveButton("tweet",
										new DialogInterface.OnClickListener() {
											@Override
											public void onClick(
													DialogInterface dialog,
													int which) {

												jskAndroidGuiNode
														.tweetTask(editText
																.getText()
																.toString());
												Toast.makeText(
														JskAndroidGui.this,
														"tasks: Tweet",
														Toast.LENGTH_SHORT)
														.show();
												Log.i("JskAndroidGui:ButtonClicked",
														"Sending Tweet");
											}
										}).show();
					}
				});
			}
		});
	}

    @Override
        public void openOptionsMenu() {

        Configuration config = getResources().getConfiguration();

        if((config.screenLayout & Configuration.SCREENLAYOUT_SIZE_MASK) 
           > Configuration.SCREENLAYOUT_SIZE_LARGE) {

            int originalScreenLayout = config.screenLayout;
            config.screenLayout = Configuration.SCREENLAYOUT_SIZE_LARGE;
            super.openOptionsMenu();
            config.screenLayout = originalScreenLayout;

        } else {
            super.openOptionsMenu();
        }
    }

	protected void setupListener() {
            menu_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
                            openOptionsMenu();
			}
		});

		yes_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				jskAndroidGuiNode.selectTask("ResultYes");
				Toast.makeText(JskAndroidGui.this, "tasks: ResultYes",
						Toast.LENGTH_SHORT).show();
				Log.i("JskAndroidGui:ButtonClicked", "Sending ResultYes");
			}
		});

		no_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				jskAndroidGuiNode.selectTask("ResultNo");
				Toast.makeText(JskAndroidGui.this, "tasks: ResultNo",
						Toast.LENGTH_SHORT).show();
				Log.i("JskAndroidGui:ButtonClicked", "Sending ResultNo");
			}
		});

		x_minus_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				if (moveit_pos)
					cameraView.SendMoveItMsg("x-", moveItWrt, posval);
				else
					cameraView.SendMoveItMsg("roll-", moveItWrt, rotval);
			}
		});
		x_plus_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				if (moveit_pos)
					cameraView.SendMoveItMsg("x+", moveItWrt, posval);
				else
					cameraView.SendMoveItMsg("roll+", moveItWrt, rotval);
			}
		});
		y_minus_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				if (moveit_pos)
					cameraView.SendMoveItMsg("y-", moveItWrt, posval);
				else
					cameraView.SendMoveItMsg("pitch-", moveItWrt, rotval);
			}
		});
		y_plus_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				if (moveit_pos)
					cameraView.SendMoveItMsg("y+", moveItWrt, posval);
				else
					cameraView.SendMoveItMsg("pitch+", moveItWrt, rotval);
			}
		});
		z_minus_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				if (moveit_pos)
					cameraView.SendMoveItMsg("z-", moveItWrt, posval);
				else
					cameraView.SendMoveItMsg("yaw-", moveItWrt, rotval);
			}
		});
		z_plus_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				if (moveit_pos)
					cameraView.SendMoveItMsg("z+", moveItWrt, posval);
				else
					cameraView.SendMoveItMsg("yaw+", moveItWrt, rotval);
			}
		});

		start_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				cameraView.SendMoveItMsg("start", moveItWrt, 0);
			}
		});
		stop_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				cameraView.SendMoveItMsg("stop", moveItWrt, 0);
			}
		});
		movearm_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				cameraView.SendMoveItMsg("movearm", moveItWrt, 0);
			}
		});

		circle_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				cameraView.SetCircularManipulation();
			}
		});

		linear_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				cameraView.SetLinearManipulation();
				// cameraView.hoge();
			}
		});

		cancel_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				cameraView.CancelManipulation();
			}
		});

		url_button.setOnClickListener(new OnClickListener() {
			public void onClick(View viw) {
				webView.loadUrl(urltv.getText().toString());
			}
		});

		try {
			jskAndroidGuiNode.getSpotsParam();
			spots_list = jskAndroidGuiNode.getSpotsList();
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
			// Toast.makeText(JskAndroidGui.this, "No Param Found: " +
			// ex.getMessage(), Toast.LENGTH_SHORT) .show();
		}

		spots_spinner.setOnItemSelectedListener(new OnItemSelectedListener() {
			public void onItemSelected(AdapterView parent, View viw, int arg2,
					long arg3) {
                            Spinner spinner = (Spinner) parent;
                            int count = spinner.getChildCount();
                            for (int i = 0; i < count; i++) {
                                View v = spinner.getChildAt(i);
                                if(v instanceof TextView) {
                                    ((TextView)v).setTextColor(Color.WHITE);
                                }
                            }
				if (isAdapterSet_spots) {
					String item = (String) spinner.getSelectedItem();
					jskAndroidGuiNode.spotsSpinnerTask(item);
					Toast.makeText(JskAndroidGui.this,
							"spots: MoveToSpot " + item, Toast.LENGTH_SHORT)
							.show();
					Log.i("JskAndroidGui:ItemSeleted",
							"Sending MoveToSpot messgae");
				} else {
					isAdapterSet_spots = true;
					Log.i("JskAndroidGui:", "spots adapter not set");
				}
			}

			public void onNothingSelected(AdapterView parent) {
				Toast.makeText(JskAndroidGui.this, "Updating Param",
						Toast.LENGTH_SHORT).show();
				GetParamAndSetSpinner();
			}
		}); // for tasks
		try {
			jskAndroidGuiNode.getTasksParam();
			tasks_list = jskAndroidGuiNode.getTasksList();
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
			// Toast.makeText(JskAndroidGui.this, "No Param Found: " +
			// ex.getMessage(), Toast.LENGTH_SHORT).show();
		}
		tasks_spinner.setOnItemSelectedListener(new OnItemSelectedListener() {
			public void onItemSelected(AdapterView parent, View viw, int arg2,
					long arg3) {
                            Spinner spinner = (Spinner) parent;
                            int count = spinner.getChildCount();
                            for (int i = 0; i < count; i++) {
                                View v = spinner.getChildAt(i);
                                if(v instanceof TextView) {
                                    ((TextView)v).setTextColor(Color.WHITE);
                                }
                            }
				if (isAdapterSet_tasks) {
					String item = (String) spinner.getSelectedItem();
					jskAndroidGuiNode.tasksSpinnerTask(item);
					Toast.makeText(JskAndroidGui.this,
							"tasks: StartDemo " + item, Toast.LENGTH_SHORT)
							.show();
					Log.i("JskAndroidGui:ItemSeleted",
							"Sending StartDemo messgae");
				} else {
					isAdapterSet_tasks = true;
					Log.i("JskAndroidGui:", "tasks adapter not set");
				}
			}

			public void onNothingSelected(AdapterView parent) {
				// Toast.makeText(JskAndroidGui.this, "Updating Param",
				// Toast.LENGTH_SHORT).show();
				GetParamAndSetSpinner();
			}
		});
		// for camera
		try {
			jskAndroidGuiNode.getCameraParam();
			image_list = jskAndroidGuiNode.getImageList();
			camera_info_list = jskAndroidGuiNode.getCameraInfoList();
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
			// Toast.makeText(JskAndroidGui.this, "No Param Found: " +
			// ex.getMessage(), Toast.LENGTH_SHORT).show();
		}
		image_spinner.setOnItemSelectedListener(new OnItemSelectedListener() {
			public void onItemSelected(AdapterView parent, View viw, int arg2,
					long arg3) {
                            Spinner spinner = (Spinner) parent;
                            int count = spinner.getChildCount();
                            for (int i = 0; i < count; i++) {
                                View v = spinner.getChildAt(i);
                                if(v instanceof TextView) {
                                    ((TextView)v).setTextColor(Color.WHITE);
                                }
                            }
				if (isAdapterSet_camera) {
					defaultImage = (String) spinner.getSelectedItem();
					// assume that the first element is "cameras", so -1
					defaultCameraInfo = camera_info_list.get(arg2 - 1);
					String str = "((:image " + defaultImage
							+ ") (:camera_info " + defaultCameraInfo
							+ ") (:points " + defaultPoints + "))";
					cameraView.PubSwitchSensor(str);
					// Toast.makeText(JskAndroidGui.this, "SwitchSensor: " +
					// str, Toast.LENGTH_SHORT).show();
					Log.i("JskAndroidGui:ItemSeleted", "Sending switch messgae");
				} else {
					isAdapterSet_camera = true;
					Log.i("JskAndroidGui:", "camera adapter not set");
				}
			}

			public void onNothingSelected(AdapterView parent) {
				Toast.makeText(JskAndroidGui.this, "Updating Param",
						Toast.LENGTH_SHORT).show();
				GetParamAndSetSpinner();
			}
		});

		// for points
		try {
			jskAndroidGuiNode.getPointsParam();
			points_list = jskAndroidGuiNode.getPointsList();
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
			// Toast.makeText(JskAndroidGui.this, "No Param Found: " +
			// ex.getMessage(), Toast.LENGTH_SHORT).show();
		}
		points_spinner.setOnItemSelectedListener(new OnItemSelectedListener() {
			public void onItemSelected(AdapterView parent, View viw, int arg2,
					long arg3) {
                            Spinner spinner = (Spinner) parent;
                            int count = spinner.getChildCount();
                            for (int i = 0; i < count; i++) {
                                View v = spinner.getChildAt(i);
                                if(v instanceof TextView) {
                                    ((TextView)v).setTextColor(Color.WHITE);
                                }
                            }
				if (isAdapterSet_points) {
					defaultPoints = (String) spinner.getSelectedItem();
					String str = "((:image " + defaultImage
							+ ") (:camera_info " + defaultCameraInfo
							+ ") (:points " + defaultPoints + "))";
					cameraView.PubSwitchSensor(str);
					Toast.makeText(JskAndroidGui.this, "SwitchSensor: " + str,
							Toast.LENGTH_SHORT).show();
					Log.i("JskAndroidGui:ItemSeleted", "Sending switch messgae");
				} else {
					isAdapterSet_points = true;
					Log.i("JskAndroidGui:", "points adapter not set");
				}
			}

			public void onNothingSelected(AdapterView parent) {
				Toast.makeText(JskAndroidGui.this, "Updating Param",
						Toast.LENGTH_SHORT).show();
				GetParamAndSetSpinner();
			}
		});

		params = jskAndroidGuiNode.getParameterTree();
		params.addParameterListener("/Tablet/UserList",
				new ParameterListener() {
					@Override
					public void onNewValue(Object value) {
						try {
							jskAndroidGuiNode.getTasksParam();
						} catch (Exception ex) {
							Log.e("JskAndroidGui",
									"Param cast error: " + ex.toString());
							// Toast.makeText(JskAndroidGui.this,
							// "No Param Found: " + ex.getMessage(),
							// Toast.LENGTH_SHORT).show();
						}

						Log.i("JskAndroidGui:GetTasksParam", "end");
						tasks_spinner
								.setOnItemSelectedListener(new OnItemSelectedListener() {
									public void onItemSelected(
											AdapterView parent, View viw,
											int arg2, long arg3) {
										if (isAdapterSet_tasks) {
											Spinner spinner = (Spinner) parent;
											String item = (String) spinner
													.getSelectedItem();
											jskAndroidGuiNode
													.tasksSpinnerTask(item);
											Toast.makeText(JskAndroidGui.this,
													"tasks: StartDemo " + item,
													Toast.LENGTH_SHORT).show();
											Log.i("JskAndroidGui:ItemSeleted",
													"Sending StartDemo messgae");

											TextView tv = (TextView) findViewById(R.id.textarea_test);
											try {
												tv.setText("param update searching");
												tv.setTextSize(14);
												tv.setTextColor(Color.WHITE);
												Log.i("JskAndroidGui:GetTasksParam",
														"setting text");
											} catch (Exception ex) {
												Log.i("JskAndroidGui:GetTasksParam",
														"set text error");
											}
										} else {
											isAdapterSet_tasks = true;
											Log.i("JskAndroidGui:",
													"tasks adapter not set");
										}
									}

									public void onNothingSelected(
											AdapterView parent) {
										Toast.makeText(JskAndroidGui.this,
												"Updating Param",
												Toast.LENGTH_SHORT).show();
										TextView tv = (TextView) findViewById(R.id.textarea_test);
										try {
											tv.setText("param update searching");
											tv.setTextSize(14);
											tv.setTextColor(Color.WHITE);
											Log.i("JskAndroidGui:GetTasksParam",
													"setting text");
										} catch (Exception ex) {
											Log.i("JskAndroidGui:GetTasksParam",
													"set text error");
										}
									}
								});

						mHandler.post(new Runnable() {
							public void run() {
								TextView tv = (TextView) findViewById(R.id.textarea_test);
								if (isParamSet) {
									try {
										tv.setText("Updated");
										tv.setTextSize(50);
										tv.setTextColor(Color.RED);
										Log.i("JskAndroidGui:GetTasksParam",
												"setting text");
									} catch (Exception ex) {
										Log.i("JskAndroidGui:GetTasksParam",
												"set text error");
									}
								} else {
									isParamSet = true;
									Log.i("JskAndroidGui:", "param not set");
								}

								Log.i("JskAndroidGui:debug", "spinner updating");
								isAdapterSet_spots = false;
								isAdapterSet_tasks = false;
								isAdapterSet_camera = false;
								isAdapterSet_points = false;
								GetParamAndSetSpinner();

							}
						});

						Log.i("JskAndroidGui:GetTasksParam", "updated");
					}
				});// end of parameter listener
		params.addParameterListener("/Tablet/Found", new ParameterListener() {
			@Override
			public void onNewValue(Object value) {
				String defaultTask_ns = "/Tablet";
				GraphName guser = GraphName.of(defaultTask_ns);
				NameResolver resolver_user = public_node.getResolver()
						.newChild(guser);
				try {
					found_task = params.getList(resolver_user.resolve("Found"))
							.toArray();
				} catch (Exception ex) {
					Log.i("JskAndroidGui:GetTasksParam", (String) found_task[0]
							+ " set text error");
				}
				mHandler.post(new Runnable() {
					public void run() {
						TextView tv = (TextView) findViewById(R.id.textarea_test);
						if (isParamSet) {
							try {
								tv.setText((String) found_task[0]);
								tv.setTextSize(40);
								tv.setTextColor(Color.GREEN);
								Log.i("JskAndroidGui:GetTasksParam",
										"setting text");
							} catch (Exception ex) {
								Log.i("JskAndroidGui:GetTasksParam",
										"set text error");
							}
						} else {
							isParamSet = true;
							Log.i("JskAndroidGui:", "param not set");
						}
						Log.i("JskAndroidGui:debug", "spinner updating");
						isAdapterSet_spots = false;
						isAdapterSet_tasks = false;
						isAdapterSet_camera = false;
						isAdapterSet_points = false;
						GetParamAndSetSpinner();

					}
				});
			}
		});// end of parameter listener

		params.addParameterListener("/Tablet/query_input",
				new ParameterListener() {

					@Override
					public void onNewValue(Object value) {
						String defaultTask_ns = "/Tablet";
						GraphName guser = GraphName.of(defaultTask_ns);
						NameResolver resolver_user = public_node.getResolver()
								.newChild(guser);
						try {
							query_input = params.getList(
									resolver_user.resolve("query_input"))
									.toArray();
						} catch (Exception ex) {
							Log.i("JskAndroidGui:GetTasksParam", query_input[0]
									+ " set text error");
						}
						LayoutInflater inflater = LayoutInflater
								.from(JskAndroidGui.this);
						final View view = inflater.inflate(R.layout.dialog,
								null);

						final EditText editText = (EditText) view
								.findViewById(R.id.editText1);
						mHandler.post(new Runnable() {
							public void run() {
								Log.i("JskAndroidGui:debug", "dialog handler");
								new AlertDialog.Builder(JskAndroidGui.this)
										.setTitle(
												"teach name: " + query_input[0])
										.setView(view)
										.setPositiveButton(
												"Save",
												new DialogInterface.OnClickListener() {

													@Override
													public void onClick(
															DialogInterface dialog,
															int which) {
														jskAndroidGuiNode
																.selectTask(editText
																		.getText()
																		.toString());
														Toast.makeText(
																JskAndroidGui.this,
																"tasks: Send dialog msg",
																Toast.LENGTH_SHORT)
																.show();
														Log.i("JskAndroidGui:debug",
																"dialog clicked");
													}
												}).show();
							}
						});
					}
				});// end of parameter listener

		Log.i("JskAndroidGui:debug", "before first spinner update");

		mHandler.post(new Runnable() {
			public void run() {
				Log.i("JskAndroidGui:debug", "spinner updating");
				GetParamAndSetSpinner();
				isParamSet = true;
			}
		});
	}

	@Override
	public void onCreateContextMenu(ContextMenu menu, View v,
			ContextMenuInfo menuInfo) {
		super.onCreateContextMenu(menu, v, menuInfo);

		Log.i("JskAndroidGui:debug", "onCreateContextMenu");
		menu.setHeaderTitle("Long touch detected");
		// Menu.add(int groupId, int itemId, int order, CharSequence title)
		menu.add(0, CONTEXT_MENU1_ID, 0, "PUSHONCE");
		menu.add(0, CONTEXT_MENU2_ID, 0, "PICKONCE");
		menu.add(0, CONTEXT_MENU3_ID, 0, "PLACEONCE");
		menu.add(0, CONTEXT_MENU4_ID, 0, "GetTemplate");
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		Log.i("JskAndroidGui:debug", "onCreateOptionsMenu");
		MenuInflater inflater = getMenuInflater();
		inflater.inflate(R.menu.jsk_android_gui, menu);
		isAdapterSet_spots = false;
		isAdapterSet_tasks = false;
		isAdapterSet_camera = false;
		isAdapterSet_points = false;
		GetParamAndSetSpinner();
                MenuItem actionItem = menu.add("Action Button");
                actionItem.setIcon(android.R.drawable.ic_menu_help);
                actionItem.setShowAsAction(MenuItem.SHOW_AS_ACTION_ALWAYS);
		return true;
	}

	@Override
	public boolean onContextItemSelected(MenuItem item) {
		switch (item.getItemId()) {
		case CONTEXT_MENU1_ID:
			Log.i("JskAndroidGui:ItemSeleted", "Publish PushOnce");
			cameraView.PublishPushOnce();
			return true;
		case CONTEXT_MENU2_ID:
			Log.i("JskAndroidGui:ItemSeleted", "Publish PickOnce");
			cameraView.PublishPickOnce();
			return true;
		case CONTEXT_MENU3_ID:
			Log.i("JskAndroidGui:ItemSeleted", "Publish PlaceOnce");
			cameraView.PublishPlaceOnce();
			return true;
		case CONTEXT_MENU4_ID:
			Log.i("JskAndroidGui:ItemSeleted", "Publish GetTemplate");
			cameraView.PublishGetTemplateOnce();
			return true;
		default:
			return super.onContextItemSelected(item);
		}
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()) {
		case R.id.getspot:
			jskAndroidGuiNode.getSpot();
			Log.i("JskAndroidGui:ItemSeleted", "Sending GetSpot messgae");
			return true;
		case R.id.gettemplate:
			cameraView.SetGetTemplate();
			return true;
		case R.id.zoomcamera:
			cameraView.SetZoomCamera();
			return true;
		case R.id.endzoomcamera:
			cameraView.unSetZoomCamera();
			return true;
		case R.id.pickonce:
			Log.i("JskAndroidGui:ItemSeleted", "Set PickOnce");
			cameraView.SetPickOnce();
			return true;
			/*
			 * case R.id.opendoor: cameraView.unSetMovingFingerInfo();
			 * cameraView.SendOpenDoorMsg(); Log.i("JskAndroidGui:ItemSeleted",
			 * "Send OpenDoorMsg"); return true;
			 */
		case R.id.pushonce:
			Log.i("JskAndroidGui:ItemSeleted", "Set PushOnce");
			cameraView.SetPushOnce();
			return true;
		case R.id.placeonce:
			Log.i("JskAndroidGui:ItemSeleted", "Set PlaceOnce");
			cameraView.SetPlaceOnce();//
			return true;
			/*
			 * case R.id.closedoor: Log.i("JskAndroidGui:ItemSeleted",
			 * "Send CloseDoorMsg"); cameraView.SendCloseDoorMsg();// return
			 * true;
			 */
		case R.id.passtohumanonce:
			Log.i("JskAndroidGui:ItemSeleted", "Set PassToHuman");
			cameraView.SetPassToHumanOnce();//
			return true;

		case R.id.tuckarmpose:
			Log.i("JskAndroidGui:ItemSeleted", "TuckArmPose");
			cameraView.SendTuckArmPoseMsg();//
			return true;
		case R.id.torsoup: // DECATED
			cameraView.SendTorsoUpMsg();//
			Log.i("JskAndroidGui:ItemSeleted", "Send TorsoUpMsg");
			return true;
		case R.id.torsodown: // DEPRECATED
			cameraView.SendTorsoDownMsg();//
			Log.i("JskAndroidGui:ItemSeleted", "Send TorsoDownMsg");
			return true;
		case R.id.opengripper: // DEPRECATED
			cameraView.SendOpenGripperMsg();
			Log.i("JskAndroidGui:ItemSeleted", "Send OpenGripperMsg");
			return true;
		case R.id.closegripper: // DEPRECATED
			cameraView.SendCloseGripperMsg();
			Log.i("JskAndroidGui:ItemSeleted", "Send CloseGripperMsg");
			return true;
		case R.id.changetouchmode:
			cameraView.ChangeTouchMode();
			Log.i("JskAndroidGui:ItemSeleted", "Change TouchMode");
			return true;
		case R.id.changelongtouch:
			Log.i("JskAndroidGui:ItemSeleted", "Change LongTouch");
			if (LongTouchFlag) {
				// ImageView ivInContext = (ImageView) findViewById(R.id.image);
				unregisterForContextMenu(boundingBoxView);
				LongTouchFlag = false;
			} else {
				// ImageView ivInContext = (ImageView) findViewById(R.id.image);
				// ivInContext.setFocusable(true);
				// ivInContext.setClickable(true);
				registerForContextMenu(boundingBoxView);
				LongTouchFlag = true;
			}
			return true;
		case R.id.resetall:
			isAdapterSet_spots = false;
			isAdapterSet_tasks = false;
			GetParamAndSetSpinner();
			cameraView.SetResetAll();
			isDrawLine = false;
			ImageView ivInContext = (ImageView) findViewById(R.id.image);
			ivInContext.setFocusable(true);
			ivInContext.setClickable(true);
			registerForContextMenu(ivInContext);
			LongTouchFlag = true;
			Log.i("JskAndroidGui:ItemSeleted", "Set ResetAll");
			return true;
		case R.id.stopjoint:
			jskAndroidGuiNode.stopJoint();
			Toast.makeText(JskAndroidGui.this, "tasks: EmergencyStopJoint",
					Toast.LENGTH_SHORT).show();
			Log.i("JskAndroidGui:ItemSeleted", "Sending EmergencyStopJoint");
			return true;
		case R.id.stopnavigation:
			jskAndroidGuiNode.stopNavigation();
			Toast.makeText(JskAndroidGui.this,
					"tasks: EmergencyStopNavigation", Toast.LENGTH_SHORT)
					.show();
			Log.i("JskAndroidGui:ItemSeleted",
					"Sending EmergencyStopNavigation");
			return true;
		case R.id.resetcollider:
			cameraView.SetResetCollider();
			return true;

		case R.id.switchjoy:
			jskAndroidGuiNode.switchJoy();
			Toast.makeText(JskAndroidGui.this, "tasks: Switchjoy",
					Toast.LENGTH_SHORT).show();
			Log.i("JskAndroidGui:ItemSelected", "Sending switchjoy");
			return true;
		case R.id.change_layout:
			viewFlipper.showNext();
			return true;
		case R.id.change_webview:
			webViewFlipper.showNext();
			return true;
		default:
			return super.onOptionsItemSelected(item);
		}
	}

	protected void GetParamAndSetSpinner() {
		// tasks_list.clear(); spots_list.clear();
		// image_list.clear(); camera_info_list.clear(); points_list.clear();
		try {
			ArrayAdapter<String> adapter_tasks = new ArrayAdapter<String>(this,
					R.layout.list);
			adapter_tasks
					.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
			for (int i = 0; i <= tasks_list.size() - 1; i++) {
				adapter_tasks.add(tasks_list.get(i));
			}
			tasks_spinner.setAdapter(adapter_tasks);
			tasks_spinner.setPromptId(R.string.SpinnerPrompt_tasks);
			ArrayAdapter<String> adapter_spots = new ArrayAdapter<String>(this,
					R.layout.list);
			adapter_spots
					.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
			for (int i = 0; i <= spots_list.size() - 1; i++) {
				adapter_spots.add(spots_list.get(i));
			}
			spots_spinner.setAdapter(adapter_spots);
			spots_spinner.setPromptId(R.string.SpinnerPrompt_spots);

			ArrayAdapter<String> adapter_image = new ArrayAdapter<String>(this,
					R.layout.list);
			adapter_image
					.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
			for (int i = 0; i <= image_list.size() - 1; i++) {
				adapter_image.add(image_list.get(i));
			}
			image_spinner.setAdapter(adapter_image);

			ArrayAdapter<String> adapter_points = new ArrayAdapter<String>(
					this, R.layout.list);
			adapter_points
					.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
			for (int i = 0; i <= points_list.size() - 1; i++) {
				adapter_points.add(points_list.get(i));
			}
			points_spinner.setAdapter(adapter_points);
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "param adapter error: " + ex.toString());
			Toast.makeText(JskAndroidGui.this,
					"adapter error: " + ex.getMessage(), Toast.LENGTH_SHORT)
					.show();
		}
	}
}