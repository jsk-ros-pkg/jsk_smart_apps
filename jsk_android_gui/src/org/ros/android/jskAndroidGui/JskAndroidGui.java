package org.ros.android.jskAndroidGui;

import android.os.Bundle;
import android.os.Handler;
import android.util.Log;

import android.app.AlertDialog;
import android.content.DialogInterface;

import android.view.Menu;
import android.view.MenuInflater;
import android.view.LayoutInflater;
import android.view.MenuItem;
import android.view.ContextMenu;
import android.view.ContextMenu.ContextMenuInfo;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Toast;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.ImageView;
import android.widget.ArrayAdapter;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.graphics.Color;

import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import org.ros.node.parameter.ParameterTree;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.service.ServiceServer;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import jsk_gui_msgs.Action;
import jsk_gui_msgs.QueryRequest;
import jsk_gui_msgs.QueryResponse;

import org.ros.address.InetAddressFactory;
import org.ros.android.view.VirtualJoystickView;
import org.ros.android.robotapp.RosAppActivity;
import java.util.ArrayList;

//import java.util.*;

/**
 * @author chen@jsk.t.u-tokyo.ac.jp (Haseru Azuma)
 */

public class JskAndroidGui extends RosAppActivity {
	
	
	public JskAndroidGui() {
		super("jsk android gui","jsk android gui");
	}

	private String robotAppName, cameraTopic;
	private SensorImageViewInfo cameraView;
	private VirtualJoystickView joystickView;
	private TextView tview;
	private JskAndroidGuiNode jskAndroidGuiNode;

	private ParameterTree params;
	private Node public_node;
	private ServiceServer<QueryRequest, QueryResponse> server;
	private Button yes_button, no_button;
	private ImageButton tweet_button;
	private RadioGroup radioGroup;
	private Spinner spots_spinner, tasks_spinner, image_spinner,
			points_spinner;
	
	
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

	@Override
	public void onCreate(Bundle savedInstanceState) {
		
		setDefaultRobotName("pr1040");
		setDefaultAppName("jsk_android_gui/jsk_android_gui");
		setDashboardResource(R.id.top_bar);
		setMainWindowResource(R.layout.main);
		super.onCreate(savedInstanceState);
		
		jskAndroidGuiNode = new JskAndroidGuiNode();

		tweet_button = (ImageButton) findViewById(R.id.tweet);
		yes_button = (Button) findViewById(R.id.resultyes);
		no_button = (Button) findViewById(R.id.resultno);

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
		joystickView = (VirtualJoystickView) findViewById(R.id.joystick);
		joystickView.setTopicName("android/cmd_vel");
		cameraView = (SensorImageViewInfo) findViewById(R.id.image);
		cameraView.setClickable(true);
		cameraView.SetRobotArm(Action.LARMID);
		cameraView.setCameraTopic(cameraTopic);
		
		mHandler = new Handler();

		ImageView ivInContext = (ImageView) findViewById(R.id.image);
		ivInContext.setFocusable(true);
		ivInContext.setClickable(true);
		registerForContextMenu(ivInContext);
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		
		super.init(nodeMainExecutor);
		
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
				InetAddressFactory.newNonLoopback().getHostAddress(),
				getMasterUri());
		
		NameResolver appNamespace = getAppNameSpace();
		nodeMainExecutor.execute(cameraView, nodeConfiguration.setNodeName("android/camera_view"));
		
		//cameraView.start(node, appNamespace.resolve(cameraTopic).toString());
		cameraView.post(new Runnable() {
			@Override
			public void run() {
				Log.i("JskAndroidGui:debug", "cameraView run");
				cameraView.setSelected(true);
			}
		});
		nodeMainExecutor.execute(jskAndroidGuiNode, nodeConfiguration.setNodeName("android/jsk_android_gui"));
		nodeMainExecutor.execute(joystickView,nodeConfiguration.setNodeName("android/joystick"));
		
		
		waitForSetupNode();
	}
	
	
	private void waitForSetupNode (){
		while (!jskAndroidGuiNode.setupEnd){

		try{
			Thread.sleep(1000);
		}catch (Exception e) {
			
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

												jskAndroidGuiNode.tweetTask(editText.getText().toString());
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

	protected void setupListener() {
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
		
		try{
			jskAndroidGuiNode.getSpotsParam();
			spots_list = jskAndroidGuiNode.getSpotsList();
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
			//Toast.makeText(JskAndroidGui.this, "No Param Found: " + ex.getMessage(), Toast.LENGTH_SHORT) .show();
		}
		
		spots_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
			public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3) { 
				if (isAdapterSet_spots) {
					Spinner spinner = (Spinner) parent; 
					String item = (String) spinner.getSelectedItem();
					jskAndroidGuiNode.spotsSpinnerTask(item);
					Toast.makeText(JskAndroidGui.this, "spots: MoveToSpot " + item, Toast.LENGTH_SHORT) .show();
					Log.i("JskAndroidGui:ItemSeleted", "Sending MoveToSpot messgae");
				} else {
					isAdapterSet_spots = true; Log.i("JskAndroidGui:", "spots adapter not set");
				}
			}
			
			public void onNothingSelected(AdapterView parent) {
				Toast.makeText(JskAndroidGui.this, "Updating Param", Toast.LENGTH_SHORT).show();
				GetParamAndSetSpinner(); }
		}); // for tasks
		try {
			jskAndroidGuiNode.getTasksParam();
			tasks_list = jskAndroidGuiNode.getTasksList();
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
		//	Toast.makeText(JskAndroidGui.this, "No Param Found: " + ex.getMessage(), Toast.LENGTH_SHORT).show();
		}
		tasks_spinner.setOnItemSelectedListener(new OnItemSelectedListener() {
			public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3){
				if (isAdapterSet_tasks) {
					Spinner spinner = (Spinner)parent; 
					String item = (String)spinner.getSelectedItem();
					jskAndroidGuiNode.tasksSpinnerTask(item);
					Toast.makeText(JskAndroidGui.this, "tasks: StartDemo " + item, Toast.LENGTH_SHORT).show();
					Log.i("JskAndroidGui:ItemSeleted", "Sending StartDemo messgae");
				} else {
					isAdapterSet_tasks = true;
					Log.i("JskAndroidGui:", "tasks adapter not set");
				}
			}
			
			public void onNothingSelected(AdapterView parent) {
			//	Toast.makeText(JskAndroidGui.this, "Updating Param", Toast.LENGTH_SHORT).show();
				GetParamAndSetSpinner();
			}
		});
		// for camera
		try {
			jskAndroidGuiNode.getCameraParam();
			image_list = jskAndroidGuiNode.getImageList();
			camera_info_list = jskAndroidGuiNode.getCameraInfoList();
		} catch (Exception ex){
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
	//		Toast.makeText(JskAndroidGui.this, "No Param Found: " + ex.getMessage(), Toast.LENGTH_SHORT).show();
		}
		image_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
			public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3){
				if (isAdapterSet_camera) {
					Spinner spinner = (Spinner)parent;
					defaultImage = (String)spinner.getSelectedItem(); // assume that the first element is "cameras", so -1
					defaultCameraInfo = camera_info_list.get(arg2 - 1);
					String str = "((:image " + defaultImage + ") (:camera_info " + defaultCameraInfo + ") (:points " + defaultPoints + "))";
					cameraView.PubSwitchSensor(str);
				//	Toast.makeText(JskAndroidGui.this, "SwitchSensor: " + str, Toast.LENGTH_SHORT).show();
					Log.i("JskAndroidGui:ItemSeleted", "Sending switch messgae");
				} else {
					isAdapterSet_camera = true; Log.i("JskAndroidGui:", "camera adapter not set");
				}
			}
			
			public void onNothingSelected(AdapterView parent) {
				Toast.makeText(JskAndroidGui.this, "Updating Param", Toast.LENGTH_SHORT).show();
				GetParamAndSetSpinner();
			}
		});
		
		// for points
		try {
			jskAndroidGuiNode.getPointsParam();
			points_list = jskAndroidGuiNode.getPointsList();
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
	//		Toast.makeText(JskAndroidGui.this, "No Param Found: " + ex.getMessage(), Toast.LENGTH_SHORT).show();
		}
		points_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
			public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3) {
				if (isAdapterSet_points) {
					Spinner spinner = (Spinner)parent;
					defaultPoints = (String)spinner.getSelectedItem();
					String str = "((:image " + defaultImage + ") (:camera_info " + defaultCameraInfo + ") (:points " + defaultPoints + "))";
					cameraView.PubSwitchSensor(str);
					Toast.makeText(JskAndroidGui.this, "SwitchSensor: " + str, Toast.LENGTH_SHORT).show();
					Log.i("JskAndroidGui:ItemSeleted", "Sending switch messgae");
				} else {
					isAdapterSet_points = true;
					Log.i("JskAndroidGui:", "points adapter not set");
				}
			}
			
			public void onNothingSelected(AdapterView parent) {
				Toast.makeText(JskAndroidGui.this, "Updating Param", Toast.LENGTH_SHORT).show();
				GetParamAndSetSpinner();
			}
		});
		
		
		params = jskAndroidGuiNode.getParameterTree();
		params.addParameterListener("/Tablet/UserList", new ParameterListener() {
			@Override
			public void onNewValue(Object value) {
				try {
					jskAndroidGuiNode.getTasksParam();
				} catch (Exception ex) {
					Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
		//			Toast.makeText(JskAndroidGui.this, "No Param Found: " + ex.getMessage(), Toast.LENGTH_SHORT).show();
				}
				
				Log.i("JskAndroidGui:GetTasksParam", "end");
				tasks_spinner.setOnItemSelectedListener(new OnItemSelectedListener() {
					public void onItemSelected( AdapterView parent, View viw, int arg2, long arg3) {
						if (isAdapterSet_tasks) {
							Spinner spinner = (Spinner)parent;
							String item = (String)spinner.getSelectedItem();
							jskAndroidGuiNode.tasksSpinnerTask(item);
							Toast.makeText(JskAndroidGui.this, "tasks: StartDemo " + item, Toast.LENGTH_SHORT).show();
							Log.i("JskAndroidGui:ItemSeleted", "Sending StartDemo messgae");
							
							TextView tv = (TextView)findViewById(R.id.textarea_test);
							try {
								tv.setText("param update searching"); tv.setTextSize(14);
								tv.setTextColor(Color.WHITE); Log.i("JskAndroidGui:GetTasksParam", "setting text");
							} catch (Exception ex) {
								Log.i("JskAndroidGui:GetTasksParam", "set text error");
							}
						} else {
							isAdapterSet_tasks = true;
							Log.i("JskAndroidGui:", "tasks adapter not set");
						}
					}
					
					public void onNothingSelected( AdapterView parent) {
						Toast.makeText(JskAndroidGui.this, "Updating Param", Toast.LENGTH_SHORT).show();
						TextView tv = (TextView)findViewById(R.id.textarea_test);
						try {
							tv.setText("param update searching");
							tv.setTextSize(14);
							tv.setTextColor(Color.WHITE);
							Log.i("JskAndroidGui:GetTasksParam", "setting text");
						} catch (Exception ex) {
							Log.i("JskAndroidGui:GetTasksParam", "set text error");
						}
					}
				});
				
				mHandler.post(new Runnable() {
					public void run() {
						TextView tv = (TextView)findViewById(R.id.textarea_test);
						if (isParamSet) {
							try {
								tv.setText("Updated");
								tv.setTextSize(50);
								tv.setTextColor(Color.RED);
								Log.i("JskAndroidGui:GetTasksParam", "setting text");
							} catch (Exception ex) {
								Log.i("JskAndroidGui:GetTasksParam", "set text error");
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
				NameResolver resolver_user = public_node.getResolver().newChild(guser);
				try {
					found_task = params.getList(resolver_user.resolve("Found")).toArray();
				} catch (Exception ex) {
					Log.i("JskAndroidGui:GetTasksParam", (String)found_task[0] + " set text error");
				}
				mHandler.post(new Runnable() {
					public void run() {
						TextView tv = (TextView)findViewById(R.id.textarea_test);
						if (isParamSet) {
							try {
								tv.setText((String)found_task[0]);
								tv.setTextSize(40);
								tv.setTextColor(Color.GREEN);
								Log.i("JskAndroidGui:GetTasksParam", "setting text");
							} catch (Exception ex) {
								Log.i("JskAndroidGui:GetTasksParam", "set text error");
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
		
		params.addParameterListener("/Tablet/query_input", new ParameterListener() {
			
			@Override public void onNewValue(Object value) {
				String defaultTask_ns = "/Tablet";
				GraphName guser = GraphName.of(defaultTask_ns);
				NameResolver resolver_user = public_node.getResolver().newChild(guser);
				try {
					query_input = params.getList(resolver_user.resolve("query_input")).toArray();
				} catch (Exception ex) {
					Log.i("JskAndroidGui:GetTasksParam", query_input[0] + " set text error");
				}
				LayoutInflater inflater = LayoutInflater.from(JskAndroidGui.this);
				final View view = inflater.inflate(R.layout.dialog, null);
				
				final EditText editText = (EditText) view.findViewById(R.id.editText1);
				mHandler.post(new Runnable() { public void run() {
					Log.i("JskAndroidGui:debug", "dialog handler");
					new AlertDialog.Builder(JskAndroidGui.this).setTitle( "teach name: " + query_input[0]).setView(view).setPositiveButton( "Save", new DialogInterface.OnClickListener() {
				
						@Override public void onClick( DialogInterface dialog, int which) {
							jskAndroidGuiNode.selectTask(editText.getText().toString());
							Toast.makeText(JskAndroidGui.this, "tasks: Send dialog msg", Toast.LENGTH_SHORT).show();
							Log.i("JskAndroidGui:debug", "dialog clicked");
						}
					}).show();
				} 
				}); 
			}
		});// end of parameter listener
		
		Log.i("JskAndroidGui:debug", "before first spinner update");
		
		mHandler.post(new Runnable() { public void run() {
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
		case R.id.setdrawline:
			if (isDrawLine) {
				Log.i("JskAndroidGui:ItemSeleted", "unSet DrawLine");
				cameraView.unSetDrawLine();
				isDrawLine = false;
			} else {
				Log.i("JskAndroidGui:ItemSeleted", "Set DrawLine");
				cameraView.SetDrawLine();
				isDrawLine = true;
			}
			return true;
		case R.id.pickonce:
			Log.i("JskAndroidGui:ItemSeleted", "Set PickOnce");
			cameraView.SetPickOnce();
			return true;
		/*case R.id.opendoor:
			cameraView.unSetMovingFingerInfo();
			cameraView.SendOpenDoorMsg();
			Log.i("JskAndroidGui:ItemSeleted", "Send OpenDoorMsg");
			return true;*/
		case R.id.pushonce:
			Log.i("JskAndroidGui:ItemSeleted", "Set PushOnce");
			cameraView.SetPushOnce();
			return true;
		case R.id.placeonce:
			Log.i("JskAndroidGui:ItemSeleted", "Set PlaceOnce");
			cameraView.SetPlaceOnce();//
			return true;
		/*case R.id.closedoor:
			Log.i("JskAndroidGui:ItemSeleted", "Send CloseDoorMsg");
			cameraView.SendCloseDoorMsg();//
			return true;
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
				ImageView ivInContext = (ImageView) findViewById(R.id.image);
				unregisterForContextMenu(ivInContext);
				LongTouchFlag = false;
			} else {
				ImageView ivInContext = (ImageView) findViewById(R.id.image);
				ivInContext.setFocusable(true);
				ivInContext.setClickable(true);
				registerForContextMenu(ivInContext);
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