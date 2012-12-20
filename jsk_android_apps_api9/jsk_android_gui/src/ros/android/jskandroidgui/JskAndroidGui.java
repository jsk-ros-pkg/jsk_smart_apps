package ros.android.jskandroidgui;

import android.os.Bundle;
import android.os.Handler;
import android.util.Log;

import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.ContextMenu;
import android.view.ContextMenu.ContextMenuInfo;
import android.view.View;
import android.view.MotionEvent;
import android.view.View.OnClickListener;
import android.widget.Toast;
import android.widget.Button;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.ImageView;
import android.widget.ArrayAdapter;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.hardware.SensorManager;

import org.ros.node.Node;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.exception.RosException;
import org.ros.message.Time;
import org.ros.message.std_msgs.Empty;
import org.ros.message.roseus.StringStamped;
import org.ros.message.jsk_gui_msgs.Action;

import ros.android.views.JoystickView;
import ros.android.activity.RosAppActivity;
import java.util.ArrayList;

//import java.util.Timer;
//import java.util.TimerTask;
//import java.util.*;

/**
 * @author chen@jsk.t.u-tokyo.ac.jp (Haseru Azuma)
 */

public class JskAndroidGui extends RosAppActivity {
    private String robotAppName, cameraTopic;
    private SensorImageViewInfo cameraView;
    private JoystickView joystickView;
    private Publisher<Empty> GetSpotPub;
    private Publisher<StringStamped> StartDemoPub;
    private Publisher<StringStamped> MoveToSpotPub;
    private Publisher<StringStamped> SelectPub;
    private Publisher<StringStamped> EmergencyStopPub;
    private ParameterTree params;

    //private Button demo_button;
    private Button yes_button;
    private Button no_button;
    private RadioGroup radioGroup;
    private Spinner spots_spinner, tasks_spinner, image_spinner, points_spinner;
    private ArrayList<String> spots_list = new ArrayList(), tasks_list = new ArrayList(),
	camera_list = new ArrayList(), points_list = new ArrayList();
    private String defaultCamera = "/openni/rgb", defaultPoints = "/openni/depth_registered/points_throttle"; // will be renamed when parameter comes
    private boolean isDrawLine = false,isAdapterSet_spots = false, isAdapterSet_tasks = false,isNotParamInit = true,isAdapterSet_camera = false, isAdapterSet_points = false;

    private Handler mHandler;
    static final int CONTEXT_MENU1_ID = 0;
    static final int CONTEXT_MENU2_ID = 1;
    static final int CONTEXT_MENU3_ID = 2;

    @Override
	public void onCreate(Bundle savedInstanceState) {
	setDefaultAppName("jsk_gui/jsk_android_gui");
	setDashboardResource(R.id.top_bar);
	setMainWindowResource(R.layout.main);
	super.onCreate(savedInstanceState);

	yes_button = (Button)findViewById(R.id.resultyes);
	no_button = (Button)findViewById(R.id.resultno);

	radioGroup = (RadioGroup) findViewById(R.id.radiogroup);
	radioGroup.check(R.id.radiobutton_L);
	radioGroup.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
		public void onCheckedChanged(RadioGroup group, int checkedId) {
		    RadioButton radioButton = (RadioButton) findViewById(checkedId);
		    if (radioButton.getText().equals("left")){
			cameraView.SetRobotArm(Action.LARMID);
			safeToastStatus("robot arm set to :larm");
			Log.i("JskAndroidGui:ItemSeleted", "Set arm to :larm");
		    } else {
			cameraView.SetRobotArm(Action.RARMID);
			safeToastStatus("robot arm set to :rarm");
			Log.i("JskAndroidGui:ItemSeleted", "Set arm to :rarm");
		    }
		}
	    });

	spots_spinner = (Spinner)findViewById(R.id.spinner_spots);
	ArrayAdapter<String> adapter_spots = new ArrayAdapter<String>(this, R.layout.list);
	spots_spinner.setAdapter(adapter_spots);
	adapter_spots.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
	spots_spinner.setPromptId(R.string.SpinnerPrompt_spots);

	tasks_spinner = (Spinner)findViewById(R.id.spinner_tasks);
	ArrayAdapter<String> adapter_tasks = new ArrayAdapter<String>(this, R.layout.list);
	tasks_spinner.setAdapter(adapter_tasks);
	adapter_tasks.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
	tasks_spinner.setPromptId(R.string.SpinnerPrompt_tasks);

	image_spinner = (Spinner)findViewById(R.id.spinner_image);
	// String[] image_list = {"cameras", "/openni/rgb", "/wide_stereo/left", "/wide_stereo/right", "/narrow_stereo/left", "/narrow_stereo/right", "/l_forearm_cam", "/r_forearm_cam"};
	//ArrayAdapter<String> adapter_image = new ArrayAdapter<String>(this, R.layout.list, image_list);
	ArrayAdapter<String> adapter_image = new ArrayAdapter<String>(this, R.layout.list);
	image_spinner.setAdapter(adapter_image);
	adapter_image.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

	points_spinner = (Spinner)findViewById(R.id.spinner_points);
	// String[] points_list = {"points", "/openni/depth_registered/points", "/tilt_laser_cloud2"};
	//ArrayAdapter<String> adapter_points = new ArrayAdapter<String>(this, R.layout.list, points_list);
	ArrayAdapter<String> adapter_points = new ArrayAdapter<String>(this, R.layout.list);
	points_spinner.setAdapter(adapter_points);
	adapter_points.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);

	if (getIntent().hasExtra("camera_topic")) {
	    cameraTopic = getIntent().getStringExtra("camera_topic");
	} else {
	    cameraTopic = "/tablet/marked/image_rect_color/compressed_throttle";
	}
	//joystickView = (JoystickView) findViewById(R.id.joystick);
	//joystickView.setBaseControlTopic("android/cmd_vel");
	cameraView = (SensorImageViewInfo) findViewById(R.id.image);
	cameraView.setClickable(true);
	cameraView.SetRobotArm(Action.LARMID);
	mHandler = new Handler();

	ImageView ivInContext = (ImageView) findViewById(R.id.image);
	ivInContext.setFocusable(true);
	ivInContext.setClickable(true);
	registerForContextMenu(ivInContext);
    }

    @Override
	protected void onNodeCreate(Node node) {
	super.onNodeCreate(node);
	try {
	    NameResolver appNamespace = getAppNamespace(node);
	    cameraView.start(node, appNamespace.resolve(cameraTopic).toString());
	    cameraView.post(new Runnable() {
		    @Override
			public void run() {
			Log.i("JskAndroidGui:debug", "cameraView run");
			cameraView.setSelected(true);
		    }
		});
	    //joystickView.start(node);
	} catch (Exception ex) {
	    Log.e("JskAndroidGui", "Init error: " + ex.toString());
	    safeToastStatus("Failed: " + ex.getMessage());
	}

	GetSpotPub =
	    node.newPublisher( "/Tablet/GetSpot" , "std_msgs/Empty" );
	StartDemoPub =
	    node.newPublisher( "/Tablet/StartDemo" , "roseus/StringStamped" );
	MoveToSpotPub =
	    node.newPublisher( "/Tablet/MoveToSpot" , "roseus/StringStamped" );
	EmergencyStopPub =
	    node.newPublisher( "/Tablet/EmergencyCommand" , "roseus/StringStamped" );
	SelectPub =
	    node.newPublisher( "/Tablet/Select" , "roseus/StringStamped" );

	// demo_button.setOnClickListener(new OnClickListener(){
	// 	public void onClick(View viw) {
	// 	    Button button = (Button)viw;
	// 	    // button.setText("starting");
	// 	    StringStamped StrMsg = new StringStamped();
	// 	    StrMsg.header.stamp = Time.fromMillis(System.currentTimeMillis());
	// 	    StrMsg.data = "StartMainDemo";
	// 	    StartDemoPub.publish( StrMsg );
	// 	    safeToastStatus("demo: " + "StartMainDemo");
	// 	    Log.i("JskAndroidGui:ItemSeleted", "Sending StartDemo main messgae");
	// 	}});

	yes_button.setOnClickListener(new OnClickListener(){
		public void onClick(View viw) {
		    Button button = (Button)viw;
		    // button.setText("starting");
		    StringStamped StrMsg_resultyes = new StringStamped();
		    StrMsg_resultyes.header.stamp = Time.fromMillis(System.currentTimeMillis());
		    StrMsg_resultyes.data = "ResultYes";
		    SelectPub.publish( StrMsg_resultyes );
		    safeToastStatus("tasks: ResultYes");
		    Log.i("JskAndroidGui:ButtonClicked", "Sending ResultYes");
		}});

	no_button.setOnClickListener(new OnClickListener(){
		public void onClick(View viw) {
		    Button button = (Button)viw;
		    // button.setText("starting");
		    StringStamped StrMsg_resultno = new StringStamped();
		    StrMsg_resultno.header.stamp = Time.fromMillis(System.currentTimeMillis());
		    StrMsg_resultno.data = "ResultNo";
		    SelectPub.publish( StrMsg_resultno );
		    safeToastStatus("tasks: ResultNo");
		    Log.i("JskAndroidGui:ButtonClicked", "Sending ResultNo");
		}});

	params = node.newParameterTree();
	/* for spots */
	try{
	    String defaultSpot_ns = "/jsk_spots";
	    String targetSpot = "/eng2/7f"; // Todo get current targetSpot
	    GraphName param_ns = new GraphName(defaultSpot_ns + targetSpot);
	    NameResolver resolver = node.getResolver().createResolver(param_ns);
	    Object[] spots_param_list = params.getList(resolver.resolve("spots")).toArray();
	    Log.i("JskAndroidGui:GetSpotsParam", "spots length = " + spots_param_list.length);
	    spots_list.clear();spots_list.add("spots");
	    for (int i = 0; i < spots_param_list.length; i++) {
		spots_list.add((String)spots_param_list[i]);
		Log.w("JskAndroidGui:GetSpotsParam", "lists:" + i + " " + spots_param_list[i]);
	    }
	} catch (Exception ex) {
	    Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
	    safeToastStatus("No Param Found: " + ex.getMessage());
	}
	spots_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
		public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3) {
		    if(isAdapterSet_spots){
			Spinner spinner = (Spinner)parent;
			String item = (String)spinner.getSelectedItem();
			StringStamped StrMsg = new StringStamped();
			StrMsg.header.stamp = Time.fromMillis(System.currentTimeMillis());
			StrMsg.data = item;
			MoveToSpotPub.publish( StrMsg );
			safeToastStatus("spots: MoveToSpot " + item);
			Log.i("JskAndroidGui:ItemSeleted", "Sending MoveToSpot messgae");
		    } else {
			isAdapterSet_spots = true;
			Log.i("JskAndroidGui:", "spots adapter not set");
		    }
		}
		public void onNothingSelected(AdapterView parent) {
		    safeToastStatus("Updating Param");
		    GetParamAndSetSpinner();
		}});
	/* for tasks */
	try{
	    String defaultTask_ns = "/Tablet";
	    GraphName gtask0 = new GraphName(defaultTask_ns);
	    NameResolver resolver0 = node.getResolver().createResolver(gtask0);
	    Object[] user_list = params.getList(resolver0.resolve("UserList")).toArray();
	    tasks_list.clear();tasks_list.add("tasks");
	    for (int i = 0; i < user_list.length; i++) {
		GraphName gtask = new GraphName(defaultTask_ns + "/User");
		NameResolver resolver = node.getResolver().createResolver(gtask);
		Object[] task_param_list = params.getList(resolver.resolve((String)user_list[i])).toArray();

		Log.i("JskAndroidGui:GetTasksParam", "task length = " + task_param_list.length);
		for (int j = 0; j < task_param_list.length; j++) {
		    Log.i("JskAndroidGui:GetTasksParam", "lists: " +  i + " " + j + " /Tablet/" + (String)user_list[i] + "/" + (String)task_param_list[j]);
		    tasks_list.add("/Tablet/" + (String)user_list[i] + "/" + (String)task_param_list[j]);
		}
	    }
	} catch (Exception ex) {
	    Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
	    safeToastStatus("No Param Found: " + ex.getMessage());
	}

	tasks_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
		public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3) {
		    if(isAdapterSet_tasks){
			Spinner spinner = (Spinner)parent;
			String item = (String)spinner.getSelectedItem();
			StringStamped StrMsg = new StringStamped();
			StrMsg.header.stamp = Time.fromMillis(System.currentTimeMillis());
			StrMsg.data = item;
			StartDemoPub.publish( StrMsg );
			safeToastStatus("tasks: StartDemo " + item);
			Log.i("JskAndroidGui:ItemSeleted", "Sending StartDemo messgae");
		    } else {
			isAdapterSet_tasks = true;
			Log.i("JskAndroidGui:", "tasks adapter not set");
		    }
		}
		public void onNothingSelected(AdapterView parent) {
		    safeToastStatus("Updating Param");
		    GetParamAndSetSpinner();
		}});

	/* for camera */
	try{
	    String defaultCamera_ns = "/jsk_ns";
	    GraphName param_ns = new GraphName(defaultCamera_ns);
	    NameResolver resolver1 = node.getResolver().createResolver(param_ns);
	    Object[] camera_param_list = params.getList(resolver1.resolve("camera")).toArray();
	    Log.i("JskAndroidGui:GetCameraParam", "camera length = " + camera_param_list.length);
	    camera_list.clear();camera_list.add("cameras");
	    for (int i = 0; i < camera_param_list.length; i++) {
		if (i == 0) {defaultCamera = (String)camera_param_list[i];}
		camera_list.add((String)camera_param_list[i]);
		Log.w("JskAndroidGui:GetCameraParam", "lists:" + i + " " + camera_param_list[i]);
	    }
	} catch (Exception ex) {
	    Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
	    safeToastStatus("No Param Found: " + ex.getMessage());
	}
	image_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
		public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3) {
		    if(isAdapterSet_camera){
			Spinner spinner = (Spinner)parent;
			//String item = (String)spinner.getSelectedItem();
			defaultCamera = (String)spinner.getSelectedItem();
			String str =  "((:image "+ defaultCamera + ") (:points " + defaultPoints + "))";
			cameraView.PubSwitchSensor(str);
			safeToastStatus("SwitchSensor: " + str);
			Log.i("JskAndroidGui:ItemSeleted", "Sending switch messgae");

		    } else {
			isAdapterSet_camera = true;
			Log.i("JskAndroidGui:", "camera adapter not set");
		    }
		}
		public void onNothingSelected(AdapterView parent) {
		    safeToastStatus("Updating Param");
		    GetParamAndSetSpinner();
		}});

	// image_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
	// 	public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3) {
	// 	    Spinner spinner = (Spinner)parent;
	// 	    defaultCamera = (String)spinner.getSelectedItem();
	// 	    String str =  "((:image "+ defaultCamera + ") (:points " + defaultPoints + "))";
	// 	    cameraView.PubSwitchSensor(str);
	// 	    safeToastStatus("SwitchSensor: " + str);
	// 	    Log.i("JskAndroidGui:ItemSeleted", "Sending switch messgae");
	// 	}
	// 	public void onNothingSelected(AdapterView parent) {
	// 	    safeToastStatus("Updating Param");
	// 	    GetParamAndSetSpinner();
	// 	}});

	/* for points */
	try{
	    String defaultCamera_ns = "/jsk_ns";
	    GraphName param_ns = new GraphName(defaultCamera_ns);
	    NameResolver resolver2 = node.getResolver().createResolver(param_ns);
	    Object[] points_param_list = params.getList(resolver2.resolve("points")).toArray();
	    Log.i("JskAndroidGui:GetPointsParam", "point length = " + points_param_list.length);
	    points_list.clear();points_list.add("points");
	    for (int i = 0; i < points_param_list.length; i++) {
		if (i == 0) {defaultPoints = (String)points_param_list[i];}
		points_list.add((String)points_param_list[i]);
		Log.w("JskAndroidGui:GetPointsParam", "lists:" + i + " " + points_param_list[i]);
	    }
	} catch (Exception ex) {
	    Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
	    safeToastStatus("No Param Found: " + ex.getMessage());
	}
	points_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
		public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3) {
		    if(isAdapterSet_points){
			Spinner spinner = (Spinner)parent;
			//String item = (String)spinner.getSelectedItem();
			defaultPoints = (String)spinner.getSelectedItem();
			String str =  "((:image "+ defaultCamera + ") (:points " + defaultPoints + "))";
			cameraView.PubSwitchSensor(str);
			safeToastStatus("SwitchSensor: " + str);
			Log.i("JskAndroidGui:ItemSeleted", "Sending switch messgae");

		    } else {
			isAdapterSet_points = true;
			Log.i("JskAndroidGui:", "camera adapter not set");
		    }
		}
		public void onNothingSelected(AdapterView parent) {
		    safeToastStatus("Updating Param");
		    GetParamAndSetSpinner();
		}});

	// points_spinner.setOnItemSelectedListener(new OnItemSelectedListener(){
	// 	public void onItemSelected(AdapterView parent, View viw, int arg2, long arg3) {
	// 	    Spinner spinner = (Spinner)parent;
	// 	    defaultPoints = (String)spinner.getSelectedItem();
	// 	    String str =  "((:image "+ defaultCamera + ") (:points " + defaultPoints + "))";
	// 	    cameraView.PubSwitchSensor(str);
	// 	    safeToastStatus("SwitchSensor: " + str);
	// 	    Log.i("JskAndroidGui:ItemSeleted", "Sending switch messgae");
	// 	}
	// 	public void onNothingSelected(AdapterView parent) {
	// 	    safeToastStatus("Updating Param");
	// 	    GetParamAndSetSpinner();
	// 	}});

	Log.i("JskAndroidGui:debug", "before first spinner update");
	mHandler.post(new Runnable() {
		public void run() {
		    Log.i("JskAndroidGui:debug", "spinner updating");
		    GetParamAndSetSpinner();
		}
	    });

    }

    @Override
	protected void onNodeDestroy(Node node) {
	super.onNodeDestroy(node);
    }

    @Override
	public void onCreateContextMenu(ContextMenu menu, View v,
					ContextMenuInfo menuInfo) {
	super.onCreateContextMenu(menu, v, menuInfo);
	Log.i("JskAndroidGui:debug", "onCreateContextMenu");
	menu.setHeaderTitle("Long touch detected");
	//Menu.add(int groupId, int itemId, int order, CharSequence title)
	menu.add(0, CONTEXT_MENU1_ID, 0, "PUSHONCE");
	menu.add(0, CONTEXT_MENU2_ID, 0, "PICKONCE");
	menu.add(0, CONTEXT_MENU3_ID, 0, "PLACEONCE");
    }

    @Override
	public boolean onCreateOptionsMenu(Menu menu) {
	Log.i("JskAndroidGui:debug", "onCreateOptionsMenu");
	MenuInflater inflater = getMenuInflater();
	inflater.inflate(R.menu.jsk_android_gui, menu);
	isAdapterSet_spots = false; isAdapterSet_tasks = false;
	isAdapterSet_camera = false; isAdapterSet_points = false;
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
	default:
	    return super.onContextItemSelected(item);
	}
    }

    @Override
	public boolean onOptionsItemSelected(MenuItem item) {
	switch (item.getItemId()) {
	case R.id.getspot:
	    Empty EmptyMsg = new Empty();
	    GetSpotPub.publish( EmptyMsg );
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
	case R.id.opendoor:
	    cameraView.unSetMovingFingerInfo();
	    cameraView.SendOpenDoorMsg();
	    Log.i("JskAndroidGui:ItemSeleted", "Send OpenDoorMsg");
	    return true;
	case R.id.pushonce:
	    Log.i("JskAndroidGui:ItemSeleted", "Set PushOnce");
	    cameraView.SetPushOnce();
	    return true;
	case R.id.placeonce:
	    Log.i("JskAndroidGui:ItemSeleted", "Set PlaceOnce");
	    cameraView.SetPlaceOnce();//
	    return true;
	case R.id.closedoor:
	    Log.i("JskAndroidGui:ItemSeleted", "Send CloseDoorMsg");
	    cameraView.SendCloseDoorMsg();//
	    return true;

	case R.id.passtohumanonce:
	    Log.i("JskAndroidGui:ItemSeleted", "Set PassToHuman");
	    cameraView.SetPassToHumanOnce();//
	    return true;

	case R.id.tuckarmpose:
	    Log.i("JskAndroidGui:ItemSeleted", "TuckArmPose");
	    cameraView.SendTuckArmPoseMsg();//
	    return true;
	case R.id.torsoup: //DEPRECATED
	    cameraView.SendTorsoUpMsg();//
	    Log.i("JskAndroidGui:ItemSeleted", "Send TorsoUpMsg");
	    return true;
	case R.id.torsodown: //DEPRECATED
	    cameraView.SendTorsoDownMsg();//
	    Log.i("JskAndroidGui:ItemSeleted", "Send TorsoDownMsg");
	    return true;
	case R.id.opengripper: //DEPRECATED
	    cameraView.SendOpenGripperMsg();
	    Log.i("JskAndroidGui:ItemSeleted", "Send OpenGripperMsg");
	    return true;
	case R.id.closegripper: //DEPRECATED
	    cameraView.SendCloseGripperMsg();
	    Log.i("JskAndroidGui:ItemSeleted", "Send CloseGripperMsg");
	    return true;
	case R.id.changetouchmode:
	    cameraView.ChangeTouchMode();
	    Log.i("JskAndroidGui:ItemSeleted", "Change TouchMode");
	    return true;
	case R.id.resetall:
	    isAdapterSet_spots = false; isAdapterSet_tasks = false;
	    GetParamAndSetSpinner();
	    cameraView.SetResetAll();
	    isDrawLine = false;
	    Log.i("JskAndroidGui:ItemSeleted", "Set ResetAll");
	    return true;
	case R.id.stopjoint:
	    StringStamped StrMsg_stopjoint = new StringStamped();
	    StrMsg_stopjoint.header.stamp = Time.fromMillis(System.currentTimeMillis());
	    StrMsg_stopjoint.data = "StopJoint";
	    EmergencyStopPub.publish( StrMsg_stopjoint );
	    safeToastStatus("tasks: EmergencyStopJoint");
	    Log.i("JskAndroidGui:ItemSeleted", "Sending EmergencyStopJoint");
	    return true;
	case R.id.stopnavigation:
	    StringStamped StrMsg_stopnavigation = new StringStamped();
	    StrMsg_stopnavigation.header.stamp = Time.fromMillis(System.currentTimeMillis());
	    StrMsg_stopnavigation.data = "StopNavigation";
	    EmergencyStopPub.publish( StrMsg_stopnavigation );
	    safeToastStatus("tasks: EmergencyStopNavigation");
	    Log.i("JskAndroidGui:ItemSeleted", "Sending EmergencyStopNavigation");
	    return true;
	case R.id.resetcollider:
	    cameraView.SetResetCollider();
	    return true;
	default:
	    return super.onOptionsItemSelected(item);
	}
    }

    protected void GetParamAndSetSpinner() {
	// tasks_list.clear(); spots_list.clear();
	// camera_list.clear(); points_list.clear();

	ArrayAdapter<String> adapter_spots = new ArrayAdapter<String>(this, R.layout.list);
	adapter_spots.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
	for (int i = 0; i <= spots_list.size() - 1; i++) {
	    adapter_spots.add(spots_list.get(i));
	}
	spots_spinner.setAdapter(adapter_spots);

	ArrayAdapter<String> adapter_tasks = new ArrayAdapter<String>(this, R.layout.list);
	adapter_tasks.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
	for (int i = 0; i <= tasks_list.size() - 1; i++) {
	    adapter_tasks.add(tasks_list.get(i));
	}
	tasks_spinner.setAdapter(adapter_tasks);

	ArrayAdapter<String> adapter_image = new ArrayAdapter<String>(this, R.layout.list);
	adapter_image.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
	for (int i = 0; i <= camera_list.size() - 1; i++) {
	    adapter_image.add(camera_list.get(i));
	}
	image_spinner.setAdapter(adapter_image);

	ArrayAdapter<String> adapter_points = new ArrayAdapter<String>(this, R.layout.list);
	adapter_points.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
	for (int i = 0; i <= points_list.size() - 1; i++) {
	    adapter_points.add(points_list.get(i));
	}
	points_spinner.setAdapter(adapter_points);
    }
}
