package ros.android.jskandroidgui;

import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.MotionEvent;
import android.widget.Toast;
import android.hardware.SensorManager;

import org.ros.node.Node;
import org.ros.namespace.NameResolver;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.exception.RosException;

import org.ros.message.std_msgs.Empty;
import org.ros.message.jsk_gui_msgs.Action;

import ros.android.views.JoystickView;
import ros.android.activity.RosAppActivity;



/**
 * @author chen@jsk.t.u-tokyo.ac.jp (Haseru Azuma)
 */

public class JskAndroidGui extends RosAppActivity {
    private boolean isDrawLine = false;
    private String robotAppName;
    private String cameraTopic;
    private SensorImageViewInfo cameraView;
    private JoystickView joystickView;
    private Publisher<Empty> GetSpotPub;

    @Override
	public void onCreate(Bundle savedInstanceState) {
	setDefaultAppName("jsk_gui/jsk_android_gui");
	setDashboardResource(R.id.top_bar);
	setMainWindowResource(R.layout.main);
	super.onCreate(savedInstanceState);

	if (getIntent().hasExtra("camera_topic")) {
	    cameraTopic = getIntent().getStringExtra("camera_topic");
	} else {
	    cameraTopic = "/camera/rgb/image_rect_color/compressed_throttle";
	}
	joystickView = (JoystickView) findViewById(R.id.joystick);
	joystickView.setBaseControlTopic("android/cmd_vel");
	cameraView = (SensorImageViewInfo) findViewById(R.id.image);
	cameraView.setClickable(true);
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
			cameraView.setSelected(true);
		    }
		});
	    joystickView.start(node);
	} catch (Exception ex) {
	    Log.e("JskAndroidGui", "Init error: " + ex.toString());
	    safeToastStatus("Failed: " + ex.getMessage());
	}

	GetSpotPub =
	    node.newPublisher( "/Tablet/GetSpot" , "std_msgs/Empty" );
    }

    @Override
	protected void onNodeDestroy(Node node) {
	super.onNodeDestroy(node);
    }

    @Override
	public boolean onCreateOptionsMenu(Menu menu) {
	MenuInflater inflater = getMenuInflater();
	inflater.inflate(R.menu.jsk_android_gui, menu);
	return true;
    }

    @Override
	public boolean onOptionsItemSelected(MenuItem item) {
	switch (item.getItemId()) {
	case R.id.kill:
	    android.os.Process.killProcess(android.os.Process.myPid());
	    return true;
	case R.id.getspot:
	    Empty EmptyMsg = new Empty();
	    GetSpotPub.publish( EmptyMsg );
	    Log.i("JskAndroidGui:ItemSeleted", "Sending GetSpot messgae");
	    return true;
	case R.id.setrarm:
	    cameraView.SetRobotArm(Action.RARMID);
	    Log.i("JskAndroidGui:ItemSeleted", "Set arm to :rarm");
	    return true;
	case R.id.setlarm:
	    cameraView.SetRobotArm(Action.LARMID);
	    Log.i("JskAndroidGui:ItemSeleted", "Set arm to :larm");
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
	case R.id.opendoor:
	    cameraView.unSetMovingFingerInfo();
	    cameraView.SendOpenDoorMsg();
	    Log.i("JskAndroidGui:ItemSeleted", "Send OpenDoorMsg");
	    return true;
	case R.id.opengripper:
	    cameraView.SendOpenGripperMsg();
	    Log.i("JskAndroidGui:ItemSeleted", "Send OpenGripperMsg");
	    return true;
	case R.id.closegripper:
	    cameraView.SendCloseGripperMsg();
	    Log.i("JskAndroidGui:ItemSeleted", "Send CloseGripperMsg");
	    return true;
	case R.id.resetall:
	    cameraView.SetResetAll();
	    isDrawLine = false;
	    Log.i("JskAndroidGui:ItemSeleted", "Set ResetAll");
	    return true;
	default:
	    return super.onOptionsItemSelected(item);
	}
    }
}
