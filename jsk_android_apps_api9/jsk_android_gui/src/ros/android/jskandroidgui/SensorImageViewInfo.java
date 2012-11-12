package ros.android.jskandroidgui;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.graphics.Color;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.view.MotionEvent;
import android.widget.Toast;
import android.widget.ImageView;

import org.ros.node.Node;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.exception.RosException;
import org.ros.message.Time;
import org.ros.message.Message;
import org.ros.message.MessageListener;
import org.ros.message.sensor_msgs.CompressedImage;
import org.ros.message.geometry_msgs.PointStamped;
import org.ros.message.jsk_gui_msgs.Tablet;
import org.ros.message.jsk_gui_msgs.Touch;
import org.ros.message.jsk_gui_msgs.Action;

import ros.android.activity.RosAppActivity;

import java.util.ArrayList;

public class SensorImageViewInfo extends ImageView implements MessageListener<CompressedImage>, Runnable {

    static{
    	System.loadLibrary("calculate");
    }
    public native long translateBGRtoRGB(int[] src,int width,int height);

    private final int minLength = 30, SWIPE_WAIT_TIME = 1, SWIPE_NONE = 0,
	SWIPE_UP = 1, SWIPE_DOWN = 2, SWIPE_RIGHT = 3, SWIPE_LEFT = 4;
    private final float DefaultHeight = 480F, DefaultWidth = 640F; //

    private Bitmap bitmap,bitmap_tmp;
    private boolean isDrawLine = false, isMovingFingerInfo = false, isPushOnce = false, isPickOnce = false, isPlaceOnce = false, isPassToHumanOnce = false;
    private int count = 0, debug_count = 0, fingerCount = 0,
	fingerCountOver = 0, SwipeCounter = 0, SwipeDetectedType,
	RobotArmId = Action.LARMID, TouchMode = 1;
    private float MaxHeight, MaxWidth;
    private ArrayList<Integer> startXList = new ArrayList(), startYList = new ArrayList(), curXList = new ArrayList(), curYList = new ArrayList(), fingerList = new ArrayList();

    private Publisher<Tablet> TabletCommandPub;
    // we separate the publisher that uses jsk_pcl_ros
    private Publisher<Tablet> TabletTaskPub;
    private Publisher<Tablet> TabletPubDebug;
    private Subscriber<CompressedImage> imageSub;

    public SensorImageViewInfo(Context ctx) {
	super(ctx);
    }
    public SensorImageViewInfo(Context context, AttributeSet attrs, int defStyle) {
	super(context, attrs, defStyle);
    }
    public SensorImageViewInfo(Context context, AttributeSet attrs) {
	super(context, attrs);
    }
    public void start(Node node, String topic) throws RosException {
	ResetValue();
	imageSub = node.newSubscriber(topic, "sensor_msgs/CompressedImage", this);
	TabletCommandPub =
	    node.newPublisher( "/Tablet/Command" , "jsk_gui_msgs/Tablet" );
	TabletTaskPub =
	    node.newPublisher( "/Tablet/Task" , "jsk_gui_msgs/Tablet" );
	TabletPubDebug =
	    node.newPublisher( "/Tablet/CommandDebug" , "jsk_gui_msgs/Tablet" );
	MaxWidth = this.getWidth();
	MaxHeight = this.getHeight();
    }
    public void stop() {
	if (imageSub != null) imageSub.shutdown();
	imageSub = null;
    }
    @Override
    public void onNewMessage(CompressedImage message) {
	bitmap = BitmapFactory.decodeByteArray(message.data, 0, message.data.length);
	post(this);
    }

    public void SetSwipeDetected (int SwipeType) {SwipeDetectedType = SwipeType; SwipeCounter = 0;}
    public void unSetSwipeDetected () {SwipeDetectedType = SWIPE_NONE; SwipeCounter = 0;}
    public void SetRobotArm (int armid) {RobotArmId = armid;}
    public void SetDrawLine () {isDrawLine = true;}
    public void unSetDrawLine () {isDrawLine = false;}
    public void SetMovingFingerInfo () {isMovingFingerInfo = true;}
    public void unSetMovingFingerInfo () {isMovingFingerInfo = false;}
    public void SetPushOnce () {isPushOnce = true;}
    public void SetPlaceOnce () {isPlaceOnce = true;}
    public void SetPickOnce () {isPickOnce = true;}
    public void SetPassToHumanOnce () {isPassToHumanOnce = true;}
    public void ChangeTouchMode () {
	if (TouchMode == 0) {TouchMode++;}
	else if (TouchMode == 1) {TouchMode = 0;}
    }
    public void PubSwitchSensor (String str) {SendCommandMsg("SwitchSensor", 0, str, 0, null, 0, fingerList, startXList, startYList, 0, 0);}

    public void SetResetAll () {ResetValue(); isDrawLine = false; isMovingFingerInfo = false; SendCommandMsg("ResetAll", 0, "SengMsg", 0, null, 0, fingerList, startXList, startYList, 0, 0);}

    public void SendCommandMsg(String task_name, int arm_id, String state, float state_value, String direction, float direction_value, ArrayList<Integer> fingerList, ArrayList<Integer> xList, ArrayList<Integer> yList, int touch_x, int touch_y) {
	Log.v("JskAndroidGui:SendCommandMsg","START");
	Tablet TabletMsg = new Tablet();
	TabletMsg.header.seq = count++; //TabletMsg.header.frame_id = "";
	TabletMsg.header.stamp = Time.fromMillis(System.currentTimeMillis());
	TabletMsg.hardware_name = "Android"; // Get From hardware?
	TabletMsg.hardware_id = "JSK Acer";
	if( task_name != null ) TabletMsg.action.task_name = task_name;
	if( arm_id != 0 ) TabletMsg.action.arm_id = arm_id;
	if( state != null ) TabletMsg.action.state = state;
	if( state_value != 0 ) TabletMsg.action.state_value = state_value;
	if( direction != null ) TabletMsg.action.direction = direction;
	if( direction_value != 0 ) TabletMsg.action.direction_value = direction_value;
	TabletMsg.touches.ensureCapacity(xList.size());
	for (int i = 0; i <= xList.size() - 1; i++) {
	    Touch TouchMsg = new Touch();
	    TouchMsg.finger_id = fingerList.get(i);
	    TouchMsg.x = xList.get(i); TouchMsg.y = yList.get(i);
	    TabletMsg.touches.add( TouchMsg );
	}
	if( touch_x != 0 ) TabletMsg.action.touch_x = touch_x;
	if( touch_y != 0 ) TabletMsg.action.touch_y = touch_y;
	TabletCommandPub.publish( TabletMsg );
	Log.v("JskAndroidGui:SendCommandMsg","END");
    }

    public void SendTaskMsg(String task_name, int arm_id, String state, float state_value, String direction, float direction_value, ArrayList<Integer> fingerList, ArrayList<Integer> xList, ArrayList<Integer> yList, int touch_x, int touch_y) {
	Log.v("JskAndroidGui:SendTaskMsg","START");
	Tablet TabletMsg = new Tablet();
	TabletMsg.header.seq = count++;
	TabletMsg.header.stamp = Time.fromMillis(System.currentTimeMillis());
	TabletMsg.hardware_name = "Android";
	TabletMsg.hardware_id = "JSK Acer";
	if( task_name != null ) TabletMsg.action.task_name = task_name;
	if( arm_id != 0 ) TabletMsg.action.arm_id = arm_id;
	if( state != null ) TabletMsg.action.state = state;
	if( state_value != 0 ) TabletMsg.action.state_value = state_value;
	if( direction != null ) TabletMsg.action.direction = direction;
	if( direction_value != 0 ) TabletMsg.action.direction_value = direction_value;
	TabletMsg.touches.ensureCapacity(xList.size());
	for (int i = 0; i <= xList.size() - 1; i++) {
	    Touch TouchMsg = new Touch();
	    TouchMsg.finger_id = fingerList.get(i);
	    TouchMsg.x = xList.get(i); TouchMsg.y = yList.get(i);
	    TabletMsg.touches.add( TouchMsg );
	}
	if( touch_x != 0 ) TabletMsg.action.touch_x = touch_x;
	if( touch_y != 0 ) TabletMsg.action.touch_y = touch_y;
	TabletTaskPub.publish( TabletMsg );
	Log.v("JskAndroidGui:SendTaskMsg","END");
    }

    public void SendDebugMsg(String state, float state_value, int x, int y) {
	Log.v("JskAndroidGui:SendDebugMsg","START");
	Tablet DebugMsg = new Tablet();
	DebugMsg.header.seq = debug_count++;
	DebugMsg.header.stamp = Time.fromMillis(System.currentTimeMillis());
	DebugMsg.action.state = state;
	DebugMsg.action.state_value = state_value;
	DebugMsg.touches.ensureCapacity(1);
	Touch TouchMsg = new Touch();
	TouchMsg.finger_id = 0;
	TouchMsg.x = x; TouchMsg.y = y;
	DebugMsg.touches.add( TouchMsg );
	TabletPubDebug.publish( DebugMsg );
	Log.v("JskAndroidGui:SendDebugMsg","END");
    }

    public void MoveNeck(String direction, float direction_value) {
	//safeToastStatus("neck moving: " + direction + " ");
	SendCommandMsg("MoveNeck", 0, "SWIPE", 0, direction, direction_value, fingerList, startXList, startYList, 0, 0);
    }

    public void ZoomCamera(int touch_x, int touch_y, float movement) {
	SendCommandMsg("ZoomCamera", 0, "PINCH", movement, null, 0, fingerList, startXList, startYList, touch_x, touch_y);
    }

    public void SendOpenDoorMsg () {
	SendCommandMsg("OpenDoor", RobotArmId, "SendMsg", 0, null, 0, fingerList, startXList, startYList, 0, 0);
	unSetMovingFingerInfo();
    }

    public void SendCloseDoorMsg () {
	SendCommandMsg("CloseDoor", RobotArmId, "SendMsg", 0, null, 0, fingerList, startXList, startYList, 0, 0);
	unSetMovingFingerInfo();
    }

    public void SendTuckArmPoseMsg () {
	SendCommandMsg("TuckArmPose", RobotArmId, "SendMsg", 0, null, 0, fingerList, startXList, startYList, 0, 0);
    }

    public void SendTorsoUpMsg () {
	SendCommandMsg("TorsoUp", RobotArmId, "SendMsg", 0, null, 0, fingerList, startXList, startYList, 0, 0);
    }

    public void SendTorsoDownMsg () {
	SendCommandMsg("TorsoDown", RobotArmId, "SendMsg", 0, null, 0, fingerList, startXList, startYList, 0, 0);
    }

    public void SendOpenGripperMsg () {
	SendCommandMsg("OpenGripper", RobotArmId, "SendMsg", 0, null, 0, fingerList, startXList, startYList, 0, 0);
    }

    public void SendCloseGripperMsg () {
	SendCommandMsg("CloseGripper", RobotArmId, "SendMsg", 0, null, 0, fingerList, startXList, startYList, 0, 0);
    }

    public void ResetValue () {
	fingerCountOver = 0;
	fingerList.clear(); startXList.clear(); startYList.clear();
	curXList.clear(); curYList.clear();
    }

    public boolean isClockWise(int p1x, int p1y, int p2x, int p2y, int p3x, int p3y) {
	//int dx2=p2x - p1x, dy2=p2y - p1y, dx3=p3x - p1x, dy3=p3y - p1y;
	//if ( ( dx2 * dy3 ) >= ( dx3 * dy2 ) ) return false;
	if ( ((p2x - p1x) * (p3y - p1y)) >= ((p3x - p1x) * (p2y - p1y)) )
	    return false;
	//	else if ( ( dx2 * dy3 ) < ( dx3 * dy2 ) ) return true;
	return true;
    }


    @Override
	public boolean onTouchEvent(MotionEvent event) {
	//this.invalidate();

	final int action = event.getAction();
	final int fingerCount_temp = event.getPointerCount();
	Log.i("JskAndroidGui:TouchEvent","START, Action is = " + action);
	Log.i("JskAndroidGui:TouchEvent","current fingerCount = " + fingerCount + ", current fingerCount_temp = " + fingerCount_temp);

	switch ( action & MotionEvent.ACTION_MASK) {
	    /* ACTION_DOWN */
	case MotionEvent.ACTION_DOWN:{
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_DOWN] START");
	    fingerCount = event.getPointerCount();
	    if (fingerCount > 0) { //fingerCount is always 1 in this case
		startXList.clear(); startYList.clear();
		startXList.ensureCapacity(fingerCount);
		startYList.ensureCapacity(fingerCount);

		startXList.add((int)(event.getX() * DefaultWidth / MaxWidth));
		startYList.add((int)(event.getY() * DefaultWidth / MaxWidth));
		Log.i("JskAndroidGui:TouchEvent","[ACTION_DOWN] startXList = " + startXList + ", startYList = " + startYList);
		SendDebugMsg("ActionDown", fingerCount, startXList.get(0), startYList.get(0));

		Log.i("JskAndroidGui:TouchEvent","[ACTION_DOWN] END");
	    break;
	    }
	}
	    /* ACTION_DOWN more than two fingers */
	case MotionEvent.ACTION_POINTER_DOWN:{
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_DOWN] START");
	    fingerCount = event.getPointerCount();
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_DOWN] current fingerCount = " + fingerCount);

	    startXList.add((int)(event.getX(fingerCount - 1) * DefaultWidth / MaxWidth));
	    startYList.add((int)(event.getY(fingerCount - 1) * DefaultWidth / MaxWidth));

	    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_DOWN] startXList = " + startXList + ", startYList = " + startYList);
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_DOWN] END");
	}
	    /* ACTION_MOVE */
	case MotionEvent.ACTION_MOVE:{
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_MOVE] START");
	    if ( fingerCount > 0 ) {
		curXList.clear(); curYList.clear();
		fingerList.ensureCapacity(100);
		curXList.ensureCapacity(100);
		curYList.ensureCapacity(100);

		if( fingerCount == 1 ) {
		    fingerList.add(0);
		    curXList.add((int)(event.getX() * DefaultWidth / MaxWidth));
		    curYList.add((int)(event.getY() * DefaultWidth / MaxWidth));
		} else {
		    for (int i = 0; i <= fingerCount_temp - 1; i++) {
			fingerList.add(i);
			curXList.add((int)(event.getX(i) * DefaultWidth / MaxWidth));
			curYList.add((int)(event.getY(i) * DefaultWidth / MaxWidth));
		    }
		}
	    }
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_MOVE] curXList = " + curXList + ", curYList = " + curYList);
	    SendDebugMsg("ActionMove", fingerCount, curXList.get(0), curYList.get(0));
	    if ( isDrawLine && fingerCount == 2 && fingerCount_temp == 2) {
		SendTaskMsg("DrawLine", 0, "TOUCHMOVE", 0, null, 0,
			fingerList, curXList, curYList, 0, 0);
	    } else if (isMovingFingerInfo) {
	    	SendTaskMsg("MovingPointInfo", 0, "TOUCHMOVE", 0, null, 0,
			    fingerList, curXList, curYList, 0, 0);
	    }

	    Log.i("JskAndroidGui:TouchEvent","[ACTION_MOVE] END");
	    break;
	}
	    /* ACTION_UP */
	case MotionEvent.ACTION_UP:{
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_UP] START");
	    // if ( fingerCount == 1 && isMovingFingerInfo) {
	    // 	SendTaskMsg("OpenDoor", RobotArmID, "TOUCH", 0, null, 0,
	    // 		    fingerList, startXList, startYList, 0, 0);
	    // 	unSetMovingFingerInfo();
	    // } else
	    if ( isMovingFingerInfo ) {
		return true;
	    }

	    if ( fingerCount == 1 && curXList.size() == 0 ) {
		fingerList.add(0);
		if (isPushOnce) {
		    isPushOnce = false;
		    SendCommandMsg("PushOnce", 0, "TOUCH", 0, null, 0,
				   fingerList, startXList, startYList, startXList.get(0), startYList.get(0));
		} else if (isPickOnce) {
		    isPickOnce = false;
		    //SendCommandMsg("PickOnce", 0, "TOUCH", 0, null, 0,
		    SendCommandMsg("PickObjectSelected", RobotArmId, "TOUCH", 0, null, 0,
				   fingerList, startXList, startYList, startXList.get(0), startYList.get(0));
		} else if (isPlaceOnce) {
		    isPlaceOnce = false;
		    SendCommandMsg("PlaceOnce", 0, "TOUCH", 0, null, 0,
				   fingerList, startXList, startYList, startXList.get(0), startYList.get(0));
		} else if (isPassToHumanOnce) {
		    isPassToHumanOnce = false;
		    SendCommandMsg("PassToHumanOnce", RobotArmId, "TOUCH", 0, null, 0,
				   fingerList, startXList, startYList, startXList.get(0), startYList.get(0));
		} else if (TouchMode == 0) {
		    SendCommandMsg("MoveCameraCenter", 0, "TOUCH", 0, null, 0,
				   fingerList, startXList, startYList, startXList.get(0), startYList.get(0));
		} else if (TouchMode == 1) {
		    SendCommandMsg("Show3DScreenPoint", 0, "TOUCH", 0, null, 0,
				   fingerList, startXList, startYList, startXList.get(0), startYList.get(0));
		}
		SendDebugMsg("TOUCH", 0, startXList.get(0), startYList.get(0));
	    } else if ( fingerCount == 1 && curXList.size() > 0 ) {
		final float swipeLength =
		    Math.round(Math.sqrt(Math.pow(curXList.get(curXList.size() - 1) - startXList.get(0) , 2)
					 + Math.pow(curYList.get(curYList.size() - 1) - startYList.get(0) , 2)));//

		if ( swipeLength >= minLength ) {
		    SendDebugMsg("SWIPE", swipeLength, curXList.get(0), curYList.get(0));
		    final int X = startXList.get(0) - curXList.get(curXList.size() - 1);
		    final int Y = curYList.get(curYList.size() - 1) - startYList.get(0);
		    final double r = Math.atan2(Y, X);
		    float swipeAngle = Math.round( r * 180 / Math.PI);
		    if ( swipeAngle < 0 ) swipeAngle =  360 - Math.abs(swipeAngle);
		    if ( (swipeAngle <= 45) && (swipeAngle >= 0) ) {
			MoveNeck("left", swipeLength);
			SetSwipeDetected(SWIPE_LEFT);
		    } else if ( (swipeAngle <= 360) && (swipeAngle >= 315) ) {
			MoveNeck("left", swipeLength);
			SetSwipeDetected(SWIPE_LEFT);
		    } else if ( (swipeAngle >= 135) && (swipeAngle <= 225) ) {
			MoveNeck("right", swipeLength);
			SetSwipeDetected(SWIPE_RIGHT);
		    } else if ( (swipeAngle > 45) && (swipeAngle < 135) ) {
			MoveNeck("down", swipeLength);
			SetSwipeDetected(SWIPE_DOWN);
		    } else {
			MoveNeck("up", swipeLength);
			SetSwipeDetected(SWIPE_UP);
		    }
		}
	    } else if ( fingerCount == 2 && curXList.size() > 0 && fingerCountOver == 0 ) {
		/* if fingercount == 2, startlist.size() = 2, curlist.size  = 1or2 */
	    	final float startDistance =
		    Math.round(Math.sqrt(Math.pow(startXList.get(1) - startXList.get(0),2)
					 + Math.pow(startYList.get(1) - startYList.get(0),2)));
		final float endDistance =
		    Math.round(Math.sqrt(Math.pow(curXList.get(curXList.size() - 1) - curXList.get(0),2)
					 + Math.pow(curYList.get(curYList.size() -1 ) - curYList.get(0),2)));
	    	final float pinchLength = startDistance - endDistance;

		final int touch_x = (curXList.get(0) + curXList.get(curXList.size() - 1))/2;
		final int touch_y = (curYList.get(0) + curYList.get(curYList.size() - 1))/2;
		SendDebugMsg("PINCH", pinchLength, touch_x, touch_y);
	    	if ( false ) {// get camera id?
	    	    ZoomCamera((startXList.get(0) + startXList.get(1))/2,
			       (startYList.get(0) + startYList.get(1))/2, pinchLength);
	    	} else {
		    SendCommandMsg("pick", RobotArmId, "PINCH", 0, null, 0,
			    fingerList, curXList, curYList, touch_x, touch_y);
		}
	    } else if ( fingerCount == 4 ) {
		if (startYList.get(0) >= DefaultHeight / 2 &&
		    DefaultHeight / 2 >= curYList.get(0)) {
		    SendCommandMsg("TorsoUp", 0, "TOUCH", 0, null, 0,
			    fingerList, startXList, startYList, 0, 0);
		} else if (startYList.get(0) <= DefaultHeight / 2 &&
			   DefaultHeight / 2 <= curYList.get(0)) {
		    SendCommandMsg("TorsoDown", 0, "TOUCH", 0, null, 0,
			    fingerList, startXList, startYList, 0, 0);
		} else {
		    SendCommandMsg("StopMotion", 0, "TOUCH", 0, null, 0,
			    fingerList, startXList, startYList, 0, 0);
		}
	    } else if ( fingerCount == 5 && (Math.abs(startXList.get(0) - curXList.get(0)) > minLength) ) {
		SendCommandMsg("TuckArmPose", RobotArmId, "TOUCH", 0, null, 0,
			fingerList, startXList, startYList, 0, 0);
	    }

	    ResetValue();
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_UP] END, ResetValue();");
	    break;
	}
	    /* ACTION_UP more than two fingers */
	case MotionEvent.ACTION_POINTER_UP:{
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] START");
	    Log.v("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] fingerCount = " + fingerCount + ", fingerCount_temp = " + fingerCount_temp);
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] isMovingFingerInfo = " + isMovingFingerInfo);
	    if ( fingerCount_temp == 2 && fingerCount == 2 && isMovingFingerInfo) {
		int p1x,p1y,p2x,p2y,p3x,p3y;
		if ( Math.abs(startXList.get(0) - curXList.get(0)) < minLength ){
		    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] true");
		    p1x = startXList.get(0); p1y = startYList.get(0);
		    p2x = startXList.get(1); p2y = startYList.get(1);
		    p3x = curXList.get(1); p3y = startYList.get(1);
		} else {
		    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] false");
		    p1x = startXList.get(1); p1y = startYList.get(1);
		    p2x = startXList.get(0); p2y = startYList.get(0);
		    p3x = curXList.get(0); p3y = startYList.get(0);
		}
		if ( isClockWise(p1x, p1y, p2x, p2y, p3x, p3y) ) {
		    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] 45");
		    SendCommandMsg("RotateGripper", RobotArmId, "PINCH", 45, null, 0,
				   fingerList, startXList, startYList, 0, 0);
		} else {
		    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] -45");
		    SendCommandMsg("RotateGripper", RobotArmId, "PINCH", -45, null, 0,
				   fingerList, startXList, startYList, 0, 0);
		}
	    } else if ( fingerCount_temp == 3 && fingerCount == 3 ) {
		Log.v("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] startXList = " + startXList + ", startYList = " + startYList);
		Log.v("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] curXList = " + curXList + ", curYList = " + curYList);

		//TODO: estimate open or slide ?
		SendTaskMsg("OpenDoorInput", RobotArmId, "TOUCH", 0, null, 0,
			    fingerList, startXList, startYList, 0, 0);
		unSetDrawLine();
		SetMovingFingerInfo();
	    }

	    Log.i("JskAndroidGui:TouchEvent","[ACTION_POINTER_UP] END");
	}
	    /* ACTION_CANCEL , called with ACTION_POINTER_UP */
	case MotionEvent.ACTION_CANCEL:{
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_CANCEL] START");
	    Log.i("JskAndroidGui:TouchEvent","[ACTION_CANCEL] END");
	    break;
	}
	}
	Log.i("JskAndroidGui:TouchEvent","END");
	return true;
    }

    @Override
	public void run() {
	final int bWidth = bitmap.getWidth();
	final int bHeight = bitmap.getHeight();
	Bitmap bitmap_tmp = Bitmap.createBitmap(bWidth, bHeight, Bitmap.Config.ARGB_8888);
	int[] bpixels = new int[bWidth*bHeight];
	bitmap.getPixels(bpixels, 0, bWidth, 0, 0, bWidth, bHeight);
	translateBGRtoRGB(bpixels,bWidth,bHeight);
	bitmap_tmp.setPixels(bpixels, 0, bWidth, 0, 0, bWidth, bHeight);

	if (SwipeDetectedType != SWIPE_NONE && SwipeCounter < SWIPE_WAIT_TIME ){
	    Log.v("JskAndroidGui:onNewMessage","[changing Bitmap] changing now " + (SwipeCounter + 1) + " / " + SWIPE_WAIT_TIME);
	    this.invalidate();
	    SwipeCounter++;
	    final int Width = bitmap_tmp.getWidth();
	    final int Height = bitmap_tmp.getHeight();
	    //final int Width = bitmap.getWidth();
	    //final int Height = bitmap.getHeight();
	    Bitmap bitmap_output = Bitmap.createBitmap(Width, Height, Bitmap.Config.ARGB_8888);
	    int[] pixels = new int[Width*Height];
	    //bitmap.getPixels(pixels, 0, Width, 0, 0, Width, Height);
	    bitmap_tmp.getPixels(pixels, 0, Width, 0, 0, Width, Height);
	    for(int x = 0; x < Width; x++){
		for(int y = 0; y < Height; y++){
		    // int pixel = pixels[x + y * Width];
		    if ( (SwipeDetectedType == SWIPE_UP && (-10 * x + y > 0  || 10 * Width  < 10 * x + y )) ||
			 (SwipeDetectedType == SWIPE_DOWN && (10 * x + y < Height  || 10 * Width - Height  < 10 * x - y )) ||
			 (SwipeDetectedType == SWIPE_LEFT && (-1 * x + 10 * y < 0  || 10 * Height < x + 10 * y )) ||
			 (SwipeDetectedType == SWIPE_RIGHT && (x + 10 * y < Width  || -1 * Width + 10 * Height < -1 * x + 10 * y )) ) {
			pixels[x + y * Width] = Color.rgb(Color.BLACK, Color.BLACK, Color.BLACK);
		    }
		}
	    }
	    bitmap_output.setPixels(pixels, 0, Width, 0, 0, Width, Height);
	    setImageBitmap(bitmap_output);

	    // Matrix matrix = new Matrix(); matrix.postSkew(skew_x, skew_y);
	    // final Bitmap bitmap_output = Bitmap.createBitmap(bitmap2, 0, 0, w, h, matrix, true);
	} else if (SwipeCounter >= SWIPE_WAIT_TIME){
	    Log.v("JskAndroidGui:onNewMessage","[changing Bitmap] back to normal");
	    unSetSwipeDetected();
	} else {
	    //this.invalidate();
	    setImageBitmap(bitmap_tmp);
	}
    }
}
