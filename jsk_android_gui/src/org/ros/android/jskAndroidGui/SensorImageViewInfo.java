package org.ros.android.jskAndroidGui;

import java.io.IOException;
import java.util.ArrayList;

import jsk_gui_msgs.Action;
import jsk_gui_msgs.Tablet;
import jsk_gui_msgs.Touch;

import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.Bitmap;
import android.graphics.BitmapRegionDecoder;
import android.graphics.Color;
import android.graphics.Point;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Toast;

public class SensorImageViewInfo extends ImageView implements Runnable,
		NodeMain {

	// static{
	// System.loadLibrary("calculate");
	// }
	public native long translateBGRtoRGB(int[] src, int width, int height);

	private final int minLength = 30, SWIPE_WAIT_TIME = 1, SWIPE_NONE = 0,
			SWIPE_UP = 1, SWIPE_DOWN = 2, SWIPE_RIGHT = 3, SWIPE_LEFT = 4;
	private float imageHeight = 480F, imageWidth = 640F;
	private boolean decodeFlag = true;
	private Bitmap bitmap;
	private BitmapRegionDecoder bitmapRegionDecoder;
	private boolean isMovingFingerInfo = false, isPushOnce = false,
			isPickOnce = false, isPlaceOnce = false, isPassToHumanOnce = false,
			isGetTemplate = false, isZoomCamera = true,
			isCircularManipulation = false, isLinearManipulation = false;
	private boolean isDrawLine = false;
	private int count = 0, debug_count = 0, fingerCount = 0, SwipeCounter = 0,
			SwipeDetectedType, RobotArmId = Action.LARMID, TouchMode = 1;
	private float MaxHeight, MaxWidth;
	private float ROIoffsetX = 0, ROIoffsetY = 0, ROIRate = 1,
			startROIRate = 1, startROIcenterX, startROIcenterY;
	private ArrayList<Integer> startXList = new ArrayList(),
			startYList = new ArrayList(), lastXList = new ArrayList(),
			lastYList = new ArrayList(), fingerList = new ArrayList();
	// private ArrayList<Integer> last_startXList = new ArrayList(),
	// last_startYList = new ArrayList(), last_curXList = new ArrayList(),
	// last_curYList = new ArrayList(), last_fingerList = new ArrayList();
	private Integer start_x = 0, start_y = 0;
	private ArrayList<Point> pl = new ArrayList<Point>();
	private LayoutInflater inflater;

	private Publisher<Tablet> TabletCommandPub;
	// we separate the publisher that uses jsk_pcl_ros
	private Publisher<Tablet> TabletTaskPub;
	private Publisher<Tablet> TabletPubDebug;
	private Publisher<Action> TabletMoveIt;
	private Publisher<Tablet> TabletTemplateInfo;
	private Subscriber<CompressedImage> imageSub;
	private Subscriber<CameraInfo> caminfoSub;
	private String cameratopic, camerainfotopic;
	private CameraInfo caminfo;
	private ConnectedNode connectedNode;
	private NodeConfiguration nodeConfiguration;
	private MessageFactory messageFactory;
	private Thread thread = null;
	private boolean nodeStart = false;
	private BoundingBoxView boundingBoxView;
	private Context context;

	public SensorImageViewInfo(Context ctx) {
		super(ctx);
		context = ctx;
	}

	public SensorImageViewInfo(Context ctx, AttributeSet attrs, int defStyle) {
		super(ctx, attrs, defStyle);
		context = ctx;
	}

	public SensorImageViewInfo(Context ctx, AttributeSet attrs) {
		super(ctx, attrs);
		context = ctx;
	}

	public void setCameraTopic(String cameratopic) {
		this.cameratopic = cameratopic;
	}

	public void setCameraInfoTopic(String camerainfotopic) {
		this.camerainfotopic = camerainfotopic;
	}

	public void setView(BoundingBoxView bb) {
		boundingBoxView = bb;
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		this.connectedNode = connectedNode;
		this.nodeConfiguration = NodeConfiguration.newPrivate();
		this.messageFactory = nodeConfiguration.getTopicMessageFactory();
		ResetValue();
		imageSub = connectedNode.newSubscriber(cameratopic,
				sensor_msgs.CompressedImage._TYPE);
		caminfoSub = connectedNode.newSubscriber(camerainfotopic,
				sensor_msgs.CameraInfo._TYPE);
		TabletCommandPub = connectedNode.newPublisher("/Tablet/Command",
				jsk_gui_msgs.Tablet._TYPE);
		TabletTaskPub = connectedNode.newPublisher("/Tablet/Task",
				jsk_gui_msgs.Tablet._TYPE);
		TabletPubDebug = connectedNode.newPublisher("/Tablet/CommandDebug",
				jsk_gui_msgs.Tablet._TYPE);
		TabletMoveIt = connectedNode.newPublisher("/Tablet/MoveIt",
				jsk_gui_msgs.Action._TYPE);
		TabletTemplateInfo = connectedNode.newPublisher("/Tablet/TemplateInfo",
				jsk_gui_msgs.Tablet._TYPE);
		MaxWidth = this.getWidth();
		MaxHeight = this.getHeight();
		imageSub.addMessageListener(new MessageListener<CompressedImage>() {
			@Override
			public void onNewMessage(CompressedImage message) {
				try {
					bitmapRegionDecoder = BitmapRegionDecoder.newInstance(
							message.getData().array(), message.getData()
									.arrayOffset(), message.getData()
									.readableBytes(), false);
				} catch (IOException e) {
					Log.e("JskAndroidGui", "failed to bitmap region decoder");
					e.printStackTrace();
					return;
				}
				setRectBitmap();
				post(SensorImageViewInfo.this);
			}
		});
		caminfoSub.addMessageListener(new MessageListener<CameraInfo>() {
			@Override
			public void onNewMessage(CameraInfo message) {
				caminfo = message;
				imageHeight = caminfo.getHeight();
				imageWidth = caminfo.getWidth();
				post(SensorImageViewInfo.this);
			}
		});
		nodeStart = true;
	}

	public void stop() {
		if (imageSub != null)
			imageSub.shutdown();
		imageSub = null;
		if (caminfoSub != null)
			caminfoSub.shutdown();
		caminfoSub = null;
	}

	public void SetSwipeDetected(int SwipeType) {
		SwipeDetectedType = SwipeType;
		SwipeCounter = 0;
	}

	public void unSetSwipeDetected() {
		SwipeDetectedType = SWIPE_NONE;
		SwipeCounter = 0;
	}

	public void SetRobotArm(int armid) {
		RobotArmId = armid;
	}

	public void PublishPushOnce() {
		SendCommandMsg("PushOnce", RobotArmId, "TOUCH", 0, null, 0, fingerList,
				startXList, startYList, start_x, start_y);
	}

	public void PublishPickOnce() {
		SendCommandMsg("PickObjectSelected", RobotArmId, "TOUCH", 0, null, 0,
				fingerList, startXList, startYList, start_x, start_y);
	}

	public void PublishPlaceOnce() {
		SendCommandMsg("PlaceObjectSelected", RobotArmId, "TOUCH", 0, null, 0,
				fingerList, startXList, startYList, start_x, start_y);
	}

	public void PublishGetTemplateOnce() {
		SendCommandMsg("GetTemplate", RobotArmId, "TOUCH", 0, null, 0,
				fingerList, startXList, startYList, start_x, start_y);
	}

	public void SetDrawLine() {
		isDrawLine = true;
	}

	public void unSetDrawLine() {
		isDrawLine = false;
	}

	public void SetCircularManipulation() {
		isCircularManipulation = true;
	}

	public void unSetCircularManipulation() {
		isCircularManipulation = false;
	}

	public void SetLinearManipulation() {
		isLinearManipulation = true;
	}

	public void unSetLinearManipulation() {
		isLinearManipulation = false;
	}

	public void CancelManipulation() {
		unSetCircularManipulation();
		unSetLinearManipulation();
		SendTaskMsg("CancelManipulation", RobotArmId, "", 0, null, 0,
				fingerList, lastXList, lastYList, 0, 0);
	}

	public void SetMovingFingerInfo() {
		isMovingFingerInfo = true;
	}

	public void unSetMovingFingerInfo() {
		isMovingFingerInfo = false;
	}

	public void SetGetTemplate() {
		isGetTemplate = true;
	}

	public void unSetGetTemplate() {
		isGetTemplate = false;
	}

	public void SetZoomCamera() {
		isZoomCamera = true;
	}

	public void unSetZoomCamera() {
		isZoomCamera = false;
	}

	public void SetPushOnce() {
		isPushOnce = true;
	}

	public void SetPickOnce() {
		isPickOnce = true;
	}

	public void SetPlaceOnce() {
		isPlaceOnce = true;
	}

	public void SetPassToHumanOnce() {
		isPassToHumanOnce = true;
	}

	public void ChangeTouchMode() {
		if (TouchMode == 0) {
			TouchMode++;
		} else if (TouchMode == 1) {
			TouchMode = 0;
		}
	}

	public void PubSwitchSensor(String str) {
		SendCommandMsg("SwitchSensor", 0, str, 0, null, 0, fingerList,
				startXList, startYList, 0, 0);
		ResetROI();
	}

	public void ResetROI() {
		ROIRate = 1;
		ROIoffsetX = 0;
		ROIoffsetY = 0;
	}

	public void SetResetAll() {
		ResetValue();
		isDrawLine = false;
		isMovingFingerInfo = false;
		SendCommandMsg("ResetAll", 0, "SengMsg", 0, null, 0, fingerList,
				startXList, startYList, 0, 0);
	}

	public void SetResetCollider() {
		SendCommandMsg("ResetCollider", 0, "SengMsg", 0, null, 0, fingerList,
				startXList, startYList, 0, 0);
	}

	public void SendCommandMsg(String task_name, int arm_id, String state,
			float state_value, String direction, float direction_value,
			ArrayList<Integer> fingerList, ArrayList<Integer> xList,
			ArrayList<Integer> yList, int touch_x, int touch_y) {
		Log.v("JskAndroidGui:SendCommandMsg", "START");
		Tablet TabletMsg = TabletCommandPub.newMessage();
		TabletMsg.getHeader().setSeq(count++);
		// TabletMsg.getHeader().setFrameId("");
		TabletMsg.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		TabletMsg.setHardwareName("Android"); // Get From hardware?
		TabletMsg.setHardwareId("JSK Acer");
		if (task_name != null)
			TabletMsg.getAction().setTaskName(task_name);
		if (arm_id != 0)
			TabletMsg.getAction().setArmId(arm_id);
		if (state != null)
			TabletMsg.getAction().setState(state);
		if (state_value != 0)
			TabletMsg.getAction().setStateValue(state_value);
		if (direction != null)
			TabletMsg.getAction().setDirection(direction);
		if (direction_value != 0)
			TabletMsg.getAction().setDirectionValue(direction_value);

		for (int i = 0; i < xList.size(); ++i) {
			Touch TouchMsg = messageFactory.newFromType(Touch._TYPE);
			TouchMsg.setFingerId(fingerList.get(i));
			TouchMsg.setX(getImageCoordX(xList.get(i)));
			TouchMsg.setY(getImageCoordY(yList.get(i)));
			TabletMsg.getTouches().add(TouchMsg);
		}
		if (touch_x != 0)
			TabletMsg.getAction().setTouchX((int) getImageCoordX(touch_x));
		if (touch_y != 0)
			TabletMsg.getAction().setTouchY((int) getImageCoordY(touch_y));
		TabletCommandPub.publish(TabletMsg);
		Log.v("JskAndroidGui:SendCommandMsg", "END");
	}

	public void SendTaskMsg(String task_name, int arm_id, String state,
			float state_value, String direction, float direction_value,
			ArrayList<Integer> fingerList, ArrayList<Integer> xList,
			ArrayList<Integer> yList, int touch_x, int touch_y) {
		Log.v("JskAndroidGui:SendTaskMsg", "START");
		Tablet TabletMsg = TabletTaskPub.newMessage();
		TabletMsg.getHeader().setSeq(count++);
		TabletMsg.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		TabletMsg.setHardwareName("Android");
		TabletMsg.setHardwareId("JSK Acer");
		if (task_name != null)
			TabletMsg.getAction().setTaskName(task_name);
		if (arm_id != 0)
			TabletMsg.getAction().setArmId(arm_id);
		if (state != null)
			TabletMsg.getAction().setState(state);
		if (state_value != 0)
			TabletMsg.getAction().setStateValue(state_value);
		if (direction != null)
			TabletMsg.getAction().setDirection(direction);
		if (direction_value != 0)
			TabletMsg.getAction().setDirectionValue(direction_value);

		for (int i = 0; i < xList.size(); i++) {
			Touch TouchMsg = messageFactory.newFromType(Touch._TYPE);
			TouchMsg.setFingerId(fingerList.get(i));
			TouchMsg.setX(getImageCoordX(xList.get(i)));
			TouchMsg.setY(getImageCoordY(yList.get(i)));
			TabletMsg.getTouches().add(TouchMsg);
		}
		if (touch_x != 0)
			TabletMsg.getAction().setTouchX((int) getImageCoordX(touch_x));
		if (touch_y != 0)
			TabletMsg.getAction().setTouchY((int) getImageCoordY(touch_y));
		TabletTaskPub.publish(TabletMsg);
		Log.v("JskAndroidGui:SendTaskMsg", "END");
	}

	public void SendDebugMsg(String state, float state_value, int x, int y) {
		Log.v("JskAndroidGui:SendDebugMsg", "START");
		Tablet DebugMsg = TabletPubDebug.newMessage();
		DebugMsg.getHeader().setSeq(debug_count++);
		DebugMsg.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		DebugMsg.getAction().setState(state);
		DebugMsg.getAction().setStateValue(state_value);
		Touch TouchMsg = messageFactory.newFromType(Touch._TYPE);
		TouchMsg.setFingerId(0);
		TouchMsg.setX(getImageCoordX(x));
		TouchMsg.setY(getImageCoordY(y));
		DebugMsg.getTouches().add(TouchMsg);
		TabletPubDebug.publish(DebugMsg);
		Log.v("JskAndroidGui:SendDebugMsg", "END");
	}

	public void SendMoveItMsg(String task_name, String direction, int resolution) {
		Action moveItMsg = TabletMoveIt.newMessage();
		moveItMsg.setArmId(RobotArmId);
		moveItMsg.setTaskName(task_name);
		moveItMsg.setDirection(direction);
		moveItMsg.setDirectionValue((double) resolution);
		TabletMoveIt.publish(moveItMsg);
	}

	public void SendTemplateInfo(String name, int startX, int startY,
			int lastX, int lastY) {
		Tablet templateMsg = TabletTemplateInfo.newMessage();
		templateMsg.getAction().setTaskName(name);
		templateMsg.getAction().setArmId(RobotArmId);
		Touch startTouchMsg = messageFactory.newFromType(Touch._TYPE);
		Touch lastTouchMsg = messageFactory.newFromType(Touch._TYPE);
		startTouchMsg.setX(getImageCoordX(startX));
		startTouchMsg.setY(getImageCoordY(startY));
		lastTouchMsg.setX(getImageCoordX(lastX));
		lastTouchMsg.setY(getImageCoordY(lastY));
		templateMsg.getTouches().add(startTouchMsg);
		templateMsg.getTouches().add(lastTouchMsg);
		TabletTemplateInfo.publish(templateMsg);
	}

	public void MoveNeck(String direction, float direction_value) {
		// safeToastStatus("neck moving: " + direction + " ");
		SendCommandMsg("MoveNeck", RobotArmId, "SWIPE", 0, direction,
				direction_value, fingerList, startXList, startYList, 0, 0);
	}

	private void ZoomCamera(int center_x, int center_y, float startDistance,
			float endDistance) {
		float ROIcenterX = startROIcenterX;
		float ROIcenterY = startROIcenterY;
		float tmpROIRate = startROIRate * startDistance / endDistance;
		if (tmpROIRate > 1) {
			ROIoffsetX = 0;
			ROIoffsetY = 0;
			ROIRate = 1;
		} else {
			if (tmpROIRate * imageWidth * (1 - center_x / MaxWidth)
					+ ROIcenterX > imageWidth) {
				ROIcenterX = imageWidth
						* (1 - tmpROIRate * (1 - center_x / MaxWidth));
			} else if (ROIcenterX - tmpROIRate * imageWidth * center_x
					/ MaxWidth < 0) {
				ROIcenterX = tmpROIRate * imageWidth * center_x / MaxWidth;
			}
			if (tmpROIRate * imageHeight * (1 - center_y / MaxHeight)
					+ ROIcenterY > imageHeight) {
				ROIcenterY = imageHeight
						* (1 - tmpROIRate * (1 - center_y / MaxHeight));
			} else if (ROIcenterY - tmpROIRate * imageHeight * center_y
					/ MaxHeight < 0) {
				ROIcenterY = tmpROIRate * imageHeight * center_y / MaxHeight;
			}
			ROIoffsetX = ROIcenterX - tmpROIRate * imageWidth * center_x
					/ MaxWidth;
			ROIoffsetY = ROIcenterY - tmpROIRate * imageHeight * center_y
					/ MaxHeight;
			ROIRate = tmpROIRate;
		}
		setRectBitmap();
	}

	private void setRectBitmap() {
		if (decodeFlag) {
			decodeFlag = false;
			Rect rect = new Rect((int) ROIoffsetX, (int) ROIoffsetY,
					(int) (ROIoffsetX + imageWidth * ROIRate),
					(int) (ROIoffsetY + imageHeight * ROIRate));
			bitmap = bitmapRegionDecoder.decodeRegion(rect, null);
			decodeFlag = true;
		}
	}

	public float getImageCoordX(float fingerX) {
		return ROIoffsetX + fingerX * imageWidth * ROIRate / MaxWidth;
	}

	public float getImageCoordY(float fingerY) {
		return ROIoffsetY + fingerY * imageHeight * ROIRate / MaxHeight;
	}

	public void SendOpenDoorMsg() {
		SendCommandMsg("OpenDoor", RobotArmId, "SendMsg", 0, null, 0,
				fingerList, startXList, startYList, 0, 0);
		unSetMovingFingerInfo();
	}

	public void SendCloseDoorMsg() {
		SendCommandMsg("CloseDoor", RobotArmId, "SendMsg", 0, null, 0,
				fingerList, startXList, startYList, 0, 0);
		unSetMovingFingerInfo();
	}

	public void SendTuckArmPoseMsg() {
		SendCommandMsg("TuckArmPose", RobotArmId, "SendMsg", 0, null, 0,
				fingerList, startXList, startYList, 0, 0);
	}

	public void SendTorsoUpMsg() {
		SendCommandMsg("TorsoUp", RobotArmId, "SendMsg", 0, null, 0,
				fingerList, startXList, startYList, 0, 0);
	}

	public void SendTorsoDownMsg() {
		SendCommandMsg("TorsoDown", RobotArmId, "SendMsg", 0, null, 0,
				fingerList, startXList, startYList, 0, 0);
	}

	public void SendOpenGripperMsg() {
		SendCommandMsg("OpenGripper", RobotArmId, "SendMsg", 0, null, 0,
				fingerList, startXList, startYList, 0, 0);
	}

	public void SendCloseGripperMsg() {
		SendCommandMsg("CloseGripper", RobotArmId, "SendMsg", 0, null, 0,
				fingerList, startXList, startYList, 0, 0);
	}

	public void ResetValue() {
		fingerList.clear();
		startXList.clear();
		startYList.clear();
		lastXList.clear();
		lastYList.clear();
	}

	public boolean isClockWise(int p1x, int p1y, int p2x, int p2y, int p3x,
			int p3y) {
		if (((p2x - p1x) * (p3y - p1y)) >= ((p3x - p1x) * (p2y - p1y)))
			return false;
		return true;
	}

	@Override
	public boolean onTouchEvent(MotionEvent event) {
		// this.invalidate();
		final int action = event.getAction();
		final int curFingerCount = event.getPointerCount();

		switch (action & MotionEvent.ACTION_MASK) {
		/* ACTION_DOWN */
		case MotionEvent.ACTION_DOWN:
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_DOWN] START");
			fingerCount = event.getPointerCount();
			fingerList.clear();
			startXList.clear();
			startYList.clear();
			lastXList.clear();
			lastYList.clear();

			fingerList.ensureCapacity(10);
			startXList.ensureCapacity(10);
			startYList.ensureCapacity(10);
			lastXList.ensureCapacity(10);
			lastYList.ensureCapacity(10);

			fingerList.add(0);
			startXList.add((int) event.getX(0));
			startYList.add((int) event.getY(0));
			lastXList.add((int) event.getX(0));
			lastYList.add((int) event.getY(0));
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_DOWN] startXList = "
					+ startXList + ", startYList = " + startYList);
			SendDebugMsg("ActionDown", fingerCount, startXList.get(0),
					startYList.get(0));

			Log.i("JskAndroidGui:TouchEvent", "[ACTION_DOWN] END");
			if (isGetTemplate) {
				pl.clear();
				pl.add(new Point((int) event.getX(0), (int) event.getY(0)));
				boundingBoxView.drawBox(pl);
			}
			break;

		/* ACTION_DOWN more than two fingers */
		case MotionEvent.ACTION_POINTER_DOWN:
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_POINTER_DOWN] START");
			fingerCount = event.getPointerCount();

			for (int i = 0; i < curFingerCount; i++) {
				int fingerId = event.getPointerId(i);
				if (!fingerList.contains(fingerId)) {
					int pointerId = event.findPointerIndex(fingerId);
					fingerList.add(fingerId);
					startXList.add((int) event.getX(pointerId));
					startYList.add((int) event.getY(pointerId));
					lastXList.add((int) event.getX(pointerId));
					lastYList.add((int) event.getY(pointerId));
				}
			}

			if (fingerCount == 2 && isZoomCamera) {
				startROIRate = ROIRate;
				startROIcenterX = getImageCoordX((startXList.get(0) + startXList
						.get(1)) / 2);
				startROIcenterY = getImageCoordY((startYList.get(0) + startYList
						.get(1)) / 2);
			}

			Log.i("JskAndroidGui:TouchEvent",
					"[ACTION_POINTER_DOWN] startXList = " + startXList
							+ ", startYList = " + startYList);
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_POINTER_DOWN] END");
			break;

		/* ACTION_MOVE */
		case MotionEvent.ACTION_MOVE:
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_MOVE] START");
			if (isGetTemplate) {
				pl.add(new Point((int) event.getX(0), (int) event.getY(0)));
				boundingBoxView.drawBox(pl);
			}
			for (int i = 0; i < curFingerCount; ++i) {
				int fingerId = event.getPointerId(i);
				int pointerId = event.findPointerIndex(fingerId);
				lastXList.set(fingerList.indexOf(fingerId),
						(int) event.getX(pointerId));
				lastYList.set(fingerList.indexOf(fingerId),
						(int) event.getY(pointerId));
			}

			Log.i("JskAndroidGui:TouchEvent", "[ACTION_MOVE] lastXList = "
					+ lastXList + ", lastYList = " + lastYList + ", fingerList"
					+ fingerList);
			SendDebugMsg("ActionMove", fingerCount, lastXList.get(0),
					lastYList.get(0));
			if (isCircularManipulation && fingerCount == 2
					&& curFingerCount == 2) {
				SendTaskMsg("DrawLine", 0, "TOUCHMOVE", 0, null, 0, fingerList,
						lastXList, lastYList, 0, 0);
			} else if (isCircularManipulation && fingerCount == 3) {
				SendTaskMsg("DrawCircle", RobotArmId, "TOUCHMOVE", 0, null, 0,
						fingerList, lastXList, lastYList, 0, 0);
			} else if (isZoomCamera && fingerCount == 2 && curFingerCount == 2) {
				final float startDistance = Math.round(Math.sqrt(Math.pow(
						startXList.get(1) - startXList.get(0), 2)
						+ Math.pow(startYList.get(1) - startYList.get(0), 2)));
				final float endDistance = Math.round(Math.sqrt(Math.pow(
						lastXList.get(1) - lastXList.get(0), 2)
						+ Math.pow(lastYList.get(1) - lastYList.get(0), 2)));
				final float pinchLength = startDistance - endDistance;
				final int center_x = (lastXList.get(0) + lastXList.get(1)) / 2;
				final int center_y = (lastYList.get(0) + lastYList.get(1)) / 2;
				SendDebugMsg("PINCH", pinchLength, center_x, center_y);
				ZoomCamera(center_x, center_y, startDistance, endDistance);
			} else if (isMovingFingerInfo) {
				SendTaskMsg("MovingPointInfo", 0, "TOUCHMOVE", 0, null, 0,
						fingerList, lastXList, lastYList, 0, 0);
			}
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_MOVE] END");
			break;

		/* ACTION_UP */
		case MotionEvent.ACTION_UP:
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_UP] START");
			if (isMovingFingerInfo) {
				return true;
			}

			if (fingerCount == 1) {
				if (isGetTemplate) {
					boundingBoxView.drawBox(pl);
					boundingBoxView.setSelect();
					unSetGetTemplate();
					// pl.clear();

					LayoutInflater inflater = LayoutInflater.from(context);
					final View view = inflater.inflate(R.layout.dialog, null);
					final EditText editText = (EditText) view
							.findViewById(R.id.editText1);
					new AlertDialog.Builder(context)
							.setTitle("Template name")
							.setView(view)
							.setPositiveButton("Save",
									new DialogInterface.OnClickListener() {
										@Override
										public void onClick(
												DialogInterface dialog,
												int which) {

											SensorImageViewInfo.this
													.SendTemplateInfo(editText
															.getText()
															.toString(),
															startXList.get(0),
															startYList.get(0),
															lastXList.get(0),
															lastYList.get(0));
											Toast.makeText(
													context,
													"Create Template Image: "
															+ editText
																	.getText()
																	.toString(),
													Toast.LENGTH_SHORT).show();
											Log.i("JskAndroidGui:ButtonClicked",
													"Create Template Image");
										}
									}).show();
				} else {
					start_x = startXList.get(0);
					start_y = startYList.get(0);
					final float swipeLength = Math.round(Math.sqrt(Math.pow(
							getImageCoordX(lastXList.get(0))
									- getImageCoordX(start_x), 2)
							+ Math.pow(getImageCoordY(lastYList.get(0))
									- getImageCoordY(start_y), 2)));

					if (swipeLength >= minLength) {
						SendDebugMsg("SWIPE", swipeLength, lastXList.get(0),
								lastYList.get(0));
						final int X = (int) (getImageCoordX(start_x) - getImageCoordX(lastXList
								.get(0)));
						final int Y = (int) (getImageCoordY(lastYList.get(0)) - getImageCoordY(start_y));
						final double r = Math.atan2(Y, X);
						float swipeAngle = Math.round(r * 180 / Math.PI);
						if (swipeAngle < 0)
							swipeAngle = 360 - Math.abs(swipeAngle);
						if ((swipeAngle <= 45) && (swipeAngle >= 0)) {
							MoveNeck("left", swipeLength);
							SetSwipeDetected(SWIPE_LEFT);
						} else if ((swipeAngle <= 360) && (swipeAngle >= 315)) {
							MoveNeck("left", swipeLength);
							SetSwipeDetected(SWIPE_LEFT);
						} else if ((swipeAngle >= 135) && (swipeAngle <= 225)) {
							MoveNeck("right", swipeLength);
							SetSwipeDetected(SWIPE_RIGHT);
						} else if ((swipeAngle > 45) && (swipeAngle < 135)) {
							MoveNeck("down", swipeLength);
							SetSwipeDetected(SWIPE_DOWN);
						} else {
							MoveNeck("up", swipeLength);
							SetSwipeDetected(SWIPE_UP);
						}
					} else {
						if (isPushOnce) {
							isPushOnce = false;
							SendCommandMsg("PushOnce", 0, "TOUCH", 0, null, 0,
									fingerList, startXList, startYList,
									start_x, start_y);
						} else if (isPickOnce) {
							isPickOnce = false;
							// SendCommandMsg("PickOnce", 0, "TOUCH", 0, null,
							// 0,
							SendCommandMsg("PickObjectSelected", RobotArmId,
									"TOUCH", 0, null, 0, fingerList,
									startXList, startYList, start_x, start_y);
						} else if (isPlaceOnce) {
							isPlaceOnce = false;
							SendCommandMsg("PlaceOnce", 0, "TOUCH", 0, null, 0,
									fingerList, startXList, startYList,
									start_x, start_y);
						} else if (isPassToHumanOnce) {
							isPassToHumanOnce = false;
							SendCommandMsg("PassToHumanOnce", RobotArmId,
									"TOUCH", 0, null, 0, fingerList,
									startXList, startYList, start_x, start_y);
						} else if (TouchMode == 0) {
							SendCommandMsg("MoveCameraCenter", 0, "TOUCH", 0,
									null, 0, fingerList, startXList,
									startYList, start_x, start_y);
						} else if (TouchMode == 1) {
							SendCommandMsg("Show3DScreenPoint", 0, "TOUCH", 0,
									null, 0, fingerList, startXList,
									startYList, start_x, start_y);
						}
					}
				}
			} else if (isCircularManipulation && fingerCount == 3) {
				SendTaskMsg("CircularManipulation", RobotArmId, "TOUCH", 0,
						null, 0, fingerList, lastXList, lastYList, 0, 0);
			} else if (fingerCount == 4) {
				if (startYList.get(0) >= imageHeight / 2
						&& imageHeight / 2 >= lastYList.get(0)) {
					SendCommandMsg("TorsoUp", 0, "TOUCH", 0, null, 0,
							fingerList, startXList, startYList, 0, 0);
				} else if (startYList.get(0) <= imageHeight / 2
						&& imageHeight / 2 <= lastYList.get(0)) {
					SendCommandMsg("TorsoDown", 0, "TOUCH", 0, null, 0,
							fingerList, startXList, startYList, 0, 0);
				} else {
					SendCommandMsg("StopMotion", 0, "TOUCH", 0, null, 0,
							fingerList, startXList, startYList, 0, 0);
				}
			} else if (fingerCount == 5
					&& (Math.abs(startXList.get(0) - lastXList.get(0)) > minLength)) {
				SendCommandMsg("TuckArmPose", RobotArmId, "TOUCH", 0, null, 0,
						fingerList, startXList, startYList, 0, 0);
			}
			// ResetValue();
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_UP] END, ResetValue();");
			break;

		/* ACTION_UP more than two fingers */
		case MotionEvent.ACTION_POINTER_UP:
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_POINTER_UP] START");
			Log.v("JskAndroidGui:TouchEvent",
					"[ACTION_POINTER_UP] fingerCount = " + fingerCount
							+ ", curFingerCount_temp = " + curFingerCount);
			Log.i("JskAndroidGui:TouchEvent",
					"[ACTION_POINTER_UP] isMovingFingerInfo = "
							+ isMovingFingerInfo);
			if (curFingerCount == 2 && fingerCount == 2 && isMovingFingerInfo) {
				int p1x, p1y, p2x, p2y, p3x, p3y;
				if (Math.abs(startXList.get(0) - lastXList.get(0)) < minLength) {
					Log.i("JskAndroidGui:TouchEvent",
							"[ACTION_POINTER_UP] true");
					p1x = startXList.get(0);
					p1y = startYList.get(0);
					p2x = startXList.get(1);
					p2y = startYList.get(1);
					p3x = lastXList.get(1);
					p3y = lastYList.get(1);
				} else {
					Log.i("JskAndroidGui:TouchEvent",
							"[ACTION_POINTER_UP] false");
					p1x = startXList.get(1);
					p1y = startYList.get(1);
					p2x = startXList.get(0);
					p2y = startYList.get(0);
					p3x = lastXList.get(0);
					p3y = lastYList.get(0);
				}
				if (isClockWise(p1x, p1y, p2x, p2y, p3x, p3y)) {
					Log.i("JskAndroidGui:TouchEvent", "[ACTION_POINTER_UP] 45");
					SendCommandMsg("RotateGripper", RobotArmId, "PINCH", 45,
							null, 0, fingerList, startXList, startYList, 0, 0);
				} else {
					Log.i("JskAndroidGui:TouchEvent", "[ACTION_POINTER_UP] -45");
					SendCommandMsg("RotateGripper", RobotArmId, "PINCH", -45,
							null, 0, fingerList, startXList, startYList, 0, 0);
				}
				// } else if (curFingerCount == 3 && fingerCount == 3) {
				// Log.v("JskAndroidGui:TouchEvent",
				// "[ACTION_POINTER_UP] startXList = " + startXList
				// + ", startYList = " + startYList);
				// Log.v("JskAndroidGui:TouchEvent",
				// "[ACTION_POINTER_UP] lastXList = " + lastXList
				// + ", lastYList = " + lastYList);
				//
				// // estimate open or slide?
				// SendTaskMsg("OpenDoorInput", RobotArmId, "TOUCH", 0, null, 0,
				// fingerList, startXList, startYList, 0, 0);
				// android.os.SystemClock.sleep(1000);
				// unSetDrawLine();
				// // SetMovingFingerInfo();
			}

			Log.i("JskAndroidGui:TouchEvent", "[ACTION_POINTER_UP] END");
			break;

		/* ACTION_OUTSIDE */
		case MotionEvent.ACTION_OUTSIDE:
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_CANCEL] START");
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_CANCEL] END");
			break;

		/* ACTION_CANCEL , called with ACTION_POINTER_UP */
		case MotionEvent.ACTION_CANCEL:
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_CANCEL] START");
			Log.i("JskAndroidGui:TouchEvent", "[ACTION_CANCEL] END");
			break;
		}
		Log.i("JskAndroidGui:TouchEvent", "END");
		// return true;
		return super.onTouchEvent(event);
	}

	@Override
	public void run() {

		// final int bWidth = bitmap.getWidth();
		// final int bHeight = bitmap.getHeight();
		// Bitmap bitmap_tmp = Bitmap.createBitmap(bWidth, bHeight,
		// Bitmap.Config.ARGB_8888);
		// int[] bpixels = new int[bWidth*bHeight];
		// bitmap.getPixels(bpixels, 0, bWidth, 0, 0, bWidth, bHeight);
		// translateBGRtoRGB(bpixels,bWidth,bHeight);
		// bitmap_tmp.setPixels(bpixels, 0, bWidth, 0, 0, bWidth, bHeight);

		if (SwipeDetectedType != SWIPE_NONE && SwipeCounter < SWIPE_WAIT_TIME) {
			Log.v("JskAndroidGui:onNewMessage",
					"[changing Bitmap] changing now " + (SwipeCounter + 1)
							+ " / " + SWIPE_WAIT_TIME);
			this.invalidate();
			SwipeCounter++;
			// final int Width = bitmap_tmp.getWidth();
			// final int Height = bitmap_tmp.getHeight();
			final int Width = bitmap.getWidth();
			final int Height = bitmap.getHeight();
			Bitmap bitmap_output = Bitmap.createBitmap(Width, Height,
					Bitmap.Config.ARGB_8888);
			int[] pixels = new int[Width * Height];
			bitmap.getPixels(pixels, 0, Width, 0, 0, Width, Height);
			// bitmap_tmp.getPixels(pixels, 0, Width, 0, 0, Width, Height);
			for (int x = 0; x < Width; x++) {
				for (int y = 0; y < Height; y++) {
					// int pixel = pixels[x + y * Width];
					if ((SwipeDetectedType == SWIPE_UP && (-10 * x + y > 0 || 10 * Width < 10
							* x + y))
							|| (SwipeDetectedType == SWIPE_DOWN && (10 * x + y < Height || 10
									* Width - Height < 10 * x - y))
							|| (SwipeDetectedType == SWIPE_LEFT && (-1 * x + 10
									* y < 0 || 10 * Height < x + 10 * y))
							|| (SwipeDetectedType == SWIPE_RIGHT && (x + 10 * y < Width || -1
									* Width + 10 * Height < -1 * x + 10 * y))) {
						pixels[x + y * Width] = Color.rgb(Color.BLACK,
								Color.BLACK, Color.BLACK);
					}
				}
			}
			bitmap_output.setPixels(pixels, 0, Width, 0, 0, Width, Height);
			setImageBitmap(bitmap_output);

			// Matrix matrix = new Matrix(); matrix.postSkew(skew_x,
			// skew_y);
			// final Bitmap bitmap_output = Bitmap.createBitmap(bitmap2, 0,
			// 0,
			// w, h, matrix, true);
		} else if (SwipeCounter >= SWIPE_WAIT_TIME) {
			Log.v("JskAndroidGui:onNewMessage",
					"[changing Bitmap] back to normal");
			unSetSwipeDetected();
		} else {
			setImageBitmap(bitmap);
		}
	}

	@Override
	public void onError(Node node, Throwable throwable) {
	}

	@Override
	public void onShutdown(Node node) {
		node.shutdown();
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("jsk_android_gui/SensorImageViewInfo");
	}
}
