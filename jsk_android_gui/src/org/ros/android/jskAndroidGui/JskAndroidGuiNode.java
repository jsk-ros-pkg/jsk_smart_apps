package org.ros.android.jskAndroidGui;

import java.util.ArrayList;

import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import android.util.Log;

import roseus.StringStamped;
import std_msgs.Empty;

public class JskAndroidGuiNode extends AbstractNodeMain {

	public boolean setupEnd = false;
	private Publisher<Empty> GetSpotPub;
	private Publisher<StringStamped> StartDemoPub, MoveToSpotPub, SelectPub,
			TweetPub, EmergencyStopPub;
	private ConnectedNode connectedNode;
	private ParameterTree parameterTree;
	private ArrayList<String> spots_list, tasks_list, image_list,
			camera_info_list, points_list;

	@Override
	public GraphName getDefaultNodeName() {
		return null;
	}

	public void tweetTask(String text) {
		StringStamped StrMsg_tweet = TweetPub.newMessage();
		StrMsg_tweet.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		StrMsg_tweet.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		StrMsg_tweet.setData(text);
		TweetPub.publish(StrMsg_tweet);

	}

	public void selectTask(String str) {
		StringStamped StrMsg_result = SelectPub.newMessage();
		StrMsg_result.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		StrMsg_result.setData(str);
		SelectPub.publish(StrMsg_result);
	}

	public void switchJoy() {
		StringStamped StrMsg_switchjoy = SelectPub.newMessage();
		StrMsg_switchjoy.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		StrMsg_switchjoy.setData("Switchjoy");
		SelectPub.publish(StrMsg_switchjoy);

	}

	public void tasksSpinnerTask(String str) {
		StringStamped StrMsg = StartDemoPub.newMessage();
		StrMsg.getHeader()
				.setStamp(Time.fromMillis(System.currentTimeMillis()));
		StrMsg.setData(str);
		StartDemoPub.publish(StrMsg);
	}

	public void spotsSpinnerTask(String str) {
		StringStamped StrMsg = MoveToSpotPub.newMessage();
		StrMsg.getHeader()
				.setStamp(Time.fromMillis(System.currentTimeMillis()));
		StrMsg.setData(str);
		MoveToSpotPub.publish(StrMsg);
	}

	public void getSpot() {
		Empty EmptyMsg = GetSpotPub.newMessage();
		GetSpotPub.publish(EmptyMsg);
	}

	public void stopJoint() {
		StringStamped StrMsg_stopjoint = EmergencyStopPub.newMessage();
		StrMsg_stopjoint.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		StrMsg_stopjoint.setData("StopJoint");
		EmergencyStopPub.publish(StrMsg_stopjoint);
	}

	public void stopNavigation() {
		StringStamped StrMsg_stopnavigation = EmergencyStopPub.newMessage();
		StrMsg_stopnavigation.getHeader().setStamp(
				Time.fromMillis(System.currentTimeMillis()));
		StrMsg_stopnavigation.setData("StopNavigation");
		EmergencyStopPub.publish(StrMsg_stopnavigation);
	}

	public void getSpotsParam() {
		String defaultSpot_ns = "/jsk_spots";
		String targetSpot = "/eng2/7f"; // TODO: get current targetSpot
		GraphName gspot = GraphName.of(defaultSpot_ns + targetSpot);
		NameResolver resolver_spot = connectedNode.getResolver()
				.newChild(gspot);
		Object[] spots_param_list = parameterTree.getList(
				resolver_spot.resolve("spots")).toArray();
		Log.i("JskAndroidGui:GetSpotsParam", "spots length = "
				+ spots_param_list.length);
		spots_list.clear();
		spots_list.add("spots");
		for (int i = 0; i < spots_param_list.length; i++) {
			spots_list.add((String) spots_param_list[i]);
			Log.w("JskAndroidGui:GetSpotsParam", "lists:" + i + " "
					+ spots_param_list[i]);
		}
	}

	public void getTasksParam() {
		String defaultTask_ns = "/Tablet";
		GraphName guser = GraphName.of(defaultTask_ns);
		NameResolver resolver_user = connectedNode.getResolver()
				.newChild(guser);
		Object[] user_list = parameterTree.getList(
				resolver_user.resolve("UserList")).toArray();
		tasks_list.clear();
		tasks_list.add("tasks");
		for (int i = 0; i < user_list.length; i++) {
			GraphName gtask = GraphName.of(defaultTask_ns + "/User");
			NameResolver resolver_task = connectedNode.getResolver().newChild(
					gtask);
			Object[] task_param_list = parameterTree.getList(
					resolver_task.resolve((String) user_list[i])).toArray();
			Log.i("JskAndroidGui:GetTasksParam", "task length = "
					+ task_param_list.length);
			for (int j = 0; j < task_param_list.length; j++) {
				Log.i("JskAndroidGui:GetTasksParam", "lists: " + i + " " + j
						+ " /Tablet/" + (String) user_list[i] + "/"
						+ (String) task_param_list[j]);
				tasks_list.add("/Tablet/" + (String) user_list[i] + "/"
						+ (String) task_param_list[j]);
			}
		}
	}

	public void getCameraParam() {
		String defaultCamera_ns = "/jsk_cameras";
		GraphName gcamera = GraphName.of(defaultCamera_ns);
		NameResolver resolver_camera = connectedNode.getResolver().newChild(
				gcamera);
		Object[] camera_names_list = parameterTree.getList(
				resolver_camera.resolve("CameraList")).toArray();
		Log.i("JskAndroidGui:GetCameraParam", "camera length = "
				+ camera_names_list.length);
		image_list.clear();
		image_list.add("cameras");
		for (int i = 0; i < camera_names_list.length; i++) {
			Object[] camera_param_list = parameterTree.getList(
					resolver_camera.resolve((String) camera_names_list[i]))
					.toArray();
			// if (i == 0) {
			// defaultImage = (String)camera_param_list[0];
			// defaultCameraInfo = (String)camera_param_list[1];
			// }
			image_list.add((String) camera_param_list[0]);
			camera_info_list.add((String) camera_param_list[1]);
			Log.w("JskAndroidGui:GetCameraParam", "lists:" + i + " "
					+ camera_param_list[0] + camera_param_list[1]);
		}
	}

	public void getPointsParam() {
		String defaultPoints_ns = "/jsk_points";
		GraphName gparam = GraphName.of(defaultPoints_ns);
		NameResolver resolver_point = connectedNode.getResolver().newChild(
				gparam);
		Object[] points_param_list = parameterTree.getList(
				resolver_point.resolve("points")).toArray();
		Log.i("JskAndroidGui:GetPointsParam", "point length = "
				+ points_param_list.length);
		points_list.clear();
		points_list.add("points");
		for (int i = 0; i < points_param_list.length; i++) {
			// if (i == 0) {
			// defaultPoints = (String)points_param_list[i];
			// }
			points_list.add((String) points_param_list[i]);
			Log.w("JskAndroidGui:GetPointsParam", "lists:" + i + " "
					+ points_param_list[i]);
		}
	}

	public ArrayList<String> getPointsList() {
		return points_list;
	}

	public ArrayList<String> getTasksList() {
		return tasks_list;
	}

	public ArrayList<String> getImageList() {
		return image_list;
	}

	public ArrayList<String> getCameraInfoList() {
		return camera_info_list;
	}

	public ArrayList<String> getSpotsList() {
		return spots_list;
	}

	public ParameterTree getParameterTree() {
		return parameterTree;
	}

	public void onStart(final ConnectedNode connectedNode) {
		this.connectedNode = connectedNode;

		spots_list = new ArrayList<String>();
		tasks_list = new ArrayList<String>();
		image_list = new ArrayList<String>();
		camera_info_list = new ArrayList<String>();
		points_list = new ArrayList<String>();

		parameterTree = connectedNode.getParameterTree();

		GetSpotPub = connectedNode.newPublisher("/Tablet/GetSpot",
				std_msgs.Empty._TYPE);
		StartDemoPub = connectedNode.newPublisher("/Tablet/StartDemo",
				roseus.StringStamped._TYPE);
		MoveToSpotPub = connectedNode.newPublisher("/Tablet/MoveToSpot",
				roseus.StringStamped._TYPE);
		EmergencyStopPub = connectedNode.newPublisher(
				"/Tablet/EmergencyCommand", roseus.StringStamped._TYPE);
		SelectPub = connectedNode.newPublisher("/Tablet/Select",
				roseus.StringStamped._TYPE);
		TweetPub = connectedNode.newPublisher("/pr2twit_from_tablet",
				roseus.StringStamped._TYPE);

		// try{
		// getSpotsParam();
		// getCameraParam();
		// getTasksParam();
		// getPointsParam();
		// }catch (Exception ex){
		// Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
		// }
		getParamLoop();

		setupEnd = true;
	}

	public void getParamLoop() {
		try {
			getSpotsParam();
			getCameraParam();
			getTasksParam();
			getPointsParam();
		} catch (Exception ex) {
			Log.e("JskAndroidGui", "Param cast error: " + ex.toString());
			try {
				Thread.sleep(1000);
			} catch (Exception e) {
				e.printStackTrace();
			}
			getParamLoop();
		}

	}

}
