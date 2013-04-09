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
import android.widget.Toast;

import roseus.StringStamped;
import std_msgs.Empty;

public class JskAndroidGuiNode extends AbstractNodeMain {

	private Publisher<Empty> GetSpotPub;
	private Publisher<StringStamped> StartDemoPub, MoveToSpotPub, SelectPub,
			TweetPub, EmergencyStopPub;
	private ConnectedNode connectedNode;
	private ParameterTree parameterTree;
	private ArrayList<String> spots_list = new ArrayList();
	
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
	
	public void yesTask() {
		StringStamped StrMsg_resultyes = SelectPub.newMessage();
		StrMsg_resultyes.getHeader().setStamp(Time.fromMillis(System
				.currentTimeMillis()));
		StrMsg_resultyes.setData("ResultYes");
		SelectPub.publish(StrMsg_resultyes);
	}
	
	public void noTask() {
		StringStamped StrMsg_resultno = SelectPub.newMessage();
		StrMsg_resultno.getHeader().setStamp(Time.fromMillis(System
				.currentTimeMillis()));
		StrMsg_resultno.setData("ResultNo");
		SelectPub.publish(StrMsg_resultno);
	}
	
	public void switchJoy() {
		StringStamped StrMsg_switchjoy = SelectPub.newMessage();
		StrMsg_switchjoy.getHeader().setStamp(Time.fromMillis(System
				.currentTimeMillis()));
		StrMsg_switchjoy.setData("Switchjoy");
		SelectPub.publish(StrMsg_switchjoy);
		
	}
	
	public void getSpot() {
		Empty EmptyMsg = GetSpotPub.newMessage();
		GetSpotPub.publish(EmptyMsg);
	}
	
	public void stopJoint() {
		StringStamped StrMsg_stopjoint = EmergencyStopPub.newMessage();
		StrMsg_stopjoint.getHeader().setStamp(Time.fromMillis(System
				.currentTimeMillis()));
		StrMsg_stopjoint.setData("StopJoint");
		EmergencyStopPub.publish(StrMsg_stopjoint);
	}
	
	public void stopNavigation() {
		StringStamped StrMsg_stopnavigation = EmergencyStopPub.newMessage();
		StrMsg_stopnavigation.getHeader().setStamp(Time.fromMillis(System
				.currentTimeMillis()));
		StrMsg_stopnavigation.setData( "StopNavigation");
		EmergencyStopPub.publish(StrMsg_stopnavigation);	
	}
	
	public void getSpotsParam() {
		
			String defaultSpot_ns = "/jsk_spots";
			String targetSpot = "/eng2/7f"; //TODO: get current targetSpot
			GraphName gspot = GraphName.of(defaultSpot_ns + targetSpot);
			NameResolver resolver_spot = connectedNode.getResolver().newChild(gspot);
			Object[] spots_param_list = parameterTree.getList( resolver_spot.resolve("spots")).toArray();
			Log.i("JskAndroidGui:GetSpotsParam", "spots length = " + spots_param_list.length);
			spots_list.clear();
			spots_list.add("spots");
			for (int i = 0; i < spots_param_list.length; i++){
				spots_list.add((String) spots_param_list[i]);
				Log.w("JskAndroidGui:GetSpotsParam", "lists:" + i + " " + spots_param_list[i]);
			} 
	}
	
	

	public void onStart(final ConnectedNode connectedNode) {
		this.connectedNode = connectedNode;
		
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
		
		getSpotsParam();
	}
}
