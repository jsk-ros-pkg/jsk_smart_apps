package org.ros.android.jskAndroidGui;

import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import roseus.StringStamped;
import std_msgs.Empty;

public class JskAndroidGuiNode extends AbstractNodeMain {

	private Publisher<Empty> GetSpotPub;
	private Publisher<StringStamped> StartDemoPub, MoveToSpotPub, SelectPub,
			TweetPub, EmergencyStopPub;
	private ConnectedNode connectedNode;

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
	
	public void switchJon() {
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
	

	public void onStart(final ConnectedNode connectedNode) {
		this.connectedNode = connectedNode;

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
	}
}
