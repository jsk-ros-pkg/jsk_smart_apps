package org.ros.android.jskAndroidGui;

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
	
	@Override
	public GraphName getDefaultNodeName() {
		return null;
	}
	
	public void onStart(final ConnectedNode connectedNode) {
		GetSpotPub = connectedNode.newPublisher("/Tablet/GetSpot",std_msgs.Empty._TYPE);
		StartDemoPub = connectedNode.newPublisher("/Tablet/StartDemo",
				roseus.StringStamped._TYPE);
		MoveToSpotPub = connectedNode.newPublisher("/Tablet/MoveToSpot",
				roseus.StringStamped._TYPE);
		EmergencyStopPub = connectedNode.newPublisher("/Tablet/EmergencyCommand",
				roseus.StringStamped._TYPE);
		SelectPub = connectedNode.newPublisher("/Tablet/Select", roseus.StringStamped._TYPE);

		TweetPub = connectedNode.newPublisher("/pr2twit_from_tablet",
				roseus.StringStamped._TYPE);
	}
}
