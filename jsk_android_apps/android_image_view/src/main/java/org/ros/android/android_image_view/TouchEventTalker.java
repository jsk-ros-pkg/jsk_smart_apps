package org.ros.android.android_image_view;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import android.view.MotionEvent;
import jsk_gui_msgs.TouchEvent;

public class TouchEventTalker extends AbstractNodeMain {

	// private boolean publish = false;

	private TouchEvent touchEvent;
	private Publisher<TouchEvent> touchEventPublisher;

	synchronized public void publish(float x, float y, float w, float h,
			int state) {
		// this.publish = true;

		this.touchEvent.setX(x);
		this.touchEvent.setY(y);
		this.touchEvent.setW(w);
		this.touchEvent.setH(h);
		this.touchEvent.setState(stateConverter(state));

		this.touchEventPublisher.publish(this.touchEvent);
	}

	public byte stateConverter(int state){
		switch ( state ){
		case MotionEvent.ACTION_DOWN :
			return TouchEvent.DOWN ;
		case MotionEvent.ACTION_UP :
			return TouchEvent.UP ;
		case MotionEvent.ACTION_MOVE :
			return TouchEvent.MOVE ;
		default :
			return (byte)-1 ;
		}
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("tap_on_image/talker");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		System.out.println(connectedNode.getClass() + "/"
				+ connectedNode.getClass().getName());

		touchEventPublisher = connectedNode.newPublisher("touchEvent",
				TouchEvent._TYPE);
		touchEvent = connectedNode.getTopicMessageFactory().newFromType(
				TouchEvent._TYPE);
	}
}