package org.ros.android.android_image_view;

import java.util.Arrays;
import java.util.List;

import geometry_msgs.Point;
import geometry_msgs.Point32;
import geometry_msgs.Polygon;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.Header;

public class Talker extends AbstractNodeMain {

	private float x, y, w, h;
	private boolean publish = false;
	private List<Point32> rectangle;
	
	private Publisher<geometry_msgs.PointStamped> pointscreen ;
	private Publisher<geometry_msgs.PolygonStamped> screenrectangle ;
	
	synchronized public void setXYWH(float x, float y, float w, float h) {
		this.publish = true;
		this.x = x;
		this.y = y;
		this.w = w;
		this.h = h;
		
		geometry_msgs.PointStamped point = pointscreen
				.newMessage();
		Point p = point.getPoint();
		p.setX(Talker.this.x);
		p.setY(Talker.this.y);
		point.setPoint(p);
		pointscreen.publish(point);
		System.out.println(point.getClass() + "/"
				+ point.getClass().getName());

		geometry_msgs.PolygonStamped rectangle = screenrectangle
				.newMessage();
		Polygon r = rectangle.getPolygon();
		setRectangle();
		r.setPoints(Talker.this.rectangle);
		screenrectangle.publish(rectangle);
		
//		try {
//			Thread.sleep(100) ;
//		} catch (InterruptedException e) {
//			e.printStackTrace();
//		}
	}

	private void setRectangle() {
		this.rectangle.get(0).setX(0);
		this.rectangle.get(0).setY(0);

		this.rectangle.get(1).setX(this.w);
		this.rectangle.get(1).setY(0);

		this.rectangle.get(2).setX(this.w);
		this.rectangle.get(2).setY(this.h);

		this.rectangle.get(3).setX(0);
		this.rectangle.get(3).setY(this.h);
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("tap_on_image/talker");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		System.out.println(connectedNode.getClass() + "/"
				+ connectedNode.getClass().getName());

		pointscreen = connectedNode
				.newPublisher("pointscreen", geometry_msgs.PointStamped._TYPE);
		screenrectangle = connectedNode
				.newPublisher("screenrectangle",
						geometry_msgs.PolygonStamped._TYPE);
		this.rectangle = Arrays.asList(new Point32[] {
				connectedNode.getTopicMessageFactory().newFromType(
						Point32._TYPE),
				connectedNode.getTopicMessageFactory().newFromType(
						Point32._TYPE),
				connectedNode.getTopicMessageFactory().newFromType(
						Point32._TYPE),
				connectedNode.getTopicMessageFactory().newFromType(
						Point32._TYPE) });

		// This CancellableLoop will be canceled automatically when the node
		// shuts
		// down.
//		connectedNode.executeCancellableLoop(new CancellableLoop() {
//			private int sequenceNumber;
//
//			@Override
//			protected void setup() {
//				sequenceNumber = 0;
//			}
//
//			@Override
//			protected void loop() throws InterruptedException {
//				synchronized (Talker.this) {
//					// System.out.println("loop");
//					if (Talker.this.publish) {
//						System.out.println("pub");
//						Talker.this.publish = false;
//						// std_msgs.String str = publisher.newMessage();
//						// str.setData("Hello world! " + sequenceNumber);
//						// publisher.publish(str);
//
//						geometry_msgs.PointStamped point = pointscreen
//								.newMessage();
//						Point p = point.getPoint();
//						p.setX(Talker.this.x);
//						p.setY(Talker.this.y);
//						point.setPoint(p);
//						pointscreen.publish(point);
//						System.out.println(point.getClass() + "/"
//								+ point.getClass().getName());
//
//						geometry_msgs.PolygonStamped rectangle = screenrectangle
//								.newMessage();
//						Polygon r = rectangle.getPolygon();
//						setRectangle();
//						r.setPoints(Talker.this.rectangle);
//						screenrectangle.publish(rectangle);
//
//						sequenceNumber++;
//						Thread.sleep(100);
//					}
//				}
//			}
//		});
	}
}