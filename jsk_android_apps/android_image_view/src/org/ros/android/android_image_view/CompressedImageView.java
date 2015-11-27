package org.ros.android.android_image_view;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.ViewGroup;
import android.widget.ImageView;

public class CompressedImageView extends ImageView implements NodeMain {

	private String topicName;
	private String messageType;
	private Bitmap showBitmap;
	private float aspect;
	private TouchEventTalker talker ;

	public CompressedImageView(Context context) {
		super(context);
	}
	
	public void setTalker( TouchEventTalker talker ){
		this.talker = talker ;
	}

	public CompressedImageView(Context context, AttributeSet attrs) {
		super(context, attrs);
	}

	public CompressedImageView(Context context, AttributeSet attrs, int defStyle) {
		super(context, attrs, defStyle);
	}

	public void setTopicName(String topicName) {
		this.topicName = topicName;
	}

	public void setMessageType(String messageType) {
		this.messageType = messageType;
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("tap_on_image/compressed_image_view");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		Subscriber<sensor_msgs.CompressedImage> subscriber = connectedNode
				.newSubscriber(topicName, messageType);
		subscriber
				.addMessageListener(new MessageListener<sensor_msgs.CompressedImage>() {
					@Override
					public void onNewMessage(
							final sensor_msgs.CompressedImage message) {
						System.out.println( "receive message" ) ;
						post(new Runnable() {
							@Override
							public void run() {
								ChannelBuffer buffer = message.getData();
								byte[] data = buffer.array();
								if (CompressedImageView.this.showBitmap != null) {
									CompressedImageView.this.showBitmap
											.recycle();
								}
								CompressedImageView.this.showBitmap = BitmapFactory
										.decodeByteArray(data,
												buffer.arrayOffset(),
												buffer.readableBytes());
								CompressedImageView.this.setBitmap();
								setImageBitmap(CompressedImageView.this.showBitmap);
							}
						});
						postInvalidate();
					}
				});
	}

	public void setBitmap() {
		setBitmap(this.showBitmap);
	}

	public void setBitmap(Bitmap bmp) {
		this.showBitmap = bmp ;
		float aspect = 1.0f * bmp.getWidth() / bmp.getHeight();
		if (Math.abs(this.aspect - aspect) > 0.1) {
			this.aspect = aspect;
			float rate = //1.0f ;
					Math.min(1.0f * MainActivity.width / bmp.getWidth(),
					1.0f * MainActivity.height / bmp.getHeight());
			ViewGroup.LayoutParams param = CompressedImageView.this
					.getLayoutParams();
			param.width = (int) (rate * bmp.getWidth());
			param.height = (int) (rate * bmp.getHeight());
			this.setLayoutParams(param);
		}
		setImageBitmap(bmp);
		postInvalidate();
	}

	@Override
	public boolean onTouchEvent(MotionEvent e) {
		System.out.println(e.getX() + ":" + e.getY());
		if ( this.talker != null ){
			this.talker.publish( e.getX(), e.getY(), this.getWidth(), this.getHeight(), e.getAction() ) ;
		}
		return true;
	}

	@Override
	public void onShutdown(Node node) {
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public void onError(Node node, Throwable throwable) {
	}
}
