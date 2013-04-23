package org.ros.android.android_image_view;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.view.Display;
import android.view.WindowManager;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.android_image_view.R;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosActivity {

	private CompressedImageView image;
	private Talker talker;
	public static float width = 480, height = 640;

	public MainActivity() {
		super("TapOnImage", "TapOnImage");
	}

	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		talker = new Talker();

		WindowManager wm = (WindowManager) getSystemService(WINDOW_SERVICE);
		Display disp = wm.getDefaultDisplay();
		width = disp.getWidth();
		height = disp.getHeight();

		setContentView(R.layout.main);
		image = (CompressedImageView) findViewById(R.id.image);
		image.setTopicName("/usb_cam/image_raw/compressed");
		image.setMessageType(sensor_msgs.CompressedImage._TYPE);
		image.setTalker(talker) ;
	}

	@Override
	public void onResume() {
		super.onResume();
		Bitmap bmp = BitmapFactory.decodeResource(this.getResources(),
				R.drawable.robot);
		this.image.setBitmap(bmp);
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
				InetAddressFactory.newNonLoopback().getHostAddress(),
				getMasterUri());
		nodeMainExecutor.execute(image, nodeConfiguration);
		nodeMainExecutor.execute(talker, nodeConfiguration);
	}

}
