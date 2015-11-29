/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.android_camera_viewer;

import android.hardware.Camera;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;
import org.ros.address.InetAddressFactory;
import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.view.RosImageView;
import org.ros.android.RosActivity;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
//add
import android.os.IBinder;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import java.net.URI;
import java.net.URISyntaxException;
import android.content.Context;
import android.util.Log;

/**
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MainActivity extends RosActivity {
    
    //add
    private URI masterUri;
    private Intent intent;
    private AndroidAudioRecordService myService;
    private String package_name;


    private int cameraId;
    private RosCameraPreviewView rosCameraPreviewView;
    private RosImageView<sensor_msgs.CompressedImage> image;



    //add
        ServiceConnection serviceConnection = new ServiceConnection(){

	    @Override
		public void onServiceConnected(ComponentName name,IBinder service){
		myService = ((AndroidAudioRecordService.MyBinder)service).getService();
		startService(intent);
		myService.setNode(package_name);
	    }

	    @Override
		public void onServiceDisconnected(ComponentName name){
		myService = null;
	    }
	};



  public MainActivity() {
    super("ROS Camera Viewer", "ROS Camera Viewer");
  }


  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    //    requestWindowFeature(Window.FEATURE_NO_TITLE);
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
    setContentView(R.layout.main);
    rosCameraPreviewView = (RosCameraPreviewView) findViewById(R.id.ros_camera_preview_view);

    image = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
    image.setTopicName("/usb_cam/image_raw/compressed");
    image.setMessageType(sensor_msgs.CompressedImage._TYPE);
    image.setMessageToBitmapCallable(new BitmapFromCompressedImage());
    //	Log.v("test","test");

    //add
    	package_name = getPackageName();
  }


    //add
     @Override
	public void onResume(){
	super.onResume();

				if(masterUri != null){
	    intent = new Intent(getBaseContext(),AndroidAudioRecordService.class);


	    intent.putExtra("masterUri",masterUri.toString());
	    bindService(intent,serviceConnection,Context.BIND_AUTO_CREATE);
	    }
	    }
    //add
    /*            protected void onActivityResult(int requestCode,int resultCode,Intent data){
	if(requestCode == 0 && resultCode == RESULT_OK){
	    try{
		masterUri = new URI(data.getStringExtra("ROS_MASTER_URI"));
	    }catch(URISyntaxException e){
		throw new RuntimeException(e);
	    }
	}
	}
    */
    
    @Override
  public boolean onTouchEvent(MotionEvent event) {
    if (event.getAction() == MotionEvent.ACTION_UP) {
      int numberOfCameras = Camera.getNumberOfCameras();
      final Toast toast;
      if (numberOfCameras > 1) {
        cameraId = (cameraId + 1) % numberOfCameras;
        rosCameraPreviewView.releaseCamera();
        rosCameraPreviewView.setCamera(Camera.open(cameraId));
        toast = Toast.makeText(this, "Switching cameras.", Toast.LENGTH_SHORT);
      } else {
        toast = Toast.makeText(this, "No alternative cameras to switch to.", Toast.LENGTH_SHORT);
      }
      runOnUiThread(new Runnable() {
        @Override
        public void run() {
          toast.show();
        }
      });
    }
    return true;
  }


  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
      cameraId = 0;
      rosCameraPreviewView.setCamera(Camera.open(cameraId));
    NodeConfiguration nodeConfiguration =
        NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());
        masterUri = getMasterUri(); //add
		if(masterUri != null){
	    intent = new Intent(getBaseContext(),AndroidAudioRecordService.class);


	    intent.putExtra("masterUri",masterUri.toString());
	    bindService(intent,serviceConnection,Context.BIND_AUTO_CREATE);
		} //add
		nodeMainExecutor.execute(image, nodeConfiguration.setNodeName("/android/image_view"));
		nodeMainExecutor.execute(rosCameraPreviewView, nodeConfiguration.setNodeName("/android/preview_view")); 

  }
}
