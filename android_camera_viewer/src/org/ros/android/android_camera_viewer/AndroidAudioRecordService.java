package org.ros.android.android_camera_viewer;


import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;

import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import android.app.IntentService;
import android.app.PendingIntent;
import android.content.Intent;


import android.os.Binder;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;


public class AndroidAudioRecordService extends IntentService{


    AndroidAudioRecordNode main_node;
    NodeMain talker;
    Intent mintent;

    private String package_name;

    final IBinder binder = new MyBinder();

    public class MyBinder extends Binder{
	AndroidAudioRecordService getService(){
	    return AndroidAudioRecordService.this;
	}
    }

    public AndroidAudioRecordService(){
	super("Service");
    }

    @Override
	public IBinder onBind(Intent intent){
	mintent = intent;

	return binder;
    }


    public void setNode(String package_name){
	showNotification();
	this.package_name = package_name;


	// We use this bundle                                 
	Bundle b = mintent.getExtras();


	if(this.main_node == null){
	    try{
		URI masterUri = new URI(b.getString("masterUri"));
		String hostLocal = InetAddressFactory.newNonLoopback().getHostAddress();
		// At this point, the user has already been prompted to either enter the URI                                  // of a master to use or to start a master locally.           
		    NodeMainExecutor e = DefaultNodeMainExecutor.newDefault();

		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostLocal,masterUri);
		nodeConfiguration.setMasterUri(masterUri);
		nodeConfiguration.setNodeName("/android/audio");

		talker = new AndroidAudioRecordNode(package_name);

		e.execute(talker, nodeConfiguration);


	    }catch(URISyntaxException e){
		e.printStackTrace();
	    }
	}
    }

    @Override
	protected void onHandleIntent(Intent intent) {
	try {
	    Thread.sleep(10000);
	} catch (InterruptedException e) {
	    e.printStackTrace();
	}

    }


    /**                                                                           
     * Show a notification while this service is running.                         
     */
    private void showNotification() {

	Intent i=new Intent(this, MainActivity.class);

	i.setFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP|
		   Intent.FLAG_ACTIVITY_SINGLE_TOP);

	PendingIntent pi=PendingIntent.getActivity(this, 0, i, 0);

    }

    @Override
	public void onDestroy(){
	this.talker.onShutdown(null);
    }
}

