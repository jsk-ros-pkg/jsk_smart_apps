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

package org.ros.android.androidSensorMessage;

import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;

import org.ros.android.MasterChooser;
import org.ros.android.RosActivity;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeMainExecutor;
import android.content.ServiceConnection;

//add libraries
import java.net.URI;
import java.net.URISyntaxException;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.widget.LinearLayout;
import android.widget.Button;
import android.util.Log;
import android.content.ComponentName;
import android.os.IBinder;
import android.content.Context;
import android.view.View;
import android.widget.TextView;
import android.widget.CheckBox;


public class AndroidSensorMessage extends RosActivity {


  private URI masterUri;
  private Intent intent;
  AndroidSensorService myService;
  private int startFlag = 0;
  private int[] useFlag;
  private int[] useFlag2;
  
  TextView text0;
  TextView text1;
  TextView text2;
  TextView text3;
  TextView text4;
  TextView text5;
  TextView text6;
  TextView text7;
  TextView text8;  
  CheckBox check0;
  CheckBox check1;
  CheckBox check2;
  CheckBox check3;
  CheckBox check4;
  CheckBox check5;
  CheckBox check6;
  CheckBox check7;
  CheckBox check8;
  Button btn;
  LinearLayout linearLayout;

  
  ServiceConnection serviceConnection = new ServiceConnection(){
	  

	    
	    @Override
	    public void onServiceConnected(ComponentName name, IBinder service) {
	      myService = ((AndroidSensorService.MyBinder)service).getService();
	      startService(intent);  	   
	      useFlag = new int[9];
	      useFlag2 = new int[9];
	       text0 = (TextView)findViewById(R.id.text_0);
	       text1 = (TextView)findViewById(R.id.text_1);
	    	 text2  = (TextView)findViewById(R.id.text_2);
		    text3 = (TextView)findViewById(R.id.text_3);
		    text4 = (TextView)findViewById(R.id.text_4);
		    text5  = (TextView)findViewById(R.id.text_5);
			 text6 = (TextView)findViewById(R.id.text_6);
			 text7 = (TextView)findViewById(R.id.text_7);
			 text8  = (TextView)findViewById(R.id.text_8);
			 check0 = (CheckBox)findViewById(R.id.checkbox0);
			 check1 = (CheckBox)findViewById(R.id.checkbox1);
			 check2 = (CheckBox)findViewById(R.id.checkbox2);
			 check3 = (CheckBox)findViewById(R.id.checkbox3);
			 check4 = (CheckBox)findViewById(R.id.checkbox4);
			 check5 = (CheckBox)findViewById(R.id.checkbox5);
			 check6 = (CheckBox)findViewById(R.id.checkbox6);
			 check7 = (CheckBox)findViewById(R.id.checkbox7);
			 check8 = (CheckBox)findViewById(R.id.checkbox8);
			 btn = (Button)findViewById(R.id.btn);

	      for(int i=0;i<9;i++){
  	    	useFlag[i]=myService.getSensor(i);
  	    	useFlag2[i]=0;
  	    	
  	    }
	      if(useFlag[0]==0){
	    	  text0.setVisibility(View.INVISIBLE);
	    	  check0.setVisibility(View.INVISIBLE);
	      }
	      if(useFlag[1]==0){
	    	  text1.setVisibility(View.INVISIBLE);
	    	  check1.setVisibility(View.INVISIBLE);
	      }
	      if(useFlag[2]==0){
	    	  text2.setVisibility(View.INVISIBLE);
	    	  check2.setVisibility(View.INVISIBLE);
	      }
	      if(useFlag[3]==0){
	    	  text3.setVisibility(View.INVISIBLE);
	    	  check3.setVisibility(View.INVISIBLE);
  }
	      if(useFlag[4]==0){
	    	  text4.setVisibility(View.INVISIBLE);
	    	  check4.setVisibility(View.INVISIBLE);
 }
	      if(useFlag[5]==0){
	    	  text5.setVisibility(View.INVISIBLE);
	    	  check5.setVisibility(View.INVISIBLE);
  }
	      if(useFlag[6]==0){
	    	  text6.setVisibility(View.INVISIBLE);
	    	  check6.setVisibility(View.INVISIBLE);
   }
	      if(useFlag[7]==0){
	    	  text7.setVisibility(View.INVISIBLE);
	    	  check7.setVisibility(View.INVISIBLE);
  }
	      if(useFlag[8]==0){
	    	  text8.setVisibility(View.INVISIBLE);
	    	  check8.setVisibility(View.INVISIBLE);
  }
	      btn.setOnClickListener(new View.OnClickListener() {
	            @Override
	            public void onClick(View v) {
	                // ボタンがクリックされた時に呼び出されます
	            	myService.setNode(useFlag2);
	            	setContentView(linearLayout);
	            }
	        });
	      check0.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[0]=1;
	            }
	        });
	      check1.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[1]=1;
	            }
	        });
	      check2.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[2]=1;
	            }
	        });	    
	      check3.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[3]=1;
	            }
	        });
	      check4.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[4]=1;
	            }
	        });	     
	      check5.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[5]=1;
	            }
	        });	   
	      check6.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[6]=1;
	            }
	        });	    
	      check7.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[7]=1;
	            }
	        });
	      check8.setOnClickListener(new View.OnClickListener() {
	            @Override
	            // チェックボックスがクリックされた時に呼び出されます
	            public void onClick(View v) {
	                CheckBox checkBox = (CheckBox) v;
	                // チェックボックスのチェック状態を取得します
	                if(checkBox.isChecked())useFlag2[8]=1;
	            }
	        });

	    }

	    @Override
	    public void onServiceDisconnected(ComponentName name) {
	      myService = null;
	    }
	    
	  };
  

  public AndroidSensorMessage() {
    // The RosActivity constructor configures the notification title and ticker
    // messages.
    super("AndroidSensorMessage", "AndroidSensorMessage");

  }

  //@SuppressWarnings("unchecked")
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
	linearLayout = new LinearLayout(this);
    setContentView(R.layout.main);
  }

  private LinearLayout.LayoutParams createParam(int w,int h){
	  return new LinearLayout.LayoutParams(w,h);
  }
  
  
    @Override
    protected void onResume(){
    	super.onResume();
    	if (masterUri != null){
    		
    		intent = new Intent(getBaseContext(),AndroidSensorService.class);
    	    intent.putExtra("masterUri", masterUri.toString());
    	    bindService(intent,serviceConnection,Context.BIND_AUTO_CREATE);
    	    
  
    	    
    	      /*intent = new Intent(this, AndroidSensorService.class);
    	      intent.putExtra("masterUri", masterUri.toString());
    	      startService(intent);*/
    	}
	
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
      if (requestCode == 0 && resultCode == RESULT_OK) {
        try {
          masterUri = new URI(data.getStringExtra("ROS_MASTER_URI"));
        } catch (URISyntaxException e) {
          throw new RuntimeException(e);
        }
      }
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
    	menu.add(0, 0, 0, R.string.app_about);
    	menu.add(0, 1, 1, R.string.str_exit);
    	return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
    	super.onOptionsItemSelected(item);
    	switch(item.getItemId())
    	{
    	case 0:
    	openOptionsDialog();
    	break;
    	case 1:
    	exitOptionsDialog();
    	break;
    	}
    	return true;
    }

    private void openOptionsDialog(){
    	new AlertDialog.Builder(this).setTitle(R.string.app_about).setMessage(R.string.app_about_message).setPositiveButton(R.string.str_ok,new DialogInterface.OnClickListener(){public void onClick(DialogInterface dialoginterface, int i){}}).show();
    }

    private void exitOptionsDialog()
    {
    	stopService(new Intent(this, AndroidSensorService.class));
    	finish();
    }
    

    @Override
	protected void onPause() {
	super.onPause();
    }
 
  


  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
    
  }
}
