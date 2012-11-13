package org.ros.android.android_camera_viewer;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import android.util.AttributeSet;
import org.ros.android.MessageCallable;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import android.content.Intent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;


import android.os.Bundle;
import android.os.Environment;
import android.os.Looper;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.widget.Button;
import jsk_gui_msgs.VoiceMessage;
import android.view.View;

//add
import android.media.AudioManager;
import android.media.AudioRecord;
import android.media.AudioTrack;
import android.media.MediaRecorder;
import android.media.AudioFormat;
import android.util.Log;
import audio_common_msgs.AudioData;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import static org.jboss.netty.buffer.ChannelBuffers.*;
import org.jboss.netty.buffer.ChannelBuffer;
import java.nio.ByteBuffer;

import org.ros.node.*;


public class AndroidAudioRecordNode implements NodeMain {



    private int[] flag;
    private int startFlag = 0;
    private int audioFlag = 0;
    private String package_name;

    //add
    private Publisher<audio_common_msgs.AudioData> audio_pub;
    private Subscriber<audio_common_msgs.AudioData> audio_sub;
    private audio_common_msgs.AudioData audio_msg;
    final static int SAMPLING_RATE = 11025; //8000,11025,16000,22050,44100
    AudioRecord audioRec = null;
    AudioTrack audioTra = null;
    int bufSize;

    
    
     
    @Override
	public GraphName getDefaultNodeName() {
	return GraphName.of("audio_common_msgs/AudioRecord");
    }

    public AndroidAudioRecordNode(String package_name){
	this.package_name = package_name;
    }



    @Override
	public void onStart(final ConnectedNode connectedNode) {
		  		Log.v("service","fire!==================================--");
		  		
		  	audio_sub = connectedNode.newSubscriber("audio2", "audio_common_msgs/AudioData");
		  	 audio_sub.addMessageListener(new MessageListener<audio_common_msgs.AudioData>() {
		  	      @Override
		  	      public void onNewMessage(final audio_common_msgs.AudioData message) {
		  	    	  //Log.v("test",message.getData());
		  	          //Log.v("test",message.toString());
		  	        ChannelBuffer heapBuffer = message.getData();
		  	        byte buf[] = new byte[heapBuffer.readableBytes()];
		  	        heapBuffer.readBytes(buf,0,heapBuffer.readableBytes());
		  	          Log.v("test",buf[0]+","+buf[1]);
		  	          if(audioFlag > 0){ // 動かすときは>0
		  	          //audioTra.reloadStaticData();
		  	          Log.v("state",""+audioTra.getState());
		  	          audioTra.write(buf, 0, buf.length);
		  	          audioTra.play();
		  	          
		  	          }
		  	      }
		  	    });
		  		
		      //バッファサイズの計算
    bufSize = AudioRecord.getMinBufferSize(
					   SAMPLING_RATE,
					   AudioFormat.CHANNEL_CONFIGURATION_MONO,
					   AudioFormat.ENCODING_PCM_16BIT)*2;
    try{

	audio_pub = connectedNode.newPublisher("audio","audio_common_msgs/AudioData");
	Log.v("test","publish start===================================="+bufSize);
	startFlag=1;

    }catch (Exception e){
			  
    }

    //AudioRecordの作成
    audioRec = new AudioRecord(
			       MediaRecorder.AudioSource.MIC,
			       SAMPLING_RATE,
			       AudioFormat.CHANNEL_CONFIGURATION_MONO,
			       AudioFormat.ENCODING_PCM_16BIT,
			       bufSize);
	
    //AudioPlayの作成

	audioTra = new AudioTrack(
			AudioManager.STREAM_MUSIC,                //streamType STREAM_MUSIC
            SAMPLING_RATE,                        //sampleRateInHz:サンプリング周波数
            AudioFormat.CHANNEL_CONFIGURATION_MONO,    //channelConfig(モノラル指定)
            AudioFormat.ENCODING_PCM_16BIT,            //audioFormat
            bufSize*10,                        //bufferSizeInBytes:バッファサイズ(奇数だとエラーになる模様
            AudioTrack.MODE_STREAM);                //mode(STATICモード指定)
			
    audioFlag = 1;
	Log.v("AudioRecord","startRecording");
	audioRec.startRecording();
	
	//Record
	new Thread(new Runnable(){
		@Override
		    public void run(){

			
		    //new!
			try{
		 // 録音用ファイル取得  SDカード状態チェック略
		    File recFile =new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/rec.raw");
		    recFile.createNewFile();
		    // ファイル書き込み  例外処理略
		    FileOutputStream out =  new FileOutputStream(recFile);
			
		    
		    
		    byte buf[] = new byte[bufSize];

		    while(startFlag<300){
		    	
			int returnSize = audioRec.read(buf,0,buf.length);
			Log.v("returnSize",""+returnSize);
			//if(startFlag > 0){
				 
					audio_msg = audio_pub.newMessage();
					ChannelBuffer heapBuffer = buffer(LITTLE_ENDIAN,bufSize);
					//Log.v("audio2",""+heapBuffer.writableBytes()+","+heapBuffer.isDirect()+","+heapBuffer.order());
					heapBuffer.writeBytes(buf);
					
					//Log.v("heap",hexDump(heapBuffer));
					
					audio_msg.setData(heapBuffer);
					
					if(startFlag<300) audio_pub.publish(audio_msg); //本当は<300というか無しで
					//Log.v("audio","published!");
			  
			
			//Thread.sleep(20);
			
			
			Log.v("AudioRecord","read:"+buf[0]+","+buf[1]);
		    //out.write(buf);
		    }
		    
		    //new!
		    Log.v("AudioRecord","stop");
		    audioRec.stop();
		    out.close();
		    out = null;
		    addWavHeader();
		    Log.v("AudioRecord","success!");

		
		}catch(Exception e){
			
		}
		}}).start();

	
	
    }
	
    //new!
 // WAVヘッダをつけて保存
 // 各種例外処理略 チェック処理略
 public void addWavHeader() {
	 
	 try{
     // 録音したファイル
     File recFile = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/rec.raw");
     // WAVファイル
     File wavFile = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/rec.wav");
     // ストリーム
     FileInputStream in = new FileInputStream(recFile);
     FileOutputStream out = new FileOutputStream(wavFile);
     
     // ヘッダ作成  サンプルレート8kHz
     byte[] header = createHeader(SAMPLING_RATE, (int)recFile.length());
     // ヘッダの書き出し
     out.write(header);
     Log.v("recFile",""+(int)recFile.length());
     // 録音したファイルのバイトデータ読み込み
     int n = 0,offset = 0;
     byte[] buffer = new byte[(int)recFile.length()];
     while (offset < buffer.length
             && (n = in.read(buffer, offset, buffer.length - offset)) >= 0) {
         offset += n;
     }
     // バイトデータ書き込み
     out.write(buffer);
      
     // 終了
     in.close();
     out.close();
	 }catch(IOException e){
		 
	 }
 }
  
    

    public static byte[] createHeader(int sampleRate, int datasize) {
	byte[] byteRIFF = {'R', 'I', 'F', 'F'};
	byte[] byteFilesizeSub8 = intToBytes((datasize + 36));  // ファイルサイズ-8バイト数
	byte[] byteWAVE = {'W', 'A', 'V', 'E'};
	byte[] byteFMT_ = {'f', 'm', 't', ' '};
	byte[] byte16bit = intToBytes(16);                  // fmtチャンクのバイト数
	byte[] byteSamplerate = intToBytes(sampleRate);     // サンプルレート
	byte[] byteBytesPerSec = intToBytes(sampleRate * 2);    // バイト/秒 = サンプルレート x 1チャンネル x 2バイト
	byte[] bytePcmMono = {0x01, 0x00, 0x01, 0x00};      // フォーマットID 1 =リニアPCM  ,  チャンネル 1 = モノラル
	byte[] byteBlockBit = {0x02, 0x00, 0x10, 0x00};     // ブロックサイズ2バイト サンプルあたりのビット数16ビット
	byte[] byteDATA = {'d', 'a', 't', 'a'};
	byte[] byteDatasize = intToBytes(datasize);         // データサイズ
 
	ByteArrayOutputStream out = new ByteArrayOutputStream();
	try {
	    out.write(byteRIFF);
	    out.write(byteFilesizeSub8);
	    out.write(byteWAVE);
	    out.write(byteFMT_);
	    out.write(byte16bit);
	    out.write(bytePcmMono);
	    out.write(byteSamplerate);
	    out.write(byteBytesPerSec);
	    out.write(byteBlockBit);
	    out.write(byteDATA);
	    out.write(byteDatasize);
 
	} catch (IOException e) {
	    return out.toByteArray();
	}
 
	return out.toByteArray();
    }
    
    // int型32ビットデータをリトルエンディアンのバイト配列にする
    public static byte[] intToBytes(int value) {
	byte[] bt = new byte[4];
	bt[0] = (byte)(value & 0x000000ff);
	bt[1] = (byte)((value & 0x0000ff00) >> 8);
	bt[2] = (byte)((value & 0x00ff0000) >> 16);
	bt[3] = (byte)((value & 0xff000000) >> 24);
	return bt;
    }

    @Override
	public void onShutdown(Node node) {
	Log.v("AudioRecord","stop");
	audioRec.stop();
	audioRec.release();
 
    }

    @Override
	public void onShutdownComplete(Node node) {
    }

    @Override
	public void onError(Node node, Throwable throwable) {
    }
	    
	 	    	
}