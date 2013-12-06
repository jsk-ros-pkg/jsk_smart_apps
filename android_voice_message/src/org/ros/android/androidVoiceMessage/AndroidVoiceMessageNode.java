package org.ros.android.androidVoiceMessage;

import static org.jboss.netty.buffer.ChannelBuffers.LITTLE_ENDIAN;
import static org.jboss.netty.buffer.ChannelBuffers.buffer;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.List;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import android.content.Context;
import android.content.Intent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;

import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import android.widget.ImageView;
import jsk_gui_msgs.VoiceMessage;
import android.view.View;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import audio_common_msgs.AudioData;
import android.media.MediaRecorder;

import org.ros.node.*;

public class AndroidVoiceMessageNode implements NodeMain,
		VoiceNodeListenerInterface, Runnable {

	// for text_message
	private Publisher<jsk_gui_msgs.VoiceMessage> voice_pub;
	private jsk_gui_msgs.VoiceMessage voice_msg;
	private int startFlag = 0;
	private int not_working_flag = 0;
	private int mode = 1;
	private SpeechRecognizer sr;
	private String package_name;
	private TextView textView;
	private ImageView imageView;
	private AndroidVoiceMessageView androidVoiceMessageView;
	private SpeechListener sl;
	// for raw_sound
	private Publisher<audio_common_msgs.AudioData> audio_pub;
	private audio_common_msgs.AudioData audio_msg;
	final static int SAMPLING_RATE = 11025; // 8000,11025,16000,22050,44100
	AudioRecord audioRec = null;
	private int bufSize;
	private int audioFlag = 0;
	private Thread managerLoop;
	private Handler handler;
	private Context context;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("jsk_gui_msgs/VoiceMessage");
	}

	public AndroidVoiceMessageNode(SensorManager manager, Context context,
			String package_name) {
		this.context = context;
		sr = SpeechRecognizer.createSpeechRecognizer(context);
		sl = new SpeechListener();
		sl.setListener(this);
		sr.setRecognitionListener(sl);
		this.package_name = package_name;

		handler = new Handler();

		managerLoop = new Thread(this);
		managerLoop.start();
	}

	public void setView(TextView textView, ImageView imageView) {
		this.textView = textView;
		this.imageView = imageView;
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {

		// for raw_audio
		bufSize = AudioRecord.getMinBufferSize(SAMPLING_RATE,
				AudioFormat.CHANNEL_CONFIGURATION_MONO,
				AudioFormat.ENCODING_PCM_16BIT) * 2;

		try {
			voice_pub = connectedNode.newPublisher("Tablet/voice",
					"jsk_gui_msgs/VoiceMessage");
			audio_pub = connectedNode.newPublisher("audio",
					"audio_common_msgs/AudioData");
			startFlag = 1;

		} catch (Exception e) {

		}

		// AudioRecord���������
		audioRec = new AudioRecord(MediaRecorder.AudioSource.MIC,
				SAMPLING_RATE, AudioFormat.CHANNEL_CONFIGURATION_MONO,
				AudioFormat.ENCODING_PCM_16BIT, bufSize);

	}

	public void stopRawSound() {
		audioFlag = 0;
	}

	public void publishRawSound(AndroidVoiceMessageView androidVoiceMessageView) {
		this.androidVoiceMessageView = androidVoiceMessageView;
		audioFlag = 1;
		audioRec.startRecording();
		// Record
		new Thread(new Runnable() {
			@Override
			public void run() {

				try {

					byte buf[] = new byte[bufSize];

					while (audioFlag == 1) {

						int returnSize = audioRec.read(buf, 0, buf.length);

						audio_msg = audio_pub.newMessage();
						ChannelBuffer heapBuffer = buffer(LITTLE_ENDIAN,
								bufSize);
						heapBuffer.writeBytes(buf);
						audio_msg.setData(heapBuffer);
						audio_pub.publish(audio_msg);

						double volume = 0;
						for (int i = 0; i < buf.length; i++) {
							volume += Math.abs(buf[i]);
						}
						volume /= buf.length;
						addValue((float) volume);
						Log.v("buf-size", "" + volume);

					}
					Log.v("AudioRecord", "stop");
					audioRec.stop();

				} catch (Exception e) {

				}
			}
		}).start();

	}

	private void addValue(float volume) {
		androidVoiceMessageView.addValues(volume);
	}

	@Override
	public void onShutdown(Node node) {
		// sr.destroy();
		// voice_pub.shutdown();
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public void onError(Node node, Throwable throwable) {
	}

	public void setMode(int mode) {
		this.mode = mode;
		if (mode == -1) {
			setFlag(-1);
		} else {
			sr.stopListening();
		}
	}

	public void setFlag(int type) {

		if (startFlag == 1 && type == mode) {
			Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
			intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
					RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
			intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE,
					package_name);
			Log.v("voice", "button pushed!");
			sr.startListening(intent);
			imageView.setImageResource(R.drawable.mike2);
		}
	}

	/* inner class */
	public class SpeechListener implements RecognitionListener {

		private VoiceNodeListenerInterface listener;

		public void setListener(VoiceNodeListenerInterface listener) {
			this.listener = listener;
		}

		@Override
		public void onBeginningOfSpeech() {
			imageView.setImageResource(R.drawable.mike3);
		}

		@Override
		public void onBufferReceived(byte[] buffer) {
			Log.e("voice", "buffer received");
			// TODO Auto-generated method stub
		}

		@Override
		public void onEndOfSpeech() {
			Log.e("voice", "speech end");
			imageView.setImageResource(R.drawable.mike);
		}

		@Override
		public void onError(int error) {
			imageView.setImageResource(R.drawable.mike);

			switch (error) {
			case SpeechRecognizer.ERROR_AUDIO:
				Log.e("voice", "save error");
				break;
			case SpeechRecognizer.ERROR_CLIENT:
				Log.e("voice", "device error");
				break;
			case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
				Log.e("voice", "nothing of right");
				break;
			case SpeechRecognizer.ERROR_NETWORK:
				Log.e("voice", "network error");
				break;
			case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
				Log.e("voice", "network timeout");
				break;
			case SpeechRecognizer.ERROR_NO_MATCH:
				Log.e("voice", "nothing recognition");
				not_working_flag = 0;
				break;
			case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
				Log.e("voice", "not request");
				if (mode == 1) {
					restart();
				}
				break;
			case SpeechRecognizer.ERROR_SERVER:
				Log.e("voice", "from server");
				break;
			case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
				Log.e("voice", "no input");
				not_working_flag = 0;
				break;
			default:
			}
			listener.end();
		}

		@Override
		public void onEvent(int eventType, Bundle params) {
			Log.e("voice", "event");
			// TODO Auto-generated method stub

		}

		@Override
		public void onPartialResults(Bundle partialResults) {
			// TODO Auto-generated method stub

		}

		@Override
		public void onReadyForSpeech(Bundle params) {
			Log.v("voice", "ready!");
			// TODO Auto-generated method stub
		}

		@Override
		public void onResults(Bundle results) {
			Log.v("voice", "result");
			voice_msg = voice_pub.newMessage();
			ArrayList<String> candidates = results
					.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
			voice_msg.setTexts(candidates);
			textView.setText("recognized text candidates:\n");
			for (int i = 0; i < candidates.size(); i++) {
				textView.setText(textView.getText() + "" + (i + 1) + ") "
						+ candidates.get(i) + "\n");
			}
			Log.v("voice", "publish ready");
			voice_msg.setTexts(candidates);
			Log.v("test", "publish ready");
			if (startFlag == 1) {
				voice_pub.publish(voice_msg);
				Log.v("voice", "publish ok");
			}
			not_working_flag = 0;
			listener.end();
		}

		@Override
		public void onRmsChanged(float rmsdB) {
		}

	}

	@Override
	public void end() {
		Log.e("voice", "end");
		setFlag(-1);
	}

	public void restart() {
		handler.post(new Runnable() {

			@Override
			public void run() {

					sr.stopListening();
					sr.destroy();

					sr = null;
					sl = null;

					try {
						Thread.sleep(1000);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}

					sr = SpeechRecognizer.createSpeechRecognizer(context);
					sl = new SpeechListener();
					sl.setListener(AndroidVoiceMessageNode.this);
					try {
						Thread.sleep(1000);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
					sr.setRecognitionListener(sl);
					

					if (mode == -1) {
						setFlag(-1);
						
					}
			}

		});
		not_working_flag = 0;
	}

	@Override
	public void run() {
		while (true) {
			if (startFlag == 1 && mode == -1) {
				not_working_flag++;
				if (not_working_flag > 2) {
					Log.v("voice", "restart");
					restart();
				} else
					Log.v("voice", "no problem");
			} else {
				not_working_flag = 0;
			}
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

	}

}
