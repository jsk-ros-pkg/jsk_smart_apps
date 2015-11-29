package org.ros.android.androidVoiceMessage;

import java.util.ArrayList;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.graphics.Paint.Style;
import android.view.SurfaceHolder;
import android.view.SurfaceView;


public class AndroidVoiceMessageView extends SurfaceView implements
		SurfaceHolder.Callback, Runnable {

	private SurfaceHolder holder;
	private Thread thread;
	private Paint paint;
	private Canvas canvas;
	private int width;
	private float originalY;
	private float scale;

	private ArrayList<Float> volumeArray;

	public AndroidVoiceMessageView(Context context) {
		super(context);
		// initialize
		volumeArray = new ArrayList<Float>();
		width = getWidth();
		originalY = getHeight();
		scale = 3;

		paint = new Paint();
		paint.setStyle(Style.FILL);
		paint.setTextSize(32);
		paint.setTypeface(Typeface.create(Typeface.SERIF, Typeface.BOLD_ITALIC));

	}
	
	public void setView(SurfaceView surfaceView){
		holder = surfaceView.getHolder();
		holder.addCallback(this);
		holder.setFixedSize(480, 262);
	}

	public void surfaceCreated(SurfaceHolder holder) {
		thread = new Thread(this);
		thread.start();
	}

	public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
		width = w;

	}

	public void surfaceDestroyed(SurfaceHolder holder) {
		thread = null;
	}

	public void run() {
		while (true) {
			try {
				int size = volumeArray.size();
				canvas = holder.lockCanvas();
				canvas.drawColor(Color.BLACK);
				paint.setColor(Color.GRAY);
					canvas.drawLine(0, originalY, width, originalY, paint);
				
				paint.setColor(Color.WHITE);
					for (int i = 0; i < size - 2; i++) {
						canvas.drawLine(i*3, volumeArray.get(i)*scale + originalY, (i + 1)*3,
								volumeArray.get(i + 1)*scale + originalY, paint);
					}
			
				holder.unlockCanvasAndPost(canvas);

			} catch (Exception e) {

			}
		}
	}

	public void addValues(float value) {
		volumeArray.add(value);
		if (volumeArray.size() > width/3+1) {
			volumeArray.remove(0);
		}
	}

}
