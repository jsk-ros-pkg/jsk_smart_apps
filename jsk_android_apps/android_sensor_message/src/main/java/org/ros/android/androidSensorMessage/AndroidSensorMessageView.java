package org.ros.android.androidSensorMessage;

import java.util.ArrayList;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.graphics.Paint.Style;
import android.view.SurfaceHolder;
import android.view.SurfaceView;


public class AndroidSensorMessageView extends SurfaceView implements
		SurfaceHolder.Callback, Runnable {

	private SurfaceHolder holder;
	private Thread thread;
	private Paint paint;
	private Canvas canvas;
	private int width;
	private int viewFlag[];
	private float originalY;
	private float scale;

	private ArrayList<Float> Xarray;
	private ArrayList<Float> Yarray;
	private ArrayList<Float> Zarray;

	public AndroidSensorMessageView(Context context, SurfaceView sv) {
		super(context);
		// initialize
		Xarray = new ArrayList<Float>();
		Yarray = new ArrayList<Float>();
		Zarray = new ArrayList<Float>();
		width = getWidth();
		viewFlag = new int[4];
		originalY = 150;
		scale = 1;
		for (int i = 0; i < 4; i++) {
			viewFlag[i] = 1;
		}

		paint = new Paint();
		paint.setStyle(Style.FILL);
		paint.setTextSize(32);
		paint.setTypeface(Typeface.create(Typeface.SERIF, Typeface.BOLD_ITALIC));

		holder = sv.getHolder();
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
				int size = Xarray.size();
				canvas = holder.lockCanvas();
				canvas.drawColor(Color.WHITE);
				paint.setColor(Color.BLACK);
				if (viewFlag[0] == 1) {
					canvas.drawLine(0, originalY, width, originalY, paint);
				}
				paint.setColor(Color.RED);
				if (viewFlag[1] == 1) {
					for (int i = 0; i < size - 2; i++) {
						canvas.drawLine(i, Xarray.get(i)*scale + originalY, i + 1,
								Xarray.get(i + 1)*scale + originalY, paint);
					}
				}
				if (viewFlag[2] == 1) {
					paint.setColor(Color.BLUE);
					for (int i = 0; i < size - 2; i++) {
						canvas.drawLine(i, Yarray.get(i)*scale + originalY, i + 1,
								Yarray.get(i + 1)*scale + originalY, paint);
					}
				}
				if (viewFlag[3] == 1) {
					paint.setColor(Color.GREEN);
					for (int i = 0; i < size - 2; i++) {
						canvas.drawLine(i, Zarray.get(i)*scale + originalY, i + 1,
								Zarray.get(i + 1)*scale + originalY, paint);
					}
				}
				holder.unlockCanvasAndPost(canvas);

			} catch (Exception e) {

			}
		}
	}

	public void addValues(float[] values) {
		Xarray.add(values[0]);
		Yarray.add(values[1]);
		Zarray.add(values[2]);
		if (Xarray.size() > width+1) {
			Xarray.remove(0);
			Yarray.remove(0);
			Zarray.remove(0);
		}
	}

	public void viewChange(int flag) {
		viewFlag[flag] *= -1;
	}
	
	public void updateOriginalY(float update){
		originalY += update;
	}
	
	public void updateScale(float scale){
		if(scale > 0){
		this.scale += 0.05f;
		}
		else if(scale < 0){
			this.scale -= 0.05f;
		}
		if(this.scale <= (float)0.01){
			this.scale = (float)0.01;
		}
	}
}
