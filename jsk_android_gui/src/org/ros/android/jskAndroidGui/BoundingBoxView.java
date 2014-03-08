package org.ros.android.jskAndroidGui;

import java.util.ArrayList;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.Point;
import android.graphics.PorterDuff.Mode;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

public class BoundingBoxView extends SurfaceView implements Runnable,
		SurfaceHolder.Callback {

	private ArrayList<Point> points;
	private SurfaceHolder surfaceHolder;
	private Thread thread;
	private Boolean selected;
	private Long selected_time;

	public BoundingBoxView(Context context) {
		super(context);
		surfaceHolder = getHolder();
		surfaceHolder.addCallback(this);
		surfaceHolder.setFormat(PixelFormat.TRANSLUCENT);
		setZOrderOnTop(true);
		points = new ArrayList<Point>();
		selected = false;
	}

	public void setSelect() {
		selected_time = System.currentTimeMillis();
		selected = true;
	}

	public void drawBox(ArrayList<Point> point_list) {
		points = point_list;
	}

	@Override
	public void run() {
		Canvas canvas = null;
		Paint paint = new Paint();
		paint.setStrokeWidth(5);
		paint.setARGB(255, 255, 0, 0);

		while (thread != null) {
			try {
				canvas = surfaceHolder.lockCanvas();
				paint.setStyle(Paint.Style.FILL);
				canvas.drawColor(0, Mode.CLEAR);
				canvas.drawColor(Color.TRANSPARENT);

				if (points.size() >= 2) {
					int size = points.size();
					paint.setStyle(Paint.Style.STROKE);
					if (selected) {
						paint.setColor(Color.BLUE);
						paint.setTextSize(55);
						canvas.drawText("Get Object Image", points.get(0).x,
								points.get(0).y - 30, paint);
					} else {
						paint.setColor(Color.RED);
					}
					canvas.drawRect(points.get(0).x, points.get(0).y,
							points.get(size - 1).x, points.get(size - 1).y,
							paint);
				}

				if (selected
						&& System.currentTimeMillis() - selected_time > 2000) {
					deleteBox();
				}
			} catch (Exception e) {

			} finally {

				surfaceHolder.unlockCanvasAndPost(canvas);
			}
		}
	}

	public void deleteBox() {
		selected = false;
		points.clear();
	}

	@Override
	public void surfaceChanged(SurfaceHolder arg0, int arg1, int arg2, int arg3) {

	}

	@Override
	public void surfaceCreated(SurfaceHolder arg0) {
		thread = new Thread(this);
		thread.start();
	}

	@Override
	public void surfaceDestroyed(SurfaceHolder arg0) {
		thread = null;
	}

}
