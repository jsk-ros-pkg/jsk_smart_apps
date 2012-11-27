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

	private SurfaceHolder holder; // サーフェイスホルダー
	private Thread thread; // スレッド
	private Paint paint; // 描画用
	private Canvas canvas;// キャンバス
	private int width;
	private int viewFlag[];

	// パラメータ
	private ArrayList<Float> Xarray;
	private ArrayList<Float> Yarray;
	private ArrayList<Float> Zarray;

	public AndroidSensorMessageView(Context context, SurfaceView sv) {
		super(context);
		// パラメータの初期化
		Xarray = new ArrayList<Float>();
		Yarray = new ArrayList<Float>();
		Zarray = new ArrayList<Float>();
		width = getWidth();
		viewFlag = new int[4];
		for (int i = 0; i < 4; i++) {
			viewFlag[i] = 1;
		}

		// 描画画面の初期化
		paint = new Paint();
		paint.setStyle(Style.FILL);
		paint.setTextSize(32);
		paint.setTypeface(Typeface.create(Typeface.SERIF, Typeface.BOLD_ITALIC));

		// サーフェイスホルダーの生成
		holder = sv.getHolder();
		holder.addCallback(this);
		holder.setFixedSize(480, 262);
	}

	// サーフェイスの生成
	public void surfaceCreated(SurfaceHolder holder) {
		// スレッドの開始
		thread = new Thread(this);
		thread.start();
	}

	// サーフェイスの変更
	public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
		width = w;

	}

	// サーフェイスの破棄
	public void surfaceDestroyed(SurfaceHolder holder) {
		thread = null;
	}

	public void run() {
		while (true) {
			try {
				//数の変動を考えなければならない
				int size = Xarray.size();
				canvas = holder.lockCanvas();
				canvas.drawColor(Color.WHITE);
				paint.setColor(Color.BLACK);
				if (viewFlag[0] == 1) {
					canvas.drawLine(0, 150, width, 150, paint);
				}
				paint.setColor(Color.RED);
				if (viewFlag[1] == 1) {
					for (int i = 0; i < size - 2; i++) {
						canvas.drawLine(i, Xarray.get(i) + 150, i + 1,
								Xarray.get(i + 1) + 150, paint);
					}
				}
				if (viewFlag[2] == 1) {
					paint.setColor(Color.BLUE);
					for (int i = 0; i < size - 2; i++) {
						canvas.drawLine(i, Yarray.get(i) + 150, i + 1,
								Yarray.get(i + 1) + 150, paint);
					}
				}
				if (viewFlag[3] == 1) {
					paint.setColor(Color.GREEN);
					for (int i = 0; i < size - 2; i++) {
						canvas.drawLine(i, Zarray.get(i) + 150, i + 1,
								Zarray.get(i + 1) + 150, paint);
					}
				}
				holder.unlockCanvasAndPost(canvas);

			} catch (Exception e) {

			}
		}
	}

	// 値の追加
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

	// 表示非表示
	public void viewChange(int flag) {
		viewFlag[flag] *= -1;
	}
	
	// クリア
	public void clearValues(){
	    /*		Xarray.clear();
		Yarray.clear();
		Zarray.clear();*/
	}

}
