package com.chris.gaze.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PointF;
import android.os.Build;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;

public class PointView extends View {
    public static final int TYPE_DEFAULT = 0;
    public static final int TYPE_OUT_OF_SCREEN = 1;
    private final int defaultColor = Color.rgb(0x00, 0x00, 0xff);
    private final int outOfScreenColor = Color.rgb(0xff, 0x00, 0x00);
    private final int type = TYPE_DEFAULT;
    private final PointF position = new PointF();
    private Paint paint;
    private boolean showLine = true;
    private float offsetX, offsetY;

    public PointView(Context context) {
        super(context);
        init();
    }

    public PointView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public PointView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    @RequiresApi(Build.VERSION_CODES.LOLLIPOP)
    public PointView(Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
        init();
    }

    private void init() {
        paint = new Paint();
        paint.setColor(defaultColor);
        paint.setStrokeWidth(2f);
    }

    public void setOffset(int x, int y) {
        offsetX = x;
        offsetY = y;
    }

    public void setPosition(float x, float y) {
        position.x = x - offsetX;
        position.y = y - offsetY;
        invalidate();
    }

    public void hideLine() {
        showLine = false;
    }

    public void showLine() {
        showLine = true;
    }

    public void setType(int type) {
        paint.setColor(type == TYPE_DEFAULT ? defaultColor : outOfScreenColor);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        canvas.drawCircle(position.x, position.y, 10, paint);
        if (showLine) {
            canvas.drawLine(0, position.y, getWidth(), position.y, paint);
            canvas.drawLine(position.x, 0, position.x, getHeight(), paint);
        }
    }
}
