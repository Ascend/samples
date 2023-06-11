package com.chris.gaze.view;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.text.TextPaint;
import android.util.AttributeSet;
import android.view.View;
import android.view.ViewGroup;
import android.view.animation.Animation;
import android.view.animation.RotateAnimation;

import androidx.annotation.Nullable;

public class CalibrationViewer extends ViewGroup {

  private final float x = -999;
  private final float y = -999;
  private final int backgroundColor = Color.rgb(0x64, 0x5E, 0x5E);
  //  private int backgroundColor = Color.argb(0x88,0x64,0x5E, 0x5E);
  private final int redColor = Color.rgb(0xEF, 0x53, 0x50);
  private final int purpleColor = Color.rgb(0xAB, 0x47, 0xBC);
  private final int orangeColor = Color.rgb(0xFF, 0xA7, 0x26);
  private final int blueColor = Color.rgb(0x42, 0xA5, 0xF5);
  private final int greenColor = Color.rgb(0x66, 0xBB, 0x6A);
  private final int brownColor = Color.rgb(0xCA, 0x9A, 0x00);
  private final int yellowColor = Color.rgb(0xFF, 0xFD, 0x00);
  private final TextPaint textPaint = new TextPaint();
  private int[] pointColors;
  private int index = 0;
  private Paint calibPoint;
  private Boolean toDraw = true;
  private CalibrationPoint calibrationPoint;
  private String msg;
  private float offsetX, offsetY;

  public CalibrationViewer(Context context) {
    super(context);
    init(context);
  }

  public CalibrationViewer(Context context, @Nullable AttributeSet attrs) {
    super(context, attrs);
    init(context);
  }

  public CalibrationViewer(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
    super(context, attrs, defStyleAttr);
    init(context);
  }

  public CalibrationViewer(Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes) {
    super(context, attrs, defStyleAttr, defStyleRes);
    init(context);
  }

  private void init(Context context) {
    textPaint.setAntiAlias(true);
    textPaint.setTextSize(16 * getResources().getDisplayMetrics().density);
    textPaint.setColor(0xFFFFFFFF);
    textPaint.setTextAlign(Paint.Align.CENTER);

    setBackgroundColor(backgroundColor);
    pointColors = new int[]{
            redColor, purpleColor, orangeColor, blueColor, greenColor, brownColor, yellowColor
    };
    calibPoint = new Paint();
    calibPoint.setAntiAlias(true);
    calibPoint.setColor(pointColors[index]);
    calibrationPoint = new CalibrationPoint(context);
    addView(calibrationPoint);
  }

  public void setOffset(int x, int y) {
    offsetX = x;
    offsetY = y;
  }

  private void drawText(Canvas canvas) {
    int xPos = (canvas.getWidth() / 2);
    int yPos = (int) ((canvas.getHeight() / 2) - ((textPaint.descent() + textPaint.ascent()) / 2));
    //((textPaint.descent() + textPaint.ascent()) / 2) is the distance from the baseline to the center.

    Rect r = new Rect();
    textPaint.getTextBounds(msg, 0, msg.length(), r);
    yPos -= (Math.abs(r.height())) / 2;

    canvas.drawText(msg, xPos, yPos, textPaint);
  }

  @Override
  protected void onLayout(boolean changed, int l, int t, int r, int b) {

  }

  @Override
  protected void onDraw(Canvas canvas) {
    super.onDraw(canvas);
    if (toDraw) {
      canvas.drawCircle(x, y, 10, calibPoint);
    }
    if (msg != null) {
      drawText(canvas);
    }
  }

  public void setPointAnimationPower(float power) {
    calibrationPoint.setPower(power);
  }

  public void setPointPosition(float x, float y) {
    float px = x - offsetX;
    float py = y - offsetY;
    calibrationPoint.layout(
            (int) px - 20,
            (int) py - 20,
            (int) px + 20,
            (int) py + 20);
    invalidate();
  }

  public void changeColor() {
    index += 1;
    if (index == pointColors.length) {
      index = 0;
    }
    calibPoint.setColor(index);
    invalidate();
  }

  public void changeDraw(boolean isDrawPoint, @Nullable String msgToShow) {
    this.toDraw = isDrawPoint;
    this.msg = msgToShow;
    invalidate();
  }

//  public void changeBackgroundColor(int color) {
//    setBackgroundColor(color);
//    invalidate();
//  }
//
//  public void setDrawGazePoint(boolean toDraw) {
//    this.toDraw = toDraw;
//    invalidate();
//  }
//
//  public void setMessage(String msg) {
//    this.msg = msg;
//    invalidate();
//  }

  //  @Override
//  protected void onLayout(boolean changed, int left, int top, int right, int bottom) {
//    super.onLayout(changed, left, top, right, bottom);
//  }

  private class CalibrationPoint extends View {
    private static final float default_radius = 20;
    private static final float default_rotate = 50;
    private final Paint point_paint;
    private final RectF oval;
    private float animation_power;
    private float center_x, center_y;
    private RotateAnimation rotateAnimation = null;
    private float last_end_degree, next_end_degree;
    Animation.AnimationListener listener = new Animation.AnimationListener() {
      @Override
      public void onAnimationStart(Animation animation) {
      }

      @Override
      public void onAnimationEnd(Animation animation) {
        last_end_degree = next_end_degree;
        next_end_degree = next_end_degree + (1 - (float) Math.pow(1 - animation_power, 5)) * default_rotate;
        setAnimation();
      }

      @Override
      public void onAnimationRepeat(Animation animation) {
      }
    };

    public CalibrationPoint(Context context) {
      super(context);
      animation_power = 0;
      point_paint = new Paint();
      point_paint.setAntiAlias(true);
      point_paint.setColor(pointColors[0]);
      oval = new RectF();
      last_end_degree = 0;
      next_end_degree = 0;
    }

    public void setPower(float power) {
      animation_power = power;
      if (rotateAnimation == null) {
        next_end_degree = next_end_degree + (1 - (float) Math.pow(1 - animation_power, 5)) * default_rotate;
        setAnimation();
      }
      invalidate();
    }

    private void setAnimation() {
      rotateAnimation = new RotateAnimation(
              last_end_degree,
              next_end_degree,
              Animation.RELATIVE_TO_SELF,
              0.5f,
              Animation.RELATIVE_TO_SELF,
              0.5f);
      rotateAnimation.setAnimationListener(listener);
      rotateAnimation.setRepeatCount(1);
      rotateAnimation.setDuration(10);
      this.startAnimation(rotateAnimation);
    }

    @Override
    protected void onDraw(Canvas canvas) {
      super.onDraw(canvas);
      if (toDraw) {
        float oval_long = default_radius * (1 + animation_power * 0.3f);
        float oval_short = default_radius * (1 - animation_power * 0.3f);

        oval.left = center_x - oval_long / 2;
        oval.top = center_y - oval_short / 2;
        oval.right = center_x + oval_long / 2;
        oval.bottom = center_y + oval_short / 2;

        canvas.drawOval(oval, point_paint);
      }
    }

    @Override
    protected void onLayout(boolean changed, int left, int top, int right, int bottom) {
      super.onLayout(changed, left, top, right, bottom);
      center_x = (right - left) / 2.0f;
      center_y = (bottom - top) / 2.0f;
    }
  }
}
