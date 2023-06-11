package com.chris.gaze.view;

import android.animation.FloatEvaluator;
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

import java.util.ArrayList;
import java.util.List;

public class GazePathView extends View {

    private static final float MIN_POINT_RADIUS = 10;
    private static final float MAX_POINT_RADIUS = 80;
    private static final long MAX_FIXATION_SIZE_TIME = 1500;
    private static final float SACCADE_LINE_WIDTH = 2.F;
    private static final long SACCADE_POINT_REFRESH_TIME_MILLIS = 350;
    private static final float MIN_FIXATION_POSITION_THRESHOLD = 80;
    private final int DEFAULT_COLOR = Color.argb(0x74, 0x34, 0x34, 0xff);
    private Paint pointPaint;
    private Paint linePaint;
    private float offsetX, offsetY;
    private List<PointF> fixationHistory;
    private PointF fixationDrawPoint = new PointF();
    private PointF fixationAnchorPoint = new PointF();
    private PointF saccadeRootPoint = new PointF();
    private PointF saccadeTarget = null;
    private boolean wasFixation = false;
    private float curPointSize;
    private long firstFixationTime = 0;
    private long lastSaccadeUpdateTime = 0;
    private FloatEvaluator evaluator;

    public GazePathView(Context context) {
        super(context);
        init();
    }

    public GazePathView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public GazePathView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    @RequiresApi(Build.VERSION_CODES.LOLLIPOP)
    public GazePathView(Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
        init();
    }

    private void init() {
        fixationHistory = new ArrayList<>();

        pointPaint = new Paint();
        pointPaint.setColor(DEFAULT_COLOR);

        linePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        linePaint.setColor(DEFAULT_COLOR);
        linePaint.setStrokeWidth(SACCADE_LINE_WIDTH);

        curPointSize = MIN_POINT_RADIUS;

        evaluator = new FloatEvaluator();
    }

    public void setOffset(int x, int y) {
        offsetX = x;
        offsetY = y;
    }

    public void onGaze(float x, float y, boolean is_fixation) {
        long curTime = System.currentTimeMillis();
        PointF curPoint = new PointF(x - offsetX, y - offsetY);
        if (!wasFixation || !is_fixation) {
            processSaccade(curTime, curPoint);
        } else {
            processFixation(curTime, curPoint);
        }
        wasFixation = is_fixation;
        invalidate();
    }

    private void processFixation(long timestamp, PointF curPoint) {
        clearSaccade();
        updateAnchorPointIfNeeded(timestamp, curPoint);
        fixationHistory.add(curPoint);
        fixationDrawPoint = getWeightedAverage(fixationHistory);
        curPointSize = calculatePointSize(timestamp);

        if (didFixationMoved(fixationAnchorPoint, curPoint, calculateFixationThreshold(curPointSize))) {
            processSaccade(timestamp, curPoint);
        }
    }

    private void processSaccade(long timeStamp, PointF curPoint) {
        clearFixation();
        if (needUpdateSaccade(timeStamp)) {
            updateSaccadeRoot(timeStamp);
        }
        saccadeTarget = curPoint;
        curPointSize = calculatePointSize(timeStamp);
    }

    private void updateAnchorPointIfNeeded(long curTime, PointF curPoint) {
        if (fixationAnchorPoint == null) {
            firstFixationTime = curTime;
            fixationAnchorPoint = curPoint;
        }
    }

    private PointF getWeightedAverage(List<PointF> points) {
        PointF center = new PointF(0, 0);
        float count = 0;
        for (int i = 0; i < points.size(); i++) {
            center.x += points.get(i).x * (points.size() - i);
            center.y += points.get(i).y * (points.size() - i);
            count += (points.size() - i);
        }
        center.x /= count;
        center.y /= count;
        return center;
    }

    private float calculateFixationThreshold(float pointSize) {
        return Math.max(pointSize * 1.5F, MIN_FIXATION_POSITION_THRESHOLD);
    }

    private boolean didFixationMoved(PointF anchorPoint, PointF curPoint, float fixationThreshold) {
        double diff_squre = Math.pow(anchorPoint.x - curPoint.x, 2) + Math.pow(anchorPoint.y - curPoint.y, 2);
        return diff_squre > fixationThreshold * fixationThreshold;
    }

    private float calculatePointSize(long timeStamp) {
        if (firstFixationTime == 0) {
            return MIN_POINT_RADIUS;
        } else {
            long timeDiff = timeStamp - firstFixationTime;
            float size = evaluator.evaluate((float) timeDiff / MAX_FIXATION_SIZE_TIME, MIN_POINT_RADIUS, MAX_POINT_RADIUS);
            return Math.min(size, MAX_POINT_RADIUS);
        }
    }

    private void clearSaccade() {
        lastSaccadeUpdateTime = 0;
        saccadeTarget = null;
    }

    private void clearFixation() {
        fixationHistory.clear();
        fixationAnchorPoint = null;
        firstFixationTime = 0;
    }

    private boolean needUpdateSaccade(long timeStamp) {
        return timeStamp - lastSaccadeUpdateTime > SACCADE_POINT_REFRESH_TIME_MILLIS;
    }

    private void updateSaccadeRoot(long timeStamp) {
        if (saccadeTarget != null) {
            saccadeRootPoint = saccadeTarget;
        } else {
            saccadeRootPoint = fixationDrawPoint;
        }
        lastSaccadeUpdateTime = timeStamp;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (saccadeTarget != null) {
            drawSaccade(canvas);
        } else {
            drawFixation(canvas);
        }
    }

    private void drawSaccade(Canvas canvas) {
        canvas.drawCircle(saccadeRootPoint.x, saccadeRootPoint.y, curPointSize, pointPaint);
        canvas.drawCircle(saccadeTarget.x, saccadeTarget.y, curPointSize, pointPaint);
        canvas.drawLine(saccadeRootPoint.x, saccadeRootPoint.y, saccadeTarget.x, saccadeTarget.y, linePaint);

    }

    private void drawFixation(Canvas canvas) {
        canvas.drawCircle(fixationDrawPoint.x, fixationDrawPoint.y, curPointSize, pointPaint);
    }
}
