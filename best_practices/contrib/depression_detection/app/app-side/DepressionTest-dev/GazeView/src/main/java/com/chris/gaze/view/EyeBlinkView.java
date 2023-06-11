package com.chris.gaze.view;

import android.content.Context;
import android.graphics.drawable.Drawable;
import android.os.Build.VERSION_CODES;
import android.os.Handler;
import android.os.Looper;
import android.util.AttributeSet;
import android.widget.ImageView;
import android.widget.LinearLayout;

import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;

import com.chris.gazeview.R;

public class EyeBlinkView extends LinearLayout {
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private ImageView imgRightEye, imgLeftEye;
    private ImageView imgEye;
    private Drawable drawableOneEyeOpen, drawableOneEyeClosed;
    private Drawable drawableEyeOpen, drawableEyeClosed;

    public EyeBlinkView(Context context) {
        super(context);
        init();
    }

    public EyeBlinkView(Context context,
                        @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public EyeBlinkView(Context context,
                        @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    @RequiresApi(VERSION_CODES.LOLLIPOP)
    public EyeBlinkView(Context context, AttributeSet attrs,
                        int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
        init();
    }

    private void init() {
        inflate(getContext(), R.layout.view_eye_blink, this);
        imgRightEye = findViewById(R.id.img_right_eye);
        imgLeftEye = findViewById(R.id.img_left_eye);
        imgEye = findViewById(R.id.img_eye);

        drawableOneEyeOpen = getContext().getDrawable(R.drawable.baseline_visibility_black_48);
        drawableOneEyeClosed = getContext().getDrawable(R.drawable.baseline_visibility_off_black_48);
        drawableEyeOpen = getContext().getDrawable(R.drawable.twotone_visibility_black_48);
        drawableEyeClosed = getContext().getDrawable(R.drawable.twotone_visibility_off_black_48);
    }

    public void setLeftEyeBlink(final boolean isBlink) {
        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                if (isBlink) {
                    imgLeftEye.setImageDrawable(drawableOneEyeClosed);
                } else {
                    imgLeftEye.setImageDrawable(drawableOneEyeOpen);
                }
            }
        });
    }

    public void setRightEyeBlink(final boolean isBlink) {
        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                if (isBlink) {
                    imgRightEye.setImageDrawable(drawableOneEyeClosed);
                } else {
                    imgRightEye.setImageDrawable(drawableOneEyeOpen);
                }
            }
        });
    }

    public void setEyeBlink(final boolean isBlink) {
        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                if (isBlink) {
                    imgEye.setImageDrawable(drawableEyeClosed);
                } else {
                    imgEye.setImageDrawable(drawableEyeOpen);
                }
            }
        });
    }
}
