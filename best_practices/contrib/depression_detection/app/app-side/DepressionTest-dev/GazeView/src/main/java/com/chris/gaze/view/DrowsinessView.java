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

public class DrowsinessView extends LinearLayout {
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private ImageView imgAttention;
    private Drawable drawableDrowsinessOn, drawableDrowsinessOff;

    public DrowsinessView(Context context) {
        super(context);
        init();
    }

    public DrowsinessView(Context context,
                          @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public DrowsinessView(Context context,
                          @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    @RequiresApi(VERSION_CODES.LOLLIPOP)
    public DrowsinessView(Context context, AttributeSet attrs,
                          int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
        init();
    }

    private void init() {
        inflate(getContext(), R.layout.view_drowsiness, this);
        imgAttention = findViewById(R.id.img_drowsiness);

        drawableDrowsinessOn = getContext().getDrawable(R.drawable.drowsiness_on_48);
        drawableDrowsinessOff = getContext().getDrawable(R.drawable.drowsiness_off_48);
    }

    public void setDrowsiness(final boolean drowsiness) {
        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                if (drowsiness) {
                    imgAttention.setImageDrawable(drawableDrowsinessOn);
                } else {
                    imgAttention.setImageDrawable(drawableDrowsinessOff);
                }
            }
        });
    }
}
