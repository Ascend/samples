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

public class AttentionView extends LinearLayout {
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private final float threshold = 0.75f;
    private ImageView imgAttention;
    private Drawable drawableAttentionOn, drawableAttentionOff;

    public AttentionView(Context context) {
        super(context);
        init();
    }

    public AttentionView(Context context,
                         @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public AttentionView(Context context,
                         @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    @RequiresApi(VERSION_CODES.LOLLIPOP)
    public AttentionView(Context context, AttributeSet attrs,
                         int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
        init();
    }

    private void init() {
        inflate(getContext(), R.layout.view_attention, this);
        imgAttention = findViewById(R.id.img_attention);

        drawableAttentionOn = getContext().getDrawable(R.drawable.attention_on_48);
        drawableAttentionOff = getContext().getDrawable(R.drawable.attention_off_48);
    }

    public void setAttention(final float attention) {
        uiHandler.post(new Runnable() {
            @Override
            public void run() {

                if (attention >= threshold) {
                    imgAttention.setImageDrawable(drawableAttentionOn);
                } else {
                    imgAttention.setImageDrawable(drawableAttentionOff);
                }
            }
        });
    }
}
