package com.chris.depression.ui.visual;

import android.content.Context;
import android.view.TextureView;

import com.chris.depression.ui.visual.calibration.CalibrationDataStorage;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.List;

import camp.visual.gazetracker.GazeTracker;
import camp.visual.gazetracker.callback.CalibrationCallback;
import camp.visual.gazetracker.callback.GazeCallback;
import camp.visual.gazetracker.callback.GazeTrackerCallback;
import camp.visual.gazetracker.callback.ImageCallback;
import camp.visual.gazetracker.callback.InitializationCallback;
import camp.visual.gazetracker.callback.StatusCallback;
import camp.visual.gazetracker.callback.UserStatusCallback;
import camp.visual.gazetracker.constant.AccuracyCriteria;
import camp.visual.gazetracker.constant.CalibrationModeType;
import camp.visual.gazetracker.constant.InitializationErrorType;
import camp.visual.gazetracker.constant.StatusErrorType;
import camp.visual.gazetracker.constant.UserStatusOption;
import camp.visual.gazetracker.device.GazeDevice;
import camp.visual.gazetracker.gaze.GazeInfo;

public class GazeTrackerManager {
    static private GazeTrackerManager mInstance = null;
    private final List<InitializationCallback> initializationCallbacks = new ArrayList<>();
    private final List<GazeCallback> gazeCallbacks = new ArrayList<>();
    private final List<CalibrationCallback> calibrationCallbacks = new ArrayList<>();
    private final List<StatusCallback> statusCallbacks = new ArrayList<>();
    private final List<ImageCallback> imageCallbacks = new ArrayList<>();
    private final List<UserStatusCallback> userStatusCallbacks = new ArrayList<>();
    private final WeakReference<Context> mContext;
    private final GazeCallback gazeCallback = new GazeCallback() {
        @Override
        public void onGaze(GazeInfo gazeInfo) {
            for (GazeCallback gazeCallback : gazeCallbacks) {
                gazeCallback.onGaze(gazeInfo);
            }
        }
    };
    private final UserStatusCallback userStatusCallback = new UserStatusCallback() {
        @Override
        public void onAttention(long timestampBegin, long timestampEnd, float attentionScore) {
            for (UserStatusCallback userStatusCallback : userStatusCallbacks) {
                userStatusCallback.onAttention(timestampBegin, timestampEnd, attentionScore);
            }
        }

        @Override
        public void onBlink(long timestamp, boolean isBlinkLeft, boolean isBlinkRight, boolean isBlink, float eyeOpenness) {
            for (UserStatusCallback userStatusCallback : userStatusCallbacks) {
                userStatusCallback.onBlink(timestamp, isBlinkLeft, isBlinkRight, isBlink, eyeOpenness);
            }
        }

        @Override
        public void onDrowsiness(long timestamp, boolean isDrowsiness) {
            for (UserStatusCallback userStatusCallback : userStatusCallbacks) {
                userStatusCallback.onDrowsiness(timestamp, isDrowsiness);
            }
        }
    };
    private final CalibrationCallback calibrationCallback = new CalibrationCallback() {
        @Override
        public void onCalibrationProgress(float v) {
            for (CalibrationCallback calibrationCallback : calibrationCallbacks) {
                calibrationCallback.onCalibrationProgress(v);
            }
        }

        @Override
        public void onCalibrationNextPoint(float v, float v1) {
            for (CalibrationCallback calibrationCallback : calibrationCallbacks) {
                calibrationCallback.onCalibrationNextPoint(v, v1);
            }
        }

        @Override
        public void onCalibrationFinished(double[] doubles) {
            CalibrationDataStorage.saveCalibrationData(mContext.get(), doubles);
            for (CalibrationCallback calibrationCallback : calibrationCallbacks) {
                calibrationCallback.onCalibrationFinished(doubles);
            }
        }
    };
    private final ImageCallback imageCallback = new ImageCallback() {
        @Override
        public void onImage(long l, byte[] bytes) {
            for (ImageCallback imageCallback : imageCallbacks) {
                imageCallback.onImage(l, bytes);
            }
        }
    };
    private final StatusCallback statusCallback = new StatusCallback() {
        @Override
        public void onStarted() {
            for (StatusCallback statusCallback : statusCallbacks) {
                statusCallback.onStarted();
            }
        }

        @Override
        public void onStopped(StatusErrorType statusErrorType) {
            for (StatusCallback statusCallback : statusCallbacks) {
                statusCallback.onStopped(statusErrorType);
            }
        }
    };
    GazeTracker gazeTracker = null;
    // TODO: change licence key
    String SEESO_LICENSE_KEY = "dev_9lnqb1glo5ypidz66euz3zp9uh7euycmx6m19ymg";
    private WeakReference<TextureView> cameraPreview = null;
    // GazeTracker Callbacks
    private final InitializationCallback initializationCallback = new InitializationCallback() {
        @Override
        public void onInitialized(GazeTracker gazeTracker, InitializationErrorType initializationErrorType) {
            setGazeTracker(gazeTracker);
            for (InitializationCallback initializationCallback : initializationCallbacks) {
                initializationCallback.onInitialized(gazeTracker, initializationErrorType);
            }
            initializationCallbacks.clear();
            if (gazeTracker != null) {
                gazeTracker.setCallbacks(gazeCallback, calibrationCallback, imageCallback, statusCallback, userStatusCallback);
                if (cameraPreview != null) {
                    gazeTracker.setCameraPreview(cameraPreview.get());
                }
            }
        }
    };

    private GazeTrackerManager(Context context) {
        this.mContext = new WeakReference<>(context);
    }

    static public GazeTrackerManager makeNewInstance(Context context) {
        if (mInstance != null) {
            mInstance.deinitGazeTracker();
        }
        mInstance = new GazeTrackerManager(context);
        return mInstance;
    }

    static public GazeTrackerManager getInstance() {
        return mInstance;
    }

    public boolean hasGazeTracker() {
        return gazeTracker != null;
    }

    public void initGazeTracker(InitializationCallback callback, UserStatusOption option) {
        GazeDevice gazeDevice = new GazeDevice();
        initializationCallbacks.add(callback);

        GazeTracker.initGazeTracker(mContext.get(), gazeDevice, SEESO_LICENSE_KEY, initializationCallback, option);
    }

    public void deinitGazeTracker() {
        if (hasGazeTracker()) {
            GazeTracker.deinitGazeTracker(gazeTracker);
            gazeTracker = null;
        }
    }

    public void setGazeTrackerCallbacks(GazeTrackerCallback... callbacks) {
        for (GazeTrackerCallback callback : callbacks) {
            if (callback instanceof GazeCallback) {
                gazeCallbacks.add((GazeCallback) callback);

            } else if (callback instanceof CalibrationCallback) {
                calibrationCallbacks.add((CalibrationCallback) callback);

            } else if (callback instanceof ImageCallback) {
                imageCallbacks.add((ImageCallback) callback);

            } else if (callback instanceof StatusCallback) {
                statusCallbacks.add((StatusCallback) callback);

            } else if (callback instanceof UserStatusCallback) {
                userStatusCallbacks.add((UserStatusCallback) callback);
            }
        }
    }

    public void removeCallbacks(GazeTrackerCallback... callbacks) {
        for (GazeTrackerCallback callback : callbacks) {
            gazeCallbacks.remove(callback);
            calibrationCallbacks.remove(callback);
            imageCallbacks.remove(callback);
            statusCallbacks.remove(callback);
        }
    }

    public boolean startGazeTracking() {
        if (hasGazeTracker()) {
            gazeTracker.startTracking();
            return true;
        }
        return false;
    }

    public boolean stopGazeTracking() {
        if (isTracking()) {
            gazeTracker.stopTracking();
            return true;
        }
        return false;
    }

    public boolean startCalibration(CalibrationModeType modeType, AccuracyCriteria criteria) {
        if (hasGazeTracker()) {
            return gazeTracker.startCalibration(modeType, criteria);
        }
        return false;
    }

    public boolean stopCalibration() {
        if (isCalibrating()) {
            gazeTracker.stopCalibration();
            return true;
        }
        return false;
    }

    public boolean startCollectingCalibrationSamples() {
        if (isCalibrating()) {
            return gazeTracker.startCollectSamples();
        }
        return false;
    }

    public boolean isTracking() {
        if (hasGazeTracker()) {
            return gazeTracker.isTracking();
        }
        return false;
    }

    public boolean isCalibrating() {
        if (hasGazeTracker()) {
            return gazeTracker.isCalibrating();
        }
        return false;
    }

    public LoadCalibrationResult loadCalibrationData() {
        if (!hasGazeTracker()) {
            return LoadCalibrationResult.FAIL_HAS_NO_TRACKER;
        }
        double[] calibrationData = CalibrationDataStorage.loadCalibrationData(mContext.get());
        if (calibrationData != null) {
            if (!gazeTracker.setCalibrationData(calibrationData)) {
                return LoadCalibrationResult.FAIL_DOING_CALIBRATION;
            } else {
                return LoadCalibrationResult.SUCCESS;
            }
        } else {
            return LoadCalibrationResult.FAIL_NO_CALIBRATION_DATA;
        }
    }

    public void setCameraPreview(TextureView preview) {
        this.cameraPreview = new WeakReference<>(preview);
        if (hasGazeTracker()) {
            gazeTracker.setCameraPreview(preview);
        }
    }

    public void removeCameraPreview(TextureView preview) {
        if (this.cameraPreview.get() == preview) {
            this.cameraPreview = null;
            if (hasGazeTracker()) {
                gazeTracker.removeCameraPreview();
            }
        }
    }

    private void setGazeTracker(GazeTracker gazeTracker) {
        this.gazeTracker = gazeTracker;
    }

    public enum LoadCalibrationResult {
        SUCCESS,
        FAIL_DOING_CALIBRATION,
        FAIL_NO_CALIBRATION_DATA,
        FAIL_HAS_NO_TRACKER
    }

}
