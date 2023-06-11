/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui.visual

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.provider.Settings
import android.view.LayoutInflater
import android.view.TextureView
import android.view.View
import android.widget.CompoundButton
import android.widget.RadioGroup
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import camp.visual.gazetracker.GazeTracker
import camp.visual.gazetracker.callback.CalibrationCallback
import camp.visual.gazetracker.callback.GazeCallback
import camp.visual.gazetracker.callback.InitializationCallback
import camp.visual.gazetracker.callback.StatusCallback
import camp.visual.gazetracker.callback.UserStatusCallback
import camp.visual.gazetracker.constant.AccuracyCriteria
import camp.visual.gazetracker.constant.CalibrationModeType
import camp.visual.gazetracker.constant.InitializationErrorType
import camp.visual.gazetracker.constant.StatusErrorType
import camp.visual.gazetracker.constant.UserStatusOption
import camp.visual.gazetracker.filter.OneEuroFilterManager
import camp.visual.gazetracker.gaze.GazeInfo
import camp.visual.gazetracker.state.ScreenState
import camp.visual.gazetracker.state.TrackingState
import camp.visual.gazetracker.util.ViewLayoutChecker
import com.chris.depression.R
import com.chris.depression.databinding.ActivityVisualBinding
import com.chris.gaze.view.PointView
import timber.log.Timber

class VisualActivity : AppCompatActivity() {

    private val PERMISSIONS = arrayOf(
        Manifest.permission.CAMERA // 시선 추적 input
    )
    private val REQ_PERMISSION = 1000
    private lateinit var gazeTrackerManager: GazeTrackerManager
    private val viewLayoutChecker = ViewLayoutChecker()
    private val backgroundThread = HandlerThread("background")
    private var backgroundHandler: Handler? = null
    private lateinit var viewBinding: ActivityVisualBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(
            ActivityVisualBinding.inflate(LayoutInflater.from(this))
                .apply { viewBinding = this }.root
        )
        gazeTrackerManager = GazeTrackerManager.makeNewInstance(this)
        Timber.i("gazeTracker version: " + GazeTracker.getVersionName())

        initView()
        checkPermission()
        initHandler()
    }

    override fun onStart() {
        super.onStart()
        if (viewBinding.preview.isAvailable) {
            // When if textureView available
            gazeTrackerManager.setCameraPreview(viewBinding.preview)
        }
        gazeTrackerManager.setGazeTrackerCallbacks(
            gazeCallback,
            calibrationCallback,
            statusCallback,
            userStatusCallback
        )
        Timber.i("onStart")
    }

    override fun onResume() {
        super.onResume()
        Timber.i("onResume")
        // 화면 전환후에도 체크하기 위해
        setOffsetOfView()
        gazeTrackerManager.startGazeTracking()
    }

    override fun onPause() {
        super.onPause()
        gazeTrackerManager.stopGazeTracking()
        Timber.i("onPause")
    }

    override fun onStop() {
        super.onStop()
        gazeTrackerManager.removeCameraPreview(viewBinding.preview)
        gazeTrackerManager.removeCallbacks(
            gazeCallback,
            calibrationCallback,
            statusCallback,
            userStatusCallback
        )
        Timber.i("onStop")
    }

    override fun onDestroy() {
        super.onDestroy()
        releaseHandler()
        viewLayoutChecker.releaseChecker()
    }

    // handler
    private fun initHandler() {
        backgroundThread.start()
        backgroundHandler = Handler(backgroundThread.looper)
    }

    private fun releaseHandler() {
        backgroundThread.quitSafely()
    }
    // handler end

    // permission
    private fun checkPermission() {
        // Check permission status
        if (!hasPermissions(PERMISSIONS)) {
            requestPermissions(PERMISSIONS, REQ_PERMISSION)
        } else {
            checkPermission(true)
        }
    }

    private fun hasPermissions(permissions: Array<String>): Boolean {
        var result: Int
        // Check permission status in string array
        for (perms in permissions) {
            if (perms == Manifest.permission.SYSTEM_ALERT_WINDOW) {
                if (!Settings.canDrawOverlays(this)) {
                    return false
                }
            }
            result = ContextCompat.checkSelfPermission(this, perms)
            if (result == PackageManager.PERMISSION_DENIED) {
                // When if unauthorized permission found
                return false
            }
        }
        // When if all permission allowed
        return true
    }

    private fun checkPermission(isGranted: Boolean) {
        if (isGranted) {
            permissionGranted()
        } else {
            showToast("not granted permissions", true)
            finish()
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String?>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        when (requestCode) {
            REQ_PERMISSION -> if (grantResults.isNotEmpty()) {
                val cameraPermissionAccepted = grantResults[0] == PackageManager.PERMISSION_GRANTED
                if (cameraPermissionAccepted) {
                    checkPermission(true)
                } else {
                    checkPermission(false)
                }
            }
        }
    }

    private fun permissionGranted() {
        setViewAtGazeTrackerState()
    }
    // permission end

    // permission end
    private var isUseGazeFilter = true
    private var isStatusBlink = false
    private var isStatusAttention = false
    private var isStatusDrowsiness = false
    private var activeStatusCount = 0

    // calibration type
    private var calibrationType = CalibrationModeType.DEFAULT
    private var criteria = AccuracyCriteria.DEFAULT

    private fun initView() {

        val versionStr = "version: " + GazeTracker.getVersionName()
        viewBinding.txtGazeVersion.text = versionStr
        viewBinding.layoutProgress.layoutProgress.setOnClickListener(null)
        viewBinding.preview.surfaceTextureListener = surfaceTextureListener
        viewBinding.btnInitGaze.setOnClickListener(onClickListener)
        viewBinding.btnReleaseGaze.setOnClickListener(onClickListener)
        viewBinding.btnStartTracking.setOnClickListener(onClickListener)
        viewBinding.btnStopTracking.setOnClickListener(onClickListener)
        viewBinding.btnStartCalibration.setOnClickListener(onClickListener)
        viewBinding.btnStopCalibration.setOnClickListener(onClickListener)
        viewBinding.btnSetCalibration.setOnClickListener(onClickListener)
        viewBinding.btnGuiDemo.setOnClickListener(onClickListener)
        viewBinding.swUseGazeFilter.isChecked = isUseGazeFilter
        viewBinding.swStatusBlink.isChecked = isStatusBlink
        viewBinding.swStatusAttention.isChecked = isStatusAttention
        viewBinding.swStatusDrowsiness.isChecked = isStatusDrowsiness
        when (calibrationType) {
            CalibrationModeType.ONE_POINT -> viewBinding.rbCalibrationOne.isChecked = true
            CalibrationModeType.SIX_POINT -> viewBinding.rbCalibrationSix.isChecked = true
            // default = five point
            else -> viewBinding.rbCalibrationFive.isChecked = true
        }
        viewBinding.swUseGazeFilter.setOnCheckedChangeListener(onCheckedChangeSwitch)
        viewBinding.swStatusBlink.setOnCheckedChangeListener(onCheckedChangeSwitch)
        viewBinding.swStatusAttention.setOnCheckedChangeListener(onCheckedChangeSwitch)
        viewBinding.swStatusDrowsiness.setOnCheckedChangeListener(onCheckedChangeSwitch)
        viewBinding.rgCalibration.setOnCheckedChangeListener(onCheckedChangeRadioButton)
        viewBinding.rgAccuracy.setOnCheckedChangeListener(onCheckedChangeRadioButton)
        viewBinding.viewEyeBlink.visibility = View.GONE
        viewBinding.viewAttention.visibility = View.GONE
        viewBinding.viewDrowsiness.visibility = View.GONE
        hideProgress()
        setOffsetOfView()
        setViewAtGazeTrackerState()
    }

    private val onCheckedChangeRadioButton =
        RadioGroup.OnCheckedChangeListener { group, checkedId ->
            if (group === viewBinding.rgCalibration) {
                when (checkedId) {
                    R.id.rb_calibration_one -> {
                        calibrationType = CalibrationModeType.ONE_POINT
                    }
                    R.id.rb_calibration_five -> {
                        calibrationType = CalibrationModeType.FIVE_POINT
                    }
                    R.id.rb_calibration_six -> {
                        calibrationType = CalibrationModeType.SIX_POINT
                    }
                }
            } else if (group === viewBinding.rgAccuracy) {
                when (checkedId) {
                    R.id.rb_accuracy_default -> {
                        criteria = AccuracyCriteria.DEFAULT
                    }
                    R.id.rb_accuracy_low -> {
                        criteria = AccuracyCriteria.LOW
                    }
                    R.id.rb_accuracy_high -> {
                        criteria = AccuracyCriteria.HIGH
                    }
                }
            }
        }

    private val onCheckedChangeSwitch =
        CompoundButton.OnCheckedChangeListener { buttonView, isChecked ->
            if (buttonView === viewBinding.swUseGazeFilter) {
                isUseGazeFilter = isChecked
            } else if (buttonView === viewBinding.swStatusBlink) {
                isStatusBlink = isChecked
                if (isStatusBlink) {
                    viewBinding.viewEyeBlink.visibility = View.VISIBLE
                    activeStatusCount++
                } else {
                    viewBinding.viewEyeBlink.visibility = View.GONE
                    activeStatusCount--
                }
            } else if (buttonView === viewBinding.swStatusAttention) {
                isStatusAttention = isChecked
                if (isStatusAttention) {
                    viewBinding.viewAttention.visibility = View.VISIBLE
                    activeStatusCount++
                } else {
                    viewBinding.viewAttention.visibility = View.GONE
                    activeStatusCount--
                }
            } else if (buttonView === viewBinding.swStatusDrowsiness) {
                isStatusDrowsiness = isChecked
                if (isStatusDrowsiness) {
                    viewBinding.viewDrowsiness.visibility = View.VISIBLE
                    activeStatusCount++
                } else {
                    viewBinding.viewDrowsiness.visibility = View.GONE
                    activeStatusCount--
                }
            }
        }

    private val surfaceTextureListener: TextureView.SurfaceTextureListener =
        object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(
                surface: SurfaceTexture,
                width: Int,
                height: Int
            ) {
                // When if textureView available
                gazeTrackerManager.setCameraPreview(viewBinding.preview)
            }

            override fun onSurfaceTextureSizeChanged(
                surface: SurfaceTexture,
                width: Int,
                height: Int
            ) {
            }

            override fun onSurfaceTextureDestroyed(surface: SurfaceTexture): Boolean {
                return false
            }

            override fun onSurfaceTextureUpdated(surface: SurfaceTexture) {}
        }

    // The gaze or calibration coordinates are delivered only to the absolute coordinates of the entire screen.
    // The coordinate system of the Android view is a relative coordinate system,
    // so the offset of the view to show the coordinates must be obtained and corrected to properly show the information on the screen.
    private fun setOffsetOfView() {
        viewLayoutChecker.setOverlayView(
            viewBinding.viewPoint
        ) { x, y ->
            viewBinding.viewPoint.setOffset(x, y)
            viewBinding.viewCalibration.setOffset(x, y)
        }
    }

    private fun showProgress() {
        runOnUiThread { viewBinding.layoutProgress.layoutProgress.visibility = View.VISIBLE }
    }

    private fun hideProgress() {
        runOnUiThread { viewBinding.layoutProgress.layoutProgress.visibility = View.INVISIBLE }
    }

    private fun showTrackingWarning() {
        runOnUiThread { viewBinding.viewWarningTracking.visibility = View.VISIBLE }
    }

    private fun hideTrackingWarning() {
        runOnUiThread { viewBinding.viewWarningTracking.visibility = View.INVISIBLE }
    }

    private val onClickListener =
        View.OnClickListener { v ->
            when {
                v === viewBinding.btnInitGaze -> {
                    initGaze()
                }
                v === viewBinding.btnReleaseGaze -> {
                    releaseGaze()
                }
                v === viewBinding.btnStartTracking -> {
                    startTracking()
                }
                v === viewBinding.btnStopTracking -> {
                    stopTracking()
                }
                v === viewBinding.btnStartCalibration -> {
                    startCalibration()
                }
                v === viewBinding.btnStopCalibration -> {
                    stopCalibration()
                }
                v === viewBinding.btnSetCalibration -> {
                    setCalibration()
                }
                v === viewBinding.btnGuiDemo -> {
                    showGuiDemo()
                }
            }
        }

    private fun showToast(msg: String, isShort: Boolean) {
        runOnUiThread {
            Toast.makeText(
                this,
                msg,
                if (isShort) Toast.LENGTH_SHORT else Toast.LENGTH_LONG
            ).show()
        }
    }

    private fun showGazePoint(x: Float, y: Float, type: ScreenState) {
        runOnUiThread {
            viewBinding.viewPoint.setType(if (type == ScreenState.INSIDE_OF_SCREEN) PointView.TYPE_DEFAULT else PointView.TYPE_OUT_OF_SCREEN)
            viewBinding.viewPoint.setPosition(x, y)
        }
    }

    private fun setCalibrationPoint(x: Float, y: Float) {
        runOnUiThread {
            viewBinding.viewCalibration.visibility = View.VISIBLE
            viewBinding.viewCalibration.changeDraw(true, null)
            viewBinding.viewCalibration.setPointPosition(x, y)
            viewBinding.viewCalibration.setPointAnimationPower(0F)
        }
    }

    private fun setCalibrationProgress(progress: Float) {
        runOnUiThread { viewBinding.viewCalibration.setPointAnimationPower(progress) }
    }

    private fun hideCalibrationView() {
        runOnUiThread { viewBinding.viewCalibration.visibility = View.INVISIBLE }
    }

    private fun setViewAtGazeTrackerState() {
        Timber.i("gaze : " + isTrackerValid() + ", tracking " + isTracking())
        runOnUiThread {
            viewBinding.btnInitGaze.isEnabled = !isTrackerValid()
            viewBinding.btnReleaseGaze.isEnabled = isTrackerValid()
            viewBinding.btnStartTracking.isEnabled = isTrackerValid() && !isTracking()
            viewBinding.btnStopTracking.isEnabled = isTracking()
            viewBinding.btnStartCalibration.isEnabled = isTracking()
            viewBinding.btnStopCalibration.isEnabled = isTracking()
            viewBinding.btnSetCalibration.isEnabled = isTrackerValid()
            if (!isTracking()) {
                hideCalibrationView()
            }
        }
    }

    private fun setStatusSwitchState(isEnabled: Boolean) {
        runOnUiThread {
            if (!isEnabled) {
                viewBinding.swStatusBlink.isEnabled = false
                viewBinding.swStatusAttention.isEnabled = false
                viewBinding.swStatusDrowsiness.isEnabled = false
            } else {
                viewBinding.swStatusBlink.isEnabled = true
                viewBinding.swStatusAttention.isEnabled = true
                viewBinding.swStatusDrowsiness.isEnabled = true
            }
        }
    }

    // view end

    // view end
    // gazeTracker
    private fun isTrackerValid(): Boolean {
        return gazeTrackerManager.hasGazeTracker()
    }

    private fun isTracking(): Boolean {
        return gazeTrackerManager.isTracking
    }

    private val initializationCallback =
        InitializationCallback { gazeTracker, error ->
            if (gazeTracker != null) {
                initSuccess(gazeTracker)
            } else {
                initFail(error)
            }
        }

    private fun initSuccess(gazeTracker: GazeTracker) {
        setViewAtGazeTrackerState()
        hideProgress()
    }

    private fun initFail(error: InitializationErrorType) {
        hideProgress()
    }

    private val oneEuroFilterManager = OneEuroFilterManager(2)
    private val gazeCallback =
        GazeCallback { gazeInfo ->
            processOnGaze(gazeInfo)
            Timber.i("check eyeMovement " + gazeInfo.eyeMovementState)
        }

    private val userStatusCallback: UserStatusCallback = object : UserStatusCallback {
        override fun onAttention(timestampBegin: Long, timestampEnd: Long, attentionScore: Float) {
            Timber.i("check User Status Attention Rate $attentionScore")
            viewBinding.viewAttention.setAttention(attentionScore)
        }

        override fun onBlink(
            timestamp: Long,
            isBlinkLeft: Boolean,
            isBlinkRight: Boolean,
            isBlink: Boolean,
            eyeOpenness: Float
        ) {
            Timber.i("check User Status Blink Left: $isBlinkLeft, Right: $isBlinkRight, Blink: $isBlink, eyeOpenness: $eyeOpenness")
            viewBinding.viewEyeBlink.setLeftEyeBlink(isBlinkLeft)
            viewBinding.viewEyeBlink.setRightEyeBlink(isBlinkRight)
            viewBinding.viewEyeBlink.setEyeBlink(isBlink)
        }

        override fun onDrowsiness(timestamp: Long, isDrowsiness: Boolean) {
            Timber.i("check User Status Drowsiness $isDrowsiness")
            viewBinding.viewDrowsiness.setDrowsiness(isDrowsiness)
        }
    }

    private fun processOnGaze(gazeInfo: GazeInfo) {
        if (gazeInfo.trackingState == TrackingState.SUCCESS) {
            hideTrackingWarning()
            if (!gazeTrackerManager.isCalibrating) {
                val filtered_gaze = filterGaze(gazeInfo)
                showGazePoint(filtered_gaze[0], filtered_gaze[1], gazeInfo.screenState)
            }
        } else {
            showTrackingWarning()
        }
    }

    private fun filterGaze(gazeInfo: GazeInfo): FloatArray {
        if (isUseGazeFilter) {
            if (oneEuroFilterManager.filterValues(gazeInfo.timestamp, gazeInfo.x, gazeInfo.y)) {
                return oneEuroFilterManager.filteredValues
            }
        }
        return floatArrayOf(gazeInfo.x, gazeInfo.y)
    }

    private val calibrationCallback: CalibrationCallback = object : CalibrationCallback {
        override fun onCalibrationProgress(progress: Float) {
            setCalibrationProgress(progress)
        }

        override fun onCalibrationNextPoint(x: Float, y: Float) {
            setCalibrationPoint(x, y)
            // Give time to eyes find calibration coordinates, then collect data samples
            backgroundHandler!!.postDelayed({ startCollectSamples() }, 1000)
        }

        override fun onCalibrationFinished(calibrationData: DoubleArray) {
            // When calibration is finished, calibration data is stored to SharedPreference
            hideCalibrationView()
            showToast("calibrationFinished", true)
        }
    }

    private val statusCallback: StatusCallback = object : StatusCallback {
        override fun onStarted() {
            // isTracking true
            // When if camera stream starting
            setViewAtGazeTrackerState()
        }

        override fun onStopped(error: StatusErrorType) {
            // isTracking false
            // When if camera stream stopping
            setViewAtGazeTrackerState()
            if (error != StatusErrorType.ERROR_NONE) {
                when (error) {
                    StatusErrorType.ERROR_CAMERA_START -> // When if camera stream can't start
                        showToast("ERROR_CAMERA_START ", false)
                    StatusErrorType.ERROR_CAMERA_INTERRUPT -> // When if camera stream interrupted
                        showToast("ERROR_CAMERA_INTERRUPT ", false)
                    else -> {}
                }
            }
        }
    }

    private fun initGaze() {
        showProgress()
        val userStatusOption = UserStatusOption()
        if (isStatusAttention) {
            userStatusOption.useAttention()
        }
        if (isStatusBlink) {
            userStatusOption.useBlink()
        }
        if (isStatusDrowsiness) {
            userStatusOption.useDrowsiness()
        }
        Timber.i("init option attention $isStatusAttention, blink $isStatusBlink, drowsiness $isStatusDrowsiness")
        gazeTrackerManager.initGazeTracker(initializationCallback, userStatusOption)
        setStatusSwitchState(false)
    }

    private fun releaseGaze() {
        gazeTrackerManager.deinitGazeTracker()
        setStatusSwitchState(true)
        setViewAtGazeTrackerState()
    }

    private fun startTracking() {
        gazeTrackerManager.startGazeTracking()
    }

    private fun stopTracking() {
        gazeTrackerManager.stopGazeTracking()
    }

    private fun startCalibration(): Boolean {
        val isSuccess = gazeTrackerManager.startCalibration(calibrationType, criteria)
        if (!isSuccess) {
            showToast("calibration start fail", false)
        }
        setViewAtGazeTrackerState()
        return isSuccess
    }

    // Collect the data samples used for calibration
    private fun startCollectSamples(): Boolean {
        val isSuccess = gazeTrackerManager.startCollectingCalibrationSamples()
        setViewAtGazeTrackerState()
        return isSuccess
    }

    private fun stopCalibration() {
        gazeTrackerManager.stopCalibration()
        hideCalibrationView()
        setViewAtGazeTrackerState()
    }

    private fun setCalibration() {
        val result: GazeTrackerManager.LoadCalibrationResult =
            gazeTrackerManager.loadCalibrationData()
        when (result) {
            GazeTrackerManager.LoadCalibrationResult.SUCCESS -> showToast(
                "setCalibrationData success",
                false
            )
            GazeTrackerManager.LoadCalibrationResult.FAIL_DOING_CALIBRATION -> showToast(
                "calibrating",
                false
            )
            GazeTrackerManager.LoadCalibrationResult.FAIL_NO_CALIBRATION_DATA -> showToast(
                "Calibration data is null",
                true
            )
            GazeTrackerManager.LoadCalibrationResult.FAIL_HAS_NO_TRACKER -> showToast(
                "No tracker has initialized",
                true
            )
        }
        setViewAtGazeTrackerState()
    }

    private fun showGuiDemo() {
//        val intent = Intent(
//            applicationContext,
//            DemoActivity::class.java
//        )
//        startActivity(intent)
    }
}
