/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import android.annotation.SuppressLint
import android.content.ContentValues
import android.media.MediaPlayer
import android.net.Uri
import android.os.Bundle
import android.provider.MediaStore
import android.provider.OpenableColumns
import android.view.LayoutInflater
import android.view.SurfaceHolder
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.camera.core.CameraSelector
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.video.MediaStoreOutputOptions
import androidx.camera.video.Quality
import androidx.camera.video.QualitySelector
import androidx.camera.video.Recorder
import androidx.camera.video.Recording
import androidx.camera.video.VideoCapture
import androidx.camera.video.VideoRecordEvent
import androidx.concurrent.futures.await
import androidx.core.content.ContextCompat
import androidx.core.util.Consumer
import androidx.core.view.isVisible
import androidx.fragment.app.viewModels
import androidx.lifecycle.ViewModelProvider
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.findNavController
import com.chris.depression.BuildConfig
import com.chris.depression.R
import com.chris.depression.databinding.FragmentVideoViewerBinding
import com.chris.depression.ui.VideoViewerViewModel.Companion.NO_IMAGE
import com.chris.depression.ui.VideoViewerViewModel.PageStates.CaptureCameraState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.CaptureStopState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.PredictResultState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.PredictingState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.StartPredictionState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.VideoPlayingState
import com.chris.depression.utils.autoCleared
import com.chris.depression.utils.getName
import com.chris.depression.utils.getPath
import com.chris.depression.utils.hasPermissions
import com.chris.depression.utils.navigateToStartFragment
import io.reactivex.Observable
import io.reactivex.android.schedulers.AndroidSchedulers
import io.reactivex.disposables.Disposable
import kotlinx.coroutines.launch
import timber.log.Timber
import java.io.File
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.concurrent.TimeUnit
import javax.inject.Inject

class VideoViewerFragment : BaseFragment() {

    private lateinit var videoCapture: VideoCapture<Recorder>
    private var activeRecording: Recording? = null
    private lateinit var recordingState: VideoRecordEvent
    private val mainThreadExecutor by lazy { ContextCompat.getMainExecutor(requireContext()) }

    private val captureListener = Consumer<VideoRecordEvent> { event ->
        if (event !is VideoRecordEvent.Status)
            recordingState = event
        updateUI(event)
//        if (event is VideoRecordEvent.Finalize) {
//            showVideo(event)
//        }
    }

    companion object {
        private const val FILENAME_FORMAT = "yyyy-MM-dd-HH-mm-ss-SSS"
    }

    @Inject
    lateinit var viewModelFactory: ViewModelProvider.Factory

    val viewModel: VideoViewerViewModel by viewModels { viewModelFactory }

    private var viewBinding by autoCleared<FragmentVideoViewerBinding>()

    val mediaPlayer = MediaPlayer().apply {
        setOnPreparedListener { start() }
        setOnCompletionListener { stop() }
    }
    private var imageDispose: Disposable? = null
    private var videoDispose: Disposable? = null

    private val holderCallback = object : SurfaceHolder.Callback {
        override fun surfaceCreated(holder: SurfaceHolder) {
            Timber.e("surfaceCreated")
            mediaPlayer.reset()
            mediaPlayer.setDisplay(holder)
        }

        override fun surfaceChanged(
            holder: SurfaceHolder,
            format: Int,
            width: Int,
            height: Int
        ) {
            Timber.e("surfaceChanged")
        }

        override fun surfaceDestroyed(holder: SurfaceHolder) {
            Timber.e("surfaceDestroyed")
            holder.removeCallback(this)
        }
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        return FragmentVideoViewerBinding.inflate(inflater, container, false).apply {
            viewBinding = this
            lifecycleOwner = viewLifecycleOwner
            viewModel = this@VideoViewerFragment.viewModel
        }.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        viewBinding.startButton.setOnClickListener {
            if (!hasPermissions())
                return@setOnClickListener
            startRecording()
        }
        viewBinding.stopButton.setOnClickListener {
            if (recordingState is VideoRecordEvent.Finalize) {
                nextQuestion()
            } else {
                stopAnswer()
            }
        }

        viewBinding.svVideo.holder.addCallback(holderCallback)

        viewModel.currentQuestion.observe(viewLifecycleOwner) { question ->
            viewBinding.ivQuestion.isVisible = false
            showVideo(question.questionVideo)
            if (question.questionImage != NO_IMAGE) {
                imageDispose =
                    Observable.just(question.questionImage).delay(8L, TimeUnit.SECONDS)
                        .observeOn(AndroidSchedulers.mainThread()).subscribe {
                            viewBinding.ivQuestion.isVisible = true
                            viewBinding.ivQuestion.setImageResource(it)
                        }
            }
        }
        viewBinding.btEnd.setOnClickListener {
            findNavController().navigateToStartFragment()
        }
        viewModel.stringMsg.observe(viewLifecycleOwner) {
//            Toast.makeText(requireContext(), "String message: $it", Toast.LENGTH_LONG).show()
//            viewBinding.btEnd.isVisible = true
            val statusStr = "Http result: $it"
            if (BuildConfig.DEBUG) viewBinding.captureStatus.isVisible = true
            viewBinding.captureStatus.text = statusStr
        }
        viewModel.pageState.observe(viewLifecycleOwner) { state ->
            when (state) {
                VideoPlayingState -> {
                    viewBinding.startButton.isVisible = true
                    viewBinding.stopButton.isVisible = false
                    viewBinding.btEnd.isVisible = false
                    viewBinding.flProgress.isVisible = false
                    viewBinding.captureStatus.isVisible = false
                }
                CaptureCameraState -> {
                    viewBinding.startButton.visibility = View.INVISIBLE
                    viewBinding.stopButton.isVisible = true
                    viewBinding.stopButton.text = getString(R.string.stop_video_answer)
                    viewBinding.btEnd.isVisible = false
                    viewBinding.flProgress.isVisible = false
                }
                CaptureStopState -> {
                    viewBinding.startButton.visibility = View.INVISIBLE
                    viewBinding.stopButton.isVisible = true
                    viewBinding.stopButton.text = getString(R.string.next_question)
                    viewBinding.btEnd.isVisible = false
                    viewBinding.flProgress.isVisible = false
                }
                StartPredictionState -> {
                    viewBinding.startButton.visibility = View.INVISIBLE
                    viewBinding.stopButton.isVisible = true
                    viewBinding.stopButton.text = getString(R.string.start_prediction)
                    viewBinding.btEnd.isVisible = false
                    viewBinding.flProgress.isVisible = false
                }
                PredictingState -> {
                    viewBinding.startButton.isVisible = false
                    viewBinding.stopButton.isVisible = false
                    viewBinding.btEnd.isVisible = false
                    viewBinding.flProgress.isVisible = true
                }
                is PredictResultState -> {
                    viewBinding.flProgress.isVisible = false
                    viewBinding.btEnd.isVisible = true
                    showResultDialog(state.result)
                }
            }
        }
        viewModel.start()
    }

    private fun showResultDialog(result: String) {
        viewBinding.clResult.isVisible = true
        viewBinding.tvResult.text = "您的检测结果为：$result"
//        findNavController().navigate(VideoViewerFragmentDirections.actionVideoViewerFragmentToPredictionResultFragment(result))
//        BottomSheetDialog(requireContext()).apply {
//            setContentView(R.layout.base_bottom_sheet_fragment)
//            findViewById<ImageView>(R.id.bt_dismiss)?.setOnClickListener { dismiss() }
//            findViewById<TextView>(R.id.tv_ok)?.apply {
//                text = "Result okay: $result"
//            }
//        }.show()
    }

    override fun onDestroy() {
        super.onDestroy()
        imageDispose?.dispose()
        videoDispose?.dispose()
    }

    private fun startRecording() {
        viewLifecycleOwner.lifecycleScope.launch {
            bindCaptureUsecase()
        }
    }

    private suspend fun bindCaptureUsecase() {
        val cameraProvider = ProcessCameraProvider.getInstance(requireContext()).await()

        val cameraSelector = CameraSelector.DEFAULT_FRONT_CAMERA
//        val preview = Preview.Builder().setTargetAspectRatio(DEFAULT_ASPECT_RATIO)
//            .build().apply {
//                setSurfaceProvider(viewBinding.previewView.surfaceProvider)
//                viewBinding.previewView.meteringPointFactory
//            }

        val qualitySelector = QualitySelector.from(Quality.HD)
        val recorder = Recorder.Builder()
            .setQualitySelector(qualitySelector)
            .build()
        videoCapture = VideoCapture.withOutput(recorder)

        try {
            cameraProvider.unbindAll()
            cameraProvider.bindToLifecycle(
                viewLifecycleOwner,
                cameraSelector,
//                if (BuildConfig.DEBUG) preview else null,
                videoCapture
            )
        } catch (exc: Exception) {
            // we are on main thread, let's reset the controls on the UI.
            Timber.e(exc, "Use case binding failed")
        }
        startAnswer()
    }

    private fun startAnswer() {
        if (!this::recordingState.isInitialized || recordingState is VideoRecordEvent.Finalize) {
            recording()
            viewModel.startAnswer()
        } else {
            when (recordingState) {
                is VideoRecordEvent.Start -> {
                    activeRecording?.pause()
                    activeRecording?.stop()
                }
                is VideoRecordEvent.Pause -> {
                    activeRecording?.resume()
                }
                is VideoRecordEvent.Resume -> {
                    activeRecording?.pause()
                    activeRecording?.stop()
                }
                else -> {
                    Timber.e("Unknown State ($recordingState) when Capture Button is pressed ")
                }
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun recording() {
        val name = "CameraX-recording-" +
                SimpleDateFormat(FILENAME_FORMAT, Locale.US)
                    .format(System.currentTimeMillis()) + ".mp4"
        val contentValues = ContentValues().apply {
            put(MediaStore.Video.Media.DISPLAY_NAME, name)
        }
        val mediaStoreOutput = MediaStoreOutputOptions.Builder(
            requireActivity().contentResolver,
            MediaStore.Video.Media.EXTERNAL_CONTENT_URI
        )
            .setContentValues(contentValues)
            .build()

        activeRecording = videoCapture.output.prepareRecording(requireActivity(), mediaStoreOutput)
            .withAudioEnabled()
            .start(
                mainThreadExecutor,
                captureListener
            )

        Timber.i("Recording started")
    }

    private fun nextQuestion() {
        viewModel.next()
    }

    private fun updateUI(event: VideoRecordEvent) {
        val state = if (event is VideoRecordEvent.Status) recordingState.getName()
        else event.getName()
        when (event) {
            is VideoRecordEvent.Status -> {
                // placeholder: we update the UI with new status after this when() block,
                // nothing needs to do here.
            }
            is VideoRecordEvent.Start -> {
                Timber.i("VideoRecordEvent.Start")
            }
            is VideoRecordEvent.Finalize -> {
                Timber.i("VideoRecordEvent.Finalize")
                if (BuildConfig.DEBUG)
                    Toast.makeText(context, "回答上传中...", Toast.LENGTH_SHORT).show()
                val outputUri =
                    (recordingState as VideoRecordEvent.Finalize).outputResults.outputUri
                Timber.i("output file path is :${outputUri.path}")
                val file = File(getPath(outputUri))
                Timber.i("output file is :${file.name} : ${file.length()}")
                viewModel.uploadQuestion(file)
            }
            is VideoRecordEvent.Pause -> {
                Timber.i("VideoRecordEvent.Pause")
            }
            is VideoRecordEvent.Resume -> {
                Timber.i("VideoRecordEvent.Resume")
            }
            else -> {
                Timber.e("Error(Unknown Event) from Recorder")
                return
            }
        }

        val stats = event.recordingStats
        val size = stats.numBytesRecorded / 1000
        val time = TimeUnit.NANOSECONDS.toSeconds(stats.recordedDurationNanos)
        var text = "$state: recorded ${size}KB, in ${time}second"
        if (event is VideoRecordEvent.Finalize)
            text = "${text}\nFile saved to: ${event.outputResults.outputUri}"

        if (BuildConfig.DEBUG) viewBinding.captureStatus.isVisible = true
        viewBinding.captureStatus.text = text
        Timber.i("recording event: $text")
    }

    private fun stopAnswer() {
        if (activeRecording == null || recordingState is VideoRecordEvent.Finalize) {
            return
        }
        val recording = activeRecording
        if (recording != null) {
            recording.stop()
            activeRecording = null
            viewModel.stopAnswer()
        }
    }

    private fun showVideo(uri: Uri) {
        if (uri.scheme.toString().compareTo("content") == 0 || uri.scheme.toString()
                .compareTo("android.resource") == 0
        ) {
            val resolver = requireContext().contentResolver
            resolver.query(uri, null, null, null, null)?.use { cursor ->
                val sizeIndex = cursor.getColumnIndex(OpenableColumns.SIZE)
                cursor.moveToFirst()

                val fileSize = cursor.getLong(sizeIndex)
                if (fileSize <= 0) {
                    Timber.i("Recorded file size is 0, could not be played!")
                    return
                }

                val dataIndex = cursor.getColumnIndexOrThrow(MediaStore.Images.Media.DATA)
                val fileInfo =
                    "FileSize: $fileSize" + dataIndex.let { "\n ${cursor.getString(it)}" }

                Timber.i(fileInfo)
                viewBinding.captureStatus.text = fileInfo
            }
            videoDispose = Observable.just(0).delay(150L, TimeUnit.MILLISECONDS).subscribe({
                Timber.i("showVideo")
                mediaPlayer.apply {
                    reset()
                    setDisplay(viewBinding.svVideo.holder)
                    setDataSource(requireContext(), uri)
                    prepareAsync()
                }
            }, {
                Timber.e(it)
            })
        }
    }
}
