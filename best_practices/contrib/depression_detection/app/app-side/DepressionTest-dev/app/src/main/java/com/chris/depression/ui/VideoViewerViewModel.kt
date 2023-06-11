/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import android.net.Uri
import androidx.annotation.DrawableRes
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import com.chris.depression.R
import com.chris.depression.domain.UseCaseExecutor
import com.chris.depression.domain.UseCaseResult
import com.chris.depression.domain.usecase.PredictionUseCase
import com.chris.depression.domain.usecase.UploadQuestionUseCase
import com.chris.depression.ui.VideoViewerViewModel.PageStates.CaptureCameraState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.CaptureStopState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.PredictResultState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.PredictingState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.StartPredictionState
import com.chris.depression.ui.VideoViewerViewModel.PageStates.VideoPlayingState
import com.chris.depression.utils.SingleLiveEvent
import io.reactivex.SingleObserver
import io.reactivex.disposables.Disposable
import timber.log.Timber
import java.io.File
import javax.inject.Inject

class VideoViewerViewModel @Inject constructor(
    private val useCaseExecutor: UseCaseExecutor,
    private val uploadQuestionUseCase: UploadQuestionUseCase,
    private val predictionUseCase: PredictionUseCase,
    packageName: String
) : BaseViewModel() {

    private val _currentQuestion = MutableLiveData<CurrentQuestion>()
    val currentQuestion: LiveData<CurrentQuestion> = _currentQuestion
    val uploaded: SingleLiveEvent<Boolean> = SingleLiveEvent()

    //    val finished: SingleLiveEvent<Boolean> = SingleLiveEvent()
//    private val fileList = ArrayList<File>()
    val stringMsg = MutableLiveData<String>()

    private val _pageStates: MutableLiveData<PageStates> = MutableLiveData()
    val pageState: LiveData<PageStates> = _pageStates
    private lateinit var tempFile: File

    companion object {
        const val NO_IMAGE = -1
    }

    private var index = 0

    fun start() {
        _currentQuestion.value = questionList[index]
    }

    fun next() {
        if (++index < questionList.size) {
            start()
            _pageStates.value = VideoPlayingState
        } else {
            _pageStates.value = PredictingState
        }
        singlePredictionUpload()
    }

//    private fun predictionUpload() {
//        useCaseExecutor.execute(predictionUseCase, fileList) { result ->
//            if (result is UseCaseResult.Success) {
//                result.data.subscribe(object : SingleObserver<String> {
//                    override fun onSubscribe(d: Disposable) {
//                    }
//
//                    override fun onSuccess(t: String) {
//                        Timber.e("success $t")
//                        setLoading(false)
//                        stringMsg.value = t
//                        finished.value = true
//                    }
//
//                    override fun onError(e: Throwable) {
//                        Timber.e(e, "errors ---------")
//                        finished.value = true
//                        setLoading(false)
//                    }
//                })
//            }
//            Timber.i("return information: $result")
//        }
//    }

    private fun singlePredictionUpload() {
        useCaseExecutor.execute(predictionUseCase, listOf(tempFile)) { result ->
            if (result is UseCaseResult.Success) {
                result.data.subscribe(object : SingleObserver<String> {
                    override fun onSubscribe(d: Disposable) {
                    }

                    override fun onSuccess(t: String) {
                        Timber.e("success $t")
                        setLoading(false)
                        stringMsg.value = t
                        if (_pageStates.value == PredictingState)
                            _pageStates.value = PredictResultState(t)
                    }

                    override fun onError(e: Throwable) {
                        Timber.e(e, "errors ---------")
                        if (_pageStates.value == PredictingState)
                            _pageStates.value = PredictResultState("error occur")
                        setLoading(false)
                    }
                })
            }
            Timber.i("return information: $result")
        }
    }

    fun uploadQuestion(file: File) {
//        fileList.add(file)
//        singlePredictionUpload(file)
        tempFile = file
        Timber.e("start upload")
        useCaseExecutor.execute(uploadQuestionUseCase, file) { result ->
            Timber.e("result: $result")
            if (result is UseCaseResult.Success) {
                result.data.subscribe(object : SingleObserver<String> {
                    override fun onSubscribe(d: Disposable) {
                    }

                    override fun onSuccess(t: String) {
                        uploaded.value = true
                    }

                    override fun onError(e: Throwable) {
                        Timber.e(e, "error happened")
                        uploaded.value = true
                    }
                })
            }
        }
    }

    fun startAnswer() {
        _pageStates.value = CaptureCameraState
    }

    fun stopAnswer() {
        _pageStates.value =
            if ((index + 1) >= questionList.size) StartPredictionState else CaptureStopState
    }

    val questionList = listOf(
        CurrentQuestion(
            "平时喜欢做什么事情？",
            Uri.parse("android.resource://" + packageName + "/" + R.raw.question2)
        ),
        CurrentQuestion(
            "最近有什么令自己高兴的事情吗？",
            Uri.parse("android.resource://" + packageName + "/" + R.raw.question3)
        ),
        CurrentQuestion(
            "阅读词组：(横向读)",
            Uri.parse("android.resource://" + packageName + "/" + R.raw.question23),
            R.drawable.question23
        )
    )

    data class CurrentQuestion(
        val questionTitle: String,
        val questionVideo: Uri,
        @DrawableRes val questionImage: Int = NO_IMAGE
    )

    sealed class PageStates {
        object VideoPlayingState : PageStates()
        object CaptureCameraState : PageStates()
        object CaptureStopState : PageStates()
        object StartPredictionState : PageStates()
        object PredictingState : PageStates()
        class PredictResultState(val result: String) : PageStates()
    }
}
