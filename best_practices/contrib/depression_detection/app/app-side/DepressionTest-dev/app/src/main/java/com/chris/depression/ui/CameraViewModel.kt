/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import androidx.lifecycle.ViewModel
import com.chris.depression.domain.UseCaseExecutor
import com.chris.depression.domain.UseCaseResult
import com.chris.depression.domain.usecase.UploadQuestionUseCase
import com.chris.depression.utils.SingleLiveEvent
import io.reactivex.SingleObserver
import io.reactivex.disposables.Disposable
import timber.log.Timber
import java.io.File
import javax.inject.Inject

class CameraViewModel @Inject constructor(
    private val useCaseExecutor: UseCaseExecutor,
    private val uploadQuestionUseCase: UploadQuestionUseCase
) : ViewModel() {

    val uploaded: SingleLiveEvent<Boolean> = SingleLiveEvent()

    fun uploadQuestion(file: File) {
        Timber.e("start upload")
        useCaseExecutor.execute(uploadQuestionUseCase, file) { result ->
            Timber.e("result: $result")
            if (result is UseCaseResult.Success) {
                result.data.subscribe(object : SingleObserver<String> {
                    override fun onSubscribe(d: Disposable) {
                    }

                    override fun onSuccess(t: String) {
                        Timber.e("success: $t")
                        uploaded.value = true
                    }

                    override fun onError(e: Throwable) {
                        Timber.e("some error")
                        Timber.e(e)
                        uploaded.value = true
                    }
                })
            }
        }
    }
}
