/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain.usecase

import com.chris.depression.domain.AnswerRepository
import com.chris.depression.domain.CoroutineContextProvider
import io.reactivex.Single
import kotlinx.coroutines.CoroutineScope
import java.io.File
import javax.inject.Inject

class UploadQuestionUseCase @Inject constructor(
    coroutineContextProvider: CoroutineContextProvider,
    private val answerRepository: AnswerRepository
) : BackgroundExecuteUseCase<File, Single<String>>(coroutineContextProvider) {
    override suspend fun executeInBackground(
        value: File,
        coroutineScope: CoroutineScope
    ): Single<String> {
        return answerRepository.uploadQuestionVideo(value)
    }
}
