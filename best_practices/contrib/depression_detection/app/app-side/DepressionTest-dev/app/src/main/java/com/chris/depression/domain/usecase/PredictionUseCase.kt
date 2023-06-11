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

class PredictionUseCase @Inject constructor(
    coroutineContextProvider: CoroutineContextProvider,
    private val answerRepository: AnswerRepository
) : BackgroundExecuteUseCase<List<File>, Single<String>>(coroutineContextProvider) {
    override suspend fun executeInBackground(
        value: List<File>,
        coroutineScope: CoroutineScope
    ): Single<String> {
        return answerRepository.predictionUploadFiles(value)
    }
}
