/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain.usecase

import com.chris.depression.domain.CoroutineContextProvider
import com.chris.depression.domain.FetchLocalQuestionRequest
import com.chris.depression.domain.QuestionPojo
import com.chris.depression.domain.QuestionRepository
import kotlinx.coroutines.CoroutineScope
import javax.inject.Inject

class FetchQuestionUseCase @Inject constructor(
    coroutineContextProvider: CoroutineContextProvider,
    private val questionRepository: QuestionRepository
) : BackgroundExecuteUseCase<FetchLocalQuestionRequest, QuestionPojo>(coroutineContextProvider) {
    override suspend fun executeInBackground(
        value: FetchLocalQuestionRequest,
        coroutineScope: CoroutineScope
    ): QuestionPojo {
        return questionRepository.fetchQuestion(value.context, value.fileName)
    }
}
