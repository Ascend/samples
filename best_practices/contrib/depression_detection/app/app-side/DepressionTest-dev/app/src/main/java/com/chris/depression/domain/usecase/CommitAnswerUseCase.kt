/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain.usecase

import com.chris.depression.domain.AnswerRepository
import com.chris.depression.domain.CoroutineContextProvider
import com.chris.depression.ui.CurrentTest
import kotlinx.coroutines.CoroutineScope
import javax.inject.Inject

class CommitAnswerUseCase @Inject constructor(
    coroutineContextProvider: CoroutineContextProvider,
    private val answerRepository: AnswerRepository
) : BackgroundExecuteUseCase<CurrentTest, Boolean>(coroutineContextProvider) {
    override suspend fun executeInBackground(
        value: CurrentTest,
        coroutineScope: CoroutineScope
    ): Boolean {
        return answerRepository.commitAnswer(value)
    }
}
