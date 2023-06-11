/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain.usecase

import com.chris.depression.domain.CoroutineContextProvider
import com.chris.depression.domain.UseCaseResult
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.withContext
import timber.log.Timber

abstract class BackgroundExecuteUseCase<INPUT_TYPE, OUTPUT_TYPE> constructor(
    private val coroutineContextProvider: CoroutineContextProvider
) : BaseUseCase<INPUT_TYPE, OUTPUT_TYPE>() {
    final override suspend fun execute(
        value: INPUT_TYPE,
        callback: (UseCaseResult<OUTPUT_TYPE>) -> Unit
    ) {
        val result = withContext(coroutineContextProvider.io) {
            try {
                Timber.d("==>> ${this@BackgroundExecuteUseCase::class.simpleName} execute input value=$value")
                UseCaseResult.Success(executeInBackground(value, this))
            } catch (throwable: Throwable) {
                UseCaseResult.Failure(throwable)
            }
        }

        callback(result)
    }

    abstract suspend fun executeInBackground(
        value: INPUT_TYPE,
        coroutineScope: CoroutineScope
    ): OUTPUT_TYPE
}
