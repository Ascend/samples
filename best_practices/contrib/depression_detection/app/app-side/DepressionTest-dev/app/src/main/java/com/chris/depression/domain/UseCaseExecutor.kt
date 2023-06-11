/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain

import com.chris.depression.JobProvider
import com.chris.depression.domain.usecase.BaseUseCase
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
class UseCaseExecutor @Inject constructor(
    jobProvider: JobProvider,
    coroutineContextProvider: CoroutineContextProvider
) {
    private var uiScope = CoroutineScope(coroutineContextProvider.main + jobProvider())

    fun <OUTPUT_TYPE> execute(
        useCase: BaseUseCase<Unit, OUTPUT_TYPE>,
        callback: (UseCaseResult<OUTPUT_TYPE>) -> Unit = {}
    ) = execute(useCase, Unit, callback)

    fun <INPUT_TYPE, OUTPUT_TYPE> execute(
        useCase: BaseUseCase<INPUT_TYPE, OUTPUT_TYPE>,
        value: INPUT_TYPE,
        callback: (UseCaseResult<OUTPUT_TYPE>) -> Unit = {}
    ) {
        uiScope.launch {
            useCase.execute(value, callback)
        }
    }

    fun stopAll() {
        uiScope.cancel()
    }
}
