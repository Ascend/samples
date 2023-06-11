/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain.usecase

import com.chris.depression.domain.UseCaseResult

abstract class BaseUseCase<INPUT_TYPE, OUTPUT_TYPE> {
    abstract suspend fun execute(value: INPUT_TYPE, callback: (UseCaseResult<OUTPUT_TYPE>) -> Unit)
}
