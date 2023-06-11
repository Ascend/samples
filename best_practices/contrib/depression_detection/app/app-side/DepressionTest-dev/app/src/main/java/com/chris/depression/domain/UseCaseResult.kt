/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain

sealed class UseCaseResult<DATA_TYPE> {
    class Success<DATA_TYPE>(val data: DATA_TYPE) : UseCaseResult<DATA_TYPE>()
    class Failure<DATA_TYPE>(val error: Throwable) : UseCaseResult<DATA_TYPE>()
}
