/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data.exception

import com.chris.depression.data.exception.ExceptionType.NETWORK
import com.chris.depression.data.exception.ExceptionType.NONE
import com.chris.depression.domain.model.dto.ErrorResponseDTO

open class ResponseException(
    open val type: ExceptionType = NONE,
    override val message: String = "ResponseException",
    val errorResponse: ErrorResponseDTO? = null
) : Exception() {

    companion object {
        fun noneException(): ResponseException {
            return ResponseException(NONE)
        }
    }
}

class NoNetworkConnectionException : ResponseException(NETWORK) {
    override val message: String = "No network connectivity exception!"
}

enum class ExceptionType {
    NONE, HTTP_EXCEPTION, NETWORK, RESULT, JSON_PARSE, OTHER
}
