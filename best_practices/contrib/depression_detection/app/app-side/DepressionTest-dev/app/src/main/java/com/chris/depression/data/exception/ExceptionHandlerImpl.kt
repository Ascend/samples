/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data.exception

import com.chris.depression.data.exception.ExceptionType.HTTP_EXCEPTION
import com.chris.depression.data.exception.ExceptionType.JSON_PARSE
import com.chris.depression.data.exception.ExceptionType.NETWORK
import com.chris.depression.data.exception.ExceptionType.OTHER
import com.chris.depression.domain.model.dto.ErrorResponseDTO
import com.google.gson.Gson
import com.google.gson.JsonParseException
import org.json.JSONException
import retrofit2.HttpException
import java.text.ParseException
import javax.inject.Inject
import javax.net.ssl.SSLHandshakeException

class ExceptionHandlerImpl @Inject constructor() : ExceptionHandler {

    private val gson = Gson()
    override fun handleException(throwable: Throwable): ResponseException {
        val responseException: ResponseException
        when (throwable) {
            is HttpException -> {
                var errorInfo: ErrorResponseDTO? = null
                try {
                    errorInfo = gson.fromJson(
                        throwable.response()?.errorBody()?.string(),
                        ErrorResponseDTO::class.java
                    )
                } catch (e: Exception) {
                }
                responseException =
                    ResponseException(HTTP_EXCEPTION, throwable.message(), errorInfo)
            }
            is JsonParseException, is JSONException, is ParseException -> {
                responseException =
                    ResponseException(
                        JSON_PARSE,
                        throwable.message ?: DEFAULT_JSON_ERROR_MESSAGE
                    )
            }
            is SSLHandshakeException -> {
                responseException = ResponseException(
                    NETWORK,
                    throwable.message ?: DEFAULT_SSL_HAND_SHAKE_ERROR_MESSAGE
                )
            }
            is NoNetworkConnectionException -> {
                responseException = ResponseException(NETWORK, throwable.message)
            }
            else -> {
                responseException =
                    ResponseException(
                        OTHER,
                        throwable.message ?: DEFAULT_OTHER_ERROR_MESSAGE
                    )
            }
        }
        return responseException
    }

    companion object {
        const val DEFAULT_JSON_ERROR_MESSAGE = "Json error"
        const val DEFAULT_SSL_HAND_SHAKE_ERROR_MESSAGE = "SSLHandshake error"
        const val DEFAULT_OTHER_ERROR_MESSAGE = "SSLHandshake error"
    }
}
