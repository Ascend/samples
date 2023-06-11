/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data.exception

interface ExceptionHandler {

    fun handleException(throwable: Throwable): ResponseException
}
