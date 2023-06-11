/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data

import okhttp3.logging.HttpLoggingInterceptor
import timber.log.Timber

class HttpLogger : HttpLoggingInterceptor.Logger {

    companion object {
        const val TAG = "HttpLogger"
    }

    override fun log(message: String) {
        Timber.tag(TAG).d(message)
    }
}
