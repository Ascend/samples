/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data

import okhttp3.Interceptor
import okhttp3.Request
import okhttp3.Response

class HeaderInterceptor : Interceptor {
    private val code = "MTQ0MDYzNDcyM0BxcS5jb206YWs0OXUyMzd0NXd4eXNtdQ=="
    override fun intercept(chain: Interceptor.Chain): Response {
//        Timber.e("------------- Try to add headers -------------")
        val request: Request = chain.request()

        val newRequest: Request = request.newBuilder()
            .addHeader("Authorization", "Basic $code")
            .addHeader("Accept", "*/*")
            .addHeader("Accept-Encoding", "gzip, deflate, br")
            .addHeader("Connection", "Keep-Alive")
            .build()
        return chain.proceed(newRequest)
    }
}
