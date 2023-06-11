/*
 * Copyright 2022 Leon.
 */
package com.chris.depression

import com.chris.depression.data.HeaderInterceptor
import com.chris.depression.data.HttpLogger
import com.chris.depression.data.api.AnswerService
import com.google.gson.Gson
import dagger.Module
import dagger.Provides
import okhttp3.OkHttpClient
import okhttp3.logging.HttpLoggingInterceptor
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import java.util.concurrent.TimeUnit
import javax.inject.Singleton

@Module
class NetModule {

//    private val username = "1440634723@qq.com"
//    private val pwd = "ak49u237t5wxysmu"

    @Provides
    @Singleton
    fun provideAccountInfoService(retrofit: Retrofit): AnswerService =
        retrofit.create(AnswerService::class.java)

    @Singleton
    @Provides
    fun logger(): HttpLoggingInterceptor.Logger {
        return HttpLogger()
    }

    @Singleton
    @Provides
    fun loggingInterceptor(logger: HttpLoggingInterceptor.Logger): HttpLoggingInterceptor {
        val httpLoggingInterceptor = HttpLoggingInterceptor(logger)
        httpLoggingInterceptor.level = HttpLoggingInterceptor.Level.BODY
        return httpLoggingInterceptor
    }

    @Singleton
    @Provides
    fun headerInterceptor(): HeaderInterceptor {
        return HeaderInterceptor()
    }

    @Singleton
    @Provides
    fun provideOkHttpClient(
        logger: HttpLoggingInterceptor,
        header: HeaderInterceptor
    ): OkHttpClient {
        return OkHttpClient.Builder()
//            .addNetworkInterceptor(header)
            .addNetworkInterceptor(logger)
//            .addInterceptor(logger)
            .addInterceptor(header)
            .connectTimeout(1L, TimeUnit.MINUTES)
            .writeTimeout(1L, TimeUnit.MINUTES)
            .readTimeout(1L, TimeUnit.MINUTES)
            .build()
    }

    @Singleton
    @Provides
    fun provideRetrofit(client: OkHttpClient, gson: Gson): Retrofit {
        return Retrofit.Builder()
            .baseUrl("https://dav.jianguoyun.com/")
            .client(client)
            .addConverterFactory(GsonConverterFactory.create(gson))
//            .addCallAdapterFactory(RxJava2CallAdapterFactory.create())
            .build()
    }
}
