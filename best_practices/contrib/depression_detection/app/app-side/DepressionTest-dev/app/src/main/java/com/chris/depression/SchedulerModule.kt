/*
 * Copyright 2022 Leon.
 */
package com.chris.depression

import com.chris.depression.domain.CoroutineContextProvider
import com.chris.depression.utils.DispatchersCoroutineContextProvider
import dagger.Module
import dagger.Provides
import kotlinx.coroutines.Job
import javax.inject.Singleton

@Module
class SchedulerModule {

//    @Provides
//    @Singleton
//    fun provideJobThread(): JobThread {
//        return object : JobThread {
//            override fun provideUI(): Scheduler {
//                return AndroidSchedulers.mainThread()
//            }
//
//            override fun provideWorker(): Scheduler {
//                return Schedulers.io()
//            }
//        }
//    }

    @Singleton
    @Provides
    fun provideJobProvider(): JobProvider = object : JobProvider {
        override fun invoke(): Job = Job()
    }

    @Singleton
    @Provides
    fun provideCoroutineContextProvider(): CoroutineContextProvider =
        DispatchersCoroutineContextProvider()
}
