/*
 * Copyright 2022 Leon.
 */
package com.chris.depression

import dagger.android.AndroidInjector
import dagger.android.support.DaggerApplication
import timber.log.Timber
import javax.inject.Inject

class App : DaggerApplication() {

    @Inject
    fun logInject() {
        Timber.tag(TAG).d("injector has invoked.")
    }

    override fun onCreate() {
        super.onCreate()
        Timber.plant(Timber.DebugTree())
    }

    override fun applicationInjector(): AndroidInjector<out DaggerApplication> {
        return DaggerAppComponent.factory().create(this)
    }

    companion object {
        const val TAG = "App"
    }
}
