/*
 * Copyright 2022 Leon.
 */
package com.chris.depression

import android.content.Context
import dagger.Module
import dagger.Provides
import javax.inject.Singleton

@Module
class AppModule {

    @Singleton
    @Provides
    fun providePackageName(app: App): String {
        return app.applicationContext.packageName
    }

    @Singleton
    @Provides
    fun provideAppContext(app: App): Context {
        return app.applicationContext
    }
}
