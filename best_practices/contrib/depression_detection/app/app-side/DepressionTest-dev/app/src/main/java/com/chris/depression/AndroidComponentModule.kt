/*
 * Copyright 2022 Leon.
 */
package com.chris.depression

import com.chris.depression.ui.MainActivity
import com.chris.depression.ui.QuestionFragment
import com.chris.depression.ui.StartFragment
import com.chris.depression.ui.TestListFragment
import com.chris.depression.ui.VideoViewerFragment
import dagger.Module
import dagger.android.ContributesAndroidInjector

@Module
internal abstract class AndroidComponentModule {

    @ContributesAndroidInjector
    abstract fun mainActivity(): MainActivity

    // ----------------  fragment   -----------------

    @ContributesAndroidInjector
    abstract fun testListFragment(): TestListFragment

    @ContributesAndroidInjector
    abstract fun startFragment(): StartFragment

    @ContributesAndroidInjector
    abstract fun videoViewerFragment(): VideoViewerFragment

    @ContributesAndroidInjector
    abstract fun questionFragment(): QuestionFragment
}
