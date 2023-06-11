/*
 * Copyright 2022 Leon.
 */
package com.chris.depression

import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.chris.depression.ui.CameraViewModel
import com.chris.depression.ui.QuestionCachedViewModel
import com.chris.depression.ui.QuestionViewModel
import com.chris.depression.ui.TestListViewModel
import com.chris.depression.ui.VideoViewerViewModel
import dagger.Binds
import dagger.Module
import dagger.multibindings.IntoMap

@Module
abstract class ViewModelModule {

    @Binds
    @IntoMap
    @ViewModelKey(QuestionViewModel::class)
    abstract fun bindQuestionViewModel(viewModel: QuestionViewModel): ViewModel

    @Binds
    @IntoMap
    @ViewModelKey(VideoViewerViewModel::class)
    abstract fun bindVideoViewerViewModel(viewModel: VideoViewerViewModel): ViewModel

    @Binds
    @IntoMap
    @ViewModelKey(CameraViewModel::class)
    abstract fun bindCameraViewModel(viewModel: CameraViewModel): ViewModel

    @Binds
    @IntoMap
    @ViewModelKey(TestListViewModel::class)
    abstract fun bindTestListViewModel(viewModel: TestListViewModel): ViewModel

    @Binds
    @IntoMap
    @ViewModelKey(QuestionCachedViewModel::class)
    abstract fun bindQuestionCachedViewModel(viewModel: QuestionCachedViewModel): ViewModel

    @Binds
    abstract fun bindViewModelFactory(factory: ViewModelProviderFactory): ViewModelProvider.Factory
}
