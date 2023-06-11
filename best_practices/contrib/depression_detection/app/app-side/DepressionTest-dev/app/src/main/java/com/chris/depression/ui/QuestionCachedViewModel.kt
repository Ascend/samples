/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import javax.inject.Inject

class QuestionCachedViewModel @Inject constructor() : ViewModel() {

    private val _currentTest: MutableLiveData<CurrentTest> = MutableLiveData()
    val currentTest: LiveData<CurrentTest> = _currentTest

    fun cacheQuestion(it: CurrentTest) {
        _currentTest.value = it
    }
}
