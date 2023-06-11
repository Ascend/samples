/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import android.content.Context
import androidx.annotation.StringRes
import androidx.lifecycle.ViewModel
import com.chris.depression.R
import com.chris.depression.domain.FetchLocalQuestionRequest
import com.chris.depression.domain.UseCaseExecutor
import com.chris.depression.domain.UseCaseResult
import com.chris.depression.domain.usecase.FetchQuestionUseCase
import com.chris.depression.utils.SingleLiveEvent
import timber.log.Timber
import javax.inject.Inject

class TestListViewModel @Inject constructor(
    private val useCaseExecutor: UseCaseExecutor,
    private val fetchQuestionUseCase: FetchQuestionUseCase
) : ViewModel() {

    val currentTest: SingleLiveEvent<CurrentTest> = SingleLiveEvent()

    fun fetchQuestion(context: Context?, item: QuestionItem) {
        context?.let {
            val quest = FetchLocalQuestionRequest(it, item.jsonFileName)
            useCaseExecutor.execute(fetchQuestionUseCase, quest) { result ->
                if (result is UseCaseResult.Success) {
                    Timber.e("result success: %s", result.data.toString())
                    result.data.let { data ->
                        currentTest.value = CurrentTest(item.stringId, item.jsonFileName, data)
                    }
                }
            }
        }
    }

    val questionList = arrayListOf(
        QuestionItem(
            R.string.test_list_phq9,
            "phq9.json"
        ),
        QuestionItem(
            R.string.test_list_ctq_sf,
            "ctq_sf.json"
        ),
        QuestionItem(
            R.string.test_list_gad7,
            "gad7.json"
        ),
        QuestionItem(
            R.string.test_list_les,
            "les.json"
        ),
        QuestionItem(
            R.string.test_list_ssrs,
            "ssrs.json"
        ),
        QuestionItem(
            R.string.test_list_psqi,
            "psqi.json"
        ),
        QuestionItem(
            R.string.test_list_test, "test.json"
        )
    )

    data class QuestionItem(@StringRes val stringId: Int, val jsonFileName: String)
}
