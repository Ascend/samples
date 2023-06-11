/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui.survey

import androidx.annotation.StringRes
import androidx.compose.runtime.Stable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue

@Stable
class QuestionState(
    val question: Question,
    val questionIndex: Int,
    val totalQuestionsCount: Int,
    val showPrevious: Boolean,
    val showDone: Boolean
) {
    var enableNext by mutableStateOf(false)
    var answer by mutableStateOf<Answer<*>?>(null)
}

sealed class SurveyState {
    data class Start(
        @StringRes val surveyTitle: Int,
        val surveyStart: SurveyStart
    ) : SurveyState()

    data class Questions(
        val surveyTitle: String,
        val questionsState: List<QuestionState>
    ) : SurveyState() {
        var currentQuestionIndex by mutableStateOf(0)
    }

    data class Result(
        val surveyTitle: String,
        val surveyResult: SurveyResult
    ) : SurveyState()
}
