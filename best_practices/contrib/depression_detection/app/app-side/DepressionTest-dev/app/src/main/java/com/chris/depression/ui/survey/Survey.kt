/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui.survey

import android.net.Uri
import androidx.annotation.StringRes

data class SurveyStart(
    val surveyName: String,
    val intro: String,
    val description: String
)

data class SurveyResult(
    val library: String,
    @StringRes val result: Int,
    @StringRes val description: Int
)

data class Survey(
    val title: String,
    val questions: List<Question>
)

data class Question(
    val id: Int,
    val questionText: String,
    val answer: PossibleAnswer,
    @StringRes val description: Int? = null,
    val permissionsRequired: List<String> = emptyList(),
    @StringRes val permissionsRationaleText: Int? = null
)

/**
 * Type of supported actions for a survey
 */
enum class SurveyActionType { PICK_DATE, TAKE_PHOTO, SELECT_CONTACT }

sealed class SurveyActionResult {
    data class Date(val date: String) : SurveyActionResult()
    data class Photo(val uri: Uri) : SurveyActionResult()
    data class Contact(val contact: String) : SurveyActionResult()
}

sealed class PossibleAnswer {
    data class SingleChoice(val optionsString: List<String>) : PossibleAnswer()
    data class SingleChoiceIcon(val optionsStringIconRes: List<Pair<Int, String>>) :
        PossibleAnswer()

    object InputAnswer : PossibleAnswer()

    data class MultipleChoice(val optionsStringRes: List<String>) : PossibleAnswer()
    data class MultipleChoiceIcon(val optionsStringIconRes: List<Pair<Int, String>>) :
        PossibleAnswer()

    data class Action(
        @StringRes val label: Int,
        val actionType: SurveyActionType
    ) : PossibleAnswer()

    data class Slider(
        val range: ClosedFloatingPointRange<Float>,
        val steps: Int,
        @StringRes val startText: Int,
        @StringRes val endText: Int,
        @StringRes val neutralText: Int,
        val defaultValue: Float = 5.5f
    ) : PossibleAnswer()
}

sealed class Answer<T : PossibleAnswer> {
    abstract var answer: String

    object PermissionsDenied : Answer<Nothing>() {
        override var answer: String
            get() = ""
            set(value) {}
    }

    data class SingleChoice(override var answer: String) : Answer<PossibleAnswer.SingleChoice>()
    data class InputAnswer(override var answer: String) : Answer<PossibleAnswer.InputAnswer>()
    data class MultipleChoice(val answersStringRes: Set<String>) :
        Answer<PossibleAnswer.MultipleChoice>() {
        override var answer: String
            get() = ""
            set(value) {}
    }

    data class Action(val result: SurveyActionResult) : Answer<PossibleAnswer.Action>() {
        override var answer: String
            get() = ""
            set(value) {}
    }

    data class Slider(val answerValue: Float) : Answer<PossibleAnswer.Slider>() {
        override var answer: String
            get() = ""
            set(value) {}
    }
}

/**
 * Add or remove an answer from the list of selected answers depending on whether the answer was
 * selected or deselected.
 */
fun Answer.MultipleChoice.withAnswerSelected(
    answer: String,
    selected: Boolean
): Answer.MultipleChoice {
    val newStringRes = answersStringRes.toMutableSet()
    if (!selected) {
        newStringRes.remove(answer)
    } else {
        newStringRes.add(answer)
    }
    return Answer.MultipleChoice(newStringRes)
}
