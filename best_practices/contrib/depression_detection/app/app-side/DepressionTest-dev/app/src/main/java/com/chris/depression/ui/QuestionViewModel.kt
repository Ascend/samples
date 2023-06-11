/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.chris.depression.R
import com.chris.depression.domain.UseCaseExecutor
import com.chris.depression.domain.UseCaseResult
import com.chris.depression.domain.usecase.CommitAnswerUseCase
import com.chris.depression.ui.survey.Answer
import com.chris.depression.ui.survey.PossibleAnswer
import com.chris.depression.ui.survey.Question
import com.chris.depression.ui.survey.QuestionState
import com.chris.depression.ui.survey.Survey
import com.chris.depression.ui.survey.SurveyResult
import com.chris.depression.ui.survey.SurveyStart
import com.chris.depression.ui.survey.SurveyState
import com.chris.depression.utils.SingleLiveEvent
import kotlinx.coroutines.launch
import timber.log.Timber
import javax.inject.Inject

class QuestionViewModel @Inject constructor(
    private val useCaseExecutor: UseCaseExecutor,
    private val commitAnswerUseCase: CommitAnswerUseCase
) : ViewModel() {

    private val _uiState = MutableLiveData<SurveyState>()
    val uiState: LiveData<SurveyState>
        get() = _uiState

    private lateinit var surveyInitialState: SurveyState

    private lateinit var currentTest: LiveData<CurrentTest>

//    private val questionPojo = MutableLiveData<QuestionPojo>()

    val commit: SingleLiveEvent<Boolean> = SingleLiveEvent()

    fun computeResult(surveyQuestions: SurveyState.Questions) {
        val answers = surveyQuestions.questionsState.mapNotNull { it.answer }
        val result = SurveyResult(
            library = surveyQuestions.surveyTitle,
            result = R.string.survey_result,
            description = R.string.survey_result_description
        )
        _uiState.value = SurveyState.Result(surveyQuestions.surveyTitle, result)
        convertAnswer(answers)
    }

    private fun convertAnswer(answers: List<Answer<*>>) {
        for (i in answers.indices) {
            currentTest.value?.question?.questionList?.get(i)?.selectedAnswer =
                answers[i].answer
        }
        Timber.e(currentTest.value.toString())
    }

    fun fetchQuestion(currentTest: LiveData<CurrentTest>) {
        this.currentTest = currentTest
        viewModelScope.launch {
            val originValue = currentTest.value

            surveyInitialState = SurveyState.Start(
                originValue?.testTitleRes ?: -1,
                SurveyStart(
                    surveyName = originValue?.fileName ?: "",
                    intro = originValue?.question?.questionIntroduction ?: "",
                    description = ""
                )
            )
            _uiState.value = surveyInitialState
        }
    }

    fun startQuestion() {
        viewModelScope.launch {
            val survey = convertQuestion(currentTest.value)
            // Create the default questions state based on the survey questions
            val questions: List<QuestionState> = survey.questions.mapIndexed { index, question ->
                val showPrevious = index > 0
                val showDone = index == survey.questions.size - 1
                QuestionState(
                    question = question,
                    questionIndex = index,
                    totalQuestionsCount = survey.questions.size,
                    showPrevious = showPrevious,
                    showDone = showDone
                )
            }
            val question = SurveyState.Questions(survey.title, questions)
            _uiState.value = question
        }
    }

    private fun convertQuestion(test: CurrentTest?): Survey {
        return if (test != null) {
            Timber.e("convertQuestion -------->")
            val questions = mutableListOf<Question>()
            val questionList = test.question.questionList
            for (i in questionList.indices) {
                val question = questionList[i]
                questions.add(
                    i,
                    Question(
                        id = i,
                        questionText = question.question,
                        answer = if (question.questionType == "I")
                            PossibleAnswer.InputAnswer
                        else
                            PossibleAnswer.SingleChoice(
                                optionsString = question.answer
                            ),
                        description = null
                    )
                )
            }
            Survey(
                title = test.fileName,
                questions = questions
            )
        } else {
            Survey("", listOf())
        }
    }

    fun commitAnswer() {
        useCaseExecutor.execute(commitAnswerUseCase, currentTest.value!!) { result ->
            if (result is UseCaseResult.Success) {
                Timber.e("commit success: %s", result.data)
                commit.value = true
            }
        }
    }
}
