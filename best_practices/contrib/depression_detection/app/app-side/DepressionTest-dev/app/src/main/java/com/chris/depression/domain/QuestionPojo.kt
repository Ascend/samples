/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain

import com.google.gson.Gson

data class QuestionPojo(
    val questionList: List<Question>,
    val questionIntroduction: String,
    val result: Result
) {
    fun toJson(): String {
        return Gson().toJson(this)
    }
}

data class Question(
    val answer: List<String>,
    val question: String,
    val questionType: String?,
    var selectedAnswer: String?
)

data class Result(
    val resultCopy: String,
    val resultOption: List<String>
)
