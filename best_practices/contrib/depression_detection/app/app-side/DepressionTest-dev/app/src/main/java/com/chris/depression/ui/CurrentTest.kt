/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import androidx.annotation.StringRes
import com.chris.depression.domain.QuestionPojo

data class CurrentTest(
    @StringRes val testTitleRes: Int,
    val fileName: String,
    val question: QuestionPojo
)
