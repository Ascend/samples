/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain

import android.content.Context

interface QuestionRepository {

    fun fetchQuestion(context: Context, fileName: String): QuestionPojo
}
