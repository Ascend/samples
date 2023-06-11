/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain

import android.content.Context

data class FetchLocalQuestionRequest(
    val context: Context,
    val fileName: String
)
