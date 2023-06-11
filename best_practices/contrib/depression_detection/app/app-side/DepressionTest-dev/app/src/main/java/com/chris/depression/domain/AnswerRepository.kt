/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain

import com.chris.depression.ui.CurrentTest
import io.reactivex.Single
import java.io.File

interface AnswerRepository {

    fun commitAnswer(answer: CurrentTest): Boolean

    fun uploadQuestionVideo(questionVideo: File): Single<String>

    fun predictionUploadFiles(files: List<File>): Single<String>
    fun fetchFile()
}
