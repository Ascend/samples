/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data

import android.content.Context
import com.chris.depression.domain.QuestionPojo
import com.chris.depression.domain.QuestionRepository
import com.chris.depression.utils.AssetReaderUtils
import com.google.gson.reflect.TypeToken
import kotlinx.coroutines.runBlocking
import timber.log.Timber
import javax.inject.Inject
import kotlin.coroutines.suspendCoroutine

class LocalQuestionRepository @Inject constructor() : QuestionRepository {
    override fun fetchQuestion(context: Context, fileName: String): QuestionPojo {
        return runBlocking {
            getLocalQuestion(context, fileName)
        }
    }

    private suspend fun getLocalQuestion(context: Context, fileName: String): QuestionPojo =
        suspendCoroutine {
            Timber.e("get file $fileName")
            it.resumeWith(
                Result.success(
                    AssetReaderUtils.readJson<QuestionPojo>(
                        context,
                        fileName,
                        object : TypeToken<QuestionPojo>() {}.type
                    )
                )
            )
        }
}
