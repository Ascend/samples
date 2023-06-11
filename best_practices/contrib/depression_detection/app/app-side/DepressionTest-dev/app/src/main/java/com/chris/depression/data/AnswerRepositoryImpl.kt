/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data

import android.content.Context
import com.chris.depression.data.api.AnswerService
import com.chris.depression.domain.AnswerRepository
import com.chris.depression.ui.CurrentTest
import io.reactivex.Single
import io.reactivex.android.schedulers.AndroidSchedulers
import io.reactivex.schedulers.Schedulers
import kotlinx.coroutines.runBlocking
import okhttp3.MediaType.Companion.toMediaTypeOrNull
import okhttp3.MultipartBody
import okhttp3.RequestBody.Companion.asRequestBody
import okhttp3.RequestBody.Companion.toRequestBody
import timber.log.Timber
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import javax.inject.Inject

class AnswerRepositoryImpl @Inject constructor(
    private val answerService: AnswerService,
    private val context: Context
) : AnswerRepository {

    companion object {
        const val serverUrl = "http://140.210.206.227:8888/number"
    }

    override fun commitAnswer(answer: CurrentTest): Boolean {
        return runBlocking {
            postAnswer(answer)
        }
    }

    override fun uploadQuestionVideo(questionVideo: File): Single<String> {
        return Single.just(questionVideo)
            .map {
                val dir = "TestResult/${getTodayStr()}"
                if (!isExist(dir)) {
                    createDirectory(dir)
                }
                uploadVideo(dir, it)
            }.map {
                "upload"
            }
            .subscribeOn(Schedulers.io())
            .observeOn(AndroidSchedulers.mainThread())
    }

    override fun predictionUploadFiles(files: List<File>): Single<String> {
        return Single.just(files)
            .map { list ->
                val audioList = ArrayList<File>()
                list.forEach {
                    val audioFromVideo = SplitMediaFile.extractAudio(context, it)
                    Timber.e("audio file: ${audioFromVideo.name} -> ${audioFromVideo.length()}")
                    audioList.add(audioFromVideo)
                }
                audioList
            }.map { audios ->
                Timber.e(audios.toString())
                val requestBody = MultipartBody.Builder().apply {
                    audios.forEach { audio ->
                        val requestBody = audio.asRequestBody()
                        addFormDataPart("file", audio.name, requestBody)
                    }
                    setType(MultipartBody.FORM)
                }.build()
                Timber.e("request Body: $requestBody")
                val result = runBlocking {
                    kotlin.runCatching {
                        answerService.prediction(serverUrl, requestBody)
                    }
                }
                Timber.e("result: $result")
                result.getOrNull()?.get("result").toString()
            }
            .subscribeOn(Schedulers.io())
            .observeOn(AndroidSchedulers.mainThread())
    }

    override fun fetchFile() {
        val dir = "TestResult/${getTodayStr()}"
        if (!isExist(dir)) {
            createDirectory(dir)
        }
    }

    private fun getTodayStr(): String {
        val date = Date()
        val format = SimpleDateFormat("yyyy_MM_dd", Locale.getDefault())
        return format.format(date)
    }

    private fun uploadVideo(dir: String, file: File) {
        baseRun {
            answerService.uploadFile(
                dir,
                file.name,
                file.asRequestBody("application/octet-stream".toMediaTypeOrNull())
            )
        }
    }

    private fun createDirectory(dir: String): Boolean {
        return baseRun { answerService.createDirectory(dir) }
    }

    private fun isExist(dir: String): Boolean {
        return baseRun { answerService.exists(dir) }
    }

    private fun <T> baseRun(action: (suspend () -> T)): Boolean {
        return runBlocking {
            var result = false
            kotlin.runCatching { action.invoke() }
                .onFailure {
                    if (it is KotlinNullPointerException)
                        result = true
                    else
                        Timber.e("this is $it")
                }.onSuccess {
                    result = true
                }
            result
        }
    }

    private suspend fun postAnswer(answer: CurrentTest): Boolean {
        return baseRun {
            Timber.e("http request json: ${answer.question.toJson()}")
            answerService.uploadFile(
                "answer/${getTodayStr()}",
                answer.fileName,
                answer.question.toJson().toRequestBody("application/octet".toMediaTypeOrNull())
            )
        }
    }
}
