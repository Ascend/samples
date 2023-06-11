/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data.api

import com.google.gson.JsonObject
import okhttp3.MultipartBody
import okhttp3.RequestBody
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.HTTP
import retrofit2.http.POST
import retrofit2.http.PUT
import retrofit2.http.Path
import retrofit2.http.Url

interface AnswerService {

    @PUT("/dav/Depression/{dir}/{fileName}")
    suspend fun uploadFile(
        @Path("dir") dir: String,
        @Path("fileName") fileName: String,
        @Body body: RequestBody
    ): String?

    @GET("/dav/Depression")
    fun getFilePath(): String

    @HTTP(method = "HEAD", path = "/dav/Depression/{dir}")
    suspend fun exists(@Path("dir", encoded = true) dir: String): Void?

    @HTTP(method = "MKCOL", path = "/dav/Depression/{dir}")
    suspend fun createDirectory(@Path("dir", encoded = true) dir: String): Void?

    @POST
    suspend fun prediction(@Url url: String, @Body body: MultipartBody): JsonObject
}
