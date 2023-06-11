/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.utils

import android.content.Context
import com.google.gson.Gson
import okio.BufferedSource
import okio.buffer
import okio.source
import java.lang.reflect.Type
import java.nio.charset.StandardCharsets

object AssetReaderUtils {

    fun <T> readJson(context: Context, filename: String, typeOfT: Type): T {
        val resultJson = readStringFromAssets(context, filename)
        return toObject(resultJson, typeOfT)
    }

    private fun readStringFromAssets(context: Context, filename: String): String {
        val inputStream = context.resources.assets.open(filename)
        val source: BufferedSource = inputStream.source().buffer()
        val result = source.readString(StandardCharsets.UTF_8)
        source.close()
        return result
    }

    fun <T> toObject(json: String, typeOfT: Type): T {
        return Gson().fromJson(json, typeOfT)
    }
}
