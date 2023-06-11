/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.utils

import android.Manifest
import android.content.pm.PackageManager
import android.database.Cursor
import android.net.Uri
import android.os.Build
import android.provider.MediaStore
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.video.VideoRecordEvent
import androidx.core.content.ContextCompat
import androidx.fragment.app.Fragment

private var PERMISSIONS_REQUIRED = arrayOf(
    Manifest.permission.CAMERA,
    Manifest.permission.READ_EXTERNAL_STORAGE,
    Manifest.permission.RECORD_AUDIO
)

fun Fragment.getPackageName(): String = this.requireContext().packageName

fun Fragment.getPath(uri: Uri): String {
    val projection = arrayOf(MediaStore.Images.Media.DATA)
    val cursor: Cursor =
        requireContext().contentResolver.query(uri, projection, null, null, null)
            ?: return ""
    val columnIndex = cursor.getColumnIndexOrThrow(MediaStore.Images.Media.DATA)
    cursor.moveToFirst()
    val s = cursor.getString(columnIndex)
    cursor.close()
    return s
}

fun Fragment.checkPermission() {
    // add the storage access permission request for Android 9 and below.
    if (Build.VERSION.SDK_INT <= Build.VERSION_CODES.P) {
        val permissionList = PERMISSIONS_REQUIRED.toMutableList()
        permissionList.add(Manifest.permission.WRITE_EXTERNAL_STORAGE)
        PERMISSIONS_REQUIRED = permissionList.toTypedArray()
    }

    if (!hasPermissions()) {
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            // Handle Permission granted/rejected
            var permissionGranted = true
            permissions.entries.forEach {
                if (it.key in PERMISSIONS_REQUIRED && !it.value)
                    permissionGranted = false
            }
            if (!permissionGranted) {
                Toast.makeText(context, "Permission request denied", Toast.LENGTH_LONG).show()
            }
        }.launch(PERMISSIONS_REQUIRED)
    }
}

fun Fragment.hasPermissions() = PERMISSIONS_REQUIRED.all {
    ContextCompat.checkSelfPermission(requireContext(), it) == PackageManager.PERMISSION_GRANTED
}

/**
 * A helper extended function to get the name(string) for the VideoRecordEvent.
 */
fun VideoRecordEvent.getName(): String {
    return when (this) {
        is VideoRecordEvent.Status -> "Status"
        is VideoRecordEvent.Start -> "Started"
        is VideoRecordEvent.Finalize -> "Finalized"
        is VideoRecordEvent.Pause -> "Paused"
        is VideoRecordEvent.Resume -> "Resumed"
        else -> "Error(Unknown)"
    }
}
