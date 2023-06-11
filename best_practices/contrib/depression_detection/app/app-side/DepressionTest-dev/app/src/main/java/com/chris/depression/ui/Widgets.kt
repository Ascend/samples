/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import androidx.camera.core.CameraSelector
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.wrapContentHeight
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material.MaterialTheme
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.chris.depression.BuildConfig
import com.chris.depression.R
import com.chris.depression.utils.supportWideScreen
import timber.log.Timber

@Composable
fun TestListItem(
    modifier: Modifier = Modifier,
    lightTheme: Boolean = MaterialTheme.colors.isLight,
    stringId: Int,
    onClick: (() -> Unit)? = null
) {
    val bgColor = if (lightTheme) R.color.test_list_item_bg else R.color.light_blue_900
    Row(
        modifier = modifier
            .supportWideScreen()
            .padding(bottom = 1.dp)
            .background(color = colorResource(id = bgColor))
            .fillMaxWidth()
            .height(56.dp)
            .clickable { onClick?.invoke() }
            .wrapContentHeight(align = Alignment.CenterVertically),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = stringResource(id = stringId),
            Modifier
                .weight(1f, fill = true)
                .padding(start = 16.dp)
        )
        Image(
            painter = painterResource(id = R.drawable.ic_action_right),
            modifier = modifier.padding(end = 16.dp),
            contentDescription = null
        )
    }
}

@Preview
@Composable
private fun PreviewTestListItem() {
    LazyColumn(
        content = {
            items(questionList) { stringId ->
                TestListItem(stringId = stringId)
            }
        }
    )
}

val questionList = arrayListOf(
    R.string.test_list_phq9,
    R.string.test_list_ctq_sf,
    R.string.test_list_gad7,
    R.string.test_list_les,
    R.string.test_list_ssrs,
    R.string.test_list_psqi,
    if (BuildConfig.DEBUG)
        R.string.test_list_test
    else {
        -1
    }
)

@Composable
fun Camera() {
    val lifecycleOwner = LocalLifecycleOwner.current
    val context = LocalContext.current
    val cameraProviderFuture = remember { ProcessCameraProvider.getInstance(context) }
    val previewView = remember { PreviewView(context) }

    AndroidView(factory = { previewView }, modifier = Modifier.fillMaxSize()) {
        cameraProviderFuture.addListener(
            {
                val cameraProvider = cameraProviderFuture.get()
                val preview = androidx.camera.core.Preview.Builder()
                    .build()
                    .also {
                        it.setSurfaceProvider(previewView.surfaceProvider)
                    }
                val cameraSelector = CameraSelector.DEFAULT_FRONT_CAMERA
                try {
                    cameraProvider.unbindAll()
                    cameraProvider.bindToLifecycle(lifecycleOwner, cameraSelector, preview)
                } catch (e: Exception) {
                    Timber.e(e)
                }
            },
            ContextCompat.getMainExecutor(context)
        )
    }
}

@Preview
@Composable
private fun PreviewCamera() {
    Camera()
}
