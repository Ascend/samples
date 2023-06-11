/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material.Button
import androidx.compose.material.ButtonDefaults
import androidx.compose.material.Scaffold
import androidx.compose.material.Surface
import androidx.compose.material.Text
import androidx.compose.material.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.ComposeView
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.fragment.findNavController
import com.chris.depression.BuildConfig
import com.chris.depression.R
import com.chris.depression.domain.AnswerRepository
import com.chris.depression.utils.checkPermission
import com.chris.depression.utils.supportWideScreen
import javax.inject.Inject

class StartFragment : BaseFragment() {

    @Inject
    lateinit var repository: AnswerRepository

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        return ComposeView(requireContext()).apply {
            setContent { StartPage() }
        }
    }

    @Composable
    fun StartPage() {
        Surface {
            Scaffold(
                topBar = {
                    TopAppBar(
                        title = { Text(stringResource(R.string.app_name)) },
                        backgroundColor = colorResource(R.color.blue_200),
                        contentColor = Color.White
                    )
                },
                content = {
                    Column(
                        modifier = Modifier
                            .supportWideScreen()
                            .padding(16.dp)
                            .fillMaxHeight(1f),
                        verticalArrangement = Arrangement.SpaceBetween,
                        horizontalAlignment = Alignment.CenterHorizontally
                    ) {
                        Text(text = stringResource(id = R.string.introduction), fontSize = 16.sp)
                        Button(
                            onClick = {
                                findNavController().navigate(
                                    StartFragmentDirections.actionStartFragmentToVideoViewerFragment()
                                )
                            },
                            colors = ButtonDefaults.buttonColors(
                                colorResource(R.color.blue_200),
                                Color.White
                            )
                        ) {
                            Text(text = "问题", fontSize = 28.sp)
                        }
                        Button(
                            onClick = { findNavController().navigate(StartFragmentDirections.actionStartFragmentToQuestionList()) },
                            colors = ButtonDefaults.buttonColors(
                                colorResource(R.color.blue_200),
                                Color.White
                            )
                        ) {
                            Text(text = "问卷", fontSize = 28.sp)
                        }
                        Button(
                            onClick = { findNavController().navigate(StartFragmentDirections.actionStartFragmentToVisualActivity()) },
                            colors = ButtonDefaults.buttonColors(
                                colorResource(R.color.blue_200),
                                Color.White
                            )
                        ) {
                            Text(text = "眼动追踪", fontSize = 28.sp)
                        }
                        Spacer(modifier = Modifier.height(1.dp))
                        Text(
                            text = "test network",
                            modifier = Modifier.clickable {
                                repository.fetchFile()
                            },
                            fontSize = 32.sp,
                            color = if (BuildConfig.DEBUG) Color.Gray else Color.White
                        )
                    }
                }
            )
        }
    }

    @Preview
    @Composable
    fun PreviewStart() {
        StartPage()
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        checkPermission()
    }
}
