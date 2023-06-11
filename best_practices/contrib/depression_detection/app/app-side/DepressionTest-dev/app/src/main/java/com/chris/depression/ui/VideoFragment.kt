/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.compose.material.MaterialTheme
import androidx.compose.material.Surface
import androidx.compose.ui.platform.ComposeView
import androidx.fragment.app.Fragment

class VideoFragment : Fragment() {

    companion object {
        fun newInstance() = VideoFragment()
    }

//    private lateinit var viewModel: VideoViewModel

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        return ComposeView(requireContext()).apply {
            setContent {
                MaterialTheme {
                    Surface {
                        Camera()
                    }
                }
            }
        }
    }
}
