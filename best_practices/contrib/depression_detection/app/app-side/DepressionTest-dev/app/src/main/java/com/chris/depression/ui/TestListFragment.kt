/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material.FloatingActionButton
import androidx.compose.material.Icon
import androidx.compose.material.MaterialTheme
import androidx.compose.material.Scaffold
import androidx.compose.material.Surface
import androidx.compose.material.Text
import androidx.compose.material.TopAppBar
import androidx.compose.ui.platform.ComposeView
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.fragment.app.viewModels
import androidx.lifecycle.ViewModelProvider
import androidx.navigation.fragment.findNavController
import androidx.navigation.navGraphViewModels
import com.chris.depression.R
import javax.inject.Inject

class TestListFragment : BaseFragment() {

    @Inject
    lateinit var viewModelFactory: ViewModelProvider.Factory

    val viewModel: TestListViewModel by viewModels { viewModelFactory }

    private val questionCachedViewModel: QuestionCachedViewModel by navGraphViewModels(R.id.nav_main) { viewModelFactory }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        return ComposeView(requireContext()).apply {
            setContent {
                MaterialTheme {
                    Surface {
                        Scaffold(
                            topBar = {
                                TopAppBar(title = { Text(stringResource(id = R.string.app_name)) })
                            },
                            floatingActionButton = {
                                FloatingActionButton(
                                    onClick = {
                                        findNavController().navigate(TestListFragmentDirections.actionQuestionListToVideoFragment())
                                    }
                                ) {
                                    Icon(
                                        painter = painterResource(id = R.drawable.ic_action_test),
                                        contentDescription = ""
                                    )
                                }
                            },
                            content = {
                                LazyColumn {
                                    items(viewModel.questionList) { item ->
                                        TestListItem(stringId = item.stringId) {
                                            viewModel.fetchQuestion(context, item)
                                        }
                                    }
                                }
                            }
                        )
                    }
                }
            }
        }
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        viewModel.currentTest.observe(viewLifecycleOwner) {
            questionCachedViewModel.cacheQuestion(it)
            findNavController().navigate(TestListFragmentDirections.actionQuestionListToQuestionFragment())
        }
    }
}
