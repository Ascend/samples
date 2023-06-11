/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import android.content.Intent
import android.net.Uri
import android.os.Bundle
import android.provider.Settings
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.compose.runtime.livedata.observeAsState
import androidx.compose.ui.platform.ComposeView
import androidx.fragment.app.viewModels
import androidx.lifecycle.ViewModelProvider
import androidx.navigation.fragment.findNavController
import androidx.navigation.navGraphViewModels
import com.chris.depression.R
import com.chris.depression.theme.JetsurveyTheme
import com.chris.depression.ui.survey.SurveyActionType
import com.chris.depression.ui.survey.SurveyQuestionsScreen
import com.chris.depression.ui.survey.SurveyResultScreen
import com.chris.depression.ui.survey.SurveyStartScreen
import com.chris.depression.ui.survey.SurveyState
import javax.inject.Inject

class QuestionFragment : BaseFragment() {

    @Inject
    lateinit var viewModelFactory: ViewModelProvider.Factory

    val viewModel: QuestionViewModel by viewModels { viewModelFactory }
    private val questionCachedViewModel: QuestionCachedViewModel by navGraphViewModels(R.id.nav_main) { viewModelFactory }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        return ComposeView(requireContext()).apply {
            // In order for savedState to work, the same ID needs to be used for all instances.
            // id = R.id.sign_in_fragment
            viewModel.fetchQuestion(questionCachedViewModel.currentTest)

            viewModel.commit.observe(viewLifecycleOwner) {
                if (it) {
                    findNavController().navigateUp()
                }
            }

            layoutParams = ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT
            )
            setContent {
                JetsurveyTheme {
                    viewModel.uiState.observeAsState().value?.let { surveyState ->
                        when (surveyState) {
                            is SurveyState.Start -> SurveyStartScreen(
                                start = surveyState,
                                onStartPressed = { viewModel.startQuestion() }
                            )
                            is SurveyState.Questions -> SurveyQuestionsScreen(
                                questions = surveyState,
                                onAction = { id, action -> handleSurveyAction(id, action) },
                                onDonePressed = { viewModel.computeResult(surveyState) },
                                onBackPressed = {
                                    activity?.onBackPressedDispatcher?.onBackPressed()
                                },
                                openSettings = {
                                    activity?.startActivity(
                                        Intent(
                                            Settings.ACTION_APPLICATION_DETAILS_SETTINGS,
                                            Uri.fromParts("package", context.packageName, null)
                                        )
                                    )
                                }
                            )
                            is SurveyState.Result -> SurveyResultScreen(
                                result = surveyState,
                                onDonePressed = {
                                    viewModel.commitAnswer()
                                }
                            )
                        }
                    }
                }
            }
        }
    }

    private fun handleSurveyAction(questionId: Int, actionType: SurveyActionType) {
        when (actionType) {
            SurveyActionType.PICK_DATE -> showDatePicker(questionId)
            SurveyActionType.TAKE_PHOTO -> takeAPhoto()
            SurveyActionType.SELECT_CONTACT -> selectContact(questionId)
        }
    }

    private fun showDatePicker(questionId: Int) {
//        val date = viewModel.getCurrentDate(questionId = questionId)
//        val picker = MaterialDatePicker.Builder.datePicker()
//            .setSelection(date)
//            .build()
//        activity?.let {
//            picker.show(it.supportFragmentManager, picker.toString())
//            picker.addOnPositiveButtonClickListener {
//                viewModel.onDatePicked(questionId, picker.selection)
//            }
//        }
    }

    private fun takeAPhoto() {
//        takePicture.launch(viewModel.getUriToSaveImage())
    }

    private fun selectContact(questionId: Int) {
        // TODO: unsupported for now
    }
}
