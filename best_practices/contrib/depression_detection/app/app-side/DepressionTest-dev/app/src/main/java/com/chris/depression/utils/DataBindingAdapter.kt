/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.utils

import android.view.View
import androidx.databinding.BindingAdapter
import androidx.databinding.BindingMethod
import androidx.databinding.BindingMethods

@BindingMethods(BindingMethod(type = View::class, attribute = "isGone", method = "bindIsGone"))
object DataBindingAdapter {

    @JvmStatic
    @BindingAdapter("isGone")
    fun bindIsGone(view: View, isGone: Boolean) {
        if (isGone) {
            view.visibility = View.GONE
        } else {
            view.visibility = View.VISIBLE
        }
    }
}
