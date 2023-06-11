/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.utils

import androidx.navigation.NavController
import com.chris.depression.NavMainDirections

fun NavController.navigateToStartFragment() {
    navigate(NavMainDirections.gotoStart())
}
