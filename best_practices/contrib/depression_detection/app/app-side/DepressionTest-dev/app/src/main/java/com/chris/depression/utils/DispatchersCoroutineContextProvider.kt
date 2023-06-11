/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.utils

import com.chris.depression.domain.CoroutineContextProvider
import kotlinx.coroutines.Dispatchers

class DispatchersCoroutineContextProvider : CoroutineContextProvider {
    override val main = Dispatchers.Main
    override val io = Dispatchers.IO
}
