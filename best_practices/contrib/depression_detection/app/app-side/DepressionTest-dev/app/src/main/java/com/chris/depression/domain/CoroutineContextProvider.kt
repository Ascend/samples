/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain

import kotlin.coroutines.CoroutineContext

interface CoroutineContextProvider {
    val main: CoroutineContext
    val io: CoroutineContext
}
