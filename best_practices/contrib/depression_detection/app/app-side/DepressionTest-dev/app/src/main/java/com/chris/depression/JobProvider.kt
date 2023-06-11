/*
 * Copyright 2022 Leon.
 */
package com.chris.depression

import kotlinx.coroutines.Job

interface JobProvider : () -> Job
