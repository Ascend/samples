/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.ui

import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.chris.depression.data.exception.ExceptionHandler
import com.chris.depression.data.exception.ExceptionHandlerImpl
import com.chris.depression.data.exception.ResponseException
import io.reactivex.Observable
import io.reactivex.Single
import io.reactivex.disposables.CompositeDisposable
import io.reactivex.schedulers.Schedulers

abstract class BaseViewModel : ViewModel() {

    private val exceptionHandler: ExceptionHandler by lazy {
        ExceptionHandlerImpl()
    }

    protected val compositeDisposable: CompositeDisposable = CompositeDisposable()

    val loading: MutableLiveData<Boolean> by lazy {
        MutableLiveData(false)
    }

    val commonError: MutableLiveData<ResponseException> by lazy {
        MutableLiveData(ResponseException.noneException())
    }

    protected fun <T> Single<T>.applyIoSchedules(): Single<T> {
        return subscribeOn(Schedulers.io())
            .doOnError { doOnException(exceptionHandler.handleException(it)) }
            .doOnSubscribe { setLoading(true) }
            .doFinally { setLoading(false) }
    }

    protected fun <T> Observable<T>.applyIoSchedules(): Observable<T> {
        return subscribeOn(Schedulers.io())
            .doOnError { doOnException(exceptionHandler.handleException(it)) }
            .doOnSubscribe { setLoading(true) }
            .doFinally { setLoading(false) }
    }

    private fun doOnException(resultException: ResponseException) {
        handleCommonException(resultException)
        handleException(resultException)
    }

    open fun handleCommonException(resultException: ResponseException) {
        commonError.postValue(resultException)
    }

    open fun handleException(resultException: ResponseException) {
        // overrider this method to handle error on you business
    }

    protected fun setLoading(isLoading: Boolean) {
        loading.postValue(isLoading)
    }

    override fun onCleared() {
        setLoading(false)
        compositeDisposable.clear()
        super.onCleared()
    }
}
