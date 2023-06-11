/*
 * Copyright 2022 Leon.
 */
package com.chris.depression

import com.chris.depression.data.AnswerRepositoryImpl
import com.chris.depression.data.LocalQuestionRepository
import com.chris.depression.domain.AnswerRepository
import com.chris.depression.domain.QuestionRepository
import dagger.Module
import dagger.Provides
import javax.inject.Singleton

@Module
class RepositoryModule {

    @Provides
    @Singleton
    fun provideQuestionRepository(localQuestionRepository: LocalQuestionRepository): QuestionRepository {
        return localQuestionRepository
    }

    @Provides
    @Singleton
    fun provideAnswerRepository(answerRepositoryImpl: AnswerRepositoryImpl): AnswerRepository {
        return answerRepositoryImpl
    }
}
