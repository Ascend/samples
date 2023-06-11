/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.domain.model.dto

data class ErrorInfoDTO(
    val code: String,
    val causes: List<String>,
    val correlationId: String
)

data class ErrorResponseDTO(
    val errorInfo: List<ErrorInfoDTO>
)

enum class ErrorCode(val value: String) {
    CardNotFound("CC106"),
    AddCardInvalidData("CC004"),
    AddCardAlreadyExist("CC008"),
    AddCardVerifyFailed("CC013");

    companion object {
        fun fromString(value: String) = values().firstOrNull { it.value == value }
    }
}
