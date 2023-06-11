#!/bin/sh
set -e
./gradlew --stop
./gradlew spotlessApply
./gradlew clean connectedAndroidTest jacocoTestReport
./gradlew jacocoTestCoverageVerification
