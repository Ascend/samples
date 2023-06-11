/**
* @file op_test.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <iostream>
#include <fstream>
#include <string>

#include "op_test.h"
#include "acl/acl.h"
namespace OpTest {
using namespace std::chrono;

/**
 * @brief      set result of part testcase
 * @param [in] assert : testcase result of run
 * @return     None
 */
void SetPartResult(bool assert)
{
    UnitTest::GetInstance().SetTestResult(assert);
}

/**
 * @brief      set result of part testcase
 * @param [in] assert : testcase result of run
 * @return     None
 */
void RecordPartResult(OpTestDesc &opDesc, bool assert, std::string caseName)
{
    uint32_t testIndex = UnitTest::GetInstance().GetTestIndex();
    std::string resStr = std::to_string(testIndex) + "  " + caseName;
    resStr += assert ? "  [pass]" : "  [fail]";
    std::ofstream resultFile;
    resultFile.open("./result_files/result.txt", std::ios::out | std::ios::app);
    if (!resultFile.is_open()) {
        std::cout << "Fail to open result summary file." << std::endl;
    } else {
        resultFile << resStr << std::endl;
        std::cout << "Result file append successfully." << std::endl;
    }
}


/**
 * @brief      create TestInfo of tasecase
 * @param [in] caseName : testcase name
 * @param [in] test     : ada testcase to TestInfo
 * @return     None
 */
std::shared_ptr<TestInfo>  MakeAndRegisterTestInfo(std::string  caseName, std::unique_ptr<Test> test)
{
    if (test == nullptr) {
        return nullptr;
    }
    std::shared_ptr<TestInfo> testInfo = nullptr;
    try {
        testInfo = std::make_shared<TestInfo>(caseName, std::move(test));
    } catch (...) {
        return nullptr;
    }

    UnitTest::GetInstance().AddTestInfo(testInfo);
    return testInfo;
}

/**
 * @brief  get test instance
 * @return test instance
 */
UnitTest& UnitTest::GetInstance()
{
    static UnitTest instance;
    return instance;
}

/**
 * @brief      test process run
 * @return     0
 */
int UnitTest::Run()
{
    testIndex_ = 0;
    failedCount_ = 0;
    runCount_ = 0;
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "[=========]" << " Running " << testInfoVec_.size() << " tests." << std::endl;
    for (auto &x : testInfoVec_) {
        testIndex_++;
        if (x->IsSkip()) {
            continue;
        }
        runCount_++;
        x->Run();
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms = end - start;
    std::cout <<"[=========]" << " Ran " << testInfoVec_.size() << " tests. ( "
        << ms.count() << " ms total )" << std::endl;

    std::cout << "[PASSED] " << (runCount_ - failedCount_) << " tests."<<std::endl;
    std::cout << "[FAILED] " << failedCount_ << " tests." << std::endl;
    return 0;
}

/**
 * @brief  get running test index
 * @return testcase index
 */
uint32_t UnitTest::GetTestIndex()
{
    return testIndex_;
}

/**
 * @brief      get running test index
 * @param [in] assert : testcase running result
 * @return     None
 */
void UnitTest::SetTestResult(bool assert)
{
    if (assert == false) {
        testInfoVec_[testIndex_ - 1]->GetTestResult().AssertionFailed();
        failedCount_++;
    }
}

/**
 * @brief      add testinfo
 * @param [in] testInfo : testcase running result
 * @return     None
 */
void UnitTest::AddTestInfo(const std::shared_ptr<TestInfo> testInfo)
{
    if (testInfo != nullptr) {
        testInfoVec_.push_back(testInfo);
    }
}

/**
 * @brief      check case skip or not
 * @return     true : skip, false : not skip
 */
bool TestInfo::IsSkip()
{
    return false;
}

/**
 * @brief      get test result
 * @return     None
 */
TestResult& TestInfo::GetTestResult()
{
    return result_;
}

/**
 * @brief      testinfo process run
 * @return     None
 */
void TestInfo::Run()
{
    result_.RecordStartTimeStamp();
    std::cout << "[ Run     ] " << testSuiteName_ << std::endl;
    test_->TestBody();
    result_.RecordEndTimeStamp();
    if (result_.Passed()) {
        std::cout << "[      OK ] ";
    } else {
        std::cout << "[  FAILED ] ";
    }
    std::cout << testSuiteName_ << " ( " << result_.GetElapsedTime() << " ms )" << std::endl;
}
}
