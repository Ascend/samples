/**
* @file op_test.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#ifndef OP_TEST_H
#define OP_TEST_H
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <iostream>

#include "op_test_desc.h"
namespace OpTest{
#define OP_TEST_DISALLOW_COPY_AND_ASSIGN(type)      \
    type(type const&) = delete;                     \
    type& operator=(type const&) = delete

#define OP_TEST_DISALLOW_MOVE_AND_ASSIGN(type)      \
    type(type&&) noexcept = delete;                 \
    type& operator=(type&&) noexcept = delete

class Test {
public:
    virtual void TestBody() = 0;
};

class TestResult {
public:
    TestResult() : assert_(true) {}
    ~TestResult() {}
    bool Passed() const { return assert_; }
    void RecordStartTimeStamp()
    {
        startTm = std::chrono::high_resolution_clock::now();
    }
    void RecordEndTimeStamp()
    {
        endTm = std::chrono::high_resolution_clock::now();
    }
    double GetElapsedTime()
    {
       std::chrono::duration<double, std::milli> elapsedTm = endTm - startTm;
       return elapsedTm.count();
    }

    void AssertionFailed() {
        assert_ = false;
    }
private:
    bool assert_;
    std::chrono::high_resolution_clock::time_point startTm;
    std::chrono::high_resolution_clock::time_point endTm;
};

class TestInfo {
public:
    TestInfo(std::string testSuiteName, std::unique_ptr<Test> test)
        : testSuiteName_(testSuiteName), test_(std::move(test)) {}
    ~TestInfo() {}
    bool IsSkip();
    void Run();
    TestResult& GetTestResult();
private:
    std::string testSuiteName_;
    std::unique_ptr<Test> test_;
    TestResult result_;
};

class UnitTest {
public:
    UnitTest() : testIndex_(0), runCount_(0), failedCount_(0) {}
    ~UnitTest() {}
    static UnitTest& GetInstance();
    void AddTestInfo(const std::shared_ptr<TestInfo> testInfo);
    void SetTestResult(bool assert);
    uint32_t GetTestIndex();
    int Run();
private:
    uint32_t testIndex_;
    uint32_t runCount_;
    uint32_t failedCount_;
    std::vector<std::shared_ptr<TestInfo>> testInfoVec_;
};

class EqHelper {
public:
    template<typename T1, typename T2>
    static bool Compare(const T1& val1, const T2& val2) {
        if (val1 == val2) {
            return true;
        }
        return false;
    }
};

extern std::shared_ptr<TestInfo> MakeAndRegisterTestInfo(std::string  caseName, std::unique_ptr<Test> testInfo);
extern void SetPartResult(bool assert);
extern void RecordPartResult(OpTestDesc &opDesc, bool assert, std::string caseName);

#define OP_TEST_CLASS_NAME(TestClass, CaseName)   \
    Op##TestClass##CaseName##Test


#define OP_TEST(TestClass, CaseName)                                                         \
    class OP_TEST_CLASS_NAME(TestClass, CaseName): public Test {                             \
    public:                                                                                  \
        OP_TEST_CLASS_NAME(TestClass, CaseName)() {}                                         \
        ~OP_TEST_CLASS_NAME(TestClass, CaseName)() {}                                        \
        OP_TEST_DISALLOW_COPY_AND_ASSIGN(OP_TEST_CLASS_NAME(TestClass,                       \
            CaseName));                                                                      \
        OP_TEST_DISALLOW_MOVE_AND_ASSIGN(OP_TEST_CLASS_NAME(TestClass,                       \
            CaseName));                                                                      \
    void TestBody() override;                                                                \
    static std::shared_ptr<TestInfo> const testInfo_;                                        \
    };                                                                                       \
    std::shared_ptr<TestInfo> const OP_TEST_CLASS_NAME(TestClass, CaseName)::testInfo_ =     \
        MakeAndRegisterTestInfo(#TestClass"."#CaseName,                                      \
        std::unique_ptr<Test>(new(std::nothrow) OP_TEST_CLASS_NAME(TestClass, CaseName)));   \
    void OP_TEST_CLASS_NAME(TestClass, CaseName)::TestBody()


#define EXPECT_EQ(val1, val2)                                            \
    if (EqHelper::Compare(val1, val2)) {                                 \
        SetPartResult(true);                                             \
    } else {                                                             \
        SetPartResult(false);                                            \
    }

#define EXPECT_EQ_AND_RECORD(val1, val2, desc, caseName)                 \
    if (EqHelper::Compare(val1, val2)) {                                 \
        SetPartResult(true);                                             \
        RecordPartResult(desc, true, caseName);                          \
    } else {                                                             \
        SetPartResult(false);                                            \
        RecordPartResult(desc, false, caseName);                         \
    }
}
#endif // OP_TEST_H
