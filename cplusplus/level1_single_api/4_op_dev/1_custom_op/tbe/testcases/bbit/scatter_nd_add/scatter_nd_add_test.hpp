#include "tvm_bbit.hpp"
#include "register.hpp"
#include <functional>
#include <map>
#include <string>
using namespace std;
class ScatterNdAddTest : public BaseBbitTest{
  public:
    ScatterNdAddTest(){
        for (const auto& item : test_calls_) {
            testcases.push_back(item.first);
        }
        testcases.push_back("scatter_nd_add_test_2_2_2_float32");
        testcases.push_back("scatter_nd_add_test_33_5_5_float16");
    };

    virtual ~ScatterNdAddTest(){

    };

    bool test(string name);

    bool scatter_nd_add_test_2_2_2_float32();
    bool scatter_nd_add_test_33_5_5_float16();

  private:
    std::map < std::string, std::function< bool(void)>> test_calls_ = {
    { "scatter_nd_add_test_2_2_2_float32", std::bind(& ScatterNdAddTest::scatter_nd_add_test_2_2_2_float32, this)},
    { "scatter_nd_add_test_33_5_5_float16", std::bind(& ScatterNdAddTest::scatter_nd_add_test_33_5_5_float16, this)},
    };

    const std::string case_ = "scatter_nd_add";
};

REGISTER_CLASS(ScatterNdAddTest)
