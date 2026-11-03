#include "cloud_point_rpc/rpc_server.hpp"
#include "server_api.h"
#include "test_api.h"
#include <future>
#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

class TestCApi : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestRPC");
        EXPECT_NO_THROW(crpc_test_init());
    }

    void TearDown() override { crpc_test_deinit(); }
};

TEST_F(TestCApi, Base) {
    rpc_string name;
    name.s = "test";
    static std::promise<bool> task;
    std::future<bool> called = task.get_future();
    DLOG(INFO) << "Test";
    crpc_test_add_method(
        +[](rpc_string *) -> rpc_string * {
            static bool installed = false;
            DLOG(INFO) << "Trying to do something";
            if (!installed) {
                DLOG(INFO) << "Trying to install";
                installed = true;
                task.set_value(installed);
            }
            DLOG(INFO) << "Go out";
            return crpc_str_create("res", sizeof("res") - 1);
        },
        &name);

    using namespace std::chrono_literals;
    const std::future_status res = called.wait_for(500ms);
    EXPECT_NE(res, std::future_status::timeout);
    EXPECT_EQ(called.get(), true);

    DLOG(INFO) << "DONE";
}

TEST_F(TestCApi, AddedMultiple) {
    constexpr int N = 4;
    std::array<std::promise<bool>, N> tasks;
    std::array<std::pair<std::future<bool>, rpc_string>, N> called;

    // The Bridge: A static pointer local to this test function
    static std::array<std::promise<bool>, N> *bridge;
    bridge = &tasks;

    for (int i = 0; i < N; i++) {
        called[i].first  = tasks[i].get_future();
        std::string n    = "test" + std::to_string(i);
        called[i].second = rpc_string{n.c_str(), n.size()};
    }

    auto register_idx = [&]<size_t I>() {
        crpc_test_add_method(
            +[](rpc_string *) -> rpc_string * {
                static bool installed = false;
                if (!installed) {
                    installed = true;
                    (*bridge)[I].set_value(true);
                }
                return crpc_str_create("res", sizeof("res"));
            },
            &called[I].second);
    };

    register_idx.template operator()<0>();
    register_idx.template operator()<1>();
    register_idx.template operator()<2>();
    register_idx.template operator()<3>();

    auto test_idx = [&]<size_t I>() {
        using namespace std::chrono_literals;
        const std::future_status res = called[I].first.wait_for(1000ms);
        EXPECT_NE(res, std::future_status::timeout);
        EXPECT_EQ(called[I].first.get(), true);
        LOG(INFO) << "Done" << "; task=" << I;
    };

    test_idx.template operator()<0>();
    test_idx.template operator()<1>();
    test_idx.template operator()<2>();
    test_idx.template operator()<3>();
}

TEST_F(TestCApi, RemoveMethod) {
    rpc_string name{"test", sizeof("test") - 1};
    EXPECT_EQ(crpc_test_remove_method(&name), -1);
}

TEST_F(TestCApi, ChangeDuration) {
    constexpr int dur = 1e3;
    EXPECT_NO_THROW(crpc_test_change_duration(dur));
    EXPECT_EQ(crpc_test_duration(), dur);
    LOG(INFO) << "Test";
}

TEST_F(TestCApi, ScheduleCall) {
    EXPECT_NO_THROW(crpc_test_auto_call(0));
    constexpr int N = 4;
    std::array<std::promise<bool>, N> tasks;
    std::array<std::pair<std::future<bool>, rpc_string>, N> called;

    // The Bridge: A static pointer local to this test function
    static std::array<std::promise<bool>, N> *bridge;
    bridge = &tasks;
    LOG(INFO) << "Started Schedule Call";
    for (int i = 0; i < N; i++) {
        called[i].first  = tasks[i].get_future();
        std::string n    = "test" + std::to_string(i);
        called[i].second = rpc_string{n.c_str(), n.size()};
    }

    auto register_idx = [&]<size_t I>() {
        crpc_test_add_method(
            +[](rpc_string *) -> rpc_string * {
                static bool installed = false;
                if (!installed) {
                    installed = true;
                    (*bridge)[I].set_value(true);
                }
                std::string_view res = "res";
                return crpc_str_create(res.data(), res.size());
            },
            &called[I].second);
    };
    auto test_idx = [&]<size_t I>() {
        using namespace std::chrono_literals;
        const std::future_status res = called[I].first.wait_for(500ms);
        EXPECT_NE(res, std::future_status::timeout);
        EXPECT_EQ(called[I].first.get(), true);
    };

    auto schedule_call = []<size_t I>() {
        rpc_string test{"test", sizeof("test") - 1};
        test.s += std::to_string(I);
        LOG(INFO) << "Trying to add :" << test.s;
        crpc_test_schedule_call(&test);
    };

    register_idx.template operator()<0>();
    register_idx.template operator()<1>();
    register_idx.template operator()<2>();
    register_idx.template operator()<3>();
    schedule_call.template operator()<0>();
    schedule_call.template operator()<1>();
    schedule_call.template operator()<2>();
    schedule_call.template operator()<3>();
    test_idx.template operator()<0>();
    test_idx.template operator()<1>();
    test_idx.template operator()<2>();
    test_idx.template operator()<3>();
}

TEST(TestTestCApi, InitDeinitOnLongRunningTask) {
    using namespace std::chrono_literals;
    FLAGS_logtostderr = true;
    if (!google::IsGoogleLoggingInitialized())
        google::InitGoogleLogging("TestRPC");
    EXPECT_NO_THROW(crpc_test_init());
    static std::promise<bool> bridge;
    std::string_view name = "long running task";
    crpc_test_add_method(
        +[](rpc_string *) -> rpc_string * {
            static bool installed = false;
            if (!installed) {
                installed = true;
                bridge.set_value(true);
            }
            std::string_view res = "res";
            std::this_thread::sleep_for(3s);
            return crpc_str_create(res.data(), res.size());
        },
        crpc_str_create(name.data(), name.size()));
    std::this_thread::sleep_for(500ms);
    crpc_test_deinit();
}

TEST_F(TestCApi, String) {
    rpc_string name;
    name.s = "test";
    EXPECT_EQ(name.s.c_str(), crpc_str_get_data(&name));
    EXPECT_EQ(name.s.size(), crpc_str_get_size(&name));
    std::string_view testString = "test 2222";
    auto creation = crpc_str_create(testString.data(), testString.size());
    EXPECT_EQ(std::string_view(crpc_str_get_data(creation)), testString);
    EXPECT_NO_THROW(crpc_str_destroy(creation));
}
