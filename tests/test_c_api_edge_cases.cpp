#include "cloud_point_rpc/rpc_server.hpp"
#include "server_api.h"
#include "test_api.h"
#include <fstream>
#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

class CApiEdgeCaseTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestRPC");
    }
};

// Null pointer tests for string functions
TEST_F(CApiEdgeCaseTest, StrGetDataNullptr) {
    EXPECT_EQ(crpc_str_get_data(nullptr), nullptr);
}

TEST_F(CApiEdgeCaseTest, StrGetSizeNullptr) {
    EXPECT_EQ(crpc_str_get_size(nullptr), 0);
}

TEST_F(CApiEdgeCaseTest, StrCreateNullptrData) {
    EXPECT_EQ(crpc_str_create(nullptr, 10), nullptr);
}

TEST_F(CApiEdgeCaseTest, StrCreateEmptyString) {
    auto str = crpc_str_create("", 0);
    EXPECT_NE(str, nullptr);
    EXPECT_EQ(crpc_str_get_size(str), 0);
    EXPECT_EQ(std::string_view(crpc_str_get_data(str)), "");
    crpc_str_destroy(str);
}

TEST_F(CApiEdgeCaseTest, StrDestroyNullptr) {
    // Should not crash
    EXPECT_NO_THROW(crpc_str_destroy(nullptr));
}

// Double destroy should be safe-ish (will just not find it)
TEST_F(CApiEdgeCaseTest, StrDoubleDestroy) {
    auto str = crpc_str_create("test", 4);
    ASSERT_NE(str, nullptr);
    crpc_str_destroy(str);
    // Second destroy should not crash (pointer not in gc anymore)
    EXPECT_NO_THROW(crpc_str_destroy(str));
}

// Create and destroy many strings
TEST_F(CApiEdgeCaseTest, StrCreateDestroyMany) {
    constexpr int N = 1000;
    std::vector<rpc_string *> ptrs;
    ptrs.reserve(N);
    for (int i = 0; i < N; ++i) {
        auto str = crpc_str_create("x", 1);
        ASSERT_NE(str, nullptr);
        ptrs.push_back(str);
    }
    // Destroy half
    for (int i = 0; i < N / 2; ++i) {
        crpc_str_destroy(ptrs[i]);
    }
    // Create more
    for (int i = 0; i < N / 2; ++i) {
        auto str = crpc_str_create("y", 1);
        ASSERT_NE(str, nullptr);
    }
    // Destroy remaining original
    for (int i = N / 2; i < N; ++i) {
        crpc_str_destroy(ptrs[i]);
    }
}

// Null pointer tests for add_method
TEST_F(CApiEdgeCaseTest, AddMethodNullName) {
    auto cb =
        +[](rpc_string *) -> rpc_string * { return crpc_str_create("res", 3); };
    // Should not crash, just log and return
    EXPECT_NO_THROW(crpc_add_method(cb, nullptr));
}

TEST_F(CApiEdgeCaseTest, AddMethodNullCallback) {
    rpc_string name{"test", 4};
    EXPECT_NO_THROW(crpc_add_method(nullptr, &name));
}

TEST_F(CApiEdgeCaseTest, AddMethodBothNull) {
    EXPECT_NO_THROW(crpc_add_method(nullptr, nullptr));
}

// crpc_init edge cases
TEST_F(CApiEdgeCaseTest, InitWithNullptr) {
    // Should not crash, just log error and return
    EXPECT_NO_THROW(crpc_init(nullptr));
}

TEST_F(CApiEdgeCaseTest, InitWithInvalidPath) {
    // Should catch exception and log, not crash
    EXPECT_NO_THROW(crpc_init("/nonexistent/path/config.yaml"));
}

// Full lifecycle: init -> add method -> deinit
TEST_F(CApiEdgeCaseTest, FullLifecycle) {
    std::ofstream config_file("test_config.yaml");
    config_file << "server:\n"
                << "  ip: \"127.0.0.1\"\n"
                << "  port: 19191\n";
    config_file.close();

    EXPECT_NO_THROW(crpc_init("test_config.yaml"));

    rpc_string name{"echo", 4};
    auto cb = +[](rpc_string *req) -> rpc_string * {
        return crpc_str_create(req->s.data(), req->s.size());
    };
    EXPECT_NO_THROW(crpc_add_method(cb, &name));

    EXPECT_NO_THROW(crpc_deinit());
    std::remove("test_config.yaml");
}

// Deinit without init should not crash
TEST_F(CApiEdgeCaseTest, DeinitWithoutInit) { EXPECT_NO_THROW(crpc_deinit()); }

// Multiple init/deinit cycles
TEST_F(CApiEdgeCaseTest, MultipleInitDeinitCycles) {
    std::ofstream config_file("test_config.yaml");
    config_file << "server:\n"
                << "  ip: \"127.0.0.1\"\n"
                << "  port: 19192\n";
    config_file.close();

    for (int i = 0; i < 3; ++i) {
        EXPECT_NO_THROW(crpc_init("test_config.yaml"));
        EXPECT_NO_THROW(crpc_deinit());
    }
    std::remove("test_config.yaml");
}

// GC cleanup on deinit
TEST_F(CApiEdgeCaseTest, GcCleanupOnDeinit) {
    auto str1 = crpc_str_create("one", 3);
    auto str2 = crpc_str_create("two", 3);
    ASSERT_NE(str1, nullptr);
    ASSERT_NE(str2, nullptr);

    // Destroy one, leave one
    crpc_str_destroy(str1);

    // deinit should clear gc including str2
    EXPECT_NO_THROW(crpc_deinit());
}

// Large string creation
TEST_F(CApiEdgeCaseTest, LargeStringCreate) {
    std::string large(1000000, 'x');
    auto str = crpc_str_create(large.data(), large.size());
    ASSERT_NE(str, nullptr);
    EXPECT_EQ(crpc_str_get_size(str), large.size());
    EXPECT_EQ(std::string(crpc_str_get_data(str), large.size()), large);
    crpc_str_destroy(str);
}

// String with embedded null bytes
TEST_F(CApiEdgeCaseTest, StringWithNullBytes) {
    std::string data("Hello\0World", 11);
    auto str = crpc_str_create(data.data(), data.size());
    ASSERT_NE(str, nullptr);
    EXPECT_EQ(crpc_str_get_size(str), 11);
    EXPECT_EQ(std::string(crpc_str_get_data(str), 11), data);
    crpc_str_destroy(str);
}

// Test API edge cases
TEST_F(CApiEdgeCaseTest, TestInitDeinit) {
    EXPECT_NO_THROW(crpc_test_init());
    EXPECT_NO_THROW(crpc_test_deinit());
}

TEST_F(CApiEdgeCaseTest, TestRemoveNonexistentMethod) {
    crpc_test_init();
    rpc_string name{"nonexistent", 11};
    EXPECT_EQ(crpc_test_remove_method(&name), -1);
    crpc_test_deinit();
}

TEST_F(CApiEdgeCaseTest, TestAutoCallToggle) {
    crpc_test_init();
    EXPECT_NO_THROW(crpc_test_auto_call(0));
    EXPECT_NO_THROW(crpc_test_auto_call(1));
    EXPECT_NO_THROW(crpc_test_auto_call(0));
    crpc_test_deinit();
}

TEST_F(CApiEdgeCaseTest, TestChangeDuration) {
    crpc_test_init();
    EXPECT_NO_THROW(crpc_test_change_duration(100));
    EXPECT_EQ(crpc_test_duration(), 100);
    EXPECT_NO_THROW(crpc_test_change_duration(500));
    EXPECT_EQ(crpc_test_duration(), 500);
    crpc_test_deinit();
}

TEST_F(CApiEdgeCaseTest, TestScheduleCallNonexistent) {
    crpc_test_init();
    rpc_string name{"nonexistent", 11};
    EXPECT_NO_THROW(crpc_test_schedule_call(&name));
    crpc_test_deinit();
}
