#include <chrono>
#include <filesystem>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <unistd.h>

#include "cloud_point_rpc/cli.hpp"
#include "cloud_point_rpc/rpc_server.hpp"
#include "cloud_point_rpc/tcp_server.hpp"

using namespace score;

namespace {
/// Number of times the menu prompt was printed.
std::size_t count_menus(const std::string &text) {
    std::size_t n = 0;
    for (auto pos = text.find("Select an option:"); pos != std::string::npos;
         pos      = text.find("Select an option:", pos + 1))
        ++n;
    return n;
}
} // namespace

class CliTest : public ::testing::Test {
  public:
    void start() { tcp_server->start(); }

  protected:
    void SetUp() override {
        FLAGS_logtostderr = true;
        if (!google::IsGoogleLoggingInitialized())
            google::InitGoogleLogging("TestCli");
        server_ip   = "127.0.0.1";
        server_port = 9096;

        rpc_server  = std::make_unique<RpcServer>();

        tcp_server  = std::make_unique<TcpServer>(
            server_ip, server_port, [this](const std::string &req) {
                return rpc_server->process(req);
            });

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void TearDown() override { tcp_server->stop(); }

    std::string server_ip;
    int server_port;
    std::unique_ptr<RpcServer> rpc_server;
    std::unique_ptr<TcpServer> tcp_server;
};

TEST_F(CliTest, SendsInputToServerAndReceivesResponse) {
    std::stringstream input;
    std::stringstream output;

    // Select option 1 (get-intrinsic-params) then 0 (exit)
    // First we need to make sure the rpc_server has the method registered.
    // Our SetUp registers "hello", let's register "get-intrinsic-params" too.
    rpc_server->register_method(
        "get-intrinsic-params", [](const nlohmann::json &) {
            return std::vector<double>{14589.0, 22489.0, 3123124.555};
        });
    this->start();
    input << "1" << std::endl;
    input << "0" << std::endl;

    int result = run_cli(input, output, server_ip, server_port);

    EXPECT_EQ(result, 0);
    std::string response = output.str();
    // Use more flexible check because of pretty printing
    EXPECT_THAT(response, ::testing::HasSubstr("14589.0"));
    EXPECT_THAT(response, ::testing::HasSubstr("22489.0"));
    EXPECT_THAT(response, ::testing::HasSubstr("3123124.555"));
}

// Option 6 against a server without stereo methods: the reconstruction fails
// (or, without OpenCV, the option is reported as not built).  Either way no
// site is written and the menu comes back.
TEST_F(CliTest, ViewerOptionWithoutStereoServerWritesNothing) {
    rpc_server->register_method(
        "get-intrinsic-params",
        [](const nlohmann::json &) { return std::vector<double>{1.0}; });
    this->start();

    const auto out_dir = std::filesystem::temp_directory_path() /
                         ("cprpc_cli_viewer_" + std::to_string(::getpid()));
    std::filesystem::remove_all(out_dir);

    CliStereoOptions stereo;
    stereo.use_gpu             = false;
    stereo.viewer.output_dir   = out_dir.string();
    stereo.viewer.open_command = "true";

    std::stringstream input("6\n0\n");
    std::stringstream output;
    const int rc = run_cli(input, output, server_ip, server_port, stereo);
    EXPECT_EQ(rc, 0);

    const std::string text = output.str();
    EXPECT_THAT(text, ::testing::AnyOf(
                          ::testing::HasSubstr("Error"),
                          ::testing::HasSubstr("OpenCV support not built")))
        << text;
    EXPECT_THAT(text, ::testing::Not(::testing::HasSubstr("Viewer:")));
    EXPECT_FALSE(std::filesystem::exists(out_dir));
    // The menu is printed once at start and again after the failed option.
    EXPECT_GE(count_menus(text), 2u);
    std::filesystem::remove_all(out_dir);
}
