#include "test_api.h"
#include "cloud_point_rpc/rpc_server.hpp"
#include "server_api.h"

#include <condition_variable>
#include <glog/logging.h>
#include <mutex>
#include <queue>
#include <set>
#include <stop_token>
#include <thread>

class TestThread {
    static std::string make_jsonrpc(const std::string &method_name) {
        return std::format(R"(
            {{
            "jsonrpc": "2.0",
            "method": "{}",
            "params": {{}},
            "id": 1
            }}
        )",
                           method_name);
    }

  public:
    TestThread() = default;
    ~TestThread() { join(); }

    void routine() {
        LOG(INFO) << "Started routine";
        size_t distance{0};
        std::unique_lock lock(mtx);
        const std::stop_token stoken = thr.get_stop_token();
        lock.unlock();
        while (!stoken.stop_requested()) {
            lock.lock();
            if (!calls_queue.empty()) {
                auto front = calls_queue.front();
                DLOG(INFO) << front << " will try to call";
                calls_queue.pop();
                LOG(INFO) << server.process(make_jsonrpc(front));
            } else if (state.load() && !methods.empty()) {
                auto it = methods.begin();
                std::advance(it, distance++ % methods.size());
                DLOG(INFO) << *it << " : Method at this position";
                LOG(INFO) << server.process(make_jsonrpc(*it));
            }

            if (state.load() && calls_queue.empty())
                cv.wait_for(lock, thr_sleep,
                            [&] { return stoken.stop_requested(); });

            lock.unlock();
        }
        LOG(INFO) << "Stopped";
    }

    void start() { thr = std::jthread(&TestThread::routine, this); }
    void join() {
        if (thr.joinable()) {
            DLOG(INFO) << "Requested thread stop";
            thr.request_stop();
            cv.notify_one();
            thr.join();
        }
    }
    void add_method(const callback_t cb, rpc_string *name) {
        if (!name || !name->s.size()) {
            LOG(ERROR) << "Tried to add method with invalid name";
            return;
        }
        LOG(INFO) << "Trying to add method: " << name->s;
        std::lock_guard lock(mtx);
        if (methods.contains(name->s)) {
            LOG(INFO) << "Method already exists: " << name->s;
            return;
        }
        methods.emplace(name->s);
        server.register_method(name->s, cb);
    }

    int remove_method(const rpc_string *name) {
        if (!name || !name->s.size()) {
            LOG(ERROR) << "Tried to remove method with invalid name";
            return -1;
        }
        LOG(INFO) << "Trying to remove method: " << name->s;
        std::lock_guard lock(mtx);
        int result = 0;
        auto it    = std::find(methods.begin(), methods.end(), name->s);
        if (it != methods.end()) {
            methods.erase(it);
        } else {
            LOG(ERROR) << "Method not found: " << name->s;
            result = -1;
        }
        return result;
    }

    void call(const rpc_string *name) {
        if (!name) {
            LOG(ERROR) << "Called with nullptr name";
            return;
        }
        std::lock_guard lock(mtx);
        LOG(INFO) << server.process(name->s);
    }

    void set_duration(uint64_t duration_ms) {
        LOG(INFO) << "Trying to install sleep duration: " << duration_ms;
        std::lock_guard lock(mtx);
        thr_sleep = std::chrono::milliseconds(duration_ms);
    }

    uint64_t get_duration() {
        std::lock_guard lock(mtx);
        return thr_sleep.count();
    }

    void add_queue_call(const std::string &name) {
        std::lock_guard lock(mtx);
        calls_queue.emplace(name);
    }

    void auto_call(bool state) {
        this->state.store(state, std::memory_order_relaxed);
    }

    void reset() {
        std::lock_guard lock(mtx);
        calls_queue = std::queue<std::string>();
        methods.clear();
        state.store(true, std::memory_order_relaxed);
        server = score::RpcServer();
    }

  private:
    std::atomic<bool> state{true};
    std::condition_variable cv;
    std::queue<std::string> calls_queue{};
    std::set<std::string> methods{};
    std::jthread thr;
    std::mutex mtx;
    score::RpcServer server;
    std::chrono::duration<int64_t, std::milli> thr_sleep{50};
} test;

extern "C" {

void crpc_test_init() {
    if (!google::IsGoogleLoggingInitialized()) {
        google::InitGoogleLogging("TestRPC");
        google::LogToStderr();
    }
    try {
        test.start();
    } catch (const std::exception &e) {
        LOG(ERROR) << "Fatal error: " << e.what();
    }
}
void crpc_test_deinit() {
    test.join();
    crpc_deinit();
    test.reset();
}

void crpc_test_add_method(callback_t cb, rpc_string *name) {
    test.add_method(cb, name);
}

void crpc_test_change_duration(uint64_t duration_ms) {
    test.set_duration(duration_ms);
}

uint64_t crpc_test_duration() { return test.get_duration(); }

int crpc_test_remove_method(rpc_string *name) {
    return test.remove_method(name);
}

void crpc_test_schedule_call(rpc_string *name) {
    if (!name) {
        LOG(ERROR) << "Called with name nullptr";
        return;
    }
    test.add_queue_call(name->s);
}

void crpc_test_auto_call(uint32_t state) {
    test.auto_call(static_cast<bool>(state));
}
}
