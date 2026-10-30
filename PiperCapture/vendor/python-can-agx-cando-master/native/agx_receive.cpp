#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "receive_queue.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

using Clock = std::chrono::steady_clock;
using ReadFunction = bool (__stdcall *)(void*, agx::Frame*, uint32_t);

double monotonic_seconds() {
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

double wall_seconds() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

struct Receiver {
    void* device;
    ReadFunction read;
    agx::ReceiveQueue queue;
    std::mutex mutex;
    std::condition_variable available;
    std::condition_variable finished;
    std::thread worker;
    std::atomic<bool> stopping{false};
    bool done = false;
    bool failed = false;
    HMODULE vendor_module = nullptr;
    HMODULE helper_module = nullptr;

    Receiver(void* handle, ReadFunction function) : device(handle), read(function) {}

    void accept(const agx::Frame& frame) {
        const double received_at = wall_seconds();
        const double now = monotonic_seconds();
        bool accepted;
        {
            std::lock_guard<std::mutex> lock(mutex);
            accepted = !stopping.load() && queue.push(frame, received_at, now);
        }
        if (accepted) {
            available.notify_one();
        }
    }

    void run() noexcept {
        try {
            while (!stopping.load()) {
                agx::Frame frame{};
                if (!read(device, &frame, 10)) {
                    std::lock_guard<std::mutex> lock(mutex);
                    queue.expire(monotonic_seconds());
                    continue;
                }
                accept(frame);
                while (!stopping.load()) {
                    agx::Frame pending{};
                    if (!read(device, &pending, 0)) {
                        break;
                    }
                    accept(pending);
                }
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex);
            failed = true;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            done = true;
        }
        available.notify_all();
        finished.notify_all();
    }
};

extern "C" uint32_t __cdecl agx_rx_abi_version() { return 2; }

extern "C" int __cdecl agx_rx_drain(void* device, void* read_address) noexcept {
    if (!device || !read_address) { return -1; }
    try {
        ReadFunction read;
        static_assert(sizeof(read) == sizeof(read_address), "Function address ABI mismatch");
        std::memcpy(&read, &read_address, sizeof(read));
        const auto started = Clock::now();
        const auto deadline = started + std::chrono::milliseconds(250);
        auto quiet_since = started;
        for (;;) {
            const auto before = Clock::now();
            if (before >= deadline) { return 0; }
            if (before - quiet_since >= std::chrono::milliseconds(100)) { return 1; }
            agx::Frame frame{};
            const bool received = read(device, &frame, 5);
            const auto now = Clock::now();
            if (now >= deadline) { return 0; }
            if (received) {
                quiet_since = now;
            } else {
                if (now - quiet_since >= std::chrono::milliseconds(100)) { return 1; }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    } catch (...) {
        return -1;
    }
}

extern "C" void* __cdecl agx_rx_create(void* device, void* read_address) noexcept {
    if (!device || !read_address) {
        return nullptr;
    }
    HMODULE vendor = nullptr;
    HMODULE helper = nullptr;
    try {
        ReadFunction function;
        static_assert(sizeof(function) == sizeof(read_address), "Function address ABI mismatch");
        std::memcpy(&function, &read_address, sizeof(function));
        if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
                               reinterpret_cast<LPCSTR>(read_address), &vendor) ||
            !GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
                               reinterpret_cast<LPCSTR>(&agx_rx_create), &helper)) {
            throw std::runtime_error("Cannot retain DLL modules");
        }
        std::unique_ptr<Receiver> receiver(new Receiver(device, function));
        receiver->vendor_module = vendor;
        receiver->helper_module = helper;
        receiver->worker = std::thread(&Receiver::run, receiver.get());
        return receiver.release();
    } catch (...) {
        if (vendor) { FreeLibrary(vendor); }
        if (helper) { FreeLibrary(helper); }
        return nullptr;
    }
}

extern "C" int __cdecl agx_rx_pop(void* context, agx::Metadata* metadata,
                                   uint8_t* payload, double timeout) noexcept {
    if (!context || !metadata || !payload || !std::isfinite(timeout)) {
        return -1;
    }
    try {
        auto& receiver = *static_cast<Receiver*>(context);
        const double deadline = monotonic_seconds() + std::max(0.0, timeout);
        std::unique_lock<std::mutex> lock(receiver.mutex);
        for (;;) {
            if (receiver.failed) { return -1; }
            if (receiver.stopping.load() || receiver.done) { return 0; }
            const double now = monotonic_seconds();
            if (receiver.queue.pop(*metadata, payload, now)) {
                return 1;
            }
            if (timeout >= 0) {
                const double remaining = deadline - now;
                if (remaining <= 0) { return 0; }
                receiver.available.wait_for(lock, std::chrono::duration<double>(std::min(remaining, 3600.0)));
            } else {
                receiver.available.wait(lock);
            }
        }
    } catch (...) {
        return -1;
    }
}

extern "C" int __cdecl agx_rx_stop(void* context, uint32_t timeout_ms) noexcept {
    if (!context) { return 1; }
    try {
        auto& receiver = *static_cast<Receiver*>(context);
        std::unique_lock<std::mutex> lock(receiver.mutex);
        receiver.stopping.store(true);
        receiver.available.notify_all();
        return receiver.finished.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                          [&]() { return receiver.done; }) ? 1 : 0;
    } catch (...) {
        return -1;
    }
}

extern "C" int __cdecl agx_rx_destroy(void* context) noexcept {
    if (!context) { return 1; }
    try {
        auto* receiver = static_cast<Receiver*>(context);
        {
            std::lock_guard<std::mutex> lock(receiver->mutex);
            if (!receiver->done) { return 0; }
        }
        if (receiver->worker.joinable()) { receiver->worker.join(); }
        HMODULE vendor = receiver->vendor_module;
        HMODULE helper = receiver->helper_module;
        delete receiver;
        FreeLibrary(vendor);
        FreeLibrary(helper);
        return 1;
    } catch (...) {
        return -1;
    }
}
