#pragma once

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace agx {

#pragma pack(push, 1)
struct Frame {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t channel;
    uint8_t flags;
    uint8_t reserved;
    uint8_t data[8];
    uint32_t timestamp_us;
};

struct Metadata {
    uint32_t can_id;
    uint32_t dlc;
    double received_at;
};
#pragma pack(pop)

struct Record {
    Frame frame;
    double received_at;
};

static_assert(sizeof(Frame) == 24, "Frame ABI mismatch");
static_assert(offsetof(Frame, timestamp_us) == 20, "Timestamp ABI mismatch");
static_assert(sizeof(Metadata) == 16, "Metadata ABI mismatch");
static_assert(sizeof(Record) == 32, "Record ABI mismatch");

class ReceiveQueue {
public:
    static constexpr size_t capacity = 4096;

    bool push(const Frame& frame, double received_at, double now) {
        expire(now);
        if (has_timestamp_ && static_cast<uint32_t>(frame.timestamp_us - timestamp_) >= 0x80000000U) {
            return false;
        }
        timestamp_ = frame.timestamp_us;
        has_timestamp_ = true;
        if (size_ == capacity) {
            head_ = (head_ + 1) % capacity;
            --size_;
        }
        records_[(head_ + size_) % capacity] = {frame, received_at};
        ++size_;
        last_enqueue_ = now;
        armed_ = true;
        return true;
    }

    bool pop(Record& record, double now) {
        expire(now);
        if (size_ == 0) {
            return false;
        }
        record = records_[head_];
        head_ = (head_ + 1) % capacity;
        --size_;
        return true;
    }

    bool pop(Metadata& metadata, uint8_t* payload, double now) {
        expire(now);
        if (size_ == 0) { return false; }
        const auto& record = records_[head_];
        metadata.can_id = record.frame.can_id;
        metadata.dlc = record.frame.can_dlc;
        metadata.received_at = record.received_at;
        std::memcpy(payload, record.frame.data, std::min<unsigned>(8, record.frame.can_dlc));
        head_ = (head_ + 1) % capacity;
        --size_;
        return true;
    }

    void expire(double now) {
        if (armed_ && now - last_enqueue_ > 1.0) {
            head_ = 0;
            size_ = 0;
            armed_ = false;
        }
    }

    size_t size() const { return size_; }
    bool armed() const { return armed_; }

private:
    std::array<Record, capacity> records_{};
    size_t head_ = 0;
    size_t size_ = 0;
    uint32_t timestamp_ = 0;
    bool has_timestamp_ = false;
    bool armed_ = false;
    double last_enqueue_ = 0;
};

}
