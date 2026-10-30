import argparse
import statistics
import threading
import time
import can


def percentile(sorted_values, ratio):
    if not sorted_values:
        return None
    idx = int(ratio * (len(sorted_values) - 1))
    return sorted_values[idx]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--send-interval-ms", type=float, default=None)
    parser.add_argument("--target-fps", type=float, default=6000)
    args = parser.parse_args()

    frame_count = args.count
    bitrate = 1000000
    base_id = 0x500

    bus = can.Bus(
        interface="agx_cando",
        channel=0,
        bitrate=bitrate
    )

    send_times = {}
    latencies_ms = []
    recv_count = 0
    send_count = 0
    failed_at = None
    lock = threading.Lock()
    done = threading.Event()

    def receiver():
        nonlocal recv_count
        while not done.is_set():
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            data = bytes(msg.data)
            if len(data) < 4:
                continue
            seq = int.from_bytes(data[:4], byteorder="little", signed=False)
            with lock:
                sent_at = send_times.get(seq)
            if sent_at is None:
                continue
            latencies_ms.append((time.perf_counter() - sent_at) * 1000.0)
            recv_count += 1
            if recv_count >= frame_count:
                done.set()

    rx_thread = threading.Thread(target=receiver, daemon=True)
    rx_thread.start()

    started_at = time.perf_counter()
    next_send_at = started_at
    try:
        for seq in range(frame_count):
            if args.target_fps and args.target_fps > 0:
                interval_s = 1.0 / args.target_fps
                now = time.perf_counter()
                if next_send_at > now:
                    time.sleep(next_send_at - now)
                next_send_at += interval_s
            with lock:
                send_times[seq] = time.perf_counter()
            try:
                bus.send(
                    can.Message(
                        arbitration_id=base_id,
                        data=list(seq.to_bytes(4, byteorder="little", signed=False)),
                        is_extended_id=False,
                    )
                )
                send_count += 1
                if args.send_interval_ms is not None and args.send_interval_ms > 0:
                    time.sleep(args.send_interval_ms / 1000.0)
            except Exception:
                failed_at = seq
                done.set()
                break
        done.wait(timeout=10.0)
        total_ms = (time.perf_counter() - started_at) * 1000.0
    finally:
        done.set()
        rx_thread.join(timeout=1.0)
        bus.shutdown()

    print("frame_count:", frame_count)
    print("sent:", send_count)
    print("received:", recv_count)
    print("failed_at:", failed_at)
    print("total_ms: %.3f" % total_ms)
    if total_ms > 0:
        print("send_fps: %.2f" % (send_count * 1000.0 / total_ms))
        print("recv_fps: %.2f" % (recv_count * 1000.0 / total_ms))
    print("dropped:", send_count - recv_count)
    if latencies_ms:
        sorted_latencies = sorted(latencies_ms)
        print("latency_min_ms: %.3f" % min(latencies_ms))
        print("latency_avg_ms: %.3f" % statistics.mean(latencies_ms))
        print("latency_median_ms: %.3f" % statistics.median(latencies_ms))
        print("latency_p50_ms: %.3f" % percentile(sorted_latencies, 0.50))
        print("latency_p95_ms: %.3f" % percentile(sorted_latencies, 0.95))
        print("latency_p99_ms: %.3f" % percentile(sorted_latencies, 0.99))
        print("latency_max_ms: %.3f" % max(latencies_ms))
    else:
        print("No latency samples collected.")

    return 0 if recv_count == frame_count else 1


if __name__ == "__main__":
    raise SystemExit(main())
