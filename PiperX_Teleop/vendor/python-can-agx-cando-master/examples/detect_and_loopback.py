import can


def main() -> int:
    configs = can.detect_available_configs(interfaces=["agx_cando"])
    print("configs:", configs)
    if not configs:
        print("No devices detected.")
        return 1

    bus = can.Bus(interface="agx_cando", channel=0, bitrate=1_000_000)
    try:
        msg = can.Message(arbitration_id=0x456, data=[0xAA, 0xBB, 0xCC, 0xDD], is_extended_id=False)
        bus.send(msg)
        rx = bus.recv(timeout=2.0)
        print("recv:", rx)
        return 0 if rx is not None else 1
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
