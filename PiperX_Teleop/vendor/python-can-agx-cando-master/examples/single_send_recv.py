import argparse
import can


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bitrate", type=int, default=1000000)
    args = parser.parse_args()

    bus = can.Bus(
        interface="agx_cando",
        channel=0,
        bitrate=args.bitrate
    )
    try:
        msg = can.Message(arbitration_id=0x123, data=[0x01, 0x02, 0x03, 0x04], is_extended_id=False)
        print("sending...")
        bus.send(msg)
        print("sent")
        rx = bus.recv(timeout=2.0)
        print("recv:", rx)
        return 0 if rx is not None else 1
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
