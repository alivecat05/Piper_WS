import contextlib
import ctypes
import importlib
import pathlib
import runpy
import sys
import time
import types
import unittest
from unittest import mock


class FakeCanModule(types.ModuleType):
    def __init__(self):
        super().__init__("can")

        class BusABC:
            def __init__(self, channel=None, can_filters=None, **kwargs):
                del kwargs
                self.channel = channel
                self.channel_info = ""
                self._filters = can_filters
                self._shutdown = False

            def set_filters(self, filters=None):
                self._filters = filters

            def shutdown(self):
                self._shutdown = True

        class CanInitializationError(Exception):
            pass

        class CanOperationError(Exception):
            pass

        class Message:
            def __init__(
                self,
                arbitration_id=0,
                data=None,
                is_extended_id=False,
                is_remote_frame=False,
                is_error_frame=False,
                dlc=None,
                channel=None,
                timestamp=None,
            ):
                payload = bytes(data or b"")
                self.arbitration_id = arbitration_id
                self.data = payload
                self.is_extended_id = is_extended_id
                self.is_remote_frame = is_remote_frame
                self.is_error_frame = is_error_frame
                self.dlc = len(payload) if dlc is None else dlc
                self.channel = channel
                self.timestamp = timestamp

        self.BusABC = BusABC
        self.CanInitializationError = CanInitializationError
        self.CanOperationError = CanOperationError
        self.Message = Message


class FakeCFunc:
    def __init__(self, func):
        self.func = func
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.func(*args)


class FakeDll:
    def __init__(self, device_count=1):
        self.device_count = device_count
        self.list_handle_value = 0x101
        self.dev_handle_value = 0x202
        self.started_mode = None
        self.sent_frames = []
        self.last_timing = None
        self.closed = False
        self.stopped = False
        self.read_frames = []
        self.events = []
        self.freed = False
        self.list_freed = False

        self.cando_list_malloc = FakeCFunc(self._cando_list_malloc)
        self.cando_list_free = FakeCFunc(self._cando_list_free)
        self.cando_list_scan = FakeCFunc(self._cando_list_scan)
        self.cando_list_num = FakeCFunc(self._cando_list_num)
        self.cando_malloc = FakeCFunc(self._cando_malloc)
        self.cando_free = FakeCFunc(self._cando_free)
        self.cando_open = FakeCFunc(self._cando_open)
        self.cando_close = FakeCFunc(self._cando_close)
        self.cando_get_serial_number_str = FakeCFunc(self._cando_get_serial_number_str)
        self.cando_get_dev_info = FakeCFunc(self._cando_get_dev_info)
        self.cando_set_timing = FakeCFunc(self._cando_set_timing)
        self.cando_start = FakeCFunc(self._cando_start)
        self.cando_stop = FakeCFunc(self._cando_stop)
        self.cando_frame_send = FakeCFunc(self._cando_frame_send)
        self.cando_frame_read = FakeCFunc(self._cando_frame_read)

    def _cando_list_malloc(self, out_ptr):
        out_ptr._obj.value = self.list_handle_value
        return True

    def _cando_list_free(self, handle):
        del handle
        self.list_freed = True
        return True

    def _cando_list_scan(self, handle):
        del handle
        return True

    def _cando_list_num(self, handle, count_ptr):
        del handle
        count_ptr._obj.value = self.device_count
        return True

    def _cando_malloc(self, list_handle, index, out_ptr):
        del list_handle
        if int(index) >= self.device_count:
            return False
        out_ptr._obj.value = self.dev_handle_value
        return True

    def _cando_free(self, handle):
        del handle
        self.freed = True
        return True

    def _cando_open(self, handle):
        del handle
        self.events.append("open")
        return True

    def _cando_close(self, handle):
        del handle
        self.closed = True
        return True

    def _cando_get_serial_number_str(self, handle):
        del handle
        return "FAKE-SERIAL"

    def _cando_get_dev_info(self, handle, fw_ptr, hw_ptr):
        del handle
        fw_ptr._obj.value = 0x11
        hw_ptr._obj.value = 0x22
        return True

    def _cando_set_timing(self, handle, timing_ptr):
        del handle
        self.events.append("timing")
        timing = timing_ptr._obj
        self.last_timing = (
            timing.prop_seg,
            timing.phase_seg1,
            timing.phase_seg2,
            timing.sjw,
            timing.brp,
        )
        return True

    def _cando_start(self, handle, mode):
        del handle
        self.events.append("start")
        self.started_mode = int(mode)
        return True

    def _cando_stop(self, handle):
        del handle
        self.events.append("stop")
        self.stopped = True
        return True

    def _cando_frame_send(self, handle, frame_ptr):
        del handle
        frame = frame_ptr._obj
        self.sent_frames.append(
            {
                "echo_id": int(frame.echo_id),
                "can_id": int(frame.can_id),
                "can_dlc": int(frame.can_dlc),
                "channel": int(frame.channel),
                "data": bytes(frame.data[: frame.can_dlc]),
            }
        )
        return True

    def _cando_frame_read(self, handle, frame_ptr, timeout_ms):
        del handle
        if not self.read_frames:
            if int(timeout_ms) > 0:
                time.sleep(min(int(timeout_ms), 5) / 1000.0)
            return False

        next_frame = self.read_frames.pop(0)
        target = frame_ptr._obj
        target.echo_id = next_frame.echo_id
        target.can_id = next_frame.can_id
        target.can_dlc = next_frame.can_dlc
        target.channel = next_frame.channel
        target.flags = next_frame.flags
        target.reserved = next_frame.reserved
        target.timestamp_us = next_frame.timestamp_us
        for idx in range(8):
            target.data[idx] = next_frame.data[idx]
        return True


def import_modules():
    for name in list(sys.modules):
        if name == "agx_cando" or name.startswith("agx_cando."):
            sys.modules.pop(name)
    sys.modules["can"] = FakeCanModule()
    package = importlib.import_module("agx_cando")
    bus_mod = importlib.import_module("agx_cando.bus")
    dll_mod = importlib.import_module("agx_cando.dll")
    return package, bus_mod, dll_mod, sys.modules["can"]


class FakeReceiver:
    @staticmethod
    def discard_pending(vendor, device):
        vendor.events.append("drain")
        vendor.read_frames.clear()

    def __init__(self, vendor, device):
        vendor.events.append("receiver_start")
        self.record = None

    def recv(self, timeout):
        record, self.record = self.record, None
        return record

    def stop(self):
        return True

    def destroy(self):
        pass


class FakeReceiveDll:
    def __init__(self, drain_status=1, abi=2):
        self.agx_rx_abi_version = FakeCFunc(mock.Mock(return_value=abi))
        self.agx_rx_drain = FakeCFunc(mock.Mock(return_value=drain_status))
        self.agx_rx_create = FakeCFunc(mock.Mock(return_value=1))
        self.agx_rx_pop = FakeCFunc(mock.Mock(return_value=0))
        self.agx_rx_stop = FakeCFunc(mock.Mock(return_value=1))
        self.agx_rx_destroy = FakeCFunc(mock.Mock(return_value=1))


@contextlib.contextmanager
def fake_native_bus(bus_mod, vendor):
    with mock.patch.object(bus_mod, "load_cando_dll", return_value=vendor):
        with mock.patch.object(bus_mod, "NativeReceiver", FakeReceiver):
            yield


class SmokeTests(unittest.TestCase):
    def tearDown(self):
        sys.modules.pop("can", None)
        for name in list(sys.modules):
            if name == "agx_cando" or name.startswith("agx_cando."):
                sys.modules.pop(name)

    def test_detect_available_configs_reports_each_device(self):
        _, bus_mod, _, _ = import_modules()
        fake_dll = FakeDll(device_count=3)

        with fake_native_bus(bus_mod, fake_dll):
            configs = bus_mod.AgxCandoBus._detect_available_configs()

        self.assertEqual(
            configs,
            [
                {"interface": "agx_cando", "channel": 0},
                {"interface": "agx_cando", "channel": 1},
                {"interface": "agx_cando", "channel": 2},
            ],
        )

    def test_send_encodes_classic_extended_remote_error_flags(self):
        _, bus_mod, _, can_mod = import_modules()
        fake_dll = FakeDll(device_count=1)

        with fake_native_bus(bus_mod, fake_dll):
            bus = bus_mod.AgxCandoBus(channel=0, bitrate=1_000_000, local_loopback=False)
            try:
                message = can_mod.Message(
                    arbitration_id=0x1ABCDE,
                    data=b"\x10\x20\x30",
                    is_extended_id=True,
                    is_remote_frame=True,
                    is_error_frame=True,
                )
                bus.send(message)
            finally:
                bus.shutdown()

        self.assertEqual(len(fake_dll.sent_frames), 1)
        self.assertEqual(fake_dll.started_mode, bus_mod.CANDO_MODE_ONE_SHOT)
        sent = fake_dll.sent_frames[0]
        self.assertEqual(sent["can_dlc"], 3)
        self.assertEqual(sent["data"], b"\x10\x20\x30")
        self.assertEqual(
            sent["can_id"],
            0x1ABCDE
            | bus_mod.CANDO_ID_EXTENDED
            | bus_mod.CANDO_ID_RTR
            | bus_mod.CANDO_ID_ERR,
        )

    def test_receive_own_messages_false_enables_no_echo_even_with_loopback(self):
        _, bus_mod, _, can_mod = import_modules()
        fake_dll = FakeDll(device_count=1)

        with fake_native_bus(bus_mod, fake_dll):
            bus = bus_mod.AgxCandoBus(
                channel=0,
                bitrate=1_000_000,
                local_loopback=True,
                receive_own_messages=False,
            )
            try:
                bus.send(can_mod.Message(arbitration_id=0x123, data=b"\x01", is_extended_id=False))
            finally:
                bus.shutdown()

        self.assertEqual(
            fake_dll.started_mode,
            bus_mod.CANDO_MODE_LOOP_BACK | bus_mod.CANDO_MODE_ONE_SHOT | bus_mod.CANDO_MODE_NO_ECHO_BACK,
        )
        self.assertEqual(fake_dll.sent_frames[0]["echo_id"], 0xFFFFFFFF)

    def test_receive_own_messages_true_keeps_loopback_echo_enabled(self):
        _, bus_mod, _, can_mod = import_modules()
        fake_dll = FakeDll(device_count=1)

        with fake_native_bus(bus_mod, fake_dll):
            bus = bus_mod.AgxCandoBus(
                channel=0,
                bitrate=1_000_000,
                local_loopback=True,
                receive_own_messages=True,
            )
            try:
                bus.send(can_mod.Message(arbitration_id=0x123, data=b"\x01", is_extended_id=False))
            finally:
                bus.shutdown()

        self.assertEqual(fake_dll.started_mode, bus_mod.CANDO_MODE_LOOP_BACK | bus_mod.CANDO_MODE_ONE_SHOT)
        self.assertEqual(fake_dll.sent_frames[0]["echo_id"], 0)

    def test_recv_internal_decodes_frame_bits_into_message(self):
        _, bus_mod, _, _ = import_modules()
        fake_dll = FakeDll(device_count=1)

        with fake_native_bus(bus_mod, fake_dll):
            bus = bus_mod.AgxCandoBus(channel=0, bitrate=500_000, local_loopback=False)
            try:
                frame = bus_mod.CandoFrame()
                frame.can_id = 0x123 | bus_mod.CANDO_ID_EXTENDED | bus_mod.CANDO_ID_RTR
                frame.can_dlc = 2
                frame.data[0] = 0xAB
                frame.data[1] = 0xCD
                frame.timestamp_us = 42
                bus._receiver.record = (frame.can_id, frame.can_dlc, bytearray(frame.data[:2]), 1000.0)

                msg, already_filtered = bus._recv_internal(timeout=0)
            finally:
                bus.shutdown()

        self.assertFalse(already_filtered)
        self.assertEqual(msg.arbitration_id, 0x123)
        self.assertTrue(msg.is_extended_id)
        self.assertTrue(msg.is_remote_frame)
        self.assertFalse(msg.is_error_frame)
        self.assertEqual(bytes(msg.data), b"\xAB\xCD")

    def test_startup_stops_and_drains_before_starting_controller_and_receiver(self):
        _, bus_mod, _, _ = import_modules()
        fake_dll = FakeDll()
        fake_dll.read_frames.append(bus_mod.CandoFrame())

        with fake_native_bus(bus_mod, fake_dll):
            bus = bus_mod.AgxCandoBus(local_loopback=False, receive_own_messages=False)
            try:
                self.assertEqual(
                    fake_dll.events,
                    ["open", "stop", "timing", "drain", "start", "receiver_start"],
                )
                self.assertEqual(fake_dll.read_frames, [])
                self.assertEqual(
                    fake_dll.started_mode,
                    bus_mod.CANDO_MODE_ONE_SHOT | bus_mod.CANDO_MODE_NO_ECHO_BACK,
                )
            finally:
                bus.shutdown()

    def test_startup_stop_failure_does_not_drain_or_start(self):
        _, bus_mod, _, can_mod = import_modules()
        fake_dll = FakeDll()
        fake_dll.cando_stop.func = mock.Mock(return_value=False)

        with fake_native_bus(bus_mod, fake_dll):
            with self.assertRaisesRegex(can_mod.CanInitializationError, "cando_stop failed"):
                bus_mod.AgxCandoBus()

        self.assertNotIn("drain", fake_dll.events)
        self.assertIsNone(fake_dll.started_mode)
        self.assertNotIn("receiver_start", fake_dll.events)
        self.assertTrue(fake_dll.closed and fake_dll.freed and fake_dll.list_freed)

    def test_startup_drain_failure_closes_handles_without_starting(self):
        _, bus_mod, _, can_mod = import_modules()
        fake_dll = FakeDll()

        with fake_native_bus(bus_mod, fake_dll):
            with mock.patch.object(
                FakeReceiver, "discard_pending",
                side_effect=can_mod.CanInitializationError("startup drain timeout"),
            ):
                with self.assertRaisesRegex(can_mod.CanInitializationError, "startup drain timeout"):
                    bus_mod.AgxCandoBus()

        self.assertIsNone(fake_dll.started_mode)
        self.assertNotIn("receiver_start", fake_dll.events)
        self.assertTrue(fake_dll.closed and fake_dll.freed and fake_dll.list_freed)

    def test_controller_start_failure_does_not_create_receiver(self):
        _, bus_mod, _, can_mod = import_modules()
        fake_dll = FakeDll()
        fake_dll.cando_start.func = mock.Mock(return_value=False)

        with fake_native_bus(bus_mod, fake_dll):
            with self.assertRaisesRegex(can_mod.CanInitializationError, "cando_start failed"):
                bus_mod.AgxCandoBus()

        self.assertIn("drain", fake_dll.events)
        self.assertNotIn("receiver_start", fake_dll.events)
        self.assertTrue(fake_dll.closed and fake_dll.freed and fake_dll.list_freed)

    def test_native_drain_calls_helper_without_creating_worker(self):
        import_modules()
        native_mod = importlib.import_module("agx_cando.native")
        native_dll = FakeReceiveDll()
        vendor = types.SimpleNamespace(cando_frame_read=ctypes.c_void_p(0x1234))
        device = ctypes.c_void_p(0x5678)

        with mock.patch.object(native_mod, "load_receive_dll", return_value=native_dll):
            native_mod.NativeReceiver.discard_pending(vendor, device)

        native_dll.agx_rx_drain.func.assert_called_once()
        actual_device, actual_address = native_dll.agx_rx_drain.func.call_args[0]
        self.assertEqual(actual_device.value, device.value)
        self.assertEqual(actual_address.value, vendor.cando_frame_read.value)
        native_dll.agx_rx_create.func.assert_not_called()

    def test_native_drain_status_failures_raise_initialization_errors(self):
        _, _, _, can_mod = import_modules()
        native_mod = importlib.import_module("agx_cando.native")
        vendor = types.SimpleNamespace(cando_frame_read=ctypes.c_void_p(0x1234))

        for status, message in ((0, "within 250 ms"), (-1, "drain failed"), (2, "drain failed")):
            with self.subTest(status=status):
                native_dll = FakeReceiveDll(drain_status=status)
                with mock.patch.object(native_mod, "load_receive_dll", return_value=native_dll):
                    with self.assertRaisesRegex(can_mod.CanInitializationError, message):
                        native_mod.NativeReceiver.discard_pending(vendor, ctypes.c_void_p(1))
                native_dll.agx_rx_create.func.assert_not_called()

    def test_incompatible_native_abi_fails_before_controller_start(self):
        _, bus_mod, _, can_mod = import_modules()
        native_mod = importlib.import_module("agx_cando.native")
        fake_dll = FakeDll()
        native_dll = FakeReceiveDll(abi=1)

        with mock.patch.object(bus_mod, "load_cando_dll", return_value=fake_dll):
            with mock.patch.object(native_mod, "load_receive_dll", return_value=native_dll):
                with self.assertRaisesRegex(can_mod.CanInitializationError, "Unsupported.*ABI"):
                    bus_mod.AgxCandoBus()

        native_dll.agx_rx_drain.func.assert_not_called()
        native_dll.agx_rx_create.func.assert_not_called()
        self.assertIsNone(fake_dll.started_mode)
        self.assertTrue(fake_dll.closed and fake_dll.freed and fake_dll.list_freed)

    def test_missing_native_dll_fails_before_controller_start(self):
        _, bus_mod, _, can_mod = import_modules()
        native_mod = importlib.import_module("agx_cando.native")
        fake_dll = FakeDll()

        with mock.patch.object(bus_mod, "load_cando_dll", return_value=fake_dll):
            with mock.patch.object(native_mod, "load_receive_dll", side_effect=OSError("missing DLL")):
                with self.assertRaisesRegex(can_mod.CanInitializationError, "Native receiver unavailable"):
                    bus_mod.AgxCandoBus()

        self.assertIsNone(fake_dll.started_mode)
        self.assertTrue(fake_dll.closed and fake_dll.freed and fake_dll.list_freed)

    def test_load_cando_dll_rejects_non_windows_platforms(self):
        _, _, dll_mod, _ = import_modules()

        with mock.patch.object(dll_mod.os, "name", "posix"):
            with self.assertRaises(OSError):
                dll_mod.load_cando_dll()

    def test_setup_py_only_allows_windows_installs(self):
        repo_root = pathlib.Path(__file__).resolve().parent.parent
        fake_setuptools = types.ModuleType("setuptools")
        fake_setuptools.find_packages = mock.Mock(return_value=["agx_cando"])
        fake_setuptools.setup = mock.Mock()

        with mock.patch.dict(sys.modules, {"setuptools": fake_setuptools}):
            with mock.patch.object(sys, "platform", "win32"):
                runpy.run_path(str(repo_root / "setup.py"), run_name="__main__")

            fake_setuptools.setup.assert_called_once()
            self.assertEqual(
                fake_setuptools.setup.call_args[1]["name"],
                "python-can-agx-cando",
            )

            for platform in ("linux", "darwin"):
                with self.subTest(platform=platform):
                    fake_setuptools.setup.reset_mock()
                    fake_setuptools.find_packages.reset_mock()
                    with mock.patch.object(sys, "platform", platform):
                        with self.assertRaisesRegex(
                            RuntimeError,
                            "can only be built or installed on Windows",
                        ):
                            runpy.run_path(
                                str(repo_root / "setup.py"), run_name="__main__"
                            )
                    fake_setuptools.setup.assert_not_called()
                    fake_setuptools.find_packages.assert_not_called()


if __name__ == "__main__":
    unittest.main()
