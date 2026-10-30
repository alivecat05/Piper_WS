import ctypes
import math

from can import CanInitializationError, CanOperationError

from .dll import load_receive_dll


class _Metadata(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("can_id", ctypes.c_uint32), ("dlc", ctypes.c_uint32),
                ("received_at", ctypes.c_double)]


def _load_receive_api():
    try:
        dll = load_receive_dll()
        dll.agx_rx_abi_version.argtypes = []
        dll.agx_rx_abi_version.restype = ctypes.c_uint32
        if dll.agx_rx_abi_version() != 2:
            raise CanInitializationError("Unsupported agx_receive.dll ABI; rebuild both native DLLs")
        dll.agx_rx_drain.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        dll.agx_rx_drain.restype = ctypes.c_int
        dll.agx_rx_create.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        dll.agx_rx_create.restype = ctypes.c_void_p
        dll.agx_rx_pop.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Metadata),
                                  ctypes.POINTER(ctypes.c_uint8), ctypes.c_double]
        dll.agx_rx_pop.restype = ctypes.c_int
        dll.agx_rx_stop.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        dll.agx_rx_stop.restype = ctypes.c_int
        dll.agx_rx_destroy.argtypes = [ctypes.c_void_p]
        dll.agx_rx_destroy.restype = ctypes.c_int
        return dll
    except (OSError, AttributeError) as error:
        raise CanInitializationError("Native receiver unavailable: %s" % error) from error


class NativeReceiver(object):
    @staticmethod
    def discard_pending(vendor, device):
        dll = _load_receive_api()
        address = ctypes.cast(vendor.cando_frame_read, ctypes.c_void_p)
        status = dll.agx_rx_drain(device, address)
        if status == 0:
            raise CanInitializationError("Native CAN startup drain did not become quiet within 250 ms")
        if status != 1:
            raise CanInitializationError("Native CAN startup drain failed")

    def __init__(self, vendor, device):
        self._vendor = vendor
        self._context = None
        try:
            self._dll = _load_receive_api()
            dll = self._dll
            address = ctypes.cast(vendor.cando_frame_read, ctypes.c_void_p)
            self._context = dll.agx_rx_create(device, address)
            if not self._context:
                raise CanInitializationError("Cannot start native CAN receiver")
        except (OSError, AttributeError) as error:
            raise CanInitializationError("Native receiver unavailable: %s" % error) from error

    def recv(self, timeout):
        wait = -1.0 if timeout is None else max(0.0, float(timeout))
        if timeout is not None and not math.isfinite(float(timeout)):
            raise ValueError("Receive timeout must be finite or None")
        metadata = _Metadata()
        payload = bytearray(8)
        buffer = (ctypes.c_uint8 * 8).from_buffer(payload)
        status = self._dll.agx_rx_pop(self._context, ctypes.byref(metadata), buffer, wait)
        del buffer
        if status < 0:
            raise CanOperationError("Native CAN receive worker failed")
        if status == 0:
            return None
        del payload[min(8, metadata.dlc):]
        return int(metadata.can_id), int(metadata.dlc), payload, float(metadata.received_at)

    def stop(self, timeout_ms=2000):
        return self._dll.agx_rx_stop(self._context, timeout_ms) == 1

    def destroy(self):
        if self._context:
            if self._dll.agx_rx_destroy(self._context) != 1:
                raise CanOperationError("Native receiver is still active; resources retained")
            self._context = None
