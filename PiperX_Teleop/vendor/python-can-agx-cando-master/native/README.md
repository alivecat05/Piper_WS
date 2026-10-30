# Native receive worker

This directory contains the C++ receive worker and bounded queue used by the
`agx_cando` Python plugin.

## Installation and loading

The plugin loads `agx_receive.dll` alongside `cando.dll` from
`agx_cando/bin/x64/` for 64-bit Python or `agx_cando/bin/x32/` for 32-bit Python.
There is no Python receive-worker fallback: a missing or incompatible native DLL
raises an initialization error.

The Python wrapper requires native ABI version **2**, including `agx_rx_drain`.
Older helper DLLs are rejected; update both packaged architectures together.

`pip3 install .` packages the prebuilt DLLs; it does not compile native sources
or detect missing DLLs and build replacements. Users do not need a local compiler.
Developers must rebuild and include both DLLs after changing native sources.

## Build

Run the following command from the repository root, not this directory.

After changing native sources, rebuild **both** packaged DLLs with an installed
MSVC toolchain. Supply your own `vcvarsall.bat` location:

```powershell
python -m scripts.build_native --vcvars "C:\path\to\VC\Auxiliary\Build\vcvarsall.bat"
```

The build produces only `agx_cando/bin/x64/agx_receive.dll` and
`agx_cando/bin/x32/agx_receive.dll`, using a static C++ runtime. Use `--arch x64`
or `--arch x32` to build only one architecture. Object/import-library files are
created in a temporary directory and automatically removed; no test executables
or test DLLs are built. A failed compilation does not replace that architecture's
existing packaged DLL.

## Startup and send policy

Before starting the controller or native receive worker, initialization:

1. Opens the device and stops its controller. A failed stop aborts initialization.
2. Sets the bit timing.
3. Calls native `agx_rx_drain` to read and discard pending vendor records, without
   enqueueing them or seeding the receive timestamp watermark.
4. Starts the controller with `CANDO_MODE_ONE_SHOT` added to the selected mode,
   then starts the native receive worker.

The drain requires **100 ms continuously without a returned frame** within a
**250 ms total polling budget**. Each returned record restarts the quiet interval;
empty reads do not. Reads request a 5 ms timeout, with a 1 ms pause after empty
reads. The budget is checked before and after each vendor call. Reaching it before
successful completion raises `CanInitializationError`; handles are cleaned up and
neither the controller nor the worker is started. These rules are mandatory and
do not expose options or discard counters.

The budget is not a hard wall-clock timeout: a vendor call or OS scheduling delay
can overrun it, and an indefinitely blocked vendor call cannot be cancelled here.
The vendor's boolean read result does not distinguish an empty read from a read
failure. Successful draining therefore indicates an observed quiet interval, not
a guarantee about hidden or subsequently delayed driver/device data.

Draining happens **before controller start**, never as an ongoing purge while
normal reception is active. It intentionally discards records from earlier
sessions and adds about 100 ms to initialization when already quiet. Draining
after controller start can discard current traffic indefinitely on a busy bus.

`ONE_SHOT` avoids the peer-offline send stalls reproduced on the tested devices
by disabling automatic retransmission; it trades retransmission reliability for
freshness. A successful `send()` return does **not** confirm delivery or peer
acknowledgement. Existing loopback and echo choices, including their defaults,
are unchanged; use `local_loopback=False` for the physical-bus scenario tested.
This does not implement or enforce `send(timeout)` and cannot guarantee that every
vendor send call will return promptly under other failures.

## Receive policy

A native thread calls the vendor DLL directly, without Python callbacks or a
second Python queue. It continues receiving while Python consumers are blocked,
including when another Python thread holds the GIL. This does not protect against
a blocked vendor DLL, process suspension, or operating-system scheduling delays.

The following rules are mandatory, with no configuration switches or drop counters:

1. Compare every incoming device timestamp against the last accepted timestamp.
   Discard backward frames **before enqueueing**; never sort or wait to reorder.
   Equal timestamps are allowed. The same check covers data, local/TX-feedback
   records and error frames, across all IDs on that device.
2. Retain at most **4096 frames** in a native FIFO. When full, remove the oldest
   frame and append the new accepted frame. Rejected frames do not evict records.
3. Each accepted enqueue arms/resets the monotonic idle timer. After **more than
   one second without an accepted enqueue**, clear the queue and disarm the timer.
   A later accepted enqueue rearms it; rejected frames and dequeues do not.

Idle checks run on idle DLL reads and before enqueue/dequeue. This is not a hard
real-time deadline. Clearing does not reset the accepted timestamp watermark.
Unsigned 32-bit timestamp wrap is supported when compared times differ by less
than half the counter range (about 35.8 minutes). Reopen after device-clock reset
or a gap that makes the comparison ambiguous.

**The one-second threshold is inactivity, not per-frame expiry.** Continuous
accepted input prevents idle clearing. `recv()` returns the oldest retained frame,
not exclusively the newest frame. This runtime queue policy does not purge unread
data inside the vendor DLL/driver/device or downstream application queues. The
separate pre-start drain described above applies only during initialization.

## Memory and ownership

The native ring stores 4096 records of 32 bytes: **128 KiB per bus**, excluding
thread stacks, synchronization objects, vendor buffers and returned messages.
Each `recv()` copies metadata and up to eight payload bytes into a Python-owned
`bytearray`; returned messages never borrow reusable ring slots. `Message.timestamp`
is host wall-clock time captured at DLL reception, not at application dequeue.

## Shutdown and limitations

Shutdown first waits for active sends while leaving reception running, then stops
the reader and wakes blocked receives. If sends or the reader fail to stop within
the bounded waits, it raises `CanOperationError` and retains the in-use handles;
retry shutdown after the blocked operation returns. Vendor `send()` itself still
has no enforced timeout. Native worker C++ failures are reported by `recv()` rather
than silently leaving it waiting forever.

This is intentionally a lossy policy, not a fix for vendor-path disorder or loss
under overload. Disabling local loopback/echo does not imply that local-ID records
will never be returned by the vendor. Their timestamp policy is unchanged.
