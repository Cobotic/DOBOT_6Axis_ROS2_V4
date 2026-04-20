# Dobot ROS2 driver: realtime stream corruption bugfix

This note documents a set of fixes made to the upstream Dobot ROS2 driver sources in this repository.

## Symptoms

- `/robot1/joint_states_robot` continues publishing at a steady rate (e.g. ~10 Hz), but joint positions sometimes become obviously wrong:
  - All zeros, or values that look like denormalized floating-point noise (e.g. ~`1e-313`).
- Restarting the driver process restores correct joint values immediately.
- The string topic published as `.../msg/FeedInfo` can contain corrupted numeric fields and “message lost” warnings.

## Root cause

The driver reads a binary realtime stream over TCP (port `30004`) into a packed struct (`RealTimeData`) and then interprets fields like `q_actual`.

Two upstream issues can independently produce the observed behavior:

### 1) Incorrect TCP receive logic for binary data (framing / desync)

The same `TcpClient::tcpRecv()` helper was used for:

- **Dashboard ASCII** responses (terminated by `;`), and
- **Realtime binary** frames (`RealTimeData`, expected length 1440 bytes)

The old `tcpRecv()` implementation treated `';'` as a message terminator and could return “success” early if the last byte of a read chunk happened to be `0x3B`.

For a binary stream, any dropped/extra byte causes permanent desynchronization: subsequent reads still return 1440 bytes, but they are no longer aligned to `RealTimeData` boundaries, so fields like `q_actual[]` become garbage.

Additionally, `TcpClient::disConnect()` previously set `fd_ = -1` before calling `close(fd_)`, which meant it often did **not actually close** the socket (it closed `-1`).

### 2) Data race on `RealTimeData` (corrupt `FeedInfo` and can leak elsewhere)

`CRRobotRos2::pubFeedBackInfo()` serializes many `RealTimeData` fields to JSON at 100 Hz while the TCP receive thread writes into the same `RealTimeData` memory.

There was no lock or snapshot, so reads could see torn/intermediate values, producing “random” corrupted numbers.

## Fix overview

### TCP layer fixes

Files:
- `dobot_bringup_v4/include/dobot_bringup/tcp_socket.h`
- `dobot_bringup_v4/src/tcp_socket.cpp`

Changes:
- Added `tcpRecvExact()` for exact-length reads (binary-safe; no terminator logic).
- Added `tcpRecvUntil(..., terminator)` for dashboard/ASCII reads.
- Kept the legacy `tcpRecv()` name as a wrapper around `tcpRecvUntil(..., ';')` to avoid breaking callers.
- Fixed `disConnect()` so it actually closes the correct file descriptor.

### Realtime receiver fixes (resync + snapshot)

Files:
- `dobot_bringup_v4/include/dobot_bringup/command.h`
- `dobot_bringup_v4/src/command.cpp`

Changes:
- Implemented a robust realtime frame read with **resynchronization**:
  - Uses the `(len==1440, test_value==0x0123456789ABCDEF)` signature at known offsets to find the true frame boundary.
  - If the connection starts mid-frame or bytes were previously dropped, the code slides a small window until a valid frame header is found.
- The TCP receive thread now reads into a local `RealTimeData frame` and then updates shared state under a mutex.
- Added `getRealDataSnapshot()` so publishers can read a consistent copy.
- Hardened thread shutdown (`thread_` null/joinability checks).

### FeedInfo publisher fix (no data race)

File:
- `dobot_bringup_v4/src/cr_robot_ros2.cpp`

Changes:
- `pubFeedBackInfo()` now publishes from a locked snapshot (`getRealDataSnapshot()`), eliminating the race.

### Dashboard command receiver fix

File:
- `dobot_bringup_v4/src/command.cpp`

Changes:
- Replaced the old busy-loop + `strlen()` parsing on non-NUL-terminated buffers with `tcpRecvUntil(..., ';')` and explicit NUL termination.

## How to validate

1. Build and run the driver as usual.
2. Monitor joint positions for an extended period:
   - `ros2 topic echo /robot1/joint_states_robot --field position --full-length`
3. Confirm that:
   - Values remain within plausible ranges over time.
   - The driver no longer gets “stuck” outputting zeros/denormals until restart.
4. Monitor FeedInfo:
   - `ros2 topic echo --once /<ns>/dobot_bringup_ros2/msg/FeedInfo`
   - Confirm fields are stable and do not show obviously torn/corrupted floats.

## Notes for an upstream PR

- The key technical change is separating binary-safe reads (`tcpRecvExact`) from terminator-based ASCII reads (`tcpRecvUntil`).
- The resync logic is intentionally defensive: even if the server sends perfectly aligned frames most of the time, any reconnection or transient byte loss must not permanently poison the decode.
- The snapshot change in `pubFeedBackInfo()` is required because the realtime struct is mutated in another thread.
