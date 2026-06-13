# IoT Fall Detection Firmware

ESP32 NodeMCU-32S firmware that reads a WT61PC over UART and streams realtime
IMU samples to the Raspberry Pi EDGE client over BLE.

This firmware does not implement fall detection, battery measurement, SMS,
calls, or backend communication.

## Wiring

| WT61PC | ESP32 NodeMCU-32S |
| --- | --- |
| TX | GPIO 16 (RX2) |
| RX | GPIO 17 (TX2) |
| GND | GND |
| VCC | Supply required by the specific WT61PC board |

The firmware starts at 9600 baud and automatically alternates between 9600 and
115200 while the IMU is unavailable. UART format is 8N1.

## BLE Interface

- Device name: `FallDetect-IMU`
- Service: `12345678-1234-1234-1234-123456789012`
- IMU notify characteristic: `12345678-1234-1234-1234-123456789abc`
- Status read characteristic: `12345678-1234-1234-1234-123456789def`

### IMU Packet V1

The notify value is exactly 61 bytes, packed and little-endian.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | `uint8[2]` | Magic `AA 55` |
| 2 | `uint16` | Sequence number |
| 4 | `float[3]` | `ax, ay, az`, m/s^2 |
| 16 | `float[3]` | `gx, gy, gz`, deg/s |
| 28 | `float[3]` | `roll, pitch, yaw`, degrees |
| 40 | `float[4]` | `q0, q1, q2, q3` |
| 56 | `uint32` | ESP32 `millis()` timestamp |
| 60 | `uint8` | XOR checksum of bytes 0 through 59 |

There is no battery or reserved battery byte in protocol V1. Byte 60 remains
the packet checksum. Quaternion values are zero until a valid WT61PC `0x59`
frame is received. Read `quat_valid` from the status characteristic before
using quaternion values. Each packet is a latest-value snapshot; the
quaternion can be from the preceding WT61PC cycle because `0x59` normally
arrives after the `0x53` angle frame.

### MTU Requirement

A 61-byte notification requires negotiated ATT MTU of at least 64 bytes. The
ESP32 advertises a local MTU capability of 512, but the client must request a
sufficient MTU. The EDGE client should:

1. Connect to `FallDetect-IMU`.
2. Negotiate MTU 64 or larger, preferably 185 or 247.
3. Read status and require `"mtu_ok":true`.
4. Subscribe to the IMU characteristic.

The firmware does not send IMU notifications while negotiated MTU is below 64.
It reports `mtu_too_small` over USB serial and exposes `mtu` and `mtu_ok` in
status. Fragmentation/protocol V2 is the fallback if a target BlueZ setup
cannot negotiate this MTU; it is not enabled in protocol V1.

## Status Characteristic

Status is a JSON string generated when the characteristic is read:

```json
{
  "ble_connected": true,
  "advertising": false,
  "imu_ready": true,
  "imu_timeout": false,
  "last_valid_imu_age_ms": 12,
  "seq": 184,
  "checksum_error_count": 0,
  "queue_drop_count": 2,
  "packets_sent_count": 181,
  "notify_error_count": 0,
  "mtu": 185,
  "mtu_ok": true,
  "notify_enabled": true,
  "quat_valid": true
}
```

The IMU enters timeout state after 1500 ms without a complete fresh
accel/gyro/angle snapshot. Timeout clears `imu_ready`, invalidates quaternion
state, removes stale queued samples, changes the LED pattern, and resumes baud
search. Valid samples restore `imu_ready`.

The queue favors realtime data. If full, the oldest item is removed before the
new sample is inserted. The BLE task also coalesces any backlog to its newest
sample. Both cases increment `queue_drop_count`.

## Serial Debug And Calibration

Normal builds print state transitions and a health line every five seconds.
To emit one JSON line for every IMU snapshot, enable:

```ini
build_flags =
    -DCORE_DEBUG_LEVEL=0
    -DCONFIG_BT_NIMBLE_ENABLED=0
    -DIMU_SERIAL_DEBUG=1
```

Then build/upload and calibrate without requiring a BLE connection:

```powershell
pio run --target upload
python tools/imu_calibrate.py --port COM5 --seconds 8
python tools/imu_visualizer.py --port COM5 --calibration tools/imu_calibration.json
```

Calibration fails with a clear error if it receives fewer than 20 samples
instead of waiting forever.

## Build

```powershell
pio run
pio run --target upload
pio device monitor
```

## Troubleshooting

- `imu_timeout`: check power, common ground, RX/TX crossing, and UART baud.
- Increasing `checksum_error_count`: check baud, wiring length, noise, and
  supply stability.
- BLE connected but no packets: enable notifications and negotiate MTU >= 64.
- `mtu_ok:false`: fix MTU negotiation in the EDGE/BlueZ client.
- Increasing `queue_drop_count`: the client/link is slower than the IMU; data
  remains current, but intermediate samples were intentionally discarded.
- `notify_error_count`: verify the client subscribed to the CCC descriptor and
  remains connected.
