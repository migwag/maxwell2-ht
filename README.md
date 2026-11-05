# Audeze Maxwell 2 head tracker for the Dolby Atmos Renderer

Audeze's `libMaxwellHT_0.3.3.dylib` supports only the original Maxwell (USB product IDs
`0x4B18`, `0x4B19`, `0x4B1A`, `0x4B1E`) and reads raw gyro/accel data from HID report `0x03`.
The Maxwell 2 (product ID `0x4BAA`) fuses its IMU on the headset and streams a unit quaternion
`(w, x, y, z)` as four little-endian floats in HID input report `0x0A`, so that plugin fails
`headtracker_create` and the Renderer logs "Found invalid head tracker plugin".

This plugin implements the Renderer's C head-tracker API for the Maxwell 2.

## Protocol

- Interface: vendor `0x3329`, HID usage page `0xFF13`, usage `1`, product
  `0x4BAA` (headset over USB cable) or `0x4BA9` ("Audeze Maxwell Dongle", wireless).
- Enable sensors: output report `06 09 RR 05 5A 05 00 00 09 2E 00 BC` (62 bytes, zero padded).
  Last byte is `0x80 | rate`; send `0x3C` to disable. `RR` is the routing flag: `00` for the
  headset over USB, `80` through the dongle (the dongle acks an unflagged command but the
  headset ignores it).
- Data: input report `0A` + 4 x float32 LE quaternion, about 60 Hz over both the cable and the dongle.
- The headset's frame matches the Renderer's (x forward, y left, z up); see `tools/calibrate.c`.

## Renderer host behaviour (learned the hard way)

- `headtracker_get_error` must return 0 right after `create`, or the Renderer never calls `run`.
  Connection state is therefore only written to the diagnostic log.
- `headtracker_run` must block until `headtracker_cancel` (like Dolby's OSC plugin); the Renderer
  calls it on its own tracking thread.
- At startup the Renderer validates each plugin with `create` + `destroy`, then creates the real instance.
- Diagnostics go to `~/Library/Logs/Maxwell2HT.log` (one stats line every 5 s while tracking).

## Build and install

```sh
./build.sh
cp build/libMaxwell2HT.dylib "$HOME/Library/Application Support/Dolby/Dolby Atmos Renderer/Head Tracking/"
```

Restart the Renderer and pick "Audeze Maxwell 2 head tracker" in the head-tracking settings.

## Test without the Renderer

```sh
clang -O1 -o tools/ht_test tools/ht_test.c
tools/ht_test build/libMaxwell2HT.dylib 60 live
```

## Recalibrating

`tools/calibrate.c` borrows the HID functions exported by Audeze's `libMaxwellHT_0.3.3.dylib`
(it must still be in the Head Tracking folder). It speaks cues through the headset and records
`tools/calibration.csv`; `tools/analyze_calibration.py` then prints the rotation axis for each movement.

```sh
clang -O1 -o tools/calibrate tools/calibrate.c
tools/run_calibration.sh
python3 tools/analyze_calibration.py tools/calibration.csv
```
