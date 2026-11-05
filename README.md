# Audeze Maxwell 2 ANC head tracker for the Dolby Atmos Renderer

Audeze's `libMaxwellHT_0.3.3.dylib` supports only the original Maxwell (USB product IDs
`0x4B18`, `0x4B19`, `0x4B1A`, `0x4B1E`) and reads raw gyro/accel data from HID report `0x03`.
The Maxwell 2 ANC (product ID `0x4BAA`) fuses its IMU on the headset and streams a unit quaternion
`(w, x, y, z)` as four little-endian floats in HID input report `0x0A`, so that plugin fails
`headtracker_create` and the Renderer logs "Found invalid head tracker plugin".

This plugin implements the Renderer's C head-tracker API for the Maxwell 2 ANC.

It works with the headset on its USB cable or through the Maxwell USB dongle, but the dongle is
preferred: it delivers fresh orientation data more consistently, while the cable connection
repeats stale samples more often, which makes tracking less smooth.

## Disclaimer

This is an independent, unofficial project. It is **not affiliated with, endorsed by, or
supported by Audeze LLC**. "Audeze", "Maxwell" and "Maxwell 2 ANC" are used only to identify
the hardware this plugin works with. The headset's head-tracking protocol was reverse
engineered for interoperability. Use it at your own risk.

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

Restart the Renderer and pick "Audeze Maxwell 2 ANC head tracker" in the head-tracking settings.

## Building with Claude Code

If you would rather not run the commands yourself, [Claude Code](https://claude.com/claude-code)
can build, test and install the plugin for you. It reads this README and the source, runs the
steps, and asks before it changes anything outside the project folder.

1. **Prerequisites.** A Mac with the Dolby Atmos Renderer installed, the Apple command line
   tools (`xcode-select --install`), and Claude Code (the `claude` command in Terminal, or
   the Code tab in the Claude desktop app).
2. **Get the code.**
   ```sh
   git clone https://github.com/migwag/maxwell2-ht.git
   cd maxwell2-ht
   ```
3. **Start Claude Code in that folder** (`claude` in Terminal, or choose the folder in the
   desktop app) and ask, for example:

   > Build the plugin, test it with tools/ht_test against my headset, and install it into
   > the Dolby Atmos Renderer's Head Tracking folder.

   Connect the headset by cable or plug in the dongle first, so the test can find it.
4. **Finish in the Renderer.** Quit and reopen it, then pick "Audeze Maxwell 2 ANC head
   tracker" in the head-tracking settings.

Other useful requests:

- *"Head tracking does nothing in the Renderer -- read ~/Library/Logs/Maxwell2HT.log and
  tell me why."* The diagnostic log records every call the Renderer makes.
- *"Run the live test so I can check the directions while I move my head."*
- *"My headset shows a different USB product ID -- add support for it."* See
  [Protocol](#protocol) for what the plugin matches.

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
