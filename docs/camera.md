# Camera

The camera renders offscreen through an EGL device context, so it works without a window and on a headless
machine with a GPU or a software EGL device.

```
hummingbird --headless --cam-profile ar0234 --cam-width 1280 --cam-height 720 --cam-fps 120
```

- The rate is exact in simulated time (a phase accumulator, not whole simulation steps): 120 fps on a 1 kHz
  step produces 120.0 frames per simulated second.
- `--cam-profile` selects a sensor model and implies `--cam-mono`: the RGB render is converted to mono8 with
  shot noise, read noise, temperature-dependent dark current, fixed-pattern and response non-uniformity,
  vignetting, hot pixels and lens defocus that grows as the lens temperature moves away from 20 C. The
  profile limits the resolution and frame rate.
- Frames are written straight into a Renoir **frame ring** (`drone_camera_frame`, 4 slots) with
  loan/commit, so there is one pixel copy from the model into shared memory and no per-frame allocation.
  Consumers open the ring in another process, take an owned in-place view and check `view_intact`; a slow
  consumer sees dropped or torn frames flagged, never corrupted ones. `sensor_tap` shows how to find and read
  the ring through discovery.
- Frame layout: `sensor_camera_rgb_hdr_t` followed by `width * height * channels` bytes (`channels` is 1
  for mono8).
- The offscreen buffer is sized to the requested resolution (MuJoCo otherwise clamps it to 640x480).

Measured on the development machine (RTX 3060 laptop): 1280x720 render with read-back about 1.2 ms, sensor
model 1.7 ms with portable flags (0.8 ms with AVX2). Re-measure on the target before relying on 120 fps
there; a Jetson will be slower.

The legacy `/drone/camera/rgb` topic is only published when the Foxglove bridge is built in, because the
bridge reads it in the same process. Other processes must use the frame ring: the topic path hands large
payloads over through per-process descriptors and other processes receive empty messages.
