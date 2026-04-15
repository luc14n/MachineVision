import time

import sensor

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot()
    s = img.get_statistics()
    print(
        "mean={} min={} max={} fps={:.1f}".format(
            int(s.l_mean()), int(s.l_min()), int(s.l_max()), clock.fps()
        )
    )
