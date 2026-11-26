"""Periodic performance line for the perception nodes (one log line every `interval` s).

    perf: 4.9 Hz processed, 21 frames skipped | infer 182/240 ms | latency 230/301 ms

  processed  frames that went through the model per second
  skipped    frames that arrived but were replaced by a newer one before processing
             (normal when max_rate < camera rate; large numbers with a low Hz = CPU too slow)
  a/b ms     mean / max over the interval
  latency    camera capture (header stamp) -> result published
"""
import time


class PerfStats:

    def __init__(self, logger, interval=10.0):
        self.logger = logger
        self.interval = interval
        self.reset(time.monotonic())

    def reset(self, now):
        self.t0, self.n, self.skip, self.times = now, 0, 0, {}

    def skipped(self, n=1):
        self.skip += n

    def add(self, **ms):
        """One processed frame with named durations in milliseconds."""
        self.n += 1
        for k, v in ms.items():
            self.times.setdefault(k, []).append(v)
        self.maybe_report()

    def maybe_report(self):
        now = time.monotonic()
        if self.interval <= 0 or now - self.t0 < self.interval:
            return
        parts = [f'perf: {self.n / (now - self.t0):.1f} Hz processed, {self.skip} frames skipped']
        for k, v in self.times.items():
            parts.append(f'{k} {sum(v) / len(v):.0f}/{max(v):.0f} ms')
        self.logger.info(' | '.join(parts))
        self.reset(now)


def age_ms(node, stamp):
    """Milliseconds since a header stamp (ROS time)."""
    t = stamp.sec + stamp.nanosec * 1e-9
    if t <= 0:
        return 0.0
    return (node.get_clock().now().nanoseconds * 1e-9 - t) * 1000.0
