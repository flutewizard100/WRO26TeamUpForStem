import argparse
import math
import statistics
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
except ImportError as exc:
    print(
        "This helper requires rclpy — run it inside a sourced ROS 2 "
        f"environment.\nImport error: {exc}",
        file=sys.stderr,
    )
    sys.exit(2)


# --- Field extractors ------------------------------------------------------

def _yaw_from_quaternion(q):
    """Yaw (Z rotation) from a geometry_msgs/Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


ODOM_FIELDS = {
    'pose.x':      lambda m: m.pose.pose.position.x,
    'pose.y':      lambda m: m.pose.pose.position.y,
    'pose.yaw':    lambda m: _yaw_from_quaternion(m.pose.pose.orientation),
    'twist.vx':    lambda m: m.twist.twist.linear.x,
    'twist.vy':    lambda m: m.twist.twist.linear.y,
    'twist.vyaw':  lambda m: m.twist.twist.angular.z,
}

IMU_FIELDS = {
    'imu.ax':   lambda m: m.linear_acceleration.x,
    'imu.ay':   lambda m: m.linear_acceleration.y,
    'imu.wz':   lambda m: m.angular_velocity.z,
}


# --- Live capture ----------------------------------------------------------

def _collect_live(duration_s):
    """Subscribe to /odom + /imu/data for `duration_s`; return {topic: [msgs]}."""
    rclpy.init()
    node = Node('calibrate_otos_live')

    samples = {'/odom': [], '/imu/data': []}
    node.create_subscription(Odometry, '/odom',      samples['/odom'].append,     50)
    node.create_subscription(Imu,      '/imu/data',  samples['/imu/data'].append, 50)

    print(f'Listening on /odom and /imu/data for {duration_s:.1f} s ...')
    end = time.monotonic() + duration_s
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    n_odom = len(samples['/odom'])
    n_imu  = len(samples['/imu/data'])
    print(f'Captured {n_odom} /odom + {n_imu} /imu/data messages.')
    if n_odom == 0:
        print(
            'No /odom received. Is hardware.launch.py running? '
            'Is otos_node up (check `ros2 topic hz /odom`)?',
            file=sys.stderr,
        )
    return samples


# --- Stats -----------------------------------------------------------------

def _trim(values, fraction):
    if fraction <= 0 or len(values) < 20:
        return values
    n = len(values)
    cut = max(1, int(n * fraction))
    return values[cut:-cut]


def _std(values):
    if len(values) < 2:
        return float('nan')
    return statistics.stdev(values)


def _mean(values):
    if not values:
        return float('nan')
    return statistics.fmean(values)


def analyze(samples, steady_state):
    """Print per-field mean/std and return a dict of stds."""
    trim_fraction = 0.10 if steady_state else 0.0

    stds = {}

    print(f"{'field':<12} {'n':>6} {'mean':>+12} {'std':>12}")
    print('-' * 44)

    if '/odom' in samples and samples['/odom']:
        for name, extract in ODOM_FIELDS.items():
            raw = [extract(m) for m in samples['/odom']]
            trimmed = _trim(raw, trim_fraction)
            m = _mean(trimmed)
            s = _std(trimmed)
            stds[name] = s
            print(f"{name:<12} {len(trimmed):>6} {m:>+12.4f} {s:>12.4f}")

    if '/imu/data' in samples and samples['/imu/data']:
        for name, extract in IMU_FIELDS.items():
            raw = [extract(m) for m in samples['/imu/data']]
            trimmed = _trim(raw, trim_fraction)
            m = _mean(trimmed)
            s = _std(trimmed)
            stds[name] = s
            print(f"{name:<12} {len(trimmed):>6} {m:>+12.4f} {s:>12.4f}")

    return stds


def _fmt(x):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return '(no data)'
    return f'{x:.4f}'


def print_recommendation(stds):
    """Suggest launch-file parameter values from the computed stds."""
    xy    = max((stds.get('pose.x') or 0.0), (stds.get('pose.y') or 0.0))
    yaw   = stds.get('pose.yaw')
    vxy   = max((stds.get('twist.vx') or 0.0), (stds.get('twist.vy') or 0.0))
    vyaw  = stds.get('twist.vyaw')
    accel = max((stds.get('imu.ax') or 0.0), (stds.get('imu.ay') or 0.0)) or None
    gyro  = stds.get('imu.wz')

    print()
    print('Suggested parameter block for hardware.launch.py')
    print('-' * 44)
    print("            parameters=[{")
    print("                'publish_tf': False,")
    print(f"                'pose_xy_std':   {_fmt(xy or None)},")
    print(f"                'pose_yaw_std':  {_fmt(yaw)},")
    print(f"                'twist_xy_std':  {_fmt(vxy or None)},")
    print(f"                'twist_yaw_std': {_fmt(vyaw)},")
    print(f"                'accel_std':     {_fmt(accel)},")
    print(f"                'gyro_std':      {_fmt(gyro)},")
    print("            }],")
    print()
    print(
        'Use the LARGER value between static and dynamic captures for each '
        'parameter. Do not trust any std < 1e-4 — that is below the '
        "sensor's real precision and likely an artifact."
    )


# --- Main ------------------------------------------------------------------

def main(argv=None):
   
    samples = _collect_live(15)
    if not samples or not any(samples.values()):
        print('No /odom or /imu/data messages captured.', file=sys.stderr)
        return 1

    stds = analyze(samples, True)
    print_recommendation(stds)
    return 0


if __name__ == '__main__':
    sys.exit(main())
