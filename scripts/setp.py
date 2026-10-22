#!/usr/bin/env python3
"""setp.py /node name=value [name=value ...]  -- set parameters through the SetParameters service (no ros2 CLI).
value types: true/false -> bool, 12 -> int, 1.5 -> float, [a,b] -> float list, else string"""
import sys, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

def parse(v):
    if v.lower() in ('true', 'false'): return v.lower() == 'true'
    if v.startswith('['): return [float(t) for t in v.strip('[]').split(',')]
    try: return int(v)
    except ValueError: pass
    try: return float(v)
    except ValueError: return v

def main():
    node_name, items = sys.argv[1], [a.split('=', 1) for a in sys.argv[2:]]
    rclpy.init(); n = Node('setp')
    cli = n.create_client(SetParameters, node_name + '/set_parameters')
    if not cli.wait_for_service(timeout_sec=8.0):
        print('%s: service not available' % node_name); sys.exit(1)
    req = SetParameters.Request()
    req.parameters = [Parameter(k, value=parse(v)).to_parameter_msg() for k, v in items]
    fut = cli.call_async(req); rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    r = fut.result()
    if r is None: print('%s: TIMEOUT' % node_name); sys.exit(1)
    bad = 0
    for (k, v), res in zip(items, r.results):
        print('  %s %s=%s -> %s %s' % (node_name, k, v, 'OK' if res.successful else 'FAILED', res.reason))
        bad += 0 if res.successful else 1
    n.destroy_node(); rclpy.shutdown(); sys.exit(1 if bad else 0)

if __name__ == '__main__':
    main()
