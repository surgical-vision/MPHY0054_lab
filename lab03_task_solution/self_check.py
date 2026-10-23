#!/usr/bin/env python3
"""Check the running Lab 03 server and the installed student client."""

import argparse
import os
import signal
import subprocess
import tempfile
import time
import uuid

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from lab03_task_solution.srv import Rotmat2quat


PACKAGE = 'lab03_task_solution'
TOLERANCE = 1e-5


def quaternion_matrix(quaternion):
    """Build test inputs and compare rotations without choosing a quaternion sign."""
    x, y, z, w = quaternion
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def test_cases():
    """Generate known input rotations; this is not a matrix-to-quaternion solver."""
    yield 'identity', np.eye(3)
    axes = [('x', [1., 0., 0.]), ('y', [0., 1., 0.]), ('z', [0., 0., 1.]),
            ('mixed_axis', [1., -2., 3.])]
    angles = [('positive_90', np.pi/2), ('negative_90', -np.pi/2),
              ('half_turn', np.pi)]
    for axis_name, axis in axes:
        axis = np.asarray(axis) / np.linalg.norm(axis)
        for angle_name, angle in angles:
            quaternion = np.r_[axis * np.sin(angle/2), np.cos(angle/2)]
            yield axis_name + '_' + angle_name, quaternion_matrix(quaternion)
    axis = np.array([1., -2., 3.]) / np.sqrt(14.)
    for name, angle in [('near_identity', 1e-7), ('near_half_turn', np.pi-1e-7)]:
        quaternion = np.r_[axis * np.sin(angle/2), np.cos(angle/2)]
        yield name, quaternion_matrix(quaternion)
    generator = np.random.default_rng(202627)
    for index in range(12):
        quaternion = generator.normal(size=4)
        quaternion /= np.linalg.norm(quaternion)
        yield f'random_{index + 1:02d}', quaternion_matrix(quaternion)


def request_problem(request):
    """Return an error for invalid client input, or an empty string."""
    data = np.asarray(request.r.data, dtype=float)
    if data.shape != (9,) or not np.all(np.isfinite(data)):
        return 'request.r.data must contain nine finite numbers'
    rotation = data.reshape(3, 3)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0):
        return 'request matrix is not orthogonal; arbitrary random entries are not a rotation'
    if not np.isclose(np.linalg.det(rotation), 1., atol=1e-6, rtol=0):
        return 'request matrix must have determinant +1'
    return ''


def response_problem(rotation, response):
    """Check the represented rotation, accepting both q and -q."""
    q = response.q
    quaternion = np.array([q.x, q.y, q.z, q.w])
    if not np.all(np.isfinite(quaternion)):
        return 'response quaternion contains NaN or infinity'
    norm = float(np.linalg.norm(quaternion))
    if not np.isclose(norm, 1., atol=1e-6, rtol=0):
        return f'response quaternion is not unit length (norm={norm:.6g})'
    error = float(np.max(np.abs(quaternion_matrix(quaternion) - rotation)))
    if error > TOLERANCE:
        return f'wrong rotation (maximum matrix error={error:.6g}); check x,y,z,w order and signs'
    return ''


class Lab03Check(Node):
    def __init__(self, service):
        super().__init__('lab03_self_check_' + uuid.uuid4().hex[:8])
        self.group = ReentrantCallbackGroup()
        self.target = self.create_client(Rotmat2quat, service, callback_group=self.group)
        self.client_completed = 0
        self.client_problems = []

    async def check_client_request(self, request, response):
        """Validate a client request and relay it to the student's tested server."""
        problem = request_problem(request)
        if problem:
            self.client_problems.append(problem)
            return response
        try:
            result = await self.target.call_async(request)
            if result is None:
                self.client_problems.append('server returned no response to the client request')
                return response
            problem = response_problem(np.asarray(request.r.data).reshape(3, 3), result)
            if problem:
                self.client_problems.append(problem)
            self.client_completed += 1
            return result
        except Exception as error:
            self.client_problems.append(f'service request failed: {error}')
            return response


def check_server(node, timeout):
    if not node.target.wait_for_service(timeout_sec=timeout):
        print('FAIL: service is unavailable. Build your work and start the server first.', flush=True)
        return False
    failures, total = 0, 0
    for name, rotation in test_cases():
        total += 1
        request = Rotmat2quat.Request()
        request.r.data = rotation.ravel().tolist()
        future = node.target.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            print(f'FAIL: {name}: no response within {timeout:g} seconds; inspect the server terminal.', flush=True)
            return False
        try:
            response = future.result()
            problem = 'no response' if response is None else response_problem(rotation, response)
        except Exception as error:
            problem = str(error)
        print(f'{"FAIL" if problem else "PASS"}: {name}' + (': ' + problem if problem else ''), flush=True)
        if problem:
            failures += 1
            print('  Input matrix (row-major):', rotation.ravel().tolist(), flush=True)
    print(f'Server: {total - failures}/{total} cases passed.', flush=True)
    return failures == 0


def stop_process(process):
    """Stop only the client launched by this check."""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def check_client(node, samples, timeout):
    namespace = PACKAGE + '_check_' + uuid.uuid4().hex[:8]
    proxy = node.create_service(Rotmat2quat, '/' + namespace + '/rotmat2quat',
                                node.check_client_request, callback_group=node.group)
    success = False
    process = None
    with tempfile.TemporaryFile() as log:
        try:
            process = subprocess.Popen(
                ['ros2', 'run', PACKAGE, 'client', '--ros-args', '-r', '__ns:=/' + namespace],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if node.client_problems:
                    print('FAIL: client: ' + node.client_problems[0], flush=True)
                    break
                if node.client_completed >= samples:
                    print(f'PASS: client sent {samples} valid requests; server replies matched each request.', flush=True)
                    success = True
                    break
                if process.poll() is not None:
                    print(f'FAIL: client exited before completing {samples} requests.', flush=True)
                    break
            else:
                print(f'FAIL: client completed {node.client_completed}/{samples} requests in {timeout:g} seconds. '
                      'Check its service name, request sending and response handling.', flush=True)
        finally:
            if process is not None:
                stop_process(process)
            node.destroy_service(proxy)
            if not success:
                size = log.seek(0, os.SEEK_END)
                log.seek(max(0, size - 3000))
                details = log.read().decode('utf-8', errors='replace').strip()
                if details:
                    print('Client output:\n' + details, flush=True)
    return success


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--service', default='/' + PACKAGE + '/rotmat2quat')
    parser.add_argument('--server-only', action='store_true', help='Skip the client exercise')
    parser.add_argument('--timeout', type=float, default=5., help='Server discovery/request timeout in seconds')
    parser.add_argument('--client-timeout', type=float, default=20., help='Total client test timeout in seconds')
    parser.add_argument('--client-samples', type=int, default=3)
    args, ros_args = parser.parse_known_args()
    if (not np.isfinite(args.timeout) or args.timeout <= 0
            or not np.isfinite(args.client_timeout) or args.client_timeout <= 0
            or args.client_samples < 1):
        parser.error('Timeouts must be finite and positive; client samples must be at least 1')
    node = None
    rclpy.init(args=ros_args)
    try:
        node = Lab03Check(args.service)
        print('Checking service:', args.service, flush=True)
        if not check_server(node, args.timeout):
            print('RESULT: FAIL. Fix the server before checking the client.', flush=True)
            return 1
        if args.server_only:
            print('RESULT: PASS (server only; client has not been checked).', flush=True)
            return 0
        if not check_client(node, args.client_samples, args.client_timeout):
            print('RESULT: FAIL (server passed; client check failed).', flush=True)
            return 1
        print('RESULT: PASS. These sampled checks passed; also review the client output and try your own cases.', flush=True)
        return 0
    except KeyboardInterrupt:
        print('Check interrupted.', flush=True)
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
