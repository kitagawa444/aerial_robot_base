#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import argparse
import sys

import rclpy
from rclpy.node import Node

from spinal_msgs.srv import ManageFlightParameters

COMMANDS = {
    "status": ManageFlightParameters.Request.STATUS,
    "commit": ManageFlightParameters.Request.COMMIT,
    "reload": ManageFlightParameters.Request.RELOAD,
}


class ParameterDatabaseClient(Node):
    def __init__(self, service_name):
        super().__init__("spinal_parameter_database")
        self.client = self.create_client(ManageFlightParameters, service_name)


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Inspect or update the Spinal flight parameter database"
    )
    parser.add_argument("command", choices=COMMANDS)
    parser.add_argument("--service", default="/fc/parameters")
    parser.add_argument("--timeout", type=float, default=5.0)
    parsed = parser.parse_args(args)

    rclpy.init()
    node = ParameterDatabaseClient(parsed.service)
    try:
        if not node.client.wait_for_service(timeout_sec=parsed.timeout):
            print(f"service unavailable: {parsed.service}", file=sys.stderr)
            return 2

        request = ManageFlightParameters.Request()
        request.command = COMMANDS[parsed.command]
        future = node.client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=parsed.timeout)
        if not future.done() or future.result() is None:
            print("parameter database request timed out", file=sys.stderr)
            return 3

        response = future.result()
        print(
            f"success={response.success} result={response.result} valid={response.valid} "
            f"dirty={response.dirty} applied={response.applied} persistent={response.persistent_storage} "
            f"schema={response.schema_version} generation={response.generation} "
            f"crc32=0x{response.crc32:08x} valid_fields=0x{response.valid_fields:08x}"
        )
        return 0 if response.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
