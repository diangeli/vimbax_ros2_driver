# Copyright (c) 2024, Allied Vision Technologies GmbH
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
import argparse
from .helper import single_service_call, feature_type_dict
from .helper import print_feature_info, get_module_from_string, build_topic_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_namespace")
    parser.add_argument("feature_type", choices=["Int", "Float", "String", "Raw", "Bool", "Enum"])
    parser.add_argument("feature_name")
    parser.add_argument(
        "-m",
        "--module",
        choices=["remote_device", "system", "interface", "local_device", "stream"],
        default="remote_device",
        dest="module",
    )

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("vimbax_feature_info_get_example")

    feature_type = feature_type_dict[args.feature_type]
    feature_service_type = feature_type.info_service_type

    if feature_service_type is None:
        print(f"Feature type {args.feature_type} does not support info query")
        exit(1)

    # Build topic path from namespace and topic name
    topic: str = build_topic_path(
        args.node_namespace, f"/{feature_type.service_base_path}_info_get"
    )

    request = feature_service_type.Request()
    request.feature_name = args.feature_name
    request.feature_module = get_module_from_string(args.module)
    response = single_service_call(node, feature_service_type, topic, request)

    if response.error.code == 0:
        print_feature_info(response)
    else:
        print(f"Getting feature {args.feature_name} info failed with {response.error}")
