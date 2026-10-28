# Copyright 2026 LimX Dynamics. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Vendor-SDK availability check with an install hint that points at this plugin's extras."""

from lerobot.utils.import_utils import is_package_available


def require_sdk(pkg_name: str, extra: str, import_name: str | None = None) -> None:
    """Raise an informative ImportError if the vendor SDK for one robot is missing."""
    if not is_package_available(pkg_name, import_name):
        raise ImportError(
            f"'{pkg_name}' is required but not installed. Install it with: "
            f"pip install 'lerobot_robot_limx[{extra}]'"
        )
