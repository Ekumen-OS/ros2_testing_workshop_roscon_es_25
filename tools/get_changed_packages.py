# Copyright 2025 Ekumen, Inc.
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
import sys
from pathlib import Path


def get_changed_packages(changed_files):
    """
    Identifies unique ROS 2 packages that contain the changed files.
    """
    changed_packages = set()
    for file_path in changed_files:
        p = Path(file_path)
        # Walk up the directory tree from the changed file
        while p != p.parent:
            # If we find a package.xml, we've found the package root
            if (p / "package.xml").is_file():
                changed_packages.add(p.name)
                break
            p = p.parent
    return changed_packages


if __name__ == "__main__":
    # The first argument (sys.argv[0]) is the script name, so we skip it.
    # The changed files are passed as space-separated arguments.
    files = sys.argv[1:]
    packages = get_changed_packages(files)
    # Print the package names as a space-separated string for colcon
    print(" ".join(sorted(list(packages))))
