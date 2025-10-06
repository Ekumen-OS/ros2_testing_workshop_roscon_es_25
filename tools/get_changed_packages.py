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
