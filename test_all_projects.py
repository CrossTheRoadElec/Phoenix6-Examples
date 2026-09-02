import os
import subprocess
import sys

PROJECTS_TO_SEARCH = ["cpp", "java"]
PYTHON_PROJECTS_TO_SEARCH = ["python"]

for project_dir in PROJECTS_TO_SEARCH:
    # Find every Java/C++ project in here and run all tests
    for project in os.listdir(project_dir):
        print(f"Testing {project_dir}/{project}")
        subprocess.run(
            ["./gradlew" if sys.platform != "win32" else "gradlew.bat", "build"],
            shell=True,
            cwd=f"{project_dir}/{project}",
            check=True,
        )
        print()

for project_dir in PYTHON_PROJECTS_TO_SEARCH:
    # Find every Python project in here and run all tests
    for project in os.listdir(project_dir):
        print(f"Testing {project_dir}/{project}")
        subprocess.run(
            ["python3", "-Wall", "-m", "robotpy", "test"],
            shell=True,
            cwd=f"{project_dir}/{project}",
            check=True,
        )
        subprocess.run(
            ["python3", "-m", "mypy", "--check-untyped-defs", "--ignore-missing-imports", "--explicit-package-bases", "."],
            shell=True,
            cwd=f"{project_dir}/{project}",
            check=True,
        )
        print()
