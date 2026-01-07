import os

WORKFLOW_TEMPLATE = """
# This is a faster workflow that parallelizes the jobs in a matrix so
# we can get faster results than waiting for the standard build_all_frc_projects
# powershell script

name: Build all FRC Projects

on:
  push:
    branches: [ "main" ]
  pull_request:
    branches: [ "main" ]

permissions:
  contents: read

jobs:
  build:

    strategy:
      fail-fast: false
      matrix:
        include:{projects_as_matrix}

    # The type of runner that the job will run on
    runs-on: ubuntu-latest

    # This grabs the WPILib docker container
    container: wpilib/roborio-cross-ubuntu:2025-22.04

    steps:
    - uses: actions/checkout@v4

    # Grant execute permission for gradlew
    - name: Grant execute permission for gradlew
      run: cd "${{{{ matrix.directory }}}}" && chmod +x gradlew

    # Runs a single command using the runners shell
    - name: Compile and run tests on robot code for project ${{{{ matrix.project-name }}}}
      run: cd "${{{{ matrix.directory }}}}" && ./gradlew build

  build-python:

    strategy:
      fail-fast: false
      matrix:
        python_version: ['3.12', '3.13']
        os: ['ubuntu-22.04', 'macos-latest', 'windows-latest']
        project-name: [{python_projects}]

    runs-on: ${{{{ matrix.os }}}}

    steps:
    - uses: actions/checkout@v4
    - uses: actions/setup-python@v4
      with:
        python-version: ${{{{ matrix.python_version }}}}
    - name: Install python dependencies
      run: |
        pip install -U pip
        pip install -r requirements.txt
    - name: Test ${{{{ matrix.project-name }}}}
      run: |
        cd "python/${{{{ matrix.project-name }}}}" && python3 -m robotpy test
"""

PROJECT_MATRIX_TEMPLATE = """
          - project-name: '{project_name}'
            directory: '{project_dir}'"""

MATRIX_TEMPLATE = "'{project_name}'"

PROJECTS_TO_SEARCH = ["cpp", "java"]
PYTHON_PROJECTS_TO_SEARCH = ["python"]


project_matrix = []
for project_dir in PROJECTS_TO_SEARCH:
    # Find every project in here and build up an array of strings to generate the workflow file
    for project in os.listdir(project_dir):
        project_matrix.append(PROJECT_MATRIX_TEMPLATE.format(project_name=project, project_dir=f"{project_dir}/{project}"))

python_project_matrix = []
for project_dir in PYTHON_PROJECTS_TO_SEARCH:
    # Find every project in here and build up an array of strings to generate the workflow file
    for project in os.listdir(project_dir):
        python_project_matrix.append(MATRIX_TEMPLATE.format(project_name=project))

with open(".github/workflows/build-all-parallel.yml", "w", encoding="utf-8") as workflow_file:
    workflow_file.write(WORKFLOW_TEMPLATE.format(projects_as_matrix="".join(project_matrix),
                                                 python_projects=", ".join(python_project_matrix)))
