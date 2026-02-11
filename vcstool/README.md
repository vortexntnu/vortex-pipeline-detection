# vcstool - Version Control System Tool

This directory contains configuration for managing external repository dependencies using [vcstool](https://github.com/dirk-thomas/vcstool).

## What is vcstool?

vcstool is a version control system (VCS) tool designed to work with multiple repositories. It's commonly used in ROS projects to manage workspace dependencies. Think of it as a more powerful alternative to git submodules, especially suited for ROS workspaces.

## Why use vcstool?

- **Simplified dependency management**: Define all external repositories in a single `.repos` file
- **Flexible version control**: Support for git, hg, svn, and bzr
- **Workspace-friendly**: Designed to work seamlessly with ROS colcon/catkin workspaces
- **Easy updates**: Update all dependencies with a single command

## Installation

Install vcstool with one of the following methods:

```bash
# Ubuntu/Debian
sudo apt-get install python3-vcstool

# or via pip
pip install vcstool
```

## Usage

### Initial setup (after cloning this repository)

Run the import script to clone all external dependencies:

```bash
./vcstool/import_repos.sh
```

This will import all repositories defined in `dependencies.repos` into your workspace.

### Manual import

Alternatively, you can manually import the repositories:

```bash
vcs import < vcstool/dependencies.repos
```

### Updating dependencies

To update all imported repositories to their latest versions:

```bash
vcs pull
```

### Checking status

To see the status of all repositories:

```bash
vcs status
```

## Repository Configuration

Dependencies are defined in `dependencies.repos`. The file format is:

```yaml
repositories:
  repository-name:
    type: git
    url: git@github.com:organization/repository.git
    version: branch-or-tag
```

### Adding new dependencies

1. Edit `dependencies.repos`
2. Add the new repository configuration
3. Run `./vcstool/import_repos.sh` or `vcs import < vcstool/dependencies.repos`

## Current Dependencies

- **vortex-utils**: Shared utilities for Vortex projects

## Further Reading

- [vcstool documentation](https://github.com/dirk-thomas/vcstool)
- [ROS 2 vcstool tutorial](https://docs.ros.org/en/humble/Tutorials/Workspace/Workspace.html)
