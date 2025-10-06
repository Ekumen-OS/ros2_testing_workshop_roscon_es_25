# Module 6 – Continuous Integration (CI)

In this final module, the focus will be in **how to automate our building, linting, and testing processes** using Continuous Integration (CI) with GitHub Actions.

## Objectives

By the end of the module, participants will be able to:

- Understand the purpose and benefits of Continuous Integration.
- Set up a basic CI workflow for a ROS 2 project using GitHub Actions.
- Configure a workflow to automatically run `colcon build` and `colcon test` on every pull request.
- Interpret the results of a CI run to approve or reject code changes.

## Motivation

So far, all our quality checks (linters, tests, etc.) have been run manually on the local machines. This works for solo development but quickly becomes unreliable in a team setting or for long-term projects.

What if a collaborator forgets to run the tests before pushing their code?

What if a change works on your machine but fails on a different operating system?

How can you be sure that a new feature doesn't accidentally break an old one?

**Continuous Integration (CI)** is the practice of automating these checks. A CI server automatically builds and tests your code every time a change is pushed, providing a powerful safety net.

### What is Continuous Integration?

CI is a software development practice where developers frequently merge their code changes into a central repository. After each merge, an automated build and test sequence is run. The goal is to find and address bugs quicker, improve software quality, and reduce the time it takes to validate and release new software updates.

Integrating CI into your workflow is essential because it:

- **Enforces Quality**: It guarantees that every pull request passes all linter and test checks before it can be merged, maintaining a high-quality main branch.
- **Prevents Regressions**: It automatically catches bugs and breaking changes, giving you and your team immediate feedback.
- **Automates Repetitive Work**: It saves developers from the tedious, error-prone task of manually running tests.
- **Creates a Single Source of Truth**: The CI server becomes the unbiased judge of whether code is ready to be merged.

For projects hosted on GitHub, the easiest and most popular way to implement CI is with **GitHub Actions**.

## Core Concepts of CI

### Introduction to GitHub Actions

GitHub Actions is a CI/CD platform built directly into GitHub. You define your automation workflows in a **YAML file** in a special directory in your repository: `.github/workflows/`. GitHub automatically detects these files and runs them in response to events like a new pull request or a push to your main branch.

For ROS 2, the community has created a set of pre-made "actions" that make setting up a CI workflow incredibly simple. The most important one is `ros-tooling/action-ros-ci`, which encapsulates the entire colcon build, colcon lint, and colcon test process into a single step.

### Anatomy of a ROS 2 Workflow File

Here is a complete, minimal ci.yml file for a typical ROS 2 workspace. You would place this file at `.github/workflows/ci.yml` in the repository.

```yaml
# .github/workflows/ci.yml

name: ROS 2 CI

# Controls when the workflow will run
on:
  pull_request: # Run on all pull requests
  push:
    branches: [ main ] # Also run on pushes to the main branch

jobs:
  build_and_test:
    name: Build and Test
    # The type of virtual machine to run the job on
    runs-on: ubuntu-latest

    steps:
      # Checks out your repository's code
      - name: Check out repository
        uses: actions/checkout@v5

      # Sets up the ROS 2 environment
      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: jazzy

      # Runs the standard ROS 2 CI workflow (build, test)
      - name: Build and test workspace
        uses: ros-tooling/action-ros-ci@v0.4
        with:
          package-name: |
            module_1
            module_2
            module_3
            module_4
            module_5
          target-ros2-distro: jazzy
```

This approach is designed for a simple use case of building and testing an entire workspace at once. If the workspace is too large, it's recommended to develop a more advanced apporach and run these checks only for the changed packages. For that, it would be required to create other helper scripts. A full example about this can be found in [this file](../../.github/workflows/ros-ci.yml).

It's important to mention that if you want to support several distros (e.g., `jazzy` and `humble`), you don't to create separate files or jobs, Github actions has a powerful feature called a **"strategy matrix"** that can run the same job with different parameters.

> **Pro Tip for Performance**: In larger projects, you can significantly speed up your CI runs by caching downloaded dependencies and build artifacts. The `ros-tooling/setup-ros` action supports this out-of-the-box with tools like `ccache`. This can reduce your build times from many minutes to just a few.

### Viewing CI Results

Once your workflow is set up, a "check" will automatically appear on every new pull request. You can see the status directly on the PR page:

- Green checkmark ✅: All checks passed! The code is safe to merge.
- Red X ❌: One or more checks failed. You can click "Details" to see the full logs and find out which colcon command failed.

### Enforcing CI with branch protection rules

A green checkmark is great, but what stops someone from merging a pull request when the tests are failing? GitHub's **branch protection rules** are the answer.

By setting up a protection rule for your `main` branch, you can require status checks to pass before merging. This makes it impossible to merge code that breaks the build or fails tests. It's the final step that turns CI from a helpful suggestion into an enforceable quality gate.

You can configure this in your GitHub repository under `Settings -> Branches -> Add branch ruleset`.

This simple setting is a cornerstone of maintaining a healthy, high-quality codebase in a team environment.

## References

- [GitHub Actions Documentation](https://docs.github.com/en/actions/get-started/quickstart)
- [ros-tooling/action-ros-ci repository](https://github.com/ros-tooling/action-ros-ci)
- [changed-files action](https://github.com/marketplace/actions/changed-files)
