# Module 6 – Continuous Integration (CI)

In this final module, the focus will be on **how to automate the building, linting, and testing processes** using Continuous Integration (CI) with GitHub Actions.

- [Module 6 – Continuous Integration (CI)](#module-6--continuous-integration-ci)
  - [Objectives](#objectives)
  - [Motivation](#motivation)
  - [What is Continuous Integration?](#what-is-continuous-integration)
    - [Introduction to GitHub Actions](#introduction-to-github-actions)
    - [Anatomy of a ROS 2 Workflow File](#anatomy-of-a-ros-2-workflow-file)
    - [Viewing CI Results](#viewing-ci-results)
    - [Enforcing CI with branch protection rules](#enforcing-ci-with-branch-protection-rules)
    - [Testing the CI in your own fork](#testing-the-ci-in-your-own-fork)
  - [References](#references)

## Objectives

By the end of the module, participants will be able to:

- Understand the purpose and benefits of Continuous Integration.
- Set up a basic CI workflow for a ROS 2 project using GitHub Actions.
- Configure a workflow to automatically run `colcon build` and `colcon test` on every pull request.
- Interpret the results of a CI run to approve or reject code changes.

## Motivation

So far, all the quality checks (linters, tests, etc.) have been run manually on the local machines. This works for solo development but quickly becomes unreliable in a team setting or for long-term projects.

What if a collaborator forgets to run the tests before pushing their code?

What if a change works on the machine but fails on a different operating system?

How can someone be sure that a new feature doesn't accidentally break an old one?

**Continuous Integration (CI)** is the practice of automating these checks.

## What is Continuous Integration?

CI is a software development practice where developers frequently merge their code changes into a central repository. After each merge, an automated build and test sequence is run. The goal is to find and address bugs quicker, improve software quality, and reduce the time it takes to validate and release new software updates.

It's important to distinguish between CI and CD. While CI focuses on testing and validation, CD (Continuous Delivery/Deployment) extends this by automatically deploying code once it passes all checks. These concepts are complementary, and a good project should have both.

Integrating CI into the workflow is essential because it:

- **Enforces Quality**: It guarantees that every pull request passes all linter and test checks before it can be merged, maintaining a high-quality main branch.
- **Prevents Regressions**: It automatically catches bugs and breaking changes, giving the team immediate feedback.
- **Automates Repetitive Work**: It saves developers from the tedious, error-prone task of manually running tests.
- **Creates a Single Source of Truth**: The CI server becomes the unbiased judge of whether code is ready to be merged.

For projects hosted on GitHub, the easiest and most popular way to implement CI is with **GitHub Actions**.

> [!TIP]
> **Use Pre-commit Hooks to Optimize Your Workflow**
>
> While **CI is the mandatory quality gate** for merging, it's often faster for developers to catch simple style errors **locally** before they push. Tools like **pre-commit hooks** (as discussed in Module 1) run fast checks like formatting locally, saving the developer time waiting for the CI pipeline to run just to fail on a style inconsistency. They complement the CI by ensuring your commits are clean and focused on functional changes.

### Introduction to GitHub Actions

GitHub Actions is a CI/CD platform built directly into GitHub. Automation workflows are defined in a **YAML file** in a special directory in the repository: `.github/workflows/`. GitHub automatically detects these files and runs them based on a set of custom-defined rules or triggers, such as when code is pushed, a pull request is opened, or a scheduled job is due.

For ROS 2, the community has created a set of pre-made "actions" that make setting up a CI workflow incredibly simple. The most important one is `ros-tooling/action-ros-ci`, which encapsulates the entire colcon build, colcon lint, and colcon test process into a single step. This one is actively maintained by the ROS 2 tooling working group, so it's a trusted source to use in the community.

There are plenty of other choices to host the CI workflow available, it always depends on the specific use case on hand. Some of the most popular ones are Jenkins, AWS, Gitlab CI/CD, Bitbucket pipelines...

> [!NOTE]
>
> - The same GitHub Actions workflow can also be executed on **self-hosted runners**, allowing the CI pipeline to run on an organization’s own infrastructure instead of GitHub’s servers.
>
> - Note that `ament_cmake` and `colcon` commands in CI behave exactly like local runs, so the developer could always **debug possible failures locally first**, before relying on CI.

### Anatomy of a ROS 2 Workflow File

Here is a complete, minimal file for a typical ROS 2 workspace. This file would be placed at `.github/workflows/ci.yml` in the repository.

```yaml
# .github/workflows/ci.yml

name: ROS 2 CI

# Controls when the workflow will run
on:
  pull_request: # Run on all pull requests
  push:
    branches: [ main ] # Also run on pushes to the main branch

jobs:
  build_and_test_ros2:
    runs-on: ubuntu-latest
    container:
      image: rostooling/setup-ros-docker:ubuntu-noble-latest
    steps:
      - name: Build and run tests
        uses: ros-tooling/action-ros-ci@v0.4
        with:
          package-name: |
            my_package_1
            my_package_2
            my_package_3
            ...
          target-ros2-distro: jazzy
          import-token: ${{ secrets.GITHUB_TOKEN }}
```

This approach is designed for a simple use case of building and testing an entire workspace at once. If the workspace is too large, it's recommended to develop a more advanced approach and run these checks only for the changed packages, to reduce the time needed for each build and make the process faster. For that, it would be required to create other helper scripts. A full example about this can be found in [this file](../../.github/workflows/specific-ci.yml).

> [!IMPORTANT]
> Always be careful when using pre-made actions from other sources, security issues might arise if these actions are updated or modified. See [this post](https://discourse.openrobotics.org/t/notice-tj-actions-changed-files-3rd-party-github-action-compromised/42540) for an example. This risk can be minimized by pinning Github actions to specific commits, to avoid them getting automatically updated in the workflow.

It's important to mention that if support for several distros (e.g., `jazzy` and `humble`) is required, it's not necessary to create separate files or jobs, Github actions has a powerful feature called a **"strategy matrix"** that can run the same job with different parameters.

Another good feature available is to configure periodic builds using these Github actions. A schedule can be added to them, specifying if they should be run nightly, weekly, monthly... This is really useful, as it makes easier the maintanance of the project, and allows to detect integration or dependency issues.

> [!TIP]
> For performance in larger projects, CI runs can significantly speed up by caching downloaded dependencies and build artifacts. The `ros-tooling/setup-ros` action supports this out-of-the-box with tools like `ccache`. This can reduce build times from several minutes to just a few.

### Viewing CI Results

Once the workflow is set up, a "check" will automatically appear on every new pull request. It's possible to see the status directly on the PR page:

- Green checkmark ✅: All checks passed! The code is safe to merge.
- Red X ❌: One or more checks failed. It's possible to click on "Details" to see the full logs and find out which colcon command failed.

### Enforcing CI with branch protection rules

A green checkmark is great, but what stops someone from merging a pull request when the tests are failing? GitHub's **branch protection rules** are the answer.

By setting up a protection rule for the `main` branch, status checks are required to pass before merging. This makes it impossible to merge code that breaks the build or fails tests. It's the final step that turns CI from a helpful suggestion into an enforceable quality gate.

The required checks can be configured in the GitHub repository under `Settings -> Branches -> Add branch ruleset`.

This simple setting is a cornerstone of maintaining a healthy, high-quality codebase in a team environment.

### Testing the CI in your own fork

This repository already includes two workflows: [complete-ci.yml](../../.github/workflows/complete-ci.yml), which builds and tests all the packages, and [specific-ci.yml](../../.github/workflows/specific-ci.yml), which only builds and tests the packages changed by a pull request.

Since the `main` branch of the workshop contains the **unsolved exercises**, its tests are expected to fail. For this reason, in the workshop repository both workflows only **build** the code when targeting `main`, and the full test suite runs on the [`solutions`](https://github.com/Ekumen-OS/ros2_testing_workshop_roscon_es_25/tree/solutions) branch. In any fork, the condition doesn't apply and the tests always run, so the CI can be used to validate the solutions to the exercises:

1. [Fork the repository](https://github.com/Ekumen-OS/ros2_testing_workshop_roscon_es_25/fork) and enable GitHub Actions in the `Actions` tab of the fork (they are disabled by default on forks).
2. Create a new branch and solve the exercises of the modules there (or start from the `solutions` branch to see what a green run looks like).
3. Push the branch and open a pull request **against the `main` branch of the fork**, not against the workshop repository.
4. Check the results of both workflows in the pull request. If an exercise is not solved correctly yet, the corresponding linter or test will fail, showing which `colcon` command failed in the logs. Note that `complete-ci` tests all the modules, so it will only turn green once every exercise is solved, while `specific-ci` only tests the packages changed in the pull request.

> [!TIP]
> Try breaking one of the solved exercises on purpose (e.g., removing a check in `module_2`) and push the change to the same pull request. The CI should turn red, and `specific-ci` should only rebuild and test the affected package.

The CI is not the only way to check the solutions: tests are never skipped locally, so the same checks can be run inside the workshop container before pushing:

```bash
colcon build
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

> [!NOTE]
> **For workshop maintainers**: since tests are skipped for the workshop `main` branch, changes to the exercises or their tests must be verified against the solved code. Merge the branch into one based on `solutions` and open a pull request against `solutions`, where the full test suite runs.

## References

- [GitHub Actions Documentation](https://docs.github.com/en/actions/get-started/quickstart)
- [ros-tooling/action-ros-ci repository](https://github.com/ros-tooling/action-ros-ci)
- [changed-files action](https://github.com/marketplace/actions/changed-files)
