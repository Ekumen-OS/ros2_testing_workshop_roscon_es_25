**Table of Contents**

- [Module 1 – Linters](#module-1--linters)
  - [Objectives](#objectives)
  - [Motivation](#motivation)
  - [Comparison of Linters in ROS 2](#comparison-of-linters-in-ros-2)
    - [Configuration](#configuration)
    - [Frequent Conflicts and Incompatibilities](#frequent-conflicts-and-incompatibilities)
    - [Recommendations](#recommendations)
  - [Other Useful Tools](#other-useful-tools)
    - [Colcon lint](#colcon-lint)
    - [ROS 2 doctor](#ros-2-doctor)
  - [Exercises](#exercises)
    - [Exercise 1](#exercise-1)
      - [Definition of success](#definition-of-success)
    - [Exercise 2](#exercise-2)
      - [Definition of success](#definition-of-success-1)
  - [References](#references)
  
---

# Module 1 – Linters

In this module,the focus is on **linters** in ROS 2.

## Objectives

By the end of the module, participants will be able to:

- Configure **static analysis and formatters** in their ROS 2 projects to enforce quality standards.
- Understand the difference between formatters and static analyzers.
- Configure and run ROS 2 linters using **ament_lint_auto** and **colcon test**.
- Identify and fix common package manifest issues using **colcon lint**.

## Motivation

Linters are automated tools that **analyze source code** to flag programming errors, bugs, style inconsistencies, and suspicious constructs. Think of them as a grammar and spell checker for the code. They don't check if logic is correct, but they ensure the code adheres to a set of predefined rules, making it **more readable, consistent, and less prone to common errors**. These linters can be integrated with IDEs to automatically provide feedback and reformat code on save for example, maintaining the quality during active development as well.

Their importance in maintaining high-quality software cannot be underestimated:

- **Enforce Consistency**: In any project, especially collaborative ones, a consistent coding style is crucial for readability. Linters automatically enforce these style guides, ensuring everyone writes code that looks and feels the same.
- **Prevent Bugs**: Static analysis tools can detect potential issues, such as memory leaks, uninitialized variables, or dangerous code patterns, long before the code is ever run. This saves significant time in debugging.
- **Improve Maintainability**: Clean, standardized code is easier for current and future developers to understand, modify, and extend.
- **Automate Code Reviews**: Linters automate the tedious part of code reviews related to style and simple errors, allowing human reviewers to focus on more important aspects like architecture and logic.

In the ROS 2 ecosystem, integrating these tools is made simple through the ament build system. **Rather than having to configure each linter manually**, the ROS 2 community provides a collection of packages, typically prefixed with `ament_`, that wrap well-known C++ and Python linters. For example:

- `ament_clang_format` wraps the popular `clang-format` tool for code formatting.
- `ament_cppcheck` integrates the static analyzer `cppcheck`.
- `ament_cpplint` uses Google's `cpplint` style checker.

These ament packages provide a standardized way to add, configure, and execute these checks as part of the normal development workflow using tools like `colcon`. This seamless integration makes it easy to maintain high code quality and is a fundamental part of building **robust, production-ready ROS 2 applications**.

## Comparison of Linters in ROS 2

In C++ ROS 2 projects, it's common to combine formatters (for style) with static analysis tools (to detect errors). The following table explains the most used options:

| Tool / Linter          | What it does / Type of check                                          | Main advantages                                                                                              | Limitations / Risks                                                                                                            | Integration comments                                                           |
| ---------------------- | --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------ |
| **ament_uncrustify**   | Formats code (spaces, indentation, braces, line breaks)               | Highly configurable; can be adapted to fine-grained style rules                                              | If used with another formatter (e.g., clang-format), conflicts can arise—each might adjust something the other "didn't expect" | Part of the set of linters that `ament_lint_auto` can activate                 |
| **ament_clang_format** | Formatting based on the Clang / LLVM AST                              | Less "cryptic" configuration, widely used in the C++ community                                               | Its style can clash with uncrustify configurations if both are used                                                            | Many IDEs natively support clang-format, useful in CI                          |
| **ament_cppcheck**     | Static analysis of C++ code (looks for risky patterns, common errors) | Detects warnings that the compiler and formatter do not see                                                  | Does not fix the code, only reports; can generate false positives if the rules are not adjusted                                | Integrates with `ament_lint` as a code analysis tool                           |
| **ament_cpplint**      | Style checker based on Google's C++ style guide                       | Good reinforcement of "conventional" style                                                                   | Does not correct the code automatically                                                                                        | Can be complemented by clang-format by formatting the code first               |
| **ament_clang_tidy**   | Advanced, compiler-based static analysis                              | Very powerful; detects complex bugs, performance issues, and modern C++ best practices. Highly configurable. | Can be slow to run and requires more complex configuration (.clang-tidy file).                                                 | An excellent choice for projects requiring the highest level of code scrutiny. |
| **ament_xmllint**      | Checks XML files (`package.xml`, launch files) for syntax errors      | Ensures configuration files are valid, preventing runtime parsing errors                                     | Doesn't validate the content's meaning (e.g., if a dependency exists), only the XML structure                                  | Essential for CI to validate package manifests and launch files                |

---

### Configuration

The `ament_lint` packages provide default configurations for the linters they wrap, and these can change between distros (although it's not common to do so). This can affect CI/CD pipelines, so for production projects, it's recommended to create custom configuration files.

To enforce a specific style in the linters, these configuration files can be added to the root of the package, for example, a `.clang-format` file. This file tells `ament_clang_format` how to format the code. A simple configuration might look like this:

```yaml
BasedOnStyle: Google
IndentWidth: 2
ColumnLimit: 120
```

The official docs also provide an official guide on the defined guidelines for each language (see [references](#references) for more information), helping to configure correctly the linters.

---

### Frequent Conflicts and Incompatibilities

- **ament_uncrustify vs ament_clang_format**:
  It's not feasible to use both with their default configurations, because `ament_uncrustify --format` can produce changes that cause `ament_clang_format` to complain, and vice versa.
- **Overlapping warnings**:
  Tools like cppcheck and others (clang-tidy or LLVM analysis) can produce similar warnings. Using them together without filtering can generate a lot of "noise" that ends up being ignored.
- **False positives**:
  Some warnings may not correspond to real errors, especially in code conditioned by macros, templates, or specific optimizations. If these false positives become a problem, the linter warnings can be suppressed (or even if the "correct" code according to the linter is not better than the current one). Some examples of how to do this:
    - C++ (clang-tidy): `// NOLINT` or `// NOLINTNEXTLINE` at the end of the line.
    - C++ (cpplint): `// NOLINT(category/rule_name)` to be more specific.
    - Python (flake8): `# noqa` at the end of the line, or `# noqa: E501` to ignore a specific error code (like line length).

---

### Recommendations

Taking into account the information above, plus experiences seen from some users, here are some recommendations:

1. **Choose a single main formatter** for the practical exercises:
   - **`ament_clang_format`** is recommended as the default option. It's simpler to configure, widely compatible with modern tools, and has a lower risk of unexpected conflicts.
2. **Activate `ament_cpplint`** as an additional analysis tool to detect logical errors that are not caught by the compiler or the formatter.
3. If an extra style layer is required, **`ament_cppcheck`** can be added, but it is not completely necessary if clang-format is already in use.

## Other Useful Tools

### Colcon lint

`colcon lint` analyzes the package and its configuration files (package.xml, CMakeLists.txt) to detect common problems:

- Detects missing or incorrectly declared dependencies.
- Verifies package naming conventions and structure.
- Can be integrated with CI execution to prevent early compilation or integration errors.

To use it with all packages in a workspace, go to the root of the workspace and run:

```bash
colcon lint
```

---


### ROS 2 doctor

Checks the ROS 2 installation, verifying paths, environment variables, and the general state of the workspace. Simply run the following command in a terminal:

```bash
ros2 doctor
```

## Exercises

For this module, the exercises to be carried out are simple, since it will mainly be correcting style errors in the source code and completing missing dependencies, but they are useful to see how these tools can help us improve the quality of the code.

### Exercise 1

<!-- use ament_clang_format and ament_cpplint for this exercise -->

The objective of this first exercise is to run the linters on the package C++ code, detect the problems, and correct them following the style recommendations. There are 2 ways to do this:

1.  Run the linters individually one by one from the terminal: the advantage of this option is that `--reformat` can be applied on linters that support it (like `ament_clang_format`) to correct the code automatically.
2.  Use `ament_lint_auto` (recommended if Ci is going to be integrated): this is essentially a CMake function that finds and adds all linters listed as `<test_depend>` in the `package.xml`. Then, when tests are run, these linters are also executed. Another advantage is that there is no need to remember individual commands.

In our case, the second option has been chosen for simplicity and ease of use (both can be combined, first executing the linters and then reformatting directly if needed). The linters to be used are defined in the [package.xml](../package.xml), and new ones are needed, the only requisite is to tweak this configuration and add the necessary dependencies. The next step is to check that these linters run correctly.

To do this, first is necessary to build the package:

```bash
cd ~/ws
colcon build --packages-select module_1
source install/setup.bash
```

And then run the tests:

```bash
colcon test --packages-select module_1
colcon test-result --verbose
```

The task for this exercise is to analyze the inconsistencies in the current code, fix them (`--reformat` can be used directly for the style ones), and see how the linter tests pass afterwards.

#### Definition of success

Your task is complete when tests are run again, and the output of `colcon test-result --verbose` shows **0 errors** and **0 failures** for the linter tests associated with `module_1`.

The only changes that hsould be applied are thoseapplied by reformatting with the formatter, and fixing the problems highlighted with the static analysis tools.

---

### Exercise 2

The objective of this exercise is to see the utility of the other tools that are not directly for the code, but for the packages themselves. The ROS 2 package for this module has some dependencies that are missing from being explicitly defined in the package.xml.

Run colcon lint for the module 1 package:

```bash
cd ~/ws
source install/setup.bash
colcon lint --packages-select module_1
```

Some indications will appear. The task is to correct these indications, adding the missing dependencies and ficing the style inconsistencies.

#### Definition of success

Your task is complete when the command `colcon lint --packages-select module_1` is run again and it produces **no warning output** and finishes with a **successful exit code (0)**. A clean run should look like this:

```bash
colcon lint --packages-select module_1
Starting >>> module_1
Finished <<< module_1 [0.5s]

Summary: 1 package finished [0.7s]
```

<a id="references"></a>
## References

- [ROS 2 code style guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- [ament_lint_auto docs](https://github.com/ament/ament_lint/blob/jazzy/ament_lint_auto/doc/index.rst)
- [ament_lint repo](https://github.com/ament/ament_lint/tree/jazzy) (all the available linters for ROS 2 packages)
- [ament_lint CLI utilities](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Ament-Lint-For-Clean-Code.html)
- [practical example in video](https://www.youtube.com/watch?v=2gIyu09UEC8)
