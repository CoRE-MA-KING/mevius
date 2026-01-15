# MEVIUS Refactoring Policy

This document outlines the policy for large-scale refactoring of the MEVIUS software codebase. The primary goals are to improve maintainability, extensibility, and reusability.

---

## 1. Comprehensive Utilization of the ROS 2 Parameter System

### Current Issues
Currently, many parameters (e.g., motor IDs, control gains, model paths) are hard-coded within Python files, particularly in `mevius/mevius_utils/parameters/parameters.py`. This lacks flexibility, requiring code editing and rebuilding for any parameter changes.

### Proposed Improvements
-   **Migrate to YAML:** Move hard-coded parameters to ROS 2's standard YAML file format.
-   **Dynamic Loading:** Refactor each ROS 2 node to dynamically load its parameters from the parameter server upon startup.
-   **Launch File Integration:** Enable loading of different parameter files from launch files, allowing for easy switching between configurations (e.g., "high-speed mode," "stability-focused mode").

### Expected Benefits
-   **Separation of Code and Configuration:** Enables tuning of robot behavior without recompiling the code.
-   **Enhanced Flexibility:** Simplifies adaptation to different environments and experimental setups.

---

## 2. Establishment of a Comprehensive Test Suite

### Current Issues
The `test` directory currently contains only static analysis tests (e.g., `flake8`), with no unit or integration tests to verify code logic. This makes it difficult to detect regressions during refactoring or feature additions.

### Proposed Improvements
-   **Unit Testing:** Introduce `pytest` to create tests that verify the logic of individual classes and functions, especially for kinematics calculations and state transition logic.
-   **Integration Testing:** Implement the `launch_testing` framework to test the interaction between multiple nodes. This can validate scenarios such as "verifying that a walk command correctly generates motor command values."
-   **Continuous Integration (CI):** Set up a CI pipeline using tools like GitHub Actions to automatically run all tests whenever new code is pushed to the repository.

### Expected Benefits
-   **Improved Code Quality and Reliability:** Significantly increases confidence in the software's correctness.
-   **Safer Development:** Provides a safety net against unintended side effects, enabling developers to make changes with greater confidence.

---

## 3. Dependency Management and Transition to a Loosely Coupled Design

### Current Issues
The `main` function in `mevius_main.py` manually handles the instantiation and dependency injection of numerous nodes. This creates tight coupling between nodes, making it difficult to reuse or replace individual components.

### Proposed Improvements
-   **Clarify Data Flow:** Transition from passing shared states like `RobotState` and `RobotCommand` as direct Python objects to communicating them via ROS 2 topics, services, or actions.
-   **Utilize ROS 2 Lifecycle Nodes:** Adopt ROS 2's standard lifecycle management for nodes to create a more robust and predictable startup and shutdown sequence.
-   **Separate Launch Responsibilities:** Move the logic for switching between simulation and real-world execution (currently `if args.sim:`) out of the `main` function and into dedicated launch files.

### Expected Benefits
-   **Increased Node Independence:** Facilitates easier reuse and testing of individual nodes.
-   **Enhanced Debugging:** Allows for visual tracking of system data flow using tools like `rqt_graph`.
-   **Robust System Management:** Leads to a safer and clearer startup/shutdown process.

---

## 4. Documentation Enhancement and Automation

### Current Issues
Many classes and functions lack docstrings, making it difficult to understand the code's intent and usage.

### Proposed Improvements
-   **Complete Docstrings:** Add comprehensive docstrings to all public classes, methods, and functions, clearly describing their roles, arguments, and return values.
-   **Automated API Documentation:** Introduce `Sphinx` to automatically generate an API reference and other documentation from the docstrings.

### Expected Benefits
-   **Improved Readability and Maintainability:** Makes the codebase easier to understand and modify.
-   **Faster Onboarding:** Helps new developers get up to speed with the project more quickly.
