# 7. Engineering: build, tests, CI, tooling

## Workspace

A colcon workspace of five packages:

| Package | Build type | Contents |
|---|---|---|
| `wdr_interfaces` | ament_cmake (rosidl) | `DeliverOrder` action, `OrderEvent`, `OperatorConfirm`, `SetBatteryLevel` |
| `wdr_mission` | ament_cmake, C++17 | task manager, BT plugin library, order queue, station registry |
| `wdr_sim` | ament_cmake, C++17 | battery simulator, obstacle mover |
| `wdr_bringup` | ament_cmake | launch files, Nav2 config, generated world/map/masks, URDF, RViz config, integration tests |
| `wdr_tools` | ament_python | `submit_orders` CLI, bag metrics |

C++ is compiled with `-Wall -Wextra -Wpedantic -Werror`. ROS-free logic (queue, registry,
battery model, path player, metrics maths) sits in small libraries so it is unit tested without a
ROS graph.

## Building and running in WSL2

```bash
bash tools/wsl/build.sh            # colcon build into ~/wdr_ws, straight from the Windows checkout
bash tools/wsl/build.sh --test     # build, then colcon test + test-result
bash tools/wsl/stop.sh             # stop leftover Gazebo / Nav2 processes
```

## Tests

| Level | What | Where |
|---|---|---|
| Unit (gtest) | order queue ordering and cancel, station validation, failure-phase → error code, battery energy model, obstacle paths, each custom BT node in a real BT.CPP factory (including that `Script` agrees with C++ `bool` flags) | `wdr_mission/test`, `wdr_sim/test` |
| Unit (pytest) | metrics maths, a real MCAP bag written and read back | `wdr_tools/test` |
| Consistency (pytest) | generated files up to date; stations in free space and outside keep-out; keep-out margin; speed-mask pixels load as the configured percent | `wdr_bringup/test/test_generated_files.py` |
| Integration (launch_testing) | whole stack on Nav2's loopback simulator, headless: rejection of bad orders, cancel of a queued order, an order delivered end to end, low battery → dock → charge → undock → deliver; every process exits cleanly | `wdr_bringup/test/test_delivery_loopback.py` |
| Scenario (manual) | Gazebo run with moving obstacles, recorded and analysed with the metrics report | README, "Results" |

The launch test's shutdown check allows exit code -6 for the Nav2 container only (a known
teardown crash, [engineering log](engineering-log.md) entry 15); any other crash fails it.

## CI (`.github/workflows/ci.yml`)

- **build-and-test** in the `ros:jazzy` container: `rosdep install` from the package manifests
  (skipping Gazebo and RViz, which the CI tests do not need), `colcon build`, `colcon test`
  including the loopback end-to-end test, then **clang-format** and **clang-tidy** (pinned
  23.1.3, same as robot-sim) on our C++ sources. Test logs are uploaded on failure.
- **docker** (on `main` and on demand): builds the dev image and runs the unit tests inside it.

## Docker / dev container

`docker/Dockerfile` (`osrf/ros:jazzy-desktop`): stage `deps` installs everything the
`package.xml` files declare via rosdep (so the image cannot drift from the manifests), stage
`build` adds the workspace. `.devcontainer/devcontainer.json` uses the `deps` stage with the
repository mounted, for VS Code or Codespaces.

## Formatting and static analysis

`.clang-format` (Google-based, 4-space indent, 120 columns) and `.clang-tidy` (bugprone,
clang-analyzer, performance and selected modernize/readability checks, warnings as errors,
own headers only). `bash tools/lint.sh` runs both like CI; `--fix` reformats.
