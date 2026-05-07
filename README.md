# Racerbot Solutions

Racerbot Solutions contains ROS 2 packages with solutions for the RoboRacer labs and other learning projects.

## How to Use
1. Set up and run your [Racerbot Workspace](https://github.com/sfu-racerbot/racerbot_ws)
2. Go into the `racerbot_ws/src` directory.
```bash
cd /racerbot_ws/src
```
3. Clone this repository
```bash
git clone https://github.com/sfu-racerbot/racerbot_solutions.git
```
4. Build and run the packages as normal.

## Selective Building
Build times can get long, so it is recommended to build packages selectively.
- To build a single package:
```bash
cd /racerbot_ws
colcon build --packages-select your_package
```
- To build only the solution packages:
```bash
# Create a symbolic link to the solutions packages (only needs to be done once)
cd /racerbot_ws
ln -s src/racerbot_solutions solutions_src

# Build only the solution packages
cd /racerbot_ws
colcon build --base-paths solutions_src
```
