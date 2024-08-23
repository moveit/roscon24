# Exercise 1

This workspace contains packages to demonstration motion planning using a UR robot assembling a hot dog.

## Basic Usage

This directory is a colcon workspace.
It contains two exercise packages, and a package containing the hot dog assets.

To build the workspace, change directories into the `exercise1` directory and build with colcon:

```bash
cd exercise1
colcon build
```

There may be build warnings for unused variables in exercise 1-1.
This is expected; these variables are used in the exercise 1-1 solution.

### Exercise 1-1

In this exercise, we have a hot dog bun and a sausage in reach of the robot.
We'd like to pick up the sausage and place it into the bun to assemble ourselves a nice snack.

Before running exercise 1-1, first source the workspace, then launch the UR:

```bash
source install/setup.bash
ros2 launch exercise 1-1 ur.launch.py
```

You should see an RViz window with the UR.

Now, you can run exercise 1-1 to try motion planning:

```bash
source install/setup.bash
ros2 launch exercise 1-1 exercise 1-1.launch.py
```

By default, planning will fail.
In exercise 1-1, we:
 1. Set the pose target to a pose defined just above the sausage
 2. Generate a motion plan and execute it
 3. Set the pose target to a pose defined just above the bun
 4. Generate a motion plan and execute it

The solution can be found [here](./src/exercise1-1/solution/main.cpp).

### Exercise 1-2

In this exercise, we have our assembled hot dog, and the robot has a bottle of mustard.
We want to add some mustard onto our hot dog.
To start off, the robot will plan to a pose just above the hot dog, similarly to how we created motion plans and executed them in exercise 1-1.

We're ready to apply the mustard, but there's one issue: we don't want to use the normal planning pipeline to generate this plan.
The normal planning pipeline will generate a path, but it may involve some unintuitive motions.
Applying mustard with these sort of motions would definitely create a mess.
We want to create a Cartesian plan to apply our mustard.

Before running exercise 1-2, first source the workspace, then launch the UR:

```bash
source install/setup.bash
ros2 launch exercise 1-2 ur.launch.py
```

You should see an RViz window with the UR.

Now, you can run exercise 1-2 to try motion planning:

```bash
source install/setup.bash
ros2 launch exercise 1-12 exercise 1-2.launch.py
```

By default, planning will fail.
In exercise 1-2, we:
 1. Configure the parameters used for Cartesian planning
 2. Generate a Cartesian motion plan and execute it

The solution can be found [here](./src/exercise1-2/solution/main.cpp).
