# Migration Guide: GraspNet → GPD

This document explains the transition from the templated Python `grasp_planner_node.py` to the production GPD-based C++ implementation.

## Overview

**Old approach:** Placeholder Python node with "TODO" comments
```python
class GraspPlannerNode(Node):
    # TODO: Load GraspNet model
    # TODO: Implement grasp planning
```

**New approach:** Production C++ implementation using GPD
```cpp
class GraspPlannerNode : public rclcpp::Node {
    // Fully implemented grasp planning using GPD
    // Processes point clouds and returns 6-DOF grasps
};
```

---

## What to Do With the Old Node

The old `grasp_planner_node.py` was a template that was never fully implemented. Now it's superseded by the GPD implementation.

**Recommended:** Delete it
```bash
rm /home/elisabeth/me326-team7/ros2_ws/src/tidybot_perception/tidybot_perception/grasp_planner_node.py
```

---

## Updated Architecture

**Before:**
```
task1_retrieve.py → /plan_grasp (not implemented)
```

**After:**
```
task1_retrieve.py 
    ↓ (via grasp_planner_client.py)
/plan_grasp service
    ↓
grasp_planner_node (C++ in tidybot_gpd)
    ↓ (processes /camera/depth/cloud)
Returns grasp_pose + pre_grasp_pose
```

---

## Code Changes Required

### In task1_retrieve.py

**Before:**
```python
# Grasp planning didn't work
def plan_grasp_state(self):
    self.state = Task1State.PRE_GRASP  # Skip
```

**After:**
```python
from utilities.grasp_planner_client import GraspPlannerClient

class Task1RetrieveNode(Node):
    def __init__(self):
        super().__init__('task1_retrieve')
        self.grasp_client = GraspPlannerClient(self)
    
    def plan_grasp_state(self):
        try:
            grasp_pose, pre_grasp_pose = self.grasp_client.plan_grasp_sync(
                object_position=(self.object_x, self.object_y, self.object_z),
                object_class=self.target_object,
                arm_name="right",
                timeout=3.0
            )
            self.state = Task1State.PRE_GRASP
        except Exception as e:
            self.get_logger().error(f"Grasp failed: {e}")
            self.state = Task1State.SEARCH
```

---

## Verification

### Confirm New Node Works

```bash
# Test the new implementation
ros2 launch tidybot_gpd grasp_planner.launch.py
ros2 service call /plan_grasp tidybot_msgs/srv/PlanGrasp "{...}"

# Should return valid grasp poses
```

---

## Checklist

- [ ] Delete old placeholder: `rm tidybot_perception/grasp_planner_node.py`
- [ ] Initialize submodules and install GPD (see [gpd_setup.md](./gpd_setup.md)):

```bash
git submodule update --init --recursive
cd /home/elisabeth/me326-team7
chmod +x grasp_libraries/install_gpd.sh
./grasp_libraries/install_gpd.sh
```

- [ ] Build tidybot_gpd: `colcon build --packages-select tidybot_gpd`
- [ ] Test grasp planner standalone (see [gpd_quick_start.md](./gpd_quick_start.md))
- [ ] Update task1_retrieve.py to use new client (see code above)
- [ ] Test full integration

Done!
