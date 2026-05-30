---
title: "Behavior Trees & Task-Level Safety"
sidebar_label: "8.3 Behavior Trees & Task-Level Safety"
sidebar_position: 4
description: "Safety is not just a hardware concern — robot behavior must be safe at the task planning level"
tags: ["behavior-trees", "task-planning", "safety", "ROS2"]
---

# Behavior Trees & Task-Level Safety

**Duration:** 45 min · **Level:** Intermediate · **Module:** 8. Safety & Human-Robot Interaction · **Tags:** `behavior-trees`, `task-planning`, `safety`, `ROS2`

## Overview

Safety is not just a hardware concern — robot behavior must be safe at the task planning level. Behavior Trees (BTs) provide a modular, verifiable framework for robot task execution with explicit failure handling and safety checks at every node.

## Key Insights

- Behavior Trees: directed acyclic graph of control nodes (Sequence, Fallback, Parallel) and leaf nodes (Actions, Conditions); composable and verifiable
- Safety conditions as BT nodes: "CheckHumanInWorkspace" as a condition node that returns Failure if human too close; inserted as precondition for all motion sequences
- BTlib and BehaviorTree.CPP: popular open-source BT implementations with ROS 2 integration; Nav2 (navigation) uses BTs for all decision-making
- Formal verification: BTs can be model-checked against safety properties using formal methods; verify "robot never exceeds 65N contact force" across all possible execution paths
- SMACH and FlexBE: alternative state machine frameworks; less modular than BTs but widely used; BTs preferred for safety-critical applications due to clear failure propagation
- Healthcare task example: "Deliver medication" BT: verify patient identity → check medication → navigate to patient → confirm patient alert → hand medication → log action

