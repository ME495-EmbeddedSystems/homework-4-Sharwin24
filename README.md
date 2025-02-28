# homework-4-Sharwin24
## Manual Exploration
To run manual exploration, which will allow you to publish a goal pose in rviz using the button at the top:
```bash
# Make sure this package is next to nubot in your ws
ros2 launch nubot_nav manual_explore.launch.xml
```

[manual_exploration.webm](https://github.com/user-attachments/assets/fb4fb84b-3f03-4292-bdcc-69e201ecb6a0)


## Autonomous Exploration
2 autonomous exploration algorithms were implemented:

1. Frontier Exploration
2. "Random" Exploration

The frontier planner explained in the notes and this paper [Ref [4]](https://ieeexplore.ieee.org/document/7276723/citations?tabFilter=papers#citations). The random explorer performs a bit better than the frontier planner since donut shaped frontiers were used to simplify the implementation. To change the autonomous exploration algorithm set `explore_type` to either `random` or `frontier` when launching `explore.launch.xml`:

```bash
# Make sure this package is next to nubot in your ws
ros2 launch nubot_nav explore.launch.xml explore_type:=random
```

[autonomous_exploration.webm](https://github.com/user-attachments/assets/e8a53555-9b9c-40f3-b729-03d2b8ef8f5c)
