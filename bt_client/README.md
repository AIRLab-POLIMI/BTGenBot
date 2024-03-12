# bt_client
## Usage
- Select the task and the corresponding behavior tree in XML format from the ones available in `/bt_xml`
- Modify `main.cpp` with the selected task
```
const std::string tree_xml = "task.xml";
```
- Build and source the package
```
colcon build
source install/setup.bash
```
- Launch the client
```
ros2 launch bt_client bt.launch.py
```