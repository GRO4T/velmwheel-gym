# Velmwheel Gym
Environment for developing and testing reinforcement learning algorithms
for WUT Velmwheel robot. Environment is compliant with OpenAI Gym's API (https://github.com/openai/gym)

## Setup up
Build and run WUT Velmwheel project. \
Open another terminal, go to root of the Velmwheel project and run
```bash
source source_me.bash
```
> **_NOTE:_** This is important because some of the packages needed to create the Gym environment are not installable using pip.
Alternatively you can install ROS2 Humble system-wide.

Create and activate Python's virtual environment
```bash
python3 -m venv venv
source venv/bin/activate
```
Install packages
```
pip install -r requirements.txt
```

> **_NOTE:_** The Gym environment depends on custom Gazebo message for passing contact state which is built as a part of the Velmwheel project.
