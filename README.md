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

## TODOs
### Refactoring
- [ ] ros2_numpy - install from pip or move to extern (consider using a git submodule)
- [ ] move Gazebo models to velmwheel_gym/gazebo_env/models
- [ ] allow to choose between weights & biases and local tensorboard
- [ ] clean up available CLI options (remove useless ones)
- [ ] update README.md
- [ ] align all algorithms parameters
- [ ] add more tests
- [ ] add more comments

### Improvements
- [] allow to specify custom maps in a file