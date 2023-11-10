Toy repository showing how to simulate and visualize a mujuco simulation using Mujoco's python bindings.

`main.py` simulates a quadruped with simple PD controller driving the joint positions of the robot to the zero position.


**How to run the code:**

First create a python virtual environment
```
python3 -m venv env
source env/bin/activate
pip install --upgrade pip
pip install mujoco
```

Run the python script from the root directory of the repository:
```
python src/main.py
```

The python script will generate a visualization of the simulation with a .mp4 in the `src` folder called "quadruped.mp4". 
