import os

import numpy as np
import mujoco
import mediapy as media

os.environ['MUJOCO_GL'] = "egl"


def main(argv=None):
    def get_image(data: mujoco.MjData) -> np.ndarray:
        """Renders the environment state."""
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        return renderer.render()

    # Load XML File:
    filename = "models/quadruped.xml"
    filepath = os.path.join(
        os.path.dirname(
            os.path.dirname(__file__),
        ),
        filename,
    )
    model = mujoco.MjModel.from_xml_path(filepath)
    data = mujoco.MjData(model)

    # Setup Camera:
    camera = mujoco.MjvCamera()
    camera.lookat[0] = 0.0
    camera.lookat[1] = 0.0
    camera.lookat[2] = 0.1
    camera.distance = 2.0
    camera.azimuth = 90
    renderer = mujoco.Renderer(env.model, height=720, width=1280)

    # Set initial q and qd:
    position = np.asarray(data.qpos)
    velocity = np.zeros_like(data.qvel)
    mujoco.mj_setState(model, data, position, 2)
    mujoco.mj_setState(model, data, velocity, 3)
    images = [get_image(data)]

    # Parameters
    fps = 20
    image_dt = step_dt * fps
    num_steps = 500

    # Joint mapping:
    """
        Joint indices are parsed relative to the XML file.
        Depending on joint type the DoF is different.
        As a result the generalized cordinates are different.
        Below is a mapping from joint type to generalized cordinates.

        Example:
        [Joint Type] - [# DoF] -> [qpos, qvel]
        revolute joints - 1 DoF -> qpos = [theta], qvel = [theta_dot]
        sphere joints - 3 DoF -> qpos = [qw, qx, qy, qz], qvel = [wx, wy, wz]
        free joints - 6 DoF -> qpos = [x, y, z, qw, qx, qy, qz], qvel = [vx, vy, vz, wx, wy, wz]
        - [x, y, z] is position
        - [qw, qx, qy, qz] is a quaternion
        - [vx, vy, vz] is linear velocity
        - [wx, wy, wz] is angular velocity

    """
    front_left_idx = np.array([7, 8])
    front_right_idx = np.array([9, 10])
    back_left_idx = np.array([11, 12])
    back_right_idx = np.array([13, 14])
    q_map = np.concatenate([
        front_left_idx, front_right_idx,
        back_left_idx, back_right_idx,
    ])
    qd_map = q_map - 1

    # Control Values:
    kp = 10.0
    kd = 2 * np.sqrt(kp)

    for simulation_step in range(num_steps):
        # Calculate control:
        q = data.qpos[q_map]
        qd = data.qvel[qd_map]
        error = np.zeros_like(q) - q
        control_input = kp * error - kd * qd

        # Apply control:
        mujoco.mj_setState(model, data, control_input, 6)

        # Simulate:
        mujoco.mj_step(model, data)

        if simulation_step % fps == 0:
            images.append(get_image(data))

    # Simulate and display video.
    videopath = os.path.join(os.path.dirname(__file__), "quadruped.mp4")
    media.write_video(videopath, images, fps=1/image_dt, qp=18)


if __name__ == "__main__":
    main()
