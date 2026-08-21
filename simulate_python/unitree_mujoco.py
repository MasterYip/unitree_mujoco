import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config
from camera_follow import CameraFollower


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

camera_follower = None
camera_follow_body_id = None
camera_last_time = None
if config.CAMERA_FOLLOW:
    camera_follow_body = config.CAMERA_FOLLOW_BODY or ("pelvis" if config.ROBOT == "g1" else "base_link")
    try:
        camera_follow_body_id = mj_model.body(camera_follow_body).id
    except KeyError as exc:
        raise ValueError(f"camera follow body {camera_follow_body!r} does not exist in the loaded model") from exc
    camera_follower = CameraFollower(
        offset=tuple(config.CAMERA_LOOKAT_OFFSET),
        azimuth=config.CAMERA_AZIMUTH,
        follow_yaw=config.CAMERA_FOLLOW_YAW,
        smoothing_tau=config.CAMERA_SMOOTHING_TAU,
    )


if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    global camera_last_time
    while viewer.is_running():
        locker.acquire()
        if camera_follower is not None:
            if camera_last_time is not None and mj_data.time < camera_last_time:
                camera_follower.reset()
            dt = 0.0 if camera_last_time is None else mj_data.time - camera_last_time
            camera_last_time = mj_data.time
            lookat, azimuth = camera_follower.update(
                mj_data.xpos[camera_follow_body_id], mj_data.xquat[camera_follow_body_id], dt
            )
            viewer.cam.lookat[:] = lookat
            viewer.cam.azimuth = azimuth
            viewer.cam.distance = config.CAMERA_DISTANCE
            viewer.cam.elevation = config.CAMERA_ELEVATION
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
