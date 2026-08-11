import mujoco
import numpy as np


class ProximityRaySensor:

    def __init__(self, sensor_range=0.5):

        self.sensor_range = sensor_range

        # 保存Body和Site的ID
        self.sensor_body_id = None
        self.sensor_site_id = None

        # 5路局部坐标系射线
        # 中间 + 四周
        self.directions = np.array(
            [
                [0.0, 0.0, 1.0],
                [0.2, 0.0, 1.0],
                [-0.2, 0.0, 1.0],
                [0.0, 0.2, 1.0],
                [0.0, -0.2, 1.0],
            ],
            dtype=np.float64
        )

        # 单位化
        self.directions /= np.linalg.norm(
            self.directions,
            axis=1,
            keepdims=True
        )

    # --------------------------------------------------
    # 初始化Body和Site ID
    # --------------------------------------------------
    def init_id(self, model):

        self.sensor_body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "wristYawRight"
        )

        self.sensor_site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            "proximity_sensor"
        )

        print("Sensor body id:", self.sensor_body_id)
        print("Sensor site id:", self.sensor_site_id)

    # --------------------------------------------------
    # 测量
    # --------------------------------------------------
    def measure(
        self,
        model,
        data,
    ):

        # Site世界坐标
        origin = data.site_xpos[
            self.sensor_site_id
        ].copy()

        # Site旋转矩阵(3×3)
        rotation = data.site_xmat[
            self.sensor_site_id
        ].reshape(3, 3)

        distances = []

        for local_direction in self.directions:

            # 局部方向 -> 世界方向
            world_direction = rotation @ local_direction

            # 再单位化一次，避免数值误差
            world_direction /= np.linalg.norm(
                world_direction
            )

            geomid = np.array(
                [-1],
                dtype=np.int32
            )

            distance = mujoco.mj_ray(
                model,
                data,
                origin.astype(np.float64),
                world_direction.astype(np.float64),
                None,
                1,
                self.sensor_body_id,
                geomid
            )

            if geomid[0] < 0:

                distances.append(
                    self.sensor_range
                )

            else:

                distances.append(
                    float(
                        min(
                            distance,
                            self.sensor_range
                        )
                    )
                )

        return np.array(
            distances,
            dtype=np.float32
        )
