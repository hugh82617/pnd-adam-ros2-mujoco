import mujoco
import config


model = mujoco.MjModel.from_xml_path(
    config.ROBOT_SCENE
)



# ===============================
# Bodies
# ===============================

print("=" * 60)
print("Bodies")
print("=" * 60)


for i in range(model.nbody):

    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        i
    )

    print(i, name)



print()



# ===============================
# Joints
# ===============================

print("=" * 60)
print("Joints")
print("=" * 60)


for i in range(model.njnt):

    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        i
    )

    print(i, name)



print()



# ===============================
# Sites
# ===============================

print("=" * 60)
print("Sites")
# ===============================

for i in range(model.nsite):

    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        i
    )

    print(i, name)



print()



# ===============================
# Geoms
# ===============================

print("=" * 60)
print("Geoms")
print("=" * 60)


for i in range(model.ngeom):

    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        i
    )

    print(i, name)
