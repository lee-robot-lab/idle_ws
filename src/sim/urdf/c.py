import pinocchio as pin

model = pin.buildModelFromUrdf("robot.urdf")
print("nq, nv =", model.nq, model.nv)

for name in ["j1","j2","j3","j4"]:
    jid = model.getJointId(name)
    print(
        name,
        "jid=", jid,
        "idx_q=", model.idx_qs[jid],
        "idx_v=", model.idx_vs[jid],
        "nqs=", model.nqs[jid],
        "nvs=", model.nvs[jid],
    )