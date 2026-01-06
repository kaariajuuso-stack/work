import mujoco
import matplotlib.pyplot as plt

aika_askel = [0.05, 0.02, 0.01, 0.005, 0.001]

for dt in aika_askel:
    model = mujoco.MjModel.from_xml_path("kaksoisheiluri.xml")
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    
    energia = []
    aika = []

    while data.time < 120:
        mujoco.mj_step(model, data)
        mekaaninen_energia = data.energy[0] + data.energy[1]
        energia.append(mekaaninen_energia)
        aika.append(data.time)
        
    plt.plot(aika, energia, label=f"dt = {dt}")
    
plt.xlabel("aika")
plt.ylabel("energia")
plt.grid(True)
plt.legend()
plt.show()

