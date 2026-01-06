import mujoco
import matplotlib.pyplot as plt

nopeudet = [20, 50, 100]

for nopeus in nopeudet:
    model = mujoco.MjModel.from_xml_path("laatikko.xml.txt")
    data = mujoco.MjData(model)
    data.qvel[2] = -nopeus 
    
    aika = []
    paikka = []
    
    while data.time < 1:
        mujoco.mj_step(model, data)
        paikka.append(data.qpos[2])
        aika.append(data.time)
        
    plt.plot(aika, paikka, label=f"v = {nopeus} m/s")
        
plt.xlabel("aika")
plt.ylabel("paikka")
plt.grid(True)
plt.legend()
plt.show()
