


from my_dmpbbo.my_dmpbbo import KulDMP
import matplotlib.pyplot as plt

tau = 10.0
kul_dmp = KulDMP(0.4, 0.0, 1.0, tau)

t = 0
dt = 0.001
i = 0
tau=15
while i <= tau/dt:
    i+=1
    print(kul_dmp.integrate_step(dt, 'euler'))
    print(kul_dmp.integrate_step(dt, 'kuta'))

kul_dmp.plot_state_traj('euler', show=False)
kul_dmp.plot_state_traj('kuta', show = False)
plt.show()



