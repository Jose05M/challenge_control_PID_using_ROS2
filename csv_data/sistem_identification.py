import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import lstsq

# =========================
# cargar datos
# =========================

data = pd.read_csv("motor_data.csv")

t = data["time"].values
u = data["setpoint"].values
y = data["velocity"].values

t = t - t[0]

Ts = np.mean(np.diff(t))

# =========================
# IDENTIFICACIÓN ARX(1,1)
# =========================

N = len(y)

phi = []
Y = []

for k in range(1,N):

    phi.append([
        -y[k-1],
        u[k-1]
    ])

    Y.append(y[k])

phi = np.array(phi)
Y = np.array(Y)

theta, _, _, _ = lstsq(phi,Y,rcond=None)

a1 = theta[0]
b1 = theta[1]

# =========================
# CONVERSIÓN A CONTINUO
# =========================

tau = -Ts / np.log(-a1)
K = b1 / (1 + a1)

# =========================
# SIMULACIÓN MODELO CONTINUO
# =========================

y_model = np.zeros(N)

for k in range(1,N):

    dt = t[k] - t[k-1]

    y_model[k] = y_model[k-1] + dt*((-y_model[k-1] + K*u[k-1]) / tau)

# =========================
# FUNCIÓN DE TRANSFERENCIA
# =========================

transfer_function = f"G(s) = {K:.3f} / ({tau:.3f}s + 1)"

# =========================
# GRÁFICA
# =========================

plt.figure(figsize=(10,6))

plt.plot(t,u,'k--',label="Setpoint")
plt.plot(t,y,'b',label="System output")
plt.plot(t,y_model,'r',label="Continuos model")

plt.xlabel("Time (s)")
plt.ylabel("Velocity normalized")
plt.title("System identification for a DC motor")

plt.grid()
plt.legend()

textstr = '\n'.join((
    'Method: ARX(1,1)',
    transfer_function
))

plt.gca().text(
    0.60,
    0.25,
    textstr,
    transform=plt.gca().transAxes,
    fontsize=11,
    bbox=dict(facecolor='white',alpha=0.85)
)

plt.show()

print("\nFunción de transferencia identificada:\n")
print(transfer_function)
