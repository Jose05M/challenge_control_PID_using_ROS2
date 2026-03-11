import pandas as pd
import matplotlib.pyplot as plt

# =========================
# Cargar datos
# =========================

data = pd.read_csv("square.csv")

t = data["time"].values
sp = data["setpoint"].values
u = data["control"].values
vel = data["velocity"].values

# Normalizar tiempo
t = t - t[0]

# =========================
# seleccionar ventana de tiempo
# =========================

t_max = 20   # segundos que quieres mostrar

mask = t <= t_max

t = t[mask]
sp = sp[mask]
u = u[mask]
vel = vel[mask]

error = sp - vel

# =========================
# Graficar
# =========================

plt.figure(figsize=(10,6))

plt.plot(t, sp, 'k--', label="Setpoint")
plt.plot(t, vel, 'b', label="Velocity")
plt.plot(t, u, 'g', label="Control signal")
plt.xlabel("Time (s)")
plt.ylabel("Normalized value")
plt.title("Motor control signals")
plt.grid()
plt.legend()

plt.figure(figsize=(10,6))
plt.plot(t, error, 'r', label="Tracking error")
plt.xlabel("Time (s)")
plt.ylabel("Error")
plt.title("Tracking error")
plt.grid()
plt.legend()

plt.show()
