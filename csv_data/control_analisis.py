import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# =========================
# Cargar datos
# =========================

data = pd.read_csv("step.csv")

t = data["time"].values
sp = data["setpoint"].values
u = data["control"].values
vel = data["velocity"].values

# Normalizar tiempo
t = t - t[0]

# =========================
# Analizar solo primeros 20 s
# =========================

t_max = 20
mask = t <= t_max

t = t[mask]
sp = sp[mask]
u = u[mask]
vel = vel[mask]

# =========================
# Error
# =========================

error = sp - vel

# =========================
# Valor final
# =========================

y_final = np.mean(vel[-20:])
sp_final = np.mean(sp[-20:])

# =========================
# Peak value y Peak time
# =========================

peak_value = np.max(vel)
peak_index = np.argmax(vel)
peak_time = t[peak_index]

# =========================
# Rise time (10% → 90%)
# =========================

y10 = 0.1 * sp_final
y90 = 0.9 * sp_final

t10 = t[np.where(vel >= y10)[0][0]]
t90 = t[np.where(vel >= y90)[0][0]]

rise_time = t90 - t10

# =========================
# Overshoot
# =========================

overshoot = ((peak_value - y_final) / y_final) * 100

# =========================
# Settling time (±2%)
# =========================

band = 0.02 * abs(sp_final)
settling_time = None

for i in range(len(vel)):
    if np.all(np.abs(vel[i:] - sp_final) <= band):
        settling_time = t[i]
        break

# =========================
# Steady-state error
# =========================

steady_state_error = abs(sp_final - y_final)

# =========================
# RMSE
# =========================

rmse = np.sqrt(np.mean(error**2))

# =========================
# Error máximo
# =========================

max_error = np.max(np.abs(error))

# =========================
# Resultados
# =========================

print("\n===== CONTROL PERFORMANCE =====")

print(f"Peak value: {peak_value:.2f}")
print(f"Peak time (s): {peak_time:.2f}")
print(f"Rise time (s): {rise_time:.2f}")
print(f"Overshoot (%): {overshoot:.2f}")

if settling_time is not None:
    print(f"Settling Time (s): {settling_time:.2f}")
else:
    print("Settling Time: not reached")

print(f"Steady-state error: {steady_state_error:.4f}")
print(f"RMSE: {rmse:.4f}")
print(f"Max error: {max_error:.4f}")

# =========================
# Graficar respuesta
# =========================

plt.figure(figsize=(10,6))

plt.plot(t, sp, 'k--', label="Setpoint")
plt.plot(t, vel, 'b', label="Velocity")

plt.scatter(peak_time, peak_value, label="Peak")

plt.xlabel("Time (s)")
plt.ylabel("Velocity")
plt.title("Step Response")

plt.grid()
plt.legend()

plt.show()

# =========================
# Graficar error
# =========================

plt.figure(figsize=(10,6))

plt.plot(t, error, 'r', label="Error")

plt.xlabel("Time (s)")
plt.ylabel("Error")
plt.title("Tracking Error")

plt.grid()
plt.legend()

plt.show()
