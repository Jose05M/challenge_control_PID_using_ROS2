import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# =========================
# Cargar datos
# =========================
# Usage: python3 plot_csv.py [filename.csv]   (default: square.csv, looked up in ../data/)

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
csv_name = sys.argv[1] if len(sys.argv) > 1 else "square.csv"
csv_path = Path(csv_name) if Path(csv_name).exists() else DATA_DIR / csv_name

data = pd.read_csv(csv_path)

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
