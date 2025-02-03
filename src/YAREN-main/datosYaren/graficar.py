import matplotlib.pyplot as plt
import pandas as pd

# Carga los datos desde un archivo CSV
data = pd.read_csv("datos_motor.csv")

# Crear las gráficas
fig, axs = plt.subplots(2, 1, figsize=(10, 6))

# Posición
axs[0].plot(data["Tiempo"], data["Posición"], label="Posición [°]")
axs[0].set_title("Posición del Motor")
axs[0].set_ylabel("Posición [°]")

# Voltaje
axs[1].plot(data["Tiempo"], data["Voltaje"], label="Voltaje [V]", color="orange")
axs[1].set_title("Voltaje del Motor")
axs[1].set_xlabel("Tiempo [ms]")
axs[1].set_ylabel("Voltaje [V]")

plt.tight_layout()
plt.show()
