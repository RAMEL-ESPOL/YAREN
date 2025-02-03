import pandas as pd
import matplotlib.pyplot as plt

# Ruta completa al archivo
data_file = "/home/quadruped/Documents/Tesis_ESPOL_Reyes_Le-n_1/src/YAREN-main/yaren_master/data_log.txt"  # Cambia esto a la ruta completa si es necesario

# Leer el archivo y dividir los datos en columnas
column_names = ['Tiempo'] + [f'Posicion_{i+1}' for i in range(12)] + [f'Voltaje_{i+1}' for i in range(12)]
df = pd.read_csv(data_file, names=column_names)

# Separar las columnas de tiempo, posiciones y voltajes
tiempo = df['Tiempo']
posiciones = df[[f'Posicion_{i+1}' for i in range(12)]]
voltajes = df[[f'Voltaje_{i+1}' for i in range(12)]]

# Calcular el promedio de las posiciones y voltajes para cada instante
posiciones_avg = posiciones.mean(axis=1)
voltajes_avg = voltajes.mean(axis=1)

# Graficar Posiciones vs Tiempo
plt.figure(figsize=(10, 5))
plt.plot(tiempo, posiciones_avg, label='Posiciones Promedio', color='blue')
plt.title('Posiciones vs Tiempo')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posiciones (grados)')
plt.grid()
plt.legend()
plt.show()

# Graficar Voltajes vs Tiempo
plt.figure(figsize=(10, 5))
plt.plot(tiempo, voltajes_avg, label='Voltajes Promedio', color='red')
plt.title('Voltajes vs Tiempo')
plt.xlabel('Tiempo (s)')
plt.ylabel('Voltajes (V)')
plt.grid()
plt.legend()
plt.show()
