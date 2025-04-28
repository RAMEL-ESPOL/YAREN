import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq
import svgwrite

# Función para leer los puntos del archivo .txt
def read_points_from_file(filename):
    points = []
    with open(filename, 'r') as file:
        for line in file:
            x, y = map(int, line.strip().split(','))
            points.append((x, y))
    return np.array(points)

# Función para ajustar una elipse a los puntos
def fit_ellipse(x, y):
    def calc_ellipse(params, x, y):
        # Parametros de la elipse: [h, k, a, b, angle]
        h, k, a, b, angle = params
        # Transformación para el ajuste elíptico
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        
        x_rot = (x - h) * cos_theta + (y - k) * sin_theta
        y_rot = -(x - h) * sin_theta + (y - k) * cos_theta
        
        return (x_rot**2) / a**2 + (y_rot**2) / b**2 - 1

    # Inicialización de parámetros para el ajuste
    h0 = np.mean(x)  # Centro en el promedio de las coordenadas x
    k0 = np.mean(y)  # Centro en el promedio de las coordenadas y
    a0 = np.max(x) - np.min(x)  # Estimación inicial de la longitud del eje mayor
    b0 = np.max(y) - np.min(y)  # Estimación inicial de la longitud del eje menor
    angle0 = 0  # Estimación inicial de ángulo
    
    params0 = [h0, k0, a0, b0, angle0]  # Parámetros iniciales

    # Ajuste de los parámetros de la elipse
    params_opt, _ = leastsq(calc_ellipse, params0, args=(x, y))

    return params_opt

# Función para generar los puntos del path de la elipse
def generate_ellipse_path(params, num_points=100):
    h, k, a, b, angle = params
    t = np.linspace(0, 2 * np.pi, num_points)
    x = h + a * np.cos(t) * np.cos(angle) - b * np.sin(t) * np.sin(angle)
    y = k + a * np.cos(t) * np.sin(angle) + b * np.sin(t) * np.cos(angle)
    return x, y

# Función para guardar el path en un archivo SVG
def save_path_to_svg(x, y, filename):
    dwg = svgwrite.Drawing(filename, profile='tiny', size=(max(x)-min(x), max(y)-min(y)))
    
    # Añadir el path de la elipse al archivo SVG
    path_data = 'M {} {}'.format(x[0], y[0])  # Movimiento inicial
    for i in range(1, len(x)):
        path_data += ' L {} {}'.format(x[i], y[i])  # Línea a los siguientes puntos
    
    path_data += ' Z'  # Cerrar el path
    
    dwg.add(dwg.Path(d=path_data, fill='none', stroke='black', stroke_width=2))
    dwg.save()

# Leer los puntos desde el archivo
filename = '15_points.txt'  # Asegúrate de cambiar el nombre del archivo
points = read_points_from_file(filename)

# Ajustar la elipse a los puntos
x, y = points[:, 0], points[:, 1]
params = fit_ellipse(x, y)

# Generar el path de la elipse ajustada
ellipse_x, ellipse_y = generate_ellipse_path(params)

# Mostrar los puntos originales y el path de la elipse
plt.figure(figsize=(6, 6))
plt.plot(x, y, 'ro', label='Puntos originales')
plt.plot(ellipse_x, ellipse_y, 'b-', label='Path ajustado (elipse)')
plt.gca().set_aspect('equal', adjustable='box')
plt.legend()
plt.title('Ajuste de Elipse a los Puntos')
plt.show()
"""
# Guardar el path en un archivo SVG
output_svg_filename = filename.split('.')[0] + '_path.svg'
save_path_to_svg(ellipse_x, ellipse_y, output_svg_filename)

print(f"Path guardado en: {output_svg_filename}")
"""