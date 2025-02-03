# Aplicación de inteligencia artificial a un robot humanoide para el tratamiento de estrés
![Imagen de Yaren](https://github.com/RAMEL-ESPOL/YAREN/blob/main/YarenPerfil.png)

## Ejecución del funcionamiento
### Primer paso
Habilitar permisos 
```bash
sudo chmod 666 /dev/ttyUSB0
```
### Segundo paso
Lanzamiento completo del sistema
```bash
roslaunch yaren_u2d2 yaren_full.launch
```

## Adición de movimientos
![Imagen de Yaren](https://github.com/RAMEL-ESPOL/YAREN/blob/main/InterfazMovimientos.png)
### Primer paso
Habilitar permisos 
```bash
sudo chmod 666 /dev/ttyUSB0
```
### Segundo paso
```bash
roslaunch yaren_u2d2 yaren_communication_datos.launch
```
Activar torque de los motores (Véase la imagen- sección 1).
Llevar las articulaciones, manualmente, a una posición deseada. 
Luego imprimir dichas posiciones (sección 2), para llevarlas a un txt. 
Guardar dicho txt y asociarlo en el config. 
La sección 3 es para escribir manualmente un arreglo de posiciones, y la sección 4 es para poner rutinas, es decir múltiples posiciones.




