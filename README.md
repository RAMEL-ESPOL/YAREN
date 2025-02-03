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
Llevar las articulaciones a una posición deseada. Luego imprimir dichas posiciones, para llevarlas a un txt. Guardar dicho txt y asociarlo en el config.   




