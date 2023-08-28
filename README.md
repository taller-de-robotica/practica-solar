# Vigilante

Contiene el espacio de trabajo necesario para poder realizar la practica de vision 

## Como compilar y ejecutar el nodo:

### Crear espacio de trabajo 

Crea una carpeta para hospedar el espacio de trabajo para este servidor: 

````bash
mkdir -vp  ~/vigilante/src
cd ~/vigilante/src
# Clonar el espacio de trabajo a la carpeta src
git clone <repository_url> .
````

#### Instalar requerimientos de python

Hemos encontrado problemas a la hora de usar ros2 y ambientes virtuales de python, por lo que recomendamos realizar la instalación a nivel del python de usuario. 

```bash
pip install -r requirements.txt
```

#### Compilar nodos

```baash
# Sourcea ros
source source /opt/ros/<ros_version>/setup.bash

colcon build --packages-select solar_interfaces
colcon build --symlink-install --packages-select practica_solar
```
La opcion `--symlink-install `, te permitira correr tu nodo despues de realizar modificaciones al código,  sin necesidad de que vuelvas a compilarlo.

#### Ejecutar los nodos:

```
# Sourcea ros
ros2 run practica_solar vigilante
```
`Importante:` Para poder completar esta práctica es necesario que la arquitectura abajo descrita se encuentre disponible. 

## Arquitectura de la solución
![](/arquitectura.png)