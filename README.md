# TP3 - Computer Vision

## Compilar

Desde la raíz del workspace:

```bash

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

## Ejecutar

Para ejecutar el proyecto se debe indicar la ruta al ROS Bag:

```bash
ros2 launch tp3 tp3.launch.py bag:=<ruta_rosbag>
```

El launch inicia la reproducción del ROS Bag y los nodos principales necesarios para procesar las imágenes.

## Nodos

```bash
ros2 run tp3 plot-features
ros2 run tp3 plot-matchings
ros2 run tp3 plot-matchings30
ros2 run tp3 plot-matchings-ransac
ros2 run tp3 plot-epilines
ros2 run tp3 plot-disparity
ros2 run tp3 plot-pose
ros2 run tp3 plot-visual-odometry
ros2 run tp3 plot-stereo-pose
```

### Procesamiento

- **`features`**: extrae features y descriptores de las imágenes estéreo.
- **`matchings`**: calcula correspondencias entre los features de las cámaras izquierda y derecha.
- **`ransac`**: filtra correspondencias espúreas mediante RANSAC y estima la matriz fundamental.
- **`triangulation`**: triangula las correspondencias y genera la nube de puntos 3D.
- **`disparidad`**: calcula el mapa de disparidad a partir de las imágenes estéreo rectificadas.
- **`stereo_pose`**: estima la transformación entre la cámara izquierda y la derecha utilizando la matriz esencial y `recoverPose()`.
- **`dense_resconstrucction`**: reconstruye de forma densa la escena 3D a partir del mapa de disparidad utilizando `reprojectImageTo3D()`.

### Visualización

- `plot-features`: visualiza los features detectados.
- `plot-matchings`: visualiza los matchings entre ambas imágenes.
- `plot-matchings30`: visualiza únicamente los matchings con distancia menor a 30.
- `plot-matchings-ransac`: visualiza los matchings que sobrevivieron al filtrado RANSAC.
- `plot-epilines`: visualiza las líneas epipolares.
- `plot-disparity`: visualiza el mapa de disparidad.
- `plot-pose`: visualiza la pose estimada.
- `plot-visual-odometry`: visualiza la trayectoria obtenida mediante odometría visual.
- `plot-stereo-pose`: visualiza la pose relativa estimada entre las cámaras estéreo.

## Visualización con RViz

Visualiza la nube de puntos 3D generada a partir de las correspondencias estéreo:

```bash
rviz2 -d src/tp3/config/triangulacion.rviz
```

Visualiza la nube original y la nube obtenida luego de filtrar las correspondencias espúreas con RANSAC:

```bash
rviz2 -d src/tp3/config/ransac.rviz
```

Visualiza la nube de puntos obtenida a partir del mapa de disparidad:

```bash
rviz2 -d src/tp3/config/reconstruccion_densa.rviz
```


## Nueva terminal

Cada vez que se abre una nueva terminal se debe cargar ROS 2 y el workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Facultad/Robotica/tp3_ws/install/setup.bash
```