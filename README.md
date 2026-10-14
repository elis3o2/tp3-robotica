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

El `launch` inicia la reproducción del ROS Bag, la rectificación de las imágenes estéreo y los nodos principales de procesamiento.

Los nodos principales iniciados por el `launch` son:

- `features`
- `matchings`
- `ransac`
- `triangulation`
- `disparity`
- `pose`
- `dense-reconstruction`

Los comandos auxiliares ubicados en `tp3/cmd` no se ejecutan desde el `launch`. Deben iniciarse manualmente en otra terminal mientras el sistema está corriendo.

---

## Procesamiento

### `features`

Extrae features y descriptores de las imágenes estéreo rectificadas.

### `matchings`

Calcula correspondencias entre los features de las cámaras izquierda y derecha.

### `ransac`

Filtra correspondencias espúreas mediante RANSAC y estima la matriz fundamental.

### `triangulation`

Triangula las correspondencias estéreo y genera una nube de puntos 3D.

### `disparity`

Calcula el mapa de disparidad a partir de las imágenes estéreo rectificadas.

### `pose`

Estima la transformación entre la cámara izquierda y la cámara derecha utilizando la matriz esencial y `recoverPose()`.

### `dense-reconstruction`

Reconstruye de forma densa la escena 3D a partir del mapa de disparidad utilizando `reprojectImageTo3D()`.

---

## Comandos auxiliares

Los siguientes comandos se encuentran en `tp3/cmd` y deben ejecutarse manualmente.

### Control del ROS Bag

```bash
ros2 run tp3 bag-control
```

### Visualización de imágenes y features

```bash
ros2 run tp3 view-cameras
ros2 run tp3 view-features
```

- `view-cameras`: visualiza las imágenes de las cámaras.
- `view-features`: visualiza los features detectados.

### Visualización de matchings

```bash
ros2 run tp3 view-matchings
ros2 run tp3 view-matchings30
ros2 run tp3 view-matchings-ransac
```

- `view-matchings`: visualiza los matchings entre ambas imágenes.
- `view-matchings30`: visualiza únicamente los matchings con distancia menor a 30.
- `view-matchings-ransac`: visualiza los matchings que sobrevivieron al filtrado RANSAC.

### Geometría epipolar

```bash
ros2 run tp3 view-epilines
```

- `view-epilines`: visualiza las líneas epipolares obtenidas a partir de la matriz fundamental.

### Disparidad

```bash
ros2 run tp3 view-disparity
```

- `view-disparity`: visualiza el mapa de disparidad.

### Pose y movimiento de la cámara

```bash
ros2 run tp3 plot-cams-pose
ros2 run tp3 plot-cam0-path
ros2 run tp3 plot-cam0-path-PnP
```

- `plot-cams-pose`: visualiza la pose relativa estimada entre la cámara izquierda y la cámara derecha.
- `plot-cam0-path`: estima el movimiento de la cámara izquierda entre frames utilizando la matriz esencial y `recoverPose()`.
- `plot-cam0-path-PnP`: estima el movimiento de la cámara izquierda mediante correspondencias 3D-2D y PnP.

### Mapping

```bash
ros2 run tp3 plot-map
ros2 run tp3 plot-map-dense
```

- `plot-map`: construye un mapa 3D utilizando las nubes trianguladas y las poses ground-truth.
- `plot-map-dense`: construye un mapa 3D denso utilizando las nubes de la reconstrucción densa y las poses ground-truth.

---

## Visualización con RViz

### Nube triangulada

Visualiza la nube de puntos 3D generada a partir de las correspondencias estéreo:

```bash
rviz2 -d src/tp3/config/triangulacion.rviz
```

### Nube triangulada luego de RANSAC

Visualiza la nube obtenida a partir de las correspondencias filtradas por RANSAC:

```bash
rviz2 -d src/tp3/config/triangulacion_ransac.rviz
```

### Reconstrucción densa

Visualiza la nube de puntos obtenida a partir del mapa de disparidad:

```bash
rviz2 -d src/tp3/config/reconstruccion_densa.rviz
```

### Mapping

Para visualizar el mapa global generado por `plot-map`:

```bash
rviz2 -d src/tp3/config/feature_map.rviz
```

### Dense Mapping

Para visualizar el mapa denso global generado por `plot-map-dense`:

```bash
rviz2 -d src/tp3/config/dense_map.rviz
```

---

## Nueva terminal

Cada vez que se abre una nueva terminal se debe cargar ROS 2 y el workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Facultad/Robotica/tp3_ws/install/setup.bash
```

---

## Ejecutables disponibles

Los ejecutables definidos para el paquete son:

```text
features
matchings
ransac
triangulation
disparity
pose
dense-reconstruction
bag-control
view-cameras
view-features
view-matchings
view-matchings30
view-matchings-ransac
view-epilines
view-disparity
plot-cam0-path
plot-cams-pose
plot-cam0-path-PnP
plot-map
plot-map-dense
```