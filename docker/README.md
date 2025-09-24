# ROS 2 Testing Workshop - Docker

Este contenedor proporciona un entorno reproducible para el taller **"ROS 2 Testing: Una Guía Práctica de Supervivencia"**, basado en la imagen `osrf/ros:jazzy-desktop-full`.

Incluye herramientas de desarrollo, linters, Google Test/Mock y utilidades necesarias para seguir los ejercicios.

---

## 🚀 Construcción de la Imagen

Para construir la imagen, utiliza el script `build.sh` provisto en este repositorio:

```bash
./docker/build.sh
```

O si quieres construir la imagen y ejecutar el contenedor directamente, puedes usar el script `run.sh` provisto junto a la flag correspondiente:

```bash
./docker/run.sh --build
```

<!-- TODO: Terminar documentación -->
