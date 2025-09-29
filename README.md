# ROS 2 Testing: Una Guía Práctica de Supervivencia

Este taller tiene como objetivo introducir las **mejores prácticas para diseñar, probar y mantener nodos de ROS 2 en C++**, asegurando calidad de código, facilidad de mantenimiento y confianza en los despliegues.

El taller combina teoría con ejercicios prácticos para que los asistentes puedan aplicar los conceptos directamente en su propio flujo de trabajo.

---

## 🎯 Objetivos de Aprendizaje

Al finalizar el taller, los participantes serán capaces de:

- Configurar análisis estático en sus proyectos ROS 2 para reforzar estándares de calidad.
- Diseñar nodos ROS 2 de forma que la lógica principal sea **testeable independientemente** de las interfaces ROS.
- Crear y ejecutar **tests unitarios** en C++ usando `ament_cmake_gtest` y `ament_cmake_gmock`.
- Implementar **tests de interfaces ROS 2** (publishers, subscribers, parámetros, servicios).
- Realizar **tests de integración** entre múltiples nodos.
- Conocer cómo realizar **tests end-to-end** con `rosbag` y simulación.
- Integrar todos estos pasos en un flujo de **Integración Continua** con GitHub Actions.

---

## 🖥️ Requisitos Técnicos

- Portátil con Linux, Docker y un IDE (libre elección) instalado.
- Cuenta de Github configurada.
- Uso de la terminal de Linux y comandos básicos.
- Conceptos fundamentales de ROS 2 y C++.

👉 Se recomienda revisar previamente los siguientes tutoriales oficiales sobre cómo trabajar con ROS 2:

- [ROS 2 Tutoriales Básicos](https://docs.ros.org/en/jazzy/Tutorials.html)

---

## 📋 Contenido del Taller

<!-- TODO Revisar cuando tengamos todo preparado, añadir links a los ejercicios practicos -->

> [!IMPORTANT]
> Antes del workshop, es recomendable construir y probar la imagen de `Docker` que contiene todo. Para ello, hay una [guía](./docker/README.md) preparada.

1. **Análisis Estático**

   - Configuración de `ament_lint` para linters, `uncrustify` y chequeos de dependencias.
   - Ejemplo práctico con `colcon lint`.

2. **Diseño Testeable**

   - Principios para desacoplar lógica y comunicación ROS 2.
   - Uso de **Dependency Injection** para publishers, subscribers y servicios.
   - Ejemplos de código refactorizado.

3. **Unit Testing en C++**

   - Configuración de `ament_cmake_gtest` y `ament_cmake_gmock`.
   - Ejemplo: testear un algoritmo en aislamiento.

4. **Testing de Interfaces ROS 2**

   - Cómo testear publishers/subscribers/servicios/parámetros.
   - Uso de `ament_add_ros_isolated_gtest` para evitar interferencias entre tests.
   - Ejercicio práctico: testear un nodo sencillo.

5. **Testing de Integración**

   - Validar comunicación y comportamiento entre varios nodos.
   - Ejemplo: interacción entre un nodo productor y un consumidor.

6. **End-to-End Testing**

   - Validación completa del sistema con `rosbag` y entornos de simulación.
   - Ejemplos de pipelines de testing.
   - Solo contenido teórico.

7. **Integración Continua**
   - Añadir análisis estático y tests a un workflow de GitHub Actions.
   - Ejemplo de workflow mínimo.

---

## 📦 Recursos y Herramientas

(Rellenar con links y recursos)
