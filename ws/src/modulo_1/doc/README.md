# Módulo 1 – Linters

En este módulo trabajaremos con **linters** en ROS 2.

## Comparativa de linters en ROS 2

En proyectos de ROS 2 en C++, es habitual combinar formateadores (para estilo) con herramientas de análisis estático (para detectar errores). La siguiente tabla explica las opciones más usadas:

| Herramienta / linter   | Qué hace / tipo de chequeo                                                  | Ventajas principales                                                  | Limitaciones / riesgos                                                                                                                                | Comentarios de integración                                          |
| ---------------------- | --------------------------------------------------------------------------- | --------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| **ament_uncrustify**   | Formatea código (espacios, indentación, llaves, saltos de línea)            | Muy configurable; puede adaptarse a reglas finas de estilo            | Si se usa junto con otro formateador (por ejemplo, clang-format), pueden generarse conflictos — cada uno puede ajustar algo que el otro “no esperaba” | Parte del conjunto de linters que `ament_lint_auto` puede activar   |
| **ament_clang_format** | Formateo basado en el AST de Clang / LLVM                                   | Menos configuración “críptica”, ampliamente usado en la comunidad C++ | Su estilo puede chocar con configuraciones de uncrustify si se usan ambos                                                                             | Muchas IDEs soportan clang-format de forma nativa, útil en CI       |
| **ament_cppcheck**     | Análisis estático de código C++ (busca patrones de riesgo, errores comunes) | Detecta advertencias que no ve el compilador ni el formateador        | No repara el código, solo reporta, puede generar falsos positivos si no se ajustan las reglas                                                         | Se integra con `ament_lint` como herramienta de análisis de código  |
| **ament_cpplint**      | Verificador de estilo basado en convenciones de Google para C++             | Buen refuerzo de estilo “convencional”                                | No corrige el código automáticamente                                                                                                                  | Puede complementarse con clang-format formateando primero el código |

---

### Conflictos e incompatibilidades frecuentes

- **ament_uncrustify vs ament_clang_format**  
  No es viable usar ambos con sus configuraciones por defecto, porque `ament_uncrustify --format` puede producir cambios que hagan que `ament_clang_format` se queje, y viceversa.
- **Solapamiento de advertencias**  
  Herramientas como cppcheck y otras (clang-tidy o análisis LLVM) pueden producir advertencias similares. Usarlas juntas sin filtrar puede generar mucho “ruido” que termine ignorándose.
- **Falsos positivos**  
  Algunas advertencias pueden no corresponder a errores reales, especialmente en código condicionado por macros, plantillas u optimizaciones específicas.

---

### Recomendaciones

Teniendo en cuenta la información anterior, más experiencias vistas en algunos usuarios, estas son algunas recomendaciones:

1. **Elige un solo formateador principal** para los ejercicios prácticos:
   - Se recomienda **`ament_clang_format`** como la opción por defecto. Es más sencilla de configurar, ampliamente compatible con herramientas modernas y con menor riesgo de conflictos inesperados.
2. **Activa `ament_cppcheck`** como herramienta de análisis adicional para detectar errores lógicos que no se capturan con el compilador ni con el formateador.
3. Si quieres una capa de estilo extra, `ament_cpplint` puede añadirse, pero no es xompletamente necesario si ya usas clang-format.

## Otras herramientas útiles

### Colcon-lint

`colcon lint` analiza tu paquete y sus archivos de configuración (package.xml, CMakeLists.txt) para detectar problemas comunes:

- Detecta dependencias que faltan o declaradas incorrectamente.
- Verifica convenciones de nombres y estructura de paquetes.
- Puede integrarse con la ejecución de CI para evitar errores de compilación o integración tempranos.

Para poder usarlo, solo tienes que estar dentro del workspace donde está tu paquete y ejecutar:

```bash
colcon-lint
```

### ROS 2 doctor

Chequea la instalación de ROS 2, verificando paths, variables de entorno y el estado general del workspace. Simplement ejecuta el siguiente comando en tu terminal:

```bash
ros2 doctor
```

## Ejercicios

Para este módulo, los ejercicios a realizar son sencillos, ya que principalmente va a ser corregir eerores de estilo en el código fuente y completar dependencias que faltan, pero son útiles para ver cómo estas herramientas nos pueden ayudar a mejorar la calidad del código.

### Ejercicio 1

El objetivo de este primer ejercicio es ejecutar los linters sobre el código C++ del paquete, detectar los problemas y corregirlos siguiendo las recomendaciones de estilo. Hay 2 formas de hacerlo:

1. Ejecutar individualmente los linters uno a uno desde la terminal: la ventaja de esta opción es que puedes aplicar `--reformat` en linters que lo sporten (como `ament_clang_format`) para corregir el código automáticamente.
2. Usar `ament_lint_auto` (recomendado si quieres integrar con CI): integras los linters declarando en el package.xml cuáles quieres que se ejecuten, y luego cuando ejecutes los tests, estos linters se ejecutan también. Otra de las ventajas es que no necesitas recordar comandos individuales.

<!-- TODO Añadir integración con CI si al final la añadimos -->
En nuestro caso, se ha elegido la segunda opción por simplicidad y facilidad de uso. Los linters a usar están definidos en el [package.xml](../package.xml), y si queremos agregar nuevos solo hay que retocar esta configuración, y añadir las dependencias necesarias. El siguiente paso es comprobar que estos linters se ejecutan correctamente.

Para ello, primero hay que construir el paquete:

```bash
cd ~/ws
colcon build --packages-select modulo_1
source install/setup.bash
```

Y luego ejecutar los tests:

```bash
colcon test --packages-select modulo_1
colcon test-result --verbose
```

La tarea de este ejercicio es analizar las inconsistencias que tiene el actual código, arreglarlas y ver cómo pasan los tests de los linters después de hacerlo.

### Ejercicio 2

El objetivo de este ejercicio es ver la utilidad de las otras herramientas que no son directamente para el código, sino para los paquetes en sí. El paquete de ROS 2 para este módulo tiene algunas dependencias que faltan por definir explícitamente en el package.xml.

Ejecuta `colcon lint` para el paquete del módulo 1:

```bash
cd ~/ws
source install/setup.bash
colcon lint --packages-select modulo_1
```

Y verás que aparecen algunas indicaciones. La tarea es corregir esas indicaciones, y volver a ejecutar el comando para ver que, efectivamente, ya no aparecen más.
