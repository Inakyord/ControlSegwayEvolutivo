# Optimización Evolutiva de Control PID con Planificación de Ganancias para Robot Auto-balanceado

Este repositorio contiene la implementación y simulación de un sistema de control robusto para un robot de dos ruedas (tipo péndulo invertido/Segway) utilizando el motor de física **PyBullet**.

El proyecto implementa una estrategia de **Planificación de Ganancias (Gain Scheduling)** optimizada mediante un **Algoritmo Genético (GA)** avanzado. El controlador resultante es capaz de mantener el equilibrio estático, realizar maniobras de locomoción y recuperarse de perturbaciones agresivas (empujones) mediante un modo de "pánico" no lineal.

<p align="center">
  <br>
  <img src="https://img.shields.io/badge/Simulador-PyBullet-blue" alt="PyBullet">
  <img src="https://img.shields.io/badge/Lenguaje-Python-yellow" alt="Python">
  <img src="https://img.shields.io/badge/Optimización-Genética-red" alt="GA">
  <img src="https://img.shields.io/badge/Licencia-MIT-green" alt="License">
</p>

## 📋 Características Principales

* **Simulación Física de Alta Fidelidad:** Entorno en PyBullet a 240Hz con dinámicas de fricción, inercia y colisiones ajustadas mediante calibración URDF.
* **Evaluación de Robustez Multi-Escenario:** Cada genoma es sometido a tres pruebas de estrés antes de ser calificado:
    1.  **Standard:** Inicio en reposo.
    2.  **Random Tilt:** Inicio con inclinación aleatoria no nula.
    3.  **Heavy Kick:** Aplicación de fuerzas laterales de 80N durante la operación.
* **Algoritmo Genético Avanzado:**
    * **Inicialización:** Muestreo de Hipercubo Latino (LHS) para cobertura óptima del espacio de búsqueda.
    * **Operadores:** Selección por torneo, cruce BLX-α y mutación gaussiana adaptativa.
    * **Optimización:** Sintonización automática de **16 parámetros** (KPs, KDs, KIs, Velocidades, Umbrales de disparo, Suavizado).
* **Control Híbrido:**
    * **Secuencial:** Transiciones suaves (Interpolación Lineal - LERP) entre modos de Balanceo, Avance y Giro.
    * **Reactivo:** Sistema de recuperación que detecta caídas inminentes basándose en un umbral evolutivo y cambia a ganancias de "recuperación" de alto torque.

## 📂 Estructura del Repositorio

```text
.
├── assets/                 # Modelos 3D y descripciones físicas
│   ├── segwayRobot.urdf    # Archivo URDF con etiquetas <inertial> calibradas
│   ├── bodySegway.obj      # Malla visual del chasis
│   └── wheelSegway.obj     # Malla visual de las ruedas
│
├── src/                    # Código fuente
│   ├── evolucion.py        # Motor Genético (Entrenamiento con Multiprocessing)
│   ├── control_manual.py   # Control por teclado (Infiere params del JSON)
│   ├── control_automatico.py # Demo autónoma de validación
│
├── resultados/             # Salida de datos (Gitignored recomendado excepto el JSON)
│   ├── mejores_parametros.json # Archivo crítico: El "cerebro" del robot
│   ├── historial_fitness.csv   # Data para gráficas de convergencia
│   └── top_10_individuos.csv   # Ranking de mejores soluciones
│
├── environment.yml         # Configuración del entorno (Conda)
└── requirements.txt        # Dependencias (Pip)

```

## 🛠️ Instalación

Se recomienda utilizar **Anaconda** para gestionar las dependencias de simulación.

### Opción 1: Usando Conda (Recomendado)

```bash
# Crear el entorno
conda env create -f environment.yml

# Activar el entorno
conda activate pybullet_env

```

### Opción 2: Usando Pip

```bash
pip install -r requirements.txt

```

## 🚀 Uso y Ejecución

⚠️ **IMPORTANTE:** El repositorio no incluye parámetros pre-entrenados por defecto. Debes ejecutar la evolución primero.

### 1. Entrenar el Algoritmo Evolutivo

Ejecuta el algoritmo genético. Este script utiliza `ProcessPoolExecutor` para paralelizar las simulaciones en todos los núcleos de tu CPU.

```bash
python src/evolucion.py

```

* **Salida:** Generará `resultados/mejores_parametros.json`.
* **Nota:** La evolución corre por 2000 generaciones. Puedes detenerla con `Ctrl + C` y el progreso se guardará automáticamente de forma segura.

### 2. Control Manual (Teclado)

Toma el control del robot usando los parámetros que la IA ha optimizado.

```bash
python src/control_manual.py

```

**Controles:**

* `Clic` en la ventana para enfocar.
* `W`: Avanzar (Inyecta target de velocidad y modifica el setpoint de inclinación).
* `A` / `D`: Giro diferencial.
* `S`: Freno / Balanceo en punto fijo.

### 3. Validación Autónoma

Ejecuta una demostración donde el robot pasa por las fases de Balanceo -> Avance -> Giro sin intervención humana.

```bash
python src/control_automatico.py

```

## 📊 Interpretación de Resultados

El sistema de puntuación (Fitness) no es lineal.

* **Fitness < 0:** El robot se cayó.
* **Fitness > 100:** El robot se mantiene en pie pero es estático.
* **Fitness > 1000:** El robot logra avanzar y girar con éxito.
* **Fitness Máximo:** Se logra cuando el robot resiste las patadas (Kick Scenario) y mantiene la trayectoria recta con mínima oscilación.

## 📄 Referencias y Créditos

Este proyecto combina teoría de control clásico con inteligencia computacional moderna.

* **Motor Físico:** [PyBullet Physics](https://pybullet.org/)
* **Método de Optimización:** Algoritmos Genéticos con Muestreo LHS.

---

**Autor:** Iñaky Ordiales Caballero
