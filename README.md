# Optimización Evolutiva de Control PID con Planificación de Ganancias para Robot Auto-balanceado

Este repositorio contiene la implementación y simulación de un sistema de control robusto para un robot de dos ruedas (tipo péndulo invertido/Segway) utilizando el motor de física **PyBullet**.

El proyecto implementa una estrategia de **Planificación de Ganancias (Gain Scheduling)** optimizada mediante **Algoritmos Genéticos (GA)**. El controlador resultante es capaz de mantener el equilibrio estático, realizar maniobras de locomoción y, crucialmente, recuperarse de perturbaciones agresivas mediante un modo de "pánico" no lineal.

<p align="center">
  <br>
  <img src="https://img.shields.io/badge/Simulador-PyBullet-blue" alt="PyBullet">
  <img src="https://img.shields.io/badge/Lenguaje-Python-yellow" alt="Python">
  <img src="https://img.shields.io/badge/Licencia-MIT-green" alt="License">
</p>

## 📋 Características Principales

* **Simulación Física de Alta Fidelidad:** Modelo URDF con colisiones ajustadas y dinámicas de fricción realistas en PyBullet.
* **Control Híbrido:**
    * **Secuencial:** Transiciones suaves (LERP) entre modos de Balanceo, Avance y Giro.
    * **Reactivo:** Sistema de recuperación basado en estado que detecta caídas inminentes ($\theta > 2.5^\circ$) y aplica torques correctivos agresivos.
* **Optimización Evolutiva:** Sintonización automática de 13 parámetros de control (KPs, KDs, velocidades, umbrales) mediante un Algoritmo Genético con elitismo y torneo.
* **Robustez:** El sistema es capaz de guardar el progreso del entrenamiento ante interrupciones inesperadas.

## 📂 Estructura del Repositorio

```text
.
├── assets/                 # Modelos 3D y descripciones físicas
│   ├── segwayRobot.urdf    # Archivo URDF principal con colisiones ajustadas
│   ├── bodySegway.obj      # Malla visual del chasis
│   └── wheelSegway.obj     # Malla visual de las ruedas
│
├── src/                    # Código fuente
│   ├── evolucion.py        # Algoritmo Genético para entrenar los controladores
│   └── control_manual.py   # Script para probar el mejor controlador con teclado
│
├── resultados/             # Salida de datos generados por la evolución
│   ├── historial_fitness.csv
│   ├── historial_mejor_individuo.csv
│   └── top_10_finales.csv
│
├── environment.yml         # Configuración del entorno (Anaconda)
└── requirements.txt        # Dependencias (Pip)

```

## 🛠️ Instalación

Se recomienda utilizar **Anaconda** o un entorno virtual de Python para evitar conflictos de dependencias.

### Opción 1: Usando Conda (Recomendado)

```bash
# Crear el entorno desde el archivo .yml
conda env create -f environment.yml

# Activar el entorno
conda activate pybullet_env

```

### Opción 2: Usando Pip

```bash
pip install -r requirements.txt

```

## 🚀 Uso y Ejecución

### 1. Probar el Controlador (Simulación Visual)

Para ver al robot operar con las mejores ganancias obtenidas (ya pre-cargadas en el script):

```bash
python src/control_manual.py

```

**Controles (Hacer clic en la ventana de PyBullet para enfocar):**

* `W`: Avanzar (Modo Locomoción)
* `A` / `D`: Girar Izquierda / Derecha
* `S`: Detenerse / Balanceo Estático
* `Q`: Salir

### 2. Entrenar el Algoritmo Evolutivo

Para iniciar el proceso de optimización desde cero y buscar nuevas ganancias:

```bash
python src/evolucion.py

```

* **Interrupción Segura:** Puedes detener el entrenamiento en cualquier momento presionando `Ctrl + C` en la terminal. El script finalizará la generación actual y guardará todos los resultados obtenidos hasta ese punto antes de cerrarse.

## 📊 Datos y Resultados Generados

El script de entrenamiento (`evolucion.py`) genera automáticamente archivos CSV en la carpeta `resultados/` para su posterior análisis:

| Archivo | Descripción |
| --- | --- |
| `historial_fitness.csv` | Contiene el Mejor Fitness, Promedio y Desviación Estándar por cada generación. Útil para graficar curvas de convergencia. |
| `historial_mejor_individuo.csv` | Registro de cómo evolucionaron los genes (KPs, KDs, etc.) del mejor individuo a lo largo del tiempo. |
| `top_10_finales.csv` | Los 10 mejores conjuntos de parámetros encontrados al finalizar (o interrumpir) la evolución. |

## 📄 Referencias Teóricas

Este trabajo se fundamenta en principios de control no lineal y computación evolutiva detallados en:

1. **Alvarez-Hidalgo, L., & Howard, I. S. (2022).** *Gain scheduling for state space control of a dual-mode inverted pendulum.* IEEE ICSSE.
2. **Memarbashi, H. R., & Chang, J. Y. (2011).** *Design and parametric control of co-axes driven two-wheeled balancing robot.* Microsystem Technologies.
3. **Font, J. M., Manrique, D., & Ríos, J. (2009).** *Redes de Neuronas Artificiales y Computación Evolutiva.*

---

**Autor:** Iñaky Ordiales Caballero
