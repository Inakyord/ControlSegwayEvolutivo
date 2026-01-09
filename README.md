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
* **Flujo de Trabajo Automatizado:** Los parámetros optimizados se guardan y cargan automáticamente mediante JSON, asegurando la consistencia entre el entrenamiento y las pruebas.
* **Control Híbrido:**
    * **Secuencial:** Transiciones suaves (LERP) entre modos de Balanceo, Avance y Giro.
    * **Reactivo:** Sistema de recuperación basado en estado que detecta caídas inminentes ($\theta > 2.5^\circ$) y aplica torques correctivos agresivos.
* **Optimización Evolutiva:** Sintonización automática de 13 parámetros de control (KPs, KDs, velocidades, umbrales) mediante un Algoritmo Genético con elitismo y torneo.
* **Feedback Visual:** Visualización en tiempo real del estado del controlador (Modo, Recuperación) mediante texto sobre el robot en la simulación.

## 📂 Estructura del Repositorio

```text
.
├── assets/                 # Modelos 3D y descripciones físicas
│   ├── segwayRobot.urdf    # Archivo URDF principal con colisiones ajustadas
│   ├── bodySegway.obj      # Malla visual del chasis
│   └── wheelSegway.obj     # Malla visual de las ruedas
│
├── src/                    # Código fuente
│   ├── evolucion.py        # Algoritmo Genético (GENERA el JSON de parámetros)
│   ├── control_manual.py   # Control por teclado (LEE el JSON)
│   └── control_automatico.py # Test autónomo de comportamiento (LEE el JSON)
│
├── resultados/             # Salida de datos generados por la evolución
│   ├── mejores_parametros.json # Archivo crítico: Contiene los genes ganadores
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

⚠️ **IMPORTANTE:** Antes de ejecutar cualquier control (manual o automático), debes ejecutar primero la evolución para generar el archivo de parámetros.

### 1. Entrenar el Algoritmo Evolutivo (Paso Obligatorio)

Inicia el proceso de optimización para encontrar las mejores ganancias PID y parámetros de comportamiento.

```bash
python src/evolucion.py

```

* **Salida:** Al finalizar (o al interrumpir con `Ctrl + C`), se generará el archivo `resultados/mejores_parametros.json`.
* **Robustez:** El script guarda el progreso automáticamente si se interrumpe manualmente.

### 2. Control Manual (Teclado)

Una vez entrenado, puedes controlar al robot utilizando los parámetros optimizados cargados desde el JSON.

```bash
python src/control_manual.py

```

**Instrucciones:**

1. Al iniciar, haz **clic con el mouse** dentro de la ventana de PyBullet para darle el foco.
2. Usa las siguientes teclas:
* `W`: Avanzar (Modo Locomoción + Impulso inicial)
* `A` / `D`: Girar Izquierda / Derecha
* `S`: Detenerse / Balanceo Estático
* `Q`: Salir



*Nota: Se han desactivado los atajos por defecto de PyBullet (como la tecla 'W' para wireframe) para mejorar la experiencia de control.*

### 3. Control Automático (Demo)

Ejecuta una secuencia de prueba autónoma con múltiples robots para validar la estabilidad y la consistencia de los parámetros aprendidos.

```bash
python src/control_automatico.py

```

## 📊 Datos y Resultados Generados

Los scripts generan y consumen archivos en la carpeta `resultados/`:

| Archivo | Descripción |
| --- | --- |
| `mejores_parametros.json` | **Crítico.** Contiene los valores exactos de Kp, Kd, velocidades y umbrales aprendidos. Es leído por los scripts de control. |
| `historial_fitness.csv` | Contiene el Mejor Fitness, Promedio y Desviación Estándar por cada generación. Útil para graficar curvas de convergencia. |
| `historial_mejor_individuo.csv` | Registro histórico de cómo evolucionaron los genes generación a generación. |
| `top_10_finales.csv` | Los 10 mejores conjuntos de parámetros encontrados en la última ejecución. |

## 📄 Referencias Teóricas

Este trabajo se fundamenta en principios de control no lineal y computación evolutiva detallados en:

1. **Alvarez-Hidalgo, L., & Howard, I. S. (2022).** *Gain scheduling for state space control of a dual-mode inverted pendulum.* IEEE ICSSE.
2. **Memarbashi, H. R., & Chang, J. Y. (2011).** *Design and parametric control of co-axes driven two-wheeled balancing robot.* Microsystem Technologies.
3. **Font, J. M., Manrique, D., & Ríos, J. (2009).** *Redes de Neuronas Artificiales y Computación Evolutiva.*

---

**Autor:** Iñaky Ordiales Caballero
