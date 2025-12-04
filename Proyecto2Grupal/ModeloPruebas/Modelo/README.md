# Sistema de Identificación de Modelos - Motor DC

Sistema modular para identificación de modelos de sistemas de primer orden usando Python.

## 📁 Estructura del Proyecto

```
Modelo/
├── config.py                  # Configuración global del sistema
├── data_loader.py            # Carga y procesamiento de datos
├── model_identification.py   # Métodos de identificación
├── frequency_analysis.py     # Análisis de frecuencias
├── visualization.py          # Generación de gráficas
├── export_utils.py           # Exportación a diferentes formatos
├── main.py                   # Script principal
├── experiments.py            # Experimentos y comparaciones
└── README.md                 # Este archivo
```

## 🚀 Uso Rápido

### Ejecución básica:
```bash
python main.py
```

### Ejecutar experimentos:
```bash
python experiments.py
```

## 📊 Módulos

### 1. **config.py**
Centraliza todos los parámetros configurables:
- Parámetros del experimento (voltaje, archivo de datos)
- Parámetros de datos de ejemplo
- Límites de ajuste de curva
- Configuración de visualización
- Opciones de exportación

**Ejemplo de uso:**
```python
from config import VOLTAJE_ESCALON, ARCHIVO_DATOS
```

### 2. **data_loader.py**
Funciones para carga y validación de datos:
- `cargar_datos(archivo)` - Carga desde CSV o genera ejemplo
- `cargar_datos_csv(archivo)` - Carga archivo CSV específico
- `generar_datos_ejemplo()` - Genera datos sintéticos para pruebas
- `suavizar_datos(velocidad)` - Aplica filtro de media móvil
- `validar_datos(tiempo, velocidad)` - Valida integridad de datos
- `guardar_datos_csv(tiempo, velocidad, archivo)` - Guarda datos

**Ejemplo de uso:**
```python
from data_loader import cargar_datos, validar_datos

tiempo, velocidad, dt = cargar_datos('datos_experimento.csv')
validacion = validar_datos(tiempo, velocidad)
```

### 3. **model_identification.py**
Implementa métodos de identificación:
- `identificar_por_ajuste()` - Ajuste de curva (curve fitting)
- `identificar_por_63()` - Método del 63.2%
- `identificar_por_tangente()` - Método de la tangente
- `comparar_metodos()` - Compara todos los métodos
- `calcular_metricas_modelo()` - Métricas de ajuste (RMSE, R², MAE)

**Ejemplo de uso:**
```python
from model_identification import identificar_por_ajuste, comparar_metodos

modelo = identificar_por_ajuste(tiempo, velocidad, voltaje=5.0)
print(f"K = {modelo['K']}, τ = {modelo['tau']}")

# Comparar múltiples métodos
resultados = comparar_metodos(tiempo, velocidad, voltaje=5.0)
```

### 4. **frequency_analysis.py**
Análisis de frecuencias y discretización:
- `crear_funcion_transferencia()` - Crea sistema continuo
- `analizar_frecuencias()` - Análisis completo de Bode
- `calcular_ancho_banda()` - Frecuencia de corte (-3dB)
- `calcular_frecuencias_muestreo()` - Nyquist y recomendada
- `discretizar_modelo()` - Tustin, ZOH, Euler
- `obtener_coeficientes_discretos()` - Ecuación en diferencias
- `analisis_polos_ceros()` - Ubicación de polos/ceros
- `respuesta_temporal()` - Simula respuesta al escalón

**Ejemplo de uso:**
```python
from frequency_analysis import analizar_frecuencias, discretizar_modelo

analisis = analizar_frecuencias(K=1000, tau=0.3)
print(f"Ancho de banda: {analisis['BW_hz']} Hz")

sys_discreto = discretizar_modelo(analisis['sys'], Ts=0.02, metodo='tustin')
```

### 5. **visualization.py**
Generación de gráficas:
- `plot_respuesta_temporal()` - Respuesta al escalón
- `plot_polos_ceros()` - Diagrama de polos y ceros
- `plot_error()` - Error de ajuste
- `plot_bode()` - Diagrama de Bode (magnitud y fase)
- `plot_tabla_parametros()` - Tabla resumen
- `graficar_resultados_completos()` - 6 subplots completos
- `graficar_discretizacion()` - Comparación continuo vs discreto
- `plot_comparacion_metodos()` - Compara múltiples métodos

**Ejemplo de uso:**
```python
from visualization import graficar_resultados_completos

fig = graficar_resultados_completos(tiempo, velocidad, modelo_ajuste, 
                                   modelo_63, analisis, voltaje, dt)
plt.show()
```

### 6. **export_utils.py**
Exportación a múltiples formatos:
- `exportar_numpy()` - Formato .npz
- `exportar_codigo_c()` - Código C para microcontrolador
- `exportar_codigo_python()` - Clase Python para simulación
- `exportar_arduino()` - Sketch de Arduino
- `exportar_matlab()` - Script MATLAB
- `exportar_modelo_completo()` - Todos los formatos

**Ejemplo de uso:**
```python
from export_utils import exportar_modelo_completo

exportar_modelo_completo(modelo, analisis, prefijo='motor_dc')
```

### 7. **experiments.py**
Scripts de experimentos:
- `experimento_comparar_metodos()` - Compara técnicas de identificación
- `experimento_variacion_parametros()` - Estudia robustez
- `experimento_efecto_ruido()` - Analiza impacto del ruido
- `experimento_frecuencias_muestreo()` - Efecto de fs
- `menu_experimentos()` - Menú interactivo

## 🔧 Configuración

Edita `config.py` para ajustar parámetros:

```python
# Cambiar fuente de datos
ARCHIVO_DATOS = 'mis_datos.csv'  # o None para datos de ejemplo

# Ajustar parámetros de ejemplo
EJEMPLO_K_REAL = 1000      # Ganancia RPM/V
EJEMPLO_TAU_REAL = 0.3     # Constante de tiempo (s)
EJEMPLO_DT = 0.02          # Período de muestreo (s)
EJEMPLO_RUIDO_STD = 20     # Ruido (RPM)

# Configurar exportación
EXPORT_NPZ = True
EXPORT_C = True
EXPORT_PYTHON = True
```

## 📥 Formato de Datos CSV

```csv
tiempo,velocidad
0.000,0
0.020,15
0.040,45
0.060,120
...
```

## 📤 Archivos Generados

El sistema genera automáticamente:
- `modelo_identificado.npz` - Datos NumPy
- `modelo_identificado.c` - Código C
- `modelo_identificado.py` - Clase Python
- `modelo_identificado.ino` - Sketch Arduino
- `modelo_identificado.m` - Script MATLAB

## 🧪 Ejemplos de Uso

### Ejemplo 1: Identificación básica
```python
from data_loader import cargar_datos
from model_identification import identificar_por_ajuste
from frequency_analysis import analizar_frecuencias

# Cargar datos
tiempo, velocidad, dt = cargar_datos('datos.csv')

# Identificar modelo
modelo = identificar_por_ajuste(tiempo, velocidad, voltaje=5.0)
print(f"Modelo: G(s) = {modelo['K']:.2f} / ({modelo['tau']:.4f}s + 1)")

# Analizar frecuencias
analisis = analizar_frecuencias(modelo['K_total'], modelo['tau'])
print(f"Frecuencia recomendada: {analisis['fs_recomendada']:.1f} Hz")
```

### Ejemplo 2: Comparar métodos
```python
from model_identification import comparar_metodos
from visualization import plot_comparacion_metodos

resultados = comparar_metodos(tiempo, velocidad, voltaje=5.0)
plot_comparacion_metodos(resultados, tiempo, velocidad)
```

### Ejemplo 3: Exportar para Arduino
```python
from export_utils import exportar_arduino

exportar_arduino(modelo, analisis, 'motor_control.ino')
```

## 🎯 Ventajas de la Modularización

✅ **Fácil de depurar** - Cada módulo es independiente
✅ **Reutilizable** - Importa solo lo que necesitas
✅ **Experimentación rápida** - Cambia parámetros en `config.py`
✅ **Mantenible** - Código organizado y documentado
✅ **Extensible** - Agrega nuevos métodos sin afectar el resto
✅ **Testeable** - Prueba cada módulo por separado

## 📝 Notas

- El sistema está optimizado para modelos de **primer orden**
- Los datos de ejemplo son sintéticos para pruebas
- Reemplaza `ARCHIVO_DATOS = None` con tu archivo CSV real
- Los archivos exportados están listos para usar en microcontroladores

## 🆘 Solución de Problemas

**Error: "No se pudo identificar el modelo"**
- Verifica que los datos tengan forma de respuesta al escalón
- Aumenta `AJUSTE_MAX_ITERATIONS` en config.py

**Error: "Frecuencia de muestreo insuficiente"**
- Aumenta la frecuencia de adquisición de datos
- El sistema calcula automáticamente la frecuencia recomendada

**Gráficas no se muestran**
- Verifica que tengas matplotlib instalado
- Usa `plt.show()` explícitamente

## 📦 Dependencias

```bash
pip install numpy scipy matplotlib pandas
```

## 👥 Autor

Proyecto de Control de Velocidad de Puerta Corrediza
