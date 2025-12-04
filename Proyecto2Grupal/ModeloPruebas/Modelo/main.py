"""
============================================================================
IDENTIFICACIÓN DE MODELO FÍSICO - MOTOR DC CON PUERTA CORREDIZA
Proyecto: Control de Velocidad de Puerta Corrediza
Método: Respuesta al Escalón (Step Response)

Equivalente a System Identification Toolbox de MATLAB usando Python

VERSIÓN MODULAR - Orquestador principal
============================================================================
"""

import matplotlib.pyplot as plt

# Importar módulos propios
from config import *
from data_loader import cargar_datos, validar_datos
from model_identification import identificar_por_ajuste, identificar_por_63
from frequency_analysis import analizar_frecuencias
from visualization import graficar_resultados_completos, graficar_discretizacion
from export_utils import exportar_modelo_completo


# ============================================================================
# PROGRAMA PRINCIPAL
# ============================================================================
def main():
    """Función principal para ejecutar todo el proceso de identificación"""
    
    print("="*70)
    print(" IDENTIFICACIÓN DE MODELO FÍSICO - MOTOR DC")
    print("="*70)
    
    # 1. Cargar datos
    print("\n[1/6] Cargando datos experimentales...")
    tiempo, velocidad, dt = cargar_datos(ARCHIVO_DATOS)
    
    # Validar datos
    validacion = validar_datos(tiempo, velocidad)
    if not validacion['valido']:
        print("\n⚠️  ADVERTENCIAS EN LOS DATOS:")
        for warning in validacion['warnings']:
            print(f"  {warning}")
        respuesta = input("\n¿Continuar de todos modos? (s/n): ")
        if respuesta.lower() != 's':
            print("Proceso cancelado")
            return
    
    # 2. Identificar modelo por ajuste
    print("\n[2/6] Identificando modelo por ajuste de curva...")
    modelo_ajuste = identificar_por_ajuste(tiempo, velocidad, VOLTAJE_ESCALON)
    
    if not modelo_ajuste.get('exito', False):
        print("❌ Error en la identificación")
        return
    
    print(f"  ✓ K = {modelo_ajuste['K']:.2f} RPM/V")
    print(f"  ✓ τ = {modelo_ajuste['tau']:.4f} s")
    print(f"  ✓ Polo = {modelo_ajuste['polo']:.2f} rad/s")
    print(f"  ✓ Ajuste = {modelo_ajuste['fit']:.1f}%")
    
    # 3. Identificar modelo por método 63.2%
    print("\n[3/6] Validando con método del 63.2%...")
    modelo_63 = identificar_por_63(tiempo, velocidad, VOLTAJE_ESCALON)
    
    if modelo_63.get('exito', False) and modelo_63['tau'] is not None:
        print(f"  ✓ K (63.2%) = {modelo_63['K']:.2f} RPM/V")
        print(f"  ✓ τ (63.2%) = {modelo_63['tau']:.4f} s")
    
    # 4. Analizar frecuencias
    print("\n[4/6] Analizando frecuencias y discretización...")
    analisis = analizar_frecuencias(modelo_ajuste['K_total'], modelo_ajuste['tau'])
    
    print(f"  ✓ Ancho de banda: {analisis['BW_hz']:.2f} Hz")
    print(f"  ✓ Frecuencia recomendada: {analisis['fs_recomendada']:.2f} Hz")
    print(f"  ✓ Período recomendado: {analisis['Ts_recomendado']*1000:.2f} ms")
    
    # Validar frecuencia de muestreo actual
    if 1/dt < analisis['fs_recomendada']:
        print(f"  ⚠️  Frecuencia actual ({1/dt:.1f} Hz) es menor que la recomendada")
    else:
        print(f"  ✓ Frecuencia actual ({1/dt:.1f} Hz) es adecuada")
    
    # 5. Generar gráficas
    print("\n[5/6] Generando gráficas...")
    fig1 = graficar_resultados_completos(tiempo, velocidad, modelo_ajuste, modelo_63, 
                                        analisis, VOLTAJE_ESCALON, dt)
    fig2 = graficar_discretizacion(analisis['sys'], analisis['Ts_recomendado'])
    
    # 6. Exportar resultados
    print("\n[6/6] Exportando modelo...")
    exportar_modelo_completo(modelo_ajuste, analisis)
    
    # Resumen final
    print("\n" + "="*70)
    print(" RESUMEN FINAL")
    print("="*70)
    print(f"\nModelo identificado:")
    print(f"  G(s) = {modelo_ajuste['K']:.2f} / ({modelo_ajuste['tau']:.4f}s + 1)")
    print(f"\nPolo: s = {modelo_ajuste['polo']:.2f} rad/s")
    print(f"Ajuste: {modelo_ajuste['fit']:.1f}%")
    print(f"\nFrecuencia de muestreo recomendada: {analisis['fs_recomendada']:.2f} Hz")
    print(f"Período de muestreo: {analisis['Ts_recomendado']*1000:.2f} ms")
    print("\n✓ Proceso completado exitosamente!")
    print("\nPresiona Ctrl+C para salir o cierra las ventanas de gráficas.")
    
    plt.show()


if __name__ == "__main__":
    main()
