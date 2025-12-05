import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Parámetros del sistema
dt = 0.05  # Tiempo de muestreo en segundos (50ms del código)

# Configuración 1: PID Bien Ajustado
Kp_good = 90.0
Ki_good = 0.01
Kd_good = 0.5

# Configuración 2: PID Mal Ajustado
Kp_bad = 45.0
Ki_bad = 10.0
Kd_bad = 0.0

def pid_discrete_tf(Kp, Ki, Kd, dt):
    """
    Función de transferencia discreta del PID usando aproximación trapezoidal
    
    PID(z) = Kp + Ki*Ts/(2*(z-1)/(z+1)) + Kd*(2/Ts)*((z-1)/z)
    
    Simplificando:
    PID(z) = [b0*z^2 + b1*z + b2] / [z^2 + a1*z + a2]
    """
    
    # Coeficientes del numerador
    b0 = Kp + Ki*dt/2 + Kd*2/dt
    b1 = -Kp - Kd*4/dt
    b2 = Ki*dt/2 + Kd*2/dt
    
    # Coeficientes del denominador
    a0 = 1
    a1 = 0
    a2 = 0
    
    # Normalizar
    num = [b0, b1, b2]
    den = [a0, a1, a2]
    
    return num, den

def plot_pole_zero(Kp, Ki, Kd, dt, title, ax):
    """Grafica el diagrama de polos y ceros"""
    
    num, den = pid_discrete_tf(Kp, Ki, Kd, dt)
    
    # Crear sistema discreto
    system = signal.TransferFunction(num, den, dt=dt)
    
    # Calcular polos y ceros
    zeros = np.roots(num)
    poles = np.roots(den)
    
    # Graficar círculo unitario
    theta = np.linspace(0, 2*np.pi, 100)
    ax.plot(np.cos(theta), np.sin(theta), 'k--', alpha=0.3, linewidth=1)
    
    # Graficar polos y ceros
    ax.plot(np.real(zeros), np.imag(zeros), 'ro', markersize=10, 
            label='Ceros', markerfacecolor='none', markeredgewidth=2)
    ax.plot(np.real(poles), np.imag(poles), 'bx', markersize=10, 
            label='Polos', markeredgewidth=2)
    
    # Configurar gráfica
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('Parte Real')
    ax.set_ylabel('Parte Imaginaria')
    ax.set_title(title)
    ax.legend()
    ax.axis('equal')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    
    # Información textual
    info_text = f"Kp={Kp}, Ki={Ki}, Kd={Kd}\n"
    info_text += f"Ts={dt}s\n\n"
    info_text += "Ceros:\n"
    for i, z in enumerate(zeros):
        info_text += f"  z{i+1} = {z.real:.4f} + {z.imag:.4f}j\n"
    info_text += "\nPolos:\n"
    for i, p in enumerate(poles):
        if np.abs(p) < 1e-10:
            info_text += f"  p{i+1} = 0 (origen)\n"
        else:
            info_text += f"  p{i+1} = {p.real:.4f} + {p.imag:.4f}j\n"
    
    # Agregar texto en la esquina
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
            fontsize=8, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    return zeros, poles

# Crear figura con dos subgráficas
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# Gráfica 1: PID Bien Ajustado
zeros_good, poles_good = plot_pole_zero(Kp_good, Ki_good, Kd_good, dt, 
                                         'PID Bien Ajustado', ax1)

# Gráfica 2: PID Mal Ajustado
zeros_bad, poles_bad = plot_pole_zero(Kp_bad, Ki_bad, Kd_bad, dt, 
                                       'PID Mal Ajustado', ax2)

plt.suptitle('Diagrama de Polos y Ceros - Controlador PID Discreto', 
             fontsize=14, fontweight='bold')
plt.tight_layout()
plt.show()

# Imprimir análisis de estabilidad
print("="*60)
print("ANÁLISIS DE ESTABILIDAD")
print("="*60)

print("\n1. PID BIEN AJUSTADO (Kp=90, Ki=0.01, Kd=0.5):")
print("-" * 60)
print(f"Ceros: {zeros_good}")
print(f"Polos: {poles_good}")
all_inside = all(np.abs(poles_good) < 1)
print(f"Estabilidad: {'ESTABLE' if all_inside else 'INESTABLE'}")
print(f"Todos los polos dentro del círculo unitario: {all_inside}")

print("\n2. PID MAL AJUSTADO (Kp=45, Ki=10, Kd=0):")
print("-" * 60)
print(f"Ceros: {zeros_bad}")
print(f"Polos: {poles_bad}")
all_inside = all(np.abs(poles_bad) < 1)
print(f"Estabilidad: {'ESTABLE' if all_inside else 'INESTABLE'}")
print(f"Todos los polos dentro del círculo unitario: {all_inside}")

print("\n" + "="*60)
print("NOTA: Para sistemas discretos, el sistema es estable si todos")
print("los polos están dentro del círculo unitario (|z| < 1)")
print("="*60)