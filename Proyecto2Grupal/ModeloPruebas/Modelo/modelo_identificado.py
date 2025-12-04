"""
Modelo identificado del motor DC
Generado automáticamente
Fecha: 2025-12-03 14:30:38
"""

import numpy as np


class ModeloMotor:
    """
    Modelo de primer orden del motor DC
    
    Función de transferencia continua:
    G(s) = 999.3659 / (0.3008*s + 1)
    
    Polo: s = -3.32 rad/s
    Ancho de banda: 0.53 Hz
    Ajuste: 100.0%
    """
    
    # Parámetros del modelo continuo
    K = 999.3659        # Ganancia [RPM/V]
    tau = 0.3008    # Constante de tiempo [s]
    Ts = 0.1887                # Período de muestreo [s]
    fs = 5.30              # Frecuencia de muestreo [Hz]
    
    # Coeficientes discretos (Tustin)
    a1 = 0.522515
    b0 = 1192.956288
    b1 = 1192.956288
    
    def __init__(self):
        """Inicializa el modelo"""
        self.y_prev = 0.0
        self.u_prev = 0.0
        self.reset()
    
    def reset(self):
        """Resetea el estado del modelo"""
        self.y_prev = 0.0
        self.u_prev = 0.0
    
    def update(self, u):
        """
        Actualiza el modelo con nueva entrada
        
        Args:
            u: Entrada de control [V]
            
        Returns:
            Velocidad estimada [RPM]
        """
        # Ecuación en diferencias
        y = self.a1 * self.y_prev + self.b0 * u + self.b1 * self.u_prev
        
        # Actualizar estados
        self.y_prev = y
        self.u_prev = u
        
        return y
    
    def simular(self, u_array, y0=0):
        """
        Simula respuesta a array de entradas
        
        Args:
            u_array: Array de entradas [V]
            y0: Condición inicial [RPM]
            
        Returns:
            Array de velocidades [RPM]
        """
        self.reset()
        self.y_prev = y0
        
        y_array = np.zeros_like(u_array)
        
        for i, u in enumerate(u_array):
            y_array[i] = self.update(u)
        
        return y_array
    
    def respuesta_escalon(self, amplitud=5.0, duracion=3.0):
        """
        Genera respuesta a escalón
        
        Args:
            amplitud: Amplitud del escalón [V]
            duracion: Duración de la simulación [s]
            
        Returns:
            tuple: (tiempo, velocidad)
        """
        n_puntos = int(duracion / self.Ts)
        tiempo = np.arange(n_puntos) * self.Ts
        u_escalon = np.ones(n_puntos) * amplitud
        
        velocidad = self.simular(u_escalon)
        
        return tiempo, velocidad
    
    def __repr__(self):
        return f"ModeloMotor(K={self.K}, tau={self.tau}, Ts={self.Ts})"


# Ejemplo de uso
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # Crear modelo
    motor = ModeloMotor()
    
    # Simular escalón de 5V
    t, v = motor.respuesta_escalon(amplitud=5.0, duracion=3.0)
    
    # Graficar
    plt.figure(figsize=(10, 6))
    plt.plot(t, v, 'b-', linewidth=2)
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Velocidad (RPM)')
    plt.title('Respuesta al Escalón del Modelo')
    plt.grid(True, alpha=0.3)
    plt.show()
    
    print(f"Modelo: {motor}")
    print(f"Velocidad final: {v[-1]:.2f} RPM")
