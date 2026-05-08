# -*- coding: utf-8 -*-
"""
Práctica 4:Sistema Inmunologico 

Departamento de Ingeniería Eléctrica y Electrónica, Ingeniería Biomédica
Tecnológico Nacional de México [TecNM - Tijuana]
Blvd. Alberto Limón Padilla s/n, C.P. 22454, Tijuana, B.C., México


Nombre del alumno: Pamela Escoebedo Sandaoval 
Número de control: 20211965
Correo institucional: alan.garciat201@tectijuana.edu.mx


Asignatura: Modelado de Sistemas Fisiológicos
Docente: Dr. Paul Antonio Valle Trujillo; paul.valle@tectijuana.edu.mx
"""
# Librerías
import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

# Configuración de tiempo
t0, tend, dt = 0, 15, 1e-3
N = round((tend - t0) / dt) + 1
t = np.linspace(t0, tend, N)

# Entrada escalón unitario en t = 1 s
u = np.zeros_like(t)
u[t >= 1] = 1.0

# Función de transferencia del sistema endocrino
def endocrino_tf(R1, R2, L, C):
    num = [L, R2]
    den = [C * L * R1, (C * R1 * R2) + L, R1 + R2]
    return ctrl.tf(num, den)

# Sistema Control (Basal)
R1_s, R2_s, L_s, C_s = 1e3, 100e3, 100e-3, 1e-6
sys_control = endocrino_tf(R1_s, R2_s, L_s, C_s)

# Sistema Caso Alterado
R1_c, R2_c, L_c, C_c = 1e3, 1e3, 100e-3, 1000e-6
sys_caso = endocrino_tf(R1_c, R2_c, L_c, C_c)

# Respuestas en lazo abierto
_, Vs_control = ctrl.forced_response(sys_control, t, u)
_, Vs_caso = ctrl.forced_response(sys_caso, t, u)

# Controlador PID
def controlador(kP, kI, kD, sys):
    Cr = 1e-6
    Re = 1 / (kI * Cr)
    Rr = kP * Re
    Ce = kD / Rr
    numPID = [Re * Rr * Ce * Cr, (Re * Ce + Rr * Cr), 1]
    denPID = [Re * Cr, 0]
    PID = ctrl.tf(numPID, denPID)
    return ctrl.feedback(ctrl.series(PID, sys), 1)

# PID aplicado al caso alterado
casoPID = controlador(1432.29305325518, 373031.607362581, 0.322738185964439, sys_caso)

# Respuesta con PID
_, PID_res = ctrl.forced_response(casoPID, t, Vs_control)



# -------- GRÁFICA ÚNICA --------
plt.figure(figsize=(10, 5))


colors = np.array([
    [10, 196, 224],
    [133, 64, 157],
    [238, 167, 39]
]) / 255


plt.plot(t, Vs_control, '-', linewidth=1, color=colors[0], label='Vs(t): Control')
plt.plot(t, Vs_caso, '-', linewidth=1, color=colors[1], label='Vs(t): Caso')
plt.plot(t, PID_res, ':', linewidth=2, color=colors[2], label='PI(t): Caso')




plt.title('sistema Endocrino')

plt.xlabel('t [s]', fontsize=11)
plt.ylabel('V(t) [V]', fontsize=11)

plt.legend(bbox_to_anchor=(0.5, -0.2), loc='center', ncol=3, fontsize=9, frameon=True)

plt.tight_layout()

plt.tight_layout()
plt.show()