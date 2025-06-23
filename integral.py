import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Чтение сглаженных данных (t_ms, доля от g)
df = pd.read_csv('smoothed.csv', header=None, names=['t_ms', 'a_frac'])
t = df['t_ms'].values / 1000.0
a = (1 - df['a_frac'].values) * 9.81  # ускорение в м/с²

# Интегратор RK4 для v' = a(t)
def integrate_rk4(t, a):
    v = np.zeros_like(t)
    for i in range(len(t)-1):
        dt = t[i+1] - t[i]
        k1 = a[i]
        k2 = np.interp(t[i] + dt/2, t, a)
        k3 = k2
        k4 = a[i+1]
        v[i+1] = v[i] + dt/6*(k1 + 2*k2 + 2*k3 + k4)
    return v

# Получаем экспериментальную v(t)
v_meas = integrate_rk4(t, a)

# Модель для аппроксимации
def tanh_model(t, v_inf, tau):
    return v_inf * np.tanh(t / tau)

# Начальное приближение: v_inf≈max(v_meas), tau≈3.0
popt, _ = curve_fit(tanh_model, t, v_meas, p0=[np.max(v_meas), 3.0])
v_inf_est, tau_est = popt

# Расширение времени на 3 секунды для асимптоты
t_ext = np.linspace(0, t[-1] + 3, 400)
v_fit = tanh_model(t_ext, v_inf_est, tau_est)

# Построение графика
plt.figure(figsize=(10,5))
# Эксперимент
plt.plot(t, v_meas, 'o-', label='Эксперимент v(t)', color='blue')
# Аппроксимация на интервале измерений — сплошная
mask = t_ext <= t[-1]
plt.plot(t_ext[mask], v_fit[mask], '-', label=f'Аппроксимация\n$v_\\infty={v_inf_est:.2f}$ м/с\n$\\tau={tau_est:.2f}$ с', color='green')
# Асимптота — пунктир
plt.plot(t_ext[~mask], v_fit[~mask], '--', color='green')
plt.xlabel('t, с')
plt.ylabel('v, м/с')
plt.title('Аппроксимация v(t) моделью $v_\\infty\\tanh(t/\\tau)$ и асимптота')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
