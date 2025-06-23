import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Загрузка данных: первый столбец – время в мс, второй – дробное значение g (0 → 1g, 1 → 0g)
data = pd.read_csv('smoothed.csv', header=None, names=['t_ms', 'a_frac'])
# Перевод времени в секунды
t = data['t_ms'].values / 1000.0
# Преобразование ускорения: 0 → g, 1 → 0
a = (1 - data['a_frac'].values) * 9.81  # м/с²

# RK4-интегратор для v' = a(t)
def integrate_rk4(t, a):
    v = np.zeros_like(t)
    for i in range(len(t) - 1):
        dt = t[i+1] - t[i]
        k1 = a[i]
        k2 = np.interp(t[i] + dt/2, t, a)
        k3 = np.interp(t[i] + dt/2, t, a)
        k4 = a[i+1]
        v[i+1] = v[i] + dt/6*(k1 + 2*k2 + 2*k3 + k4)
    return v

# Интегрирование и оценка погрешности
v = integrate_rk4(t, a)
# Шаг вдвое меньше
t2 = np.linspace(t[0], t[-1], 2*(len(t)-1) + 1)
a2 = np.interp(t2, t, a)
v2 = integrate_rk4(t2, a2)
v2_on_t = np.interp(t, t2, v2)
eps = np.abs(v2_on_t - v)

# Построение графика
plt.figure(figsize=(10, 5))
plt.plot(t, v, label='v(t) RK4')
plt.fill_between(t, v-eps, v+eps, color='orange', alpha=0.3, label='Ошибка интегрирования')
plt.xlabel('t, с')
plt.ylabel('v, м/с')
plt.title('Скорость v(t) методом Рунге–Кутты 4-го порядка')
plt.legend()
plt.grid(True)
plt.show()
