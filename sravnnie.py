import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_acceleration_comparison_extended(csv_file):
    # Чтение CSV: время в мс, доля от g (0→1g,1→0g)
    df = pd.read_csv(csv_file, header=None, names=['t_ms', 'a_frac'])
    t_meas = df['t_ms'].values / 1000.0
    meas_frac = df['a_frac'].values

    # Параметры для теоретической кривой (m=0.093 кг)
    g = 9.81
    m = 0.093
    d = 0.14
    r = d / 2
    rho = 1.20
    Cd = 0.47
    V = 4/3 * np.pi * r**3
    A = np.pi * r**2
    a0 = (m*g - rho * V * g) / m
    b = (0.5 * rho * Cd * A) / m

    # Временная сетка для теории до 4 секунд
    t_theor = np.linspace(0, 4, 400)
    a_t = a0 * (1 / np.cosh(np.sqrt(a0 * b) * t_theor))**2
    theor_frac = 1 - a_t / g

    # Сдвиг для совмещения с первым измерением
    offset = meas_frac[0] - theor_frac[0]
    theor_frac_shifted = theor_frac + offset

    # Построение графика
    plt.figure(figsize=(8, 4))
    plt.plot(t_meas, meas_frac, 'o', label='Измеренная доля от g')
    plt.plot(t_theor, theor_frac_shifted, '-', label='Теоретическая доля от g (до 4 с)')
    plt.xlabel('t, с')
    plt.ylabel('Доля от g')
    plt.title('Сравнение измеренной и теоретической доли ускорения\n(теория до 4 секунд)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


plot_acceleration_comparison_extended('smoothed.csv')
