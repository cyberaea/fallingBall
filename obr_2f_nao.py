#!/usr/bin/env python3
import pandas as pd
import numpy as np
import glob
import sys
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def hampel_filter(x, window_size=10, n_sigmas=3):
    x = np.asarray(x, dtype=float)
    n = len(x)
    new_x = x.copy()
    k = (window_size - 1) // 2
    for i in range(n):
        start = max(0, i - k)
        end   = min(n, i + k + 1)
        window = x[start:end]
        med = np.median(window)
        mad = np.median(np.abs(window - med))
        if mad > 0 and abs(x[i] - med) > n_sigmas * mad:
            new_x[i] = med
    return new_x

def preprocess_file(fname):
    """
    Для одного CSV-файла:
    - читывает колонки time_ms, ax, ay, az
    - приводит в float, отбрасывает NaN
    - вычисляет acc = sqrt(ax^2+ay^2+az^2)
    - применяет Hampel-фильтр
    - затем Savitzky–Golay с динамическим окном
    Возвращает DataFrame ['time_ms','acc']
    """
    df = pd.read_csv(fname, header=None, names=['time_ms','ax','ay','az'])
    df = df.apply(pd.to_numeric, errors='coerce').dropna()
    acc = np.sqrt(df['ax']**2 + df['ay']**2 + df['az']**2)
    # Hampel
    acc_h = hampel_filter(acc, window_size=60, n_sigmas=1)
    # Savitzky–Golay: подбираем нечётное window_length <= len(acc_h)
    n = len(acc_h)
    if n >= 3:
        # максимально 11, но не больше n, и нечётное
        w = min(11, n if n % 2 == 1 else n-1)
        if w < 3:
            acc_s = acc_h
        else:
            acc_s = savgol_filter(acc_h, window_length=w, polyorder=2, mode='interp')
    else:
        # слишком мало точек для SG
        acc_s = acc_h

    return pd.DataFrame({
        'time_ms': df['time_ms'].values,
        'acc':     acc_s
    })

def combine_by_index_avg(dfs):
    max_len = max(len(df) for df in dfs)
    times, accs = [], []
    for i in range(max_len):
        vals, t0 = [], None
        for df in dfs:
            if i < len(df):
                if t0 is None:
                    t0 = df.at[i, 'time_ms']
                vals.append(df.at[i, 'acc'])
        times.append(t0)
        accs.append(np.mean(vals))
    return pd.DataFrame({'time_ms': times, 'acc': accs})

def plot_combined(df):
    t = df['time_ms'].values / 1000.0
    a = df['acc'].values
    plt.figure(figsize=(8,5))
    plt.plot(t, a, 'b.-', label='Усреднённый & сглаженный сигнал')
    plt.xlabel('Время (с)')
    plt.ylabel('Ускорение (g)')
    plt.title('Объединение по строкам после фильтрации')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    files = sys.argv[1:] or glob.glob('*.csv')
    if not files:
        print("Не найдено CSV-файлов.")
        sys.exit(1)

    # Сначала фильтруем каждый файл
    dfs = [preprocess_file(f) for f in files]
    # Потом усредняем по строкам
    df_combined = combine_by_index_avg(dfs)
    df_combined.to_csv('combined_smoothed.csv', index=False, header=False)
    print(f"Обработано {len(files)} файлов, результат в combined_smoothed.csv, объединено ({len(df_combined)} строк).")
    # И сразу выводим график
    plot_combined(df_combined)
