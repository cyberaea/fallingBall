import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_and_save_moving_average(csv_file, window_size, window_type='uniform', output_file='smoothed_output.csv'):
    # Загрузка данных
    df = pd.read_csv(csv_file, header=None, names=['t_ms', 'a_frac'])
    t_ms = df['t_ms'].values
    t = t_ms / 1000.0
    a_frac = df['a_frac'].values

    # Формируем веса окна
    if window_type == 'uniform':
        w = np.ones(window_size)
    elif window_type == 'triangular':
        w = np.arange(1, window_size+1)
        w = np.minimum(w, w[::-1])
    else:
        raise ValueError("window_type must be 'uniform' or 'triangular'")
    w = w / w.sum()

    # Паддим данные зеркально, чтобы свёртка 'same' дала корректные края
    pad_len = window_size // 2
    a_padded = np.pad(a_frac, pad_len, mode='reflect')
    
    # Сглаживаем полным окном и берём центральную часть
    smoothed_full = np.convolve(a_padded, w, mode='same')
    smoothed = smoothed_full[pad_len:pad_len + len(a_frac)]

    # Сохранение в файл
    out_df = pd.DataFrame({'t_ms': t_ms, 'smoothed_frac': smoothed})
    out_df.to_csv(output_file, index=False, header=False)

    # Рисуем
    plt.figure(figsize=(8,4))
    plt.plot(t, a_frac, 'o-', label='Исходные')
    plt.plot(t, smoothed, '-', label=f'Сглажено (N={window_size})')
    plt.xlabel('t, с')
    plt.ylabel('Доля от g')
    plt.title('Сглаживание без провалов на краях (reflect padding)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

plot_and_save_moving_average('combined_smoothed.csv', window_size=10, window_type='triangular', output_file='smoothed.csv')
