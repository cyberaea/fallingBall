import pandas as pd
import matplotlib.pyplot as plt

# Чтение файла accel.csv
df = pd.read_csv("accel9.csv", header=None, names=["t_ms", "ax", "ay", "az"])

# Вычисление модуля ускорения
df["g_total"] = (df["ax"]**2 + df["ay"]**2 + df["az"]**2)**0.5

# Отбор данных до 4 секунд
df_limited = df[df["t_ms"] <= 4000]

# Построение графика
plt.figure(figsize=(10, 5))
plt.plot(df_limited["t_ms"] / 1000, df_limited["g_total"], marker='o')
plt.title("Полное ускорение (g_total) от времени (0–4 с)")
plt.xlabel("Время, сек")
plt.ylabel("Ускорение, g")
plt.grid(True)
plt.xlim(0, 4)
plt.tight_layout()
plt.show()
