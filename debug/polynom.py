import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# --- Initialparameter ---
v = -2.0/30.0
init_T_sw, init_T_st = 0.5, 0.5
init_L, init_A = init_T_st*v, 0.02
init_y_0, init_y_1 = 0.08, 0.00
init_T0, init_T1 = 0.1, 0.75  # T1 leicht unter 2*Tsw
init_j = 800.0


# --- Berechnungsfunktionen ---
def eval_poly(t, coeffs):
    order = len(coeffs) - 1
    p = sum(coeffs[i] * t**i for i in range(order + 1))
    v = sum(i * coeffs[i] * t ** (i - 1) for i in range(1, order + 1))
    a = sum(i * (i - 1) * coeffs[i] * t ** (i - 2) for i in range(2, order + 1))
    return p, v, a


def calculate_coeffs_x_I(v, T_0, x_0, T_st, T_sw):
    coeffs = [0] * 7
    coeffs[0] = v * T_st - x_0
    coeffs[1] = v
    # Nenner für die x-Koeffizienten (basierend auf deiner Matrix-Lösung)
    d1 = 96 * T_0**5 * T_sw**4 - 90 * T_0**4 * T_sw**5 + 20 * T_0**3 * T_sw**6
    d2 = 48 * T_0**5 * T_sw**5 - 45 * T_0**4 * T_sw**6 + 10 * T_0**3 * T_sw**7

    coeffs[4] = (
        -3840 * T_0**5 * T_st * v
        - 1920 * T_0**5 * T_sw * v
        + 3840 * T_0**5 * x_0
        + 2400 * T_0**4 * T_st * T_sw * v
        + 1200 * T_0**4 * T_sw**2 * v
        - 2400 * T_0**4 * T_sw * x_0
        - 5 * T_sw**6 * v
    ) / d1
    coeffs[5] = (
        2304 * T_0**5 * T_st * v
        + 1152 * T_0**5 * T_sw * v
        - 2304 * T_0**5 * x_0
        - 960 * T_0**3 * T_st * T_sw**2 * v
        - 480 * T_0**3 * T_sw**3 * v
        + 960 * T_0**3 * T_sw**2 * x_0
        + 9 * T_sw**6 * v
    ) / d2
    coeffs[6] = (
        -1920 * T_0**4 * T_st * v
        - 960 * T_0**4 * T_sw * v
        + 1920 * T_0**4 * x_0
        + 1280 * T_0**3 * T_st * T_sw * v
        + 640 * T_0**3 * T_sw**2 * v
        - 1280 * T_0**3 * T_sw * x_0
        - 8 * T_sw**5 * v
    ) / d2

    print(f"swing 1 x coeffs: {coeffs}")
    return coeffs


def calculate_coeffs_y_I(v, y_0, A, L, y_1, T_sw,j):
    pi = np.pi
    coeffs = [0] * 8

    coeffs[0] = -y_0
    coeffs[1] = A * pi * v / L
    coeffs[2] = 0
    coeffs[3] = -A * pi**3 * (v**3 / (6 * L**3))
    coeffs[4] = (
        0.333333333333333
        * (
            -480.0 * A * L**2 * T_sw * pi * v
            + 4.0 * A * T_sw**3 * pi**3 * v**3
            - L**3 * T_sw**3 * j
            + 1680.0 * L**3 * y_0
            + 1680.0 * L**3 * y_1
        )
        / (L**3 * T_sw**4)
    )
    coeffs[5] = (
        720.0 * A * L**2 * T_sw * pi * v
        - 4.0 * A * T_sw**3 * pi**3 * v**3
        + 2.0 * L**3 * T_sw**3 * j
        - 2688.0 * L**3 * y_0
        - 2688.0 * L**3 * y_1
    ) / (L**3 * T_sw**5)
    coeffs[6] = (
        0.333333333333333
        * (
            -3456.0 * A * L**2 * T_sw * pi * v
            + 16.0 * A * T_sw**3 * pi**3 * v**3
            - 12.0 * L**3 * T_sw**3 * j
            + 13440.0 * L**3 * y_0
            + 13440.0 * L**3 * y_1
        )
        / (L**3 * T_sw**6)
    )
    coeffs[7] = (
        0.333333333333333
        * (
            1920.0 * A * L**2 * T_sw * pi * v
            - 8.0 * A * T_sw**3 * pi**3 * v**3
            + 8.0 * L**3 * T_sw**3 * j
            - 7680.0 * L**3 * y_0
            - 7680.0 * L**3 * y_1
        )
        / (L**3 * T_sw**7)
    )
    print(f"swing 1 y coeffs: {coeffs}")
    return coeffs


def calculate_coeffs_x_II(v, T_1, x_0, T_st, T_sw):
    coeffs = [0] * 7
    coeffs[0] = (
        -480 * T_1**5 * T_sw * v
        - 864 * T_1**5 * x_0
        + 1590 * T_1**4 * T_sw**2 * v
        + 2790 * T_1**4 * T_sw * x_0
        - 1900 * T_1**3 * T_sw**3 * v
        - 3180 * T_1**3 * T_sw**2 * x_0
        + 960 * T_1**2 * T_sw**4 * v
        + 1440 * T_1**2 * T_sw**3 * x_0
        - 180 * T_1 * T_sw**5 * v
        - 180 * T_1 * T_sw**4 * x_0
        + 7 * T_sw**6 * v
        - 6 * T_sw**5 * x_0
    ) / (
        96 * T_1**5
        - 390 * T_1**4 * T_sw
        + 620 * T_1**3 * T_sw**2
        - 480 * T_1**2 * T_sw**3
        + 180 * T_1 * T_sw**4
        - 26 * T_sw**5
    )
    coeffs[1] = (
        1968 * T_1**5 * T_sw * v
        + 3840 * T_1**5 * x_0
        - 6435 * T_1**4 * T_sw**2 * v
        - 12480 * T_1**4 * T_sw * x_0
        + 7510 * T_1**3 * T_sw**3 * v
        + 14400 * T_1**3 * T_sw**2 * x_0
        - 3600 * T_1**2 * T_sw**4 * v
        - 6720 * T_1**2 * T_sw**3 * x_0
        + 570 * T_1 * T_sw**5 * v
        + 960 * T_1 * T_sw**4 * x_0
    ) / (
        48 * T_1**5 * T_sw
        - 195 * T_1**4 * T_sw**2
        + 310 * T_1**3 * T_sw**3
        - 240 * T_1**2 * T_sw**4
        + 90 * T_1 * T_sw**5
        - 13 * T_sw**6
    )
    coeffs[2] = (
        -5760 * T_1**5 * T_sw * v
        - 11520 * T_1**5 * x_0
        + 18000 * T_1**4 * T_sw**2 * v
        + 36000 * T_1**4 * T_sw * x_0
        - 19200 * T_1**3 * T_sw**3 * v
        - 38400 * T_1**3 * T_sw**2 * x_0
        + 7200 * T_1**2 * T_sw**4 * v
        + 14400 * T_1**2 * T_sw**3 * x_0
        - 285 * T_sw**6 * v
        - 480 * T_sw**5 * x_0
    ) / (
        48 * T_1**5 * T_sw**2
        - 195 * T_1**4 * T_sw**3
        + 310 * T_1**3 * T_sw**4
        - 240 * T_1**2 * T_sw**5
        + 90 * T_1 * T_sw**6
        - 13 * T_sw**7
    )
    coeffs[3] = (
        7680 * T_1**5 * T_sw * v
        + 15360 * T_1**5 * x_0
        - 21600 * T_1**4 * T_sw**2 * v
        - 43200 * T_1**4 * T_sw * x_0
        + 17600 * T_1**3 * T_sw**3 * v
        + 35200 * T_1**3 * T_sw**2 * x_0
        - 4800 * T_1 * T_sw**5 * v
        - 9600 * T_1 * T_sw**4 * x_0
        + 1200 * T_sw**6 * v
        + 2240 * T_sw**5 * x_0
    ) / (
        48 * T_1**5 * T_sw**3
        - 195 * T_1**4 * T_sw**4
        + 310 * T_1**3 * T_sw**5
        - 240 * T_1**2 * T_sw**6
        + 90 * T_1 * T_sw**7
        - 13 * T_sw**8
    )
    coeffs[4] = (
        -9600 * T_1**5 * T_sw * v
        - 19200 * T_1**5 * x_0
        + 20400 * T_1**4 * T_sw**2 * v
        + 40800 * T_1**4 * T_sw * x_0
        - 26400 * T_1**2 * T_sw**4 * v
        - 52800 * T_1**2 * T_sw**3 * x_0
        + 19200 * T_1 * T_sw**5 * v
        + 38400 * T_1 * T_sw**4 * x_0
        - 3755 * T_sw**6 * v
        - 7200 * T_sw**5 * x_0
    ) / (
        96 * T_1**5 * T_sw**4
        - 390 * T_1**4 * T_sw**5
        + 620 * T_1**3 * T_sw**6
        - 480 * T_1**2 * T_sw**7
        + 180 * T_1 * T_sw**8
        - 26 * T_sw**9
    )
    coeffs[5] = (
        1152 * T_1**5 * T_sw * v
        + 2304 * T_1**5 * x_0
        - 8160 * T_1**3 * T_sw**3 * v
        - 16320 * T_1**3 * T_sw**2 * x_0
        + 12960 * T_1**2 * T_sw**4 * v
        + 25920 * T_1**2 * T_sw**3 * x_0
        - 7200 * T_1 * T_sw**5 * v
        - 14400 * T_1 * T_sw**4 * x_0
        + 1287 * T_sw**6 * v
        + 2496 * T_sw**5 * x_0
    ) / (
        48 * T_1**5 * T_sw**5
        - 195 * T_1**4 * T_sw**6
        + 310 * T_1**3 * T_sw**7
        - 240 * T_1**2 * T_sw**8
        + 90 * T_1 * T_sw**9
        - 13 * T_sw**10
    )
    coeffs[6] = (
        -960 * T_1**4 * T_sw * v
        - 1920 * T_1**4 * x_0
        + 3200 * T_1**3 * T_sw**2 * v
        + 6400 * T_1**3 * T_sw * x_0
        - 3840 * T_1**2 * T_sw**3 * v
        - 7680 * T_1**2 * T_sw**2 * x_0
        + 1920 * T_1 * T_sw**4 * v
        + 3840 * T_1 * T_sw**3 * x_0
        - 328 * T_sw**5 * v
        - 640 * T_sw**4 * x_0
    ) / (
        48 * T_1**5 * T_sw**5
        - 195 * T_1**4 * T_sw**6
        + 310 * T_1**3 * T_sw**7
        - 240 * T_1**2 * T_sw**8
        + 90 * T_1 * T_sw**9
        - 13 * T_sw**10
    )

    print(f"swing 2 x coeffs: {coeffs}")
    return coeffs
def calculate_coeffs_y_II(v, y_0, A, L, y_1, T_sw,j):
    coeffs = [0]*8
    coeffs[0] = (294*A*L**2*T_sw*np.pi*v - A*T_sw**3*np.pi**3*v**3 - 2*L**3*T_sw**3*j - 1254*L**3*y_0 - 1248*L**3*y_1)/(6*L**3)
    coeffs[1] = (-3174*A*L**2*T_sw*np.pi*v + 11*A*T_sw**3*np.pi**3*v**3 + 20*L**3*T_sw**3*j + 13440*L**3*y_0 + 13440*L**3*y_1)/(6*L**3*T_sw)
    coeffs[2] = (4800*A*L**2*T_sw*np.pi*v - 17*A*T_sw**3*np.pi**3*v**3 - 28*L**3*T_sw**3*j - 20160*L**3*y_0 - 20160*L**3*y_1)/(2*L**3*T_sw**2)
    coeffs[3] = (-11840*A*L**2*T_sw*np.pi*v + 43*A*T_sw**3*np.pi**3*v**3 + 64*L**3*T_sw**3*j + 49280*L**3*y_0 + 49280*L**3*y_1)/(2*L**3*T_sw**3)
    coeffs[4] = (8560*A*L**2*T_sw*np.pi*v - 32*A*T_sw**3*np.pi**3*v**3 - 43*L**3*T_sw**3*j - 35280*L**3*y_0 - 35280*L**3*y_1)/(L**3*T_sw**4)
    coeffs[5] = (-7248*A*L**2*T_sw*np.pi*v + 28*A*T_sw**3*np.pi**3*v**3 + 34*L**3*T_sw**3*j + 29568*L**3*y_0 + 29568*L**3*y_1)/(L**3*T_sw**5)
    coeffs[6] = (9984*A*L**2*T_sw*np.pi*v - 40*A*T_sw**3*np.pi**3*v**3 - 44*L**3*T_sw**3*j - 40320*L**3*y_0 - 40320*L**3*y_1)/(3*L**3*T_sw**6)
    coeffs[7] = (-1920*A*L**2*T_sw*np.pi*v + 8*A*T_sw**3*np.pi**3*v**3 + 8*L**3*T_sw**3*j + 7680*L**3*y_0 + 7680*L**3*y_1)/(3*L**3*T_sw**7)
    print(f"swing 2 y coeffs: {coeffs}")
    return coeffs

# --- Plot Setup ---
fig = plt.figure(figsize=(14, 9))
gs = plt.GridSpec(3, 3, figure=fig, height_ratios=[1, 1, 1])

# Subplots für x, v, a
ax_px = fig.add_subplot(gs[0, 0])
ax_vx = fig.add_subplot(gs[1, 0])
ax_ax = fig.add_subplot(gs[2, 0])
ax_py = fig.add_subplot(gs[0, 1])
ax_vy = fig.add_subplot(gs[1, 1])
ax_ay = fig.add_subplot(gs[2, 1])
ax_2d = fig.add_subplot(gs[:, 2])  # 2D Ansicht über die gesamte Höhe rechts

plt.subplots_adjust(bottom=0.25, left=0.1, right=0.95, hspace=0.4, wspace=0.3)

# Linien-Objekte initialisieren
lines = []
for ax_sub, title in zip(
    [ax_px, ax_vx, ax_ax, ax_py, ax_vy, ax_ay],
    ["X Pos", "X Vel", "X Acc", "Y Pos", "Y Vel", "Y Acc"],
):
    (l_st,) = ax_sub.plot([], [], "orange", lw=1.5, label="Stance")
    (l_sw,) = ax_sub.plot([], [], "b", lw=1.5, label="Swing")
    ax_sub.set_title(title)
    ax_sub.grid(True)
    lines.append((l_st, l_sw))

(l_2d_st,) = ax_2d.plot([], [], "orange", lw=2, label="Stance")
(l_2d_sw,) = ax_2d.plot([], [], "b", lw=2, label="Swing")
ax_2d.set_title("2D Footpath")
ax_2d.grid(True)
ax_2d.set_aspect("equal")

# --- Slider ---
slider_ax = [plt.axes([0.15, 0.16 - i * 0.025, 0.7, 0.015]) for i in range(7)]
s_L = Slider(slider_ax[0], "L", -0.4, 0.4, valinit=init_L)
s_A = Slider(slider_ax[1], "A", 0.005, 0.05, valinit=init_A)
s_Tst = Slider(slider_ax[2], "T_st", 0.1, 0.8, valinit=init_T_st)
s_Tsw = Slider(slider_ax[3], "T_sw", 0.1, 0.8, valinit=init_T_sw)
s_y0 = Slider(slider_ax[4], "y_0", 0.01, 0.12, valinit=init_y_0)
s_T0 = Slider(slider_ax[5], "T_0", 0.01, 0.3, valinit=init_T0)
s_j = Slider(slider_ax[6], "j", 200.0, 1500.0, valinit=init_j)


def update(val):
    L, A, T_st, T_sw, y_0, T_0,j = (
        s_L.val,
        s_A.val,
        s_Tst.val,
        s_Tsw.val,
        s_y0.val,
        s_T0.val,
        s_j.val
    )
    T_1 = T_sw - T_0
    v, x_0 = L / T_st, L / 2

    t_st = np.linspace(0, T_st, 100)
    t_sw = np.linspace(0, T_sw, 200)
    t_half = t_sw[t_sw <= T_sw / 2]

    # --- Stance Phase ---
    x_st_p = v * t_st - x_0
    y_st_p = -A * np.cos(np.pi * (0.5 - (v * t_st) / L)) - y_0
    v_x_st, a_x_st = np.full_like(t_st, v), np.zeros_like(t_st)
    v_y_st = -A * np.sin(np.pi * (0.5 - (v * t_st) / L)) * (np.pi * v / L)
    a_y_st = A * np.cos(np.pi * (0.5 - (v * t_st) / L)) * (np.pi * v / L) ** 2

    # --- Swing Phase ---
    x_coeffs = calculate_coeffs_x_I(v, T_0, x_0, T_st, T_sw)
    y_coeffs = calculate_coeffs_y_I(v, y_0, A, L, init_y_1, T_sw,j)

    xp1, xv1, xa1 = eval_poly(t_half, x_coeffs)
    yp1, yv1, ya1 = eval_poly(t_half, y_coeffs)
    
    t_half2 = t_sw[t_sw > T_sw / 2]

    x_coeffs_II = calculate_coeffs_x_II(v, T_1, x_0, T_st, T_sw)
    y_coeffs_II = calculate_coeffs_y_II(v, y_0, A, L, init_y_1, T_sw,j)
    
    xp2, xv2, xa2 = eval_poly(t_half2, x_coeffs_II)
    yp2, yv2, ya2 = eval_poly(t_half2, y_coeffs_II)

    xp_sw = np.concatenate([xp1, xp2])
    xv_sw = np.concatenate([xv1, xv2])
    xa_sw = np.concatenate([xa1, xa2])

    yp_sw = np.concatenate([yp1, yp2])
    yv_sw = np.concatenate([yv1, yv2])
    ya_sw = np.concatenate([ya1, ya2])

    # --- Daten Plotten ---
    # --- Daten Plotten ---
    t_f_sw = t_st[-1] + t_sw
    target_axes = [ax_px, ax_vx, ax_ax, ax_py, ax_vy, ax_ay]
    # Wir speichern die Datenpaare (Stance + Swing) für jeden Subplot
    data_pairs = [
        (np.concatenate([x_st_p, xp_sw])),  # X Pos
        (np.concatenate([v_x_st, xv_sw])),  # X Vel
        (np.concatenate([a_x_st, xa_sw])),  # X Acc
        (np.concatenate([y_st_p, yp_sw])),  # Y Pos
        (np.concatenate([v_y_st, yv_sw])),  # Y Vel
        (np.concatenate([a_y_st, ya_sw])),  # Y Acc
    ]

    for i, full_data in enumerate(data_pairs):
        # 1. Daten der Linien aktualisieren
        # Da wir oben konkateniert haben, müssen wir hier wieder splitten oder
        # einfach die ursprünglichen Segmente nutzen:
        lines[i][0].set_data(t_st, data_pairs[i][: len(t_st)])
        lines[i][1].set_data(t_f_sw, data_pairs[i][len(t_st) :])

        cur_ax = target_axes[i]

        # 2. Manuelle Skalierung für absolute Stabilität
        d_min = np.min(full_data)
        d_max = np.max(full_data)
        d_range = d_max - d_min

        # Falls range fast 0 ist (z.B. Beschleunigung bei konstantem v)
        if d_range < 1e-6:
            d_range = 0.1

        cur_ax.set_ylim(d_min - d_range * 0.15, d_max + d_range * 0.15)

        # Zeitachse (X-Achse) anpassen
        cur_ax.set_xlim(0, t_f_sw[-1])

    # --- 2D Plot (bleibt dynamisch) ---
    full_x = np.concatenate([x_st_p, xp_sw])
    full_y = np.concatenate([y_st_p, yp_sw])

    l_2d_st.set_data(x_st_p, y_st_p)  # Nur Stützphase
    l_2d_sw.set_data(xp_sw, yp_sw)    # Nur Schwungphase


    # Dynamische Skalierung basierend auf den echten Daten-Extrema
    x_min, x_max = np.min(full_x), np.max(full_x)
    y_min, y_max = np.min(full_y), np.max(full_y)

    x_range = max(x_max - x_min, 0.1)
    y_range = max(y_max - y_min, 0.1)

    ax_2d.set_xlim(x_min - x_range * 0.2, x_max + x_range * 0.2)
    ax_2d.set_ylim(y_min - y_range * 0.2, y_max + y_range * 0.2)

    fig.canvas.draw_idle()


for s in [s_L, s_A, s_Tst, s_Tsw, s_y0, s_T0, s_j]:
    s.on_changed(update)

update(None)
plt.show()
