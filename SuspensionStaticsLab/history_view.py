"""Time cursor, playback and axial force chart for a solved load history."""

from time import monotonic
import tkinter as tk
from tkinter import ttk

import numpy as np

from .viewport import ACCENT, AXIS_COLORS, BG, FONT, GRID, INK, MUTED, ROD


def signed_force(force):
    if abs(force) < 1e-9:
        return "0 N"
    state = "拉伸" if force > 0. else "壓縮"
    return f"{force:+.6g} N（{state}）"


class HistoryPanel(ttk.Frame):
    def __init__(self, parent, rise, generate, import_csv, on_frame, on_rod):
        super().__init__(parent, padding=12)
        self.history = None
        self.index = 0
        self.on_frame = on_frame
        self.on_rod = on_rod
        self.timer = None
        self.elapsed_start = 0.
        self.time_start = 0.
        self.time = tk.DoubleVar(self)
        self.source = tk.StringVar(self, value="產生重心加速歷程，或匯入圈速 CSV。")
        self.readout = tk.StringVar(self)
        self.largest_readout = tk.StringVar(self)
        self.statistics_readout = tk.StringVar(self)
        self.rod = tk.StringVar(self)
        self.track_largest = tk.BooleanVar(self)
        self.plot_bounds = None
        self.cursor_x = None
        self.cursor_plots = []

        inputs = ttk.Frame(self)
        inputs.pack(fill="x", pady=(0, 6))
        ttk.Label(inputs, text="上升時間 (s)").pack(side="left")
        ttk.Entry(inputs, textvariable=rise, width=7).pack(side="left", padx=(5, 6))
        ttk.Button(inputs, text="產生加速歷程", command=generate).pack(side="left")
        ttk.Button(inputs, text="匯入圈速 CSV", command=import_csv).pack(side="left", padx=6)
        ttk.Label(self, text="加速歷程採外載工況的質量、三軸加速度與重力設定。", style="Muted.TLabel",
                  wraplength=410).pack(anchor="w")
        ttk.Label(self, textvariable=self.source, style="Heading.TLabel", wraplength=410).pack(anchor="w", pady=(7, 6))

        selection = ttk.Frame(self)
        selection.pack(fill="x", pady=(0, 7))
        ttk.Label(selection, text="桿件").pack(side="left", padx=(0, 6))
        self.track = ttk.Checkbutton(selection, text="追蹤當下最大軸力", variable=self.track_largest,
                                     command=self.change_mode, state="disabled")
        self.track.pack(side="right", padx=(7, 0))
        self.picker = ttk.Combobox(selection, textvariable=self.rod, state="disabled", width=14)
        self.picker.pack(side="left", fill="x", expand=True)
        self.picker.bind("<<ComboboxSelected>>", self.select_rod)
        ttk.Label(self, textvariable=self.statistics_readout, style="Muted.TLabel", wraplength=410,
                  justify="left").pack(anchor="w", pady=(0, 5))

        controls = ttk.Frame(self)
        controls.pack(side="bottom", fill="x", pady=(7, 0))
        ttk.Label(controls, textvariable=self.largest_readout, style="Heading.TLabel", wraplength=410).pack(
            side="bottom", anchor="w", pady=(2, 0))
        ttk.Label(controls, textvariable=self.readout, style="Muted.TLabel", wraplength=460).pack(
            side="bottom", anchor="w", pady=(4, 0))
        self.play_button = ttk.Button(controls, text="播放", width=6, command=self.toggle_play, state="disabled")
        self.play_button.pack(side="left", padx=(0, 7))
        self.seek = ttk.Scale(controls, variable=self.time, orient="horizontal", command=self.scrub, state="disabled")
        self.seek.pack(side="left", fill="x", expand=True)

        self.chart = tk.Canvas(self, bg=BG, width=385, height=300, highlightthickness=0)
        self.chart.pack(fill="both", expand=True)
        self.chart.bind("<Configure>", self.draw)
        for event in ("<Button-1>", "<B1-Motion>"):
            self.chart.bind(event, self.seek_chart)

    def set_history(self, history, preferred=None):
        self.pause()
        self.history = history
        self.index = 0
        names = tuple(history.force_n) if history else ()
        self.picker.configure(values=names, state="readonly" if names else "disabled")
        state = "normal" if names else "disabled"
        self.play_button.configure(state=state)
        self.seek.configure(state=state)
        self.track.configure(state=state)
        if history:
            times = history.time_s
            self.rod.set(preferred if preferred in names else max(names, key=lambda name: np.max(np.abs(history.force_n[name]))))
            self.source.set(f"{history.source}　·　{len(times):,} 筆")
            self.seek.configure(from_=times[0], to=times[-1])
            self.time.set(times[0])
        else:
            self.rod.set("")
            self.source.set("產生重心加速歷程，或匯入圈速 CSV。")
            self.readout.set("")
            self.largest_readout.set("")
        self.update_statistics()
        self.draw()

    def select_rod(self, _event=None):
        self.track_largest.set(False)
        self.change_mode()

    def show_rod(self, name):
        if self.history and name in self.history.force_n and (self.rod.get() != name or self.track_largest.get()):
            self.track_largest.set(False)
            self.rod.set(name)
            self.update_statistics()
            self.draw()

    def active_rod(self):
        return str(self.history.largest_loads[0][self.index]) if self.track_largest.get() else self.rod.get()

    def change_mode(self):
        self.update_statistics()
        self.draw()
        self.on_rod(self.active_rod())

    def update_statistics(self):
        if self.history is None:
            self.statistics_readout.set("")
        elif self.track_largest.get():
            self.statistics_readout.set("每個時間點顯示全車桿件中最大的 |軸力|。\n選取桿件可查看全段峰值與平均荷載。")
        else:
            stats = self.history.statistics(self.rod.get())
            self.statistics_readout.set(f"全段峰值 {signed_force(stats.peak_n)} · t = {stats.peak_time_s:.6g} s\n"
                                        f"平均 |軸力| {stats.mean_absolute_n:.6g} N · 帶符號平均 {signed_force(stats.mean_n)}")

    def scrub(self, value):
        if self.history is None:
            return
        self.pause()
        times = self.history.time_s
        time = float(value)
        index = min(len(times) - 1, int(np.searchsorted(times, time)))
        if index and abs(time - times[index - 1]) < abs(time - times[index]):
            index -= 1
        self.set_index(index)

    def seek_chart(self, event):
        if self.plot_bounds:
            left, _, right, _ = self.plot_bounds
            ratio = max(0., min(1., (event.x - left) / (right - left)))
            times = self.history.time_s
            self.scrub(times[0] + ratio * (times[-1] - times[0]))

    def set_index(self, index):
        changed = self.index != index
        self.index = index
        self.time.set(self.history.time_s[index])
        if changed:
            self.draw_cursor()
            self.on_frame()

    def toggle_play(self):
        if self.history is None:
            return
        if self.timer is not None:
            self.pause()
            return
        if self.index == len(self.history.time_s) - 1:
            self.set_index(0)
        self.elapsed_start = monotonic()
        self.time_start = float(self.history.time_s[self.index])
        self.play_button.configure(text="暫停")
        self.timer = self.after(25, self.tick)

    def tick(self):
        self.timer = None
        times = self.history.time_s
        time = self.time_start + (monotonic() - self.elapsed_start)
        self.set_index(int(np.searchsorted(times, time, side="right")) - 1)
        if self.index < len(times) - 1:
            self.timer = self.after(25, self.tick)
        else:
            self.play_button.configure(text="播放")

    def pause(self):
        if self.timer is not None:
            self.after_cancel(self.timer)
            self.timer = None
        self.play_button.configure(text="播放")

    def draw(self, _event=None):
        canvas = self.chart
        canvas.delete("all")
        self.plot_bounds = None
        self.cursor_x = None
        self.cursor_plots = []
        width, height = canvas.winfo_width(), canvas.winfo_height()
        left, top, right, bottom = 68, 26, width - 24, height - 43
        if self.history is None:
            return
        if right <= left + 30 or bottom <= top + 72:
            self.draw_cursor()
            return
        middle = (top + bottom - 25) / 2.
        force_top = middle + 25
        times = self.history.time_s
        self.plot_bounds = left, top, right, bottom
        self.cursor_x = left + (times - times[0]) / (times[-1] - times[0]) * (right - left)
        acceleration = self.history.acceleration_mps2.T
        self.draw_plot(top, middle, "重心加速度 (m/s²)", acceleration, AXIS_COLORS, ("ax", "ay", "az"))
        for index, (label, color) in enumerate(zip(("ax", "ay", "az"), AXIS_COLORS)):
            canvas.create_text(right - (2 - index) * 27, top - 5, anchor="se", text=label,
                               fill=color if np.any(np.isfinite(acceleration[index])) else MUTED, font=(FONT, 9))
        if not np.isfinite(acceleration).any():
            canvas.create_text((left + right) / 2, (top + middle) / 2, text="CSV 未提供加速度資料", fill=INK, font=(FONT, 9))
        if self.track_largest.get():
            force = np.abs(self.history.largest_loads[1])
            title = "當下最大 |軸力| (N)"
        else:
            force = self.history.force_n[self.rod.get()]
            title = "桿件軸力 (N)　+拉伸 / −壓縮"
        self.draw_plot(force_top, bottom, title, (force,), (ROD,), ("force",))
        for ratio in np.linspace(0., 1., 5):
            label = times[0] + ratio * (times[-1] - times[0])
            canvas.create_text(left + ratio * (right - left), bottom + 7, anchor="n", text=f"{label:.4g}", fill=MUTED, font=(FONT, 9))
        canvas.create_text((left + right) / 2, height - 7, anchor="s", text="時間 (s)", fill=INK, font=(FONT, 9))
        self.draw_cursor()

    def draw_plot(self, top, bottom, title, series, colors, tags):
        canvas = self.chart
        left, _, right, _ = self.plot_bounds
        finite = np.asarray(series)
        finite = finite[np.isfinite(finite)]
        low = min(0., float(np.min(finite, initial=0.)))
        high = max(0., float(np.max(finite, initial=0.)))
        padding = max((high - low) * .08, 1e-9) if high > low else 1.
        low, high = low - padding, high + padding
        canvas.create_text(left, top - 5, anchor="sw", text=title, fill=INK, font=(FONT, 9))
        for p in np.linspace(left, right, 5):
            canvas.create_line(p, top, p, bottom, fill=GRID)
        ticks = np.linspace(low, high, 4 if bottom - top >= 85 else 3)
        closest = np.argmin(np.abs(ticks))
        if abs(ticks[closest]) < .18 * (ticks[1] - ticks[0]):
            ticks[closest] = 0.
        for level in ticks:
            p = bottom - (level - low) / (high - low) * (bottom - top)
            canvas.create_line(left, p, right, p, fill=GRID)
            canvas.create_text(left - 6, p, anchor="e", text=f"{level:.4g}", fill=MUTED, font=(FONT, 9))
        zero = bottom + low / (high - low) * (bottom - top)
        canvas.create_line(left, zero, right, zero, fill=MUTED, dash=(4, 4))
        traces = []
        for values, color, tag in zip(series, colors, tags):
            y = bottom - (values - low) / (high - low) * (bottom - top)
            traces.append((y, color))
            indices = np.flatnonzero(np.isfinite(y))
            for run in np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1):
                if len(run) == 1:
                    x_at, y_at = self.cursor_x[run[0]], y[run[0]]
                    canvas.create_oval(x_at - 2, y_at - 2, x_at + 2, y_at + 2, fill=color, outline="", tags=(f"curve-{tag}",))
                elif len(run) > 1:
                    points = np.column_stack((self.cursor_x[run], y[run]))
                    for start in range(0, len(points) - 1, 1500):
                        canvas.create_line(*points[start:start + 1501].ravel(), fill=color, width=2, tags=(f"curve-{tag}",))
        self.cursor_plots.append((top, bottom, traces))

    def draw_cursor(self):
        time = self.history.time_s[self.index]
        largest, forces = self.history.largest_loads
        leader = signed_force(forces[self.index])
        self.largest_readout.set(f"當下最大 |軸力|　{largest[self.index]}  {leader}" if leader != "0 N" else "當下最大 |軸力|　0 N")
        values = self.history.acceleration_mps2[self.index]
        acceleration = "   ".join(f"a{axis} {value:+.5g}" if np.isfinite(value) else f"a{axis} —"
                                 for axis, value in zip("xyz", values))
        detail = f"t = {time:.6g} s　·　{acceleration} m/s²"
        if not self.track_largest.get():
            force = self.history.force_n[self.rod.get()][self.index]
            detail += f"\n選取桿件軸力 {signed_force(force)}"
        self.readout.set(detail)
        self.chart.delete("cursor")
        if self.cursor_x is not None:
            x = self.cursor_x[self.index]
            for top, bottom, traces in self.cursor_plots:
                self.chart.create_line(x, top, x, bottom, fill=ACCENT, tags=("cursor",))
                for values, color in traces:
                    y = values[self.index]
                    if np.isfinite(y):
                        self.chart.create_oval(x - 4, y - 4, x + 4, y + 4, fill=color, outline=BG, tags=("cursor",))
