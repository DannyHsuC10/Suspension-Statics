"""Select a horizontal acceleration inside the friction circle."""

from math import hypot
import tkinter as tk

from .viewport import FONT


class GripPlot(tk.Canvas):
    def __init__(self, parent, on_change):
        super().__init__(parent, width=180, height=143, background="#111d31", highlightthickness=0,
                         takefocus=True, cursor="crosshair")
        self.limit = None
        self.on_change = on_change
        self.point = (0., 0.)
        self.bind("<Configure>", lambda _event: self.draw())
        self.bind("<Button-1>", self.pick)
        self.bind("<B1-Motion>", self.pick)
        self.bind("<KeyPress>", self.move_with_key)
    def set_data(self, limit, ax=0., ay=0.):
        self.limit = limit
        self.point = (0., 0.)
        if limit is not None and limit > 0.:
            scale = max(limit, abs(ax), abs(ay))
            x, y = ax / scale, ay / scale
            radius = max(limit / scale, hypot(x, y))
            self.point = (x / radius, y / radius)
        self.draw()

    def dimensions(self):
        width, height = self.winfo_width(), self.winfo_height()
        return width / 2., height / 2., max(1., width / 2. - 39.), max(1., height / 2. - 32.)

    def select(self, x, y):
        if self.limit is None:
            return
        if self.limit > 0.:
            scale = max(1., hypot(x, y))
            self.point = (x / scale, y / scale)
        self.on_change(*self.acceleration)

    @property
    def acceleration(self):
        return tuple(component * (self.limit or 0.) for component in self.point)

    def pick(self, event):
        self.focus_set()
        cx, cy, rx, ry = self.dimensions()
        self.select((cy - event.y) / ry, (event.x - cx) / rx)

    def move_with_key(self, event):
        ax, ay = self.point
        moves = {"Up": (.05, 0.), "Down": (-.05, 0.), "Left": (0., -.05), "Right": (0., .05)}
        if event.keysym == "Home":
            self.select(0., 0.)
        elif event.keysym in moves:
            dx, dy = moves[event.keysym]
            self.select(ax + dx, ay + dy)
        else:
            return None
        return "break"

    def draw(self):
        self.delete("all")
        if self.winfo_width() <= 2 or self.winfo_height() <= 2:
            return
        cx, cy, rx, ry = self.dimensions()
        if self.limit is not None and self.limit > 0.:
            self.create_oval(cx-rx, cy-ry, cx+rx, cy+ry, fill="#193348", outline="#68c6c3", width=2)
            self.create_oval(cx-rx/2, cy-ry/2, cx+rx/2, cy+ry/2, outline="#365a70", dash=(3, 3))
        self.create_line(cx-rx-12, cy, cx+rx+12, cy, fill="#a9bbcd")
        self.create_line(cx, cy+ry+12, cx, cy-ry-12, fill="#a9bbcd")
        for x, y, label in ((cx+rx+29, cy, "+ay"), (cx-rx-29, cy, "−ay"),
                            (cx, cy-ry-20, "+ax"), (cx, cy+ry+20, "−ax")):
            self.create_text(x, y, text=label, fill="#e5edf5", font=(FONT, 10))
        if self.limit is not None:
            ax, ay = self.point
            x, y = cx + ay * rx, cy - ax * ry
            self.create_oval(x-6, y-6, x+6, y+6, fill="#f5cf77", outline="#111d31", width=2)
