# -*- coding: utf-8 -*-
"""Generate the block and timing diagrams (SVG) used in the READMEs. No dependencies.

python gen_diagrams.py   -> writes *.svg next to this file

Visual language shared with the instructor's course material (nrf-manaus-2):
white ground, light panels, navy/blue for peripherals, green for the data
path, red for CPU/interrupts, amber for points of attention.
"""
import os

OUT = os.path.dirname(os.path.abspath(__file__))

W = 960
LEFT = 178          # left column with the signal names
RIGHT = W - 24
ROW_H = 54
FONT_FAMILY = "Segoe UI, Helvetica Neue, Helvetica, Arial, sans-serif"
MONO_FAMILY = "Consolas, Menlo, monospace"

# palette (nrf-manaus-2 theme tokens)
NAVY, BLUE, CYAN = "#00224E", "#0093D0", "#22C3E6"
INK, SLATE, LINE, PANEL, ZEBRA, BLUE_TINT = "#0B1B2B", "#566577", "#D3E0EB", "#EDF3F8", "#F3F7FB", "#E1F0FA"
OK, OK_TINT = "#1FA971", "#E3F5EC"
WARN, WARN_TINT = "#E8A33D", "#FDF3E3"
OFF, OFF_TINT = "#C94F4F", "#FBE9E9"

COL = {"sig": INK, "hw": BLUE, "cpu": OFF, "dma": OK, "grid": LINE, "muted": SLATE, "warn": WARN}
CHAR_W = 6.3  # rough average glyph width at 11 px, used to keep labels inside the canvas


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def font(size, mono=False, bold=False):
    fam = MONO_FAMILY if mono else FONT_FAMILY
    w = " font-weight='600'" if bold else ""
    return f"font-family='{fam}' font-size='{size}'{w}"


def header(title, subtitle):
    return (f"<text x='24' y='30' fill='{INK}' {font(16, bold=True)}>{esc(title)}</text>"
            f"<text x='24' y='50' fill='{SLATE}' {font(11.5)}>{esc(subtitle)}</text>"
            f"<line x1='24' y1='62' x2='{W-24}' y2='62' stroke='{LINE}' stroke-width='1'/>")


def defs():
    return (f"<defs>"
            f"<marker id='arr' markerWidth='9' markerHeight='9' refX='7' refY='4.5' orient='auto' markerUnits='userSpaceOnUse'>"
            f"<path d='M0,0.5 L8,4.5 L0,8.5 z' fill='{SLATE}'/></marker>"
            f"<marker id='arr-blue' markerWidth='9' markerHeight='9' refX='7' refY='4.5' orient='auto' markerUnits='userSpaceOnUse'>"
            f"<path d='M0,0.5 L8,4.5 L0,8.5 z' fill='{BLUE}'/></marker>"
            f"<marker id='arr-green' markerWidth='9' markerHeight='9' refX='7' refY='4.5' orient='auto' markerUnits='userSpaceOnUse'>"
            f"<path d='M0,0.5 L8,4.5 L0,8.5 z' fill='{OK}'/></marker>"
            f"<marker id='arr-red' markerWidth='9' markerHeight='9' refX='7' refY='4.5' orient='auto' markerUnits='userSpaceOnUse'>"
            f"<path d='M0,0.5 L8,4.5 L0,8.5 z' fill='{OFF}'/></marker>"
            f"<filter id='shadow' x='-5%' y='-5%' width='110%' height='120%'>"
            f"<feDropShadow dx='0' dy='1.5' stdDeviation='1.5' flood-color='{NAVY}' flood-opacity='0.12'/></filter>"
            f"<pattern id='hatch' width='6' height='6' patternUnits='userSpaceOnUse' patternTransform='rotate(45)'>"
            f"<line x1='0' y1='0' x2='0' y2='6' stroke='{LINE}' stroke-width='2'/></pattern>"
            f"</defs>")


def legend_svg(x, y, items):
    parts = []
    for color, text in items:
        parts.append(f"<rect x='{x}' y='{y-9}' width='14' height='9' rx='2' fill='{color}'/>")
        parts.append(f"<text x='{x+19}' y='{y}' fill='{SLATE}' {font(10.5)}>{esc(text)}</text>")
        x += 19 + CHAR_W * len(text) * 0.95 + 18
    return "".join(parts)


# --------------------------------------------------------------------------- timing
class Diagram:
    """Timing diagram: one row per signal, time on the horizontal axis."""

    def __init__(self, title, subtitle, t_end, rows, legend=None):
        self.title, self.subtitle, self.t_end = title, subtitle, t_end
        self.rows = rows
        self.legend = legend
        self.y0 = 100                      # baseline of the first row
        self.h = self.y0 + ROW_H * len(rows) + 30 + (28 if legend else 0)
        self.parts = []

    # geometry -------------------------------------------------------------
    def x(self, t):
        return LEFT + (RIGHT - LEFT) * t / self.t_end

    def yrow(self, name):
        return self.y0 + self.rows.index(name) * ROW_H

    def _fit(self, xc, s, anchor):
        """Shift a centred label so it stays inside the drawing area."""
        w = CHAR_W * len(s)
        if anchor == "middle":
            xc = max(LEFT + 4 + w / 2, min(RIGHT - 4 - w / 2, xc))
        elif anchor == "start":
            xc = min(RIGHT - 4 - w, xc)
        return xc

    # primitives -----------------------------------------------------------
    def text(self, x, y, s, color=INK, anchor="start", mono=False, size=11, bold=False):
        x = self._fit(x, s, anchor)
        self.parts.append(f"<text x='{x:.1f}' y='{y:.1f}' fill='{color}' text-anchor='{anchor}' "
                          f"{font(size, mono, bold)}>{esc(s)}</text>")

    def pulse(self, row, t0, t1, color=BLUE, label=None, amp=22, dashed=False, label_side="top"):
        y = self.yrow(row)
        x0, x1 = self.x(t0), self.x(t1)
        dash = " stroke-dasharray='4 3'" if dashed else ""
        self.parts.append(f"<path d='M{x0:.1f},{y:.1f} V{y-amp:.1f} H{x1:.1f} V{y:.1f}' fill='none' "
                          f"stroke='{color}' stroke-width='2' stroke-linejoin='round'{dash}/>")
        if label and label_side == "right":      # beside the pulse, clear of arrows coming from above
            self.text(x1 + 6, y - amp / 2 + 4, label, color, "start", size=10.5)
        elif label:
            self.text((x0 + x1) / 2, y - amp - 5, label, color, "middle", size=10.5)

    def level(self, row, t0, t1, high, color=BLUE):
        y = self.yrow(row) - (22 if high else 0)
        self.parts.append(f"<line x1='{self.x(t0):.1f}' y1='{y:.1f}' x2='{self.x(t1):.1f}' y2='{y:.1f}' "
                          f"stroke='{color}' stroke-width='2'/>")

    def edge(self, row, t, up, color=BLUE):
        y = self.yrow(row)
        self.parts.append(f"<line x1='{self.x(t):.1f}' y1='{y:.1f}' x2='{self.x(t):.1f}' y2='{y-22:.1f}' "
                          f"stroke='{color}' stroke-width='2'/>")

    def block(self, row, t0, t1, label, color=OK, fill=OK_TINT):
        y = self.yrow(row)
        x0, x1 = self.x(t0), self.x(t1)
        self.parts.append(f"<rect x='{x0:.1f}' y='{y-25:.1f}' width='{max(x1-x0, 2):.1f}' height='25' rx='3' "
                          f"fill='{fill}' stroke='{color}' stroke-width='1.4'/>")
        if label and CHAR_W * len(label) < (x1 - x0) - 6:
            self.text((x0 + x1) / 2, y - 8.5, label, color, "middle", size=10.5, bold=True)
        elif label:
            self.text((x0 + x1) / 2, y - 29, label, color, "middle", size=10.5)

    def arrow(self, t0, row0, t1, row1, label=None, color=SLATE):
        x0, y0 = self.x(t0), self.yrow(row0) - 22
        x1, y1 = self.x(t1), self.yrow(row1) - 26
        self.parts.append(f"<line x1='{x0:.1f}' y1='{y0:.1f}' x2='{x1:.1f}' y2='{y1:.1f}' stroke='{color}' "
                          f"stroke-width='1.3' stroke-dasharray='3 3' marker-end='url(#arr)'/>")
        if label:
            self.text((x0 + x1) / 2 + 8, (y0 + y1) / 2, label, color, "start", size=10.5)

    def note(self, t, row, s, color=WARN):
        """Annotation under a row, on a white backing so crossing lines do not cut through the text."""
        x = self._fit(self.x(t), s, "middle")
        y = self.yrow(row) + 15
        w = CHAR_W * len(s) * 0.92
        self.parts.append(f"<rect x='{x - w/2 - 3:.1f}' y='{y - 11}' width='{w + 6:.1f}' height='14' rx='3' fill='white' opacity='0.85'/>")
        self.text(x, y, s, color, "middle", size=10.5)

    def marker(self, t, label, color=SLATE):
        x = self.x(t)
        yb = self.y0 + ROW_H * len(self.rows) - 30
        self.parts.append(f"<line x1='{x:.1f}' y1='{self.y0-30}' x2='{x:.1f}' y2='{yb+8}' "
                          f"stroke='{color}' stroke-width='1' stroke-dasharray='2 4'/>")
        if label:                          # label under the time axis, next to the marker
            self.text(x + 4, yb + 22, label, color, "start", size=10.5)

    # output ---------------------------------------------------------------
    def save(self, name):
        bg = []
        top = self.y0 - 34
        # zebra bands and baselines
        for i, r in enumerate(self.rows):
            y = self.y0 + i * ROW_H
            if i % 2 == 0:
                bg.append(f"<rect x='{LEFT-6}' y='{y-34}' width='{RIGHT-LEFT+6}' height='{ROW_H}' fill='{ZEBRA}'/>")
            bg.append(f"<line x1='{LEFT}' y1='{y}' x2='{RIGHT}' y2='{y}' stroke='{LINE}' stroke-width='1'/>")
            label = r.strip()
            if CHAR_W * len(label) > LEFT - 26:      # two lines, split near the middle
                words = label.split(" ")
                best, first = None, ""
                for i in range(1, len(words)):
                    a, b = " ".join(words[:i]), " ".join(words[i:])
                    score = abs(len(a) - len(b))
                    if best is None or score < best:
                        best, first = score, a
                lines = [first, label[len(first) + 1:]]
            else:
                lines = [label]
            for j, s in enumerate(lines):
                yy = y - 7 - (len(lines) - 1 - j) * 13
                bg.append(f"<text x='{LEFT-12}' y='{yy}' text-anchor='end' fill='{INK}' {font(11.5, bold=True)}>{esc(s)}</text>")
        # light time grid
        ticks = 10
        for k in range(ticks + 1):
            x = LEFT + (RIGHT - LEFT) * k / ticks
            bg.append(f"<line x1='{x:.1f}' y1='{top}' x2='{x:.1f}' y2='{self.y0 + ROW_H*len(self.rows) - 30}' "
                      f"stroke='{LINE}' stroke-width='0.8' stroke-dasharray='1 5'/>")
        yb = self.y0 + ROW_H * len(self.rows) - 30
        axis = (f"<line x1='{LEFT}' y1='{yb+8}' x2='{RIGHT}' y2='{yb+8}' stroke='{SLATE}' stroke-width='1' marker-end='url(#arr)'/>"
                f"<text x='{RIGHT}' y='{yb+22}' text-anchor='end' fill='{SLATE}' {font(10.5)}>tempo</text>")
        leg = legend_svg(24, self.h - 8, self.legend) if self.legend else ""
        svg = (f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{self.h}' viewBox='0 0 {W} {self.h}'>"
               f"{defs()}<rect width='100%' height='100%' fill='white'/>"
               f"{header(self.title, self.subtitle)}{''.join(bg)}{''.join(self.parts)}{axis}{leg}</svg>")
        with open(os.path.join(OUT, name), "w", encoding="utf-8") as fh:
            fh.write(svg)
        print("wrote", name)


LEGEND_TIMING = [(BLUE, "periférico (evento/tarefa)"), (OK, "EasyDMA / dados"), (OFF, "CPU / interrupção"),
                 (SLATE, "sensor / externo"), (WARN, "ponto de atenção")]


# --------------------------------------------------------------------------- blocks
class Blocks:
    """Block/connection diagram: rounded boxes and orthogonal connectors, no time axis."""

    def __init__(self, title, subtitle, h=440):
        self.title, self.subtitle, self.h = title, subtitle, h
        self.parts, self.boxes = [], {}

    def box(self, key, x, y, w, h, lines, color=BLUE, fill=BLUE_TINT):
        self.boxes[key] = (x, y, w, h)
        self.parts.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='8' fill='{fill}' stroke='{color}' "
                          f"stroke-width='1.5' filter='url(#shadow)'/>")
        n = len(lines)
        for i, s in enumerate(lines):
            yy = y + h / 2 + (i - (n - 1) / 2) * 15 + 4.5
            col = color if i == 0 else INK
            self.parts.append(f"<text x='{x + w/2}' y='{yy:.1f}' text-anchor='middle' fill='{col}' "
                              f"{font(12 if i == 0 else 10.5, bold=(i == 0))}>{esc(s)}</text>")

    def _edge(self, key, side):
        x, y, w, h = self.boxes[key]
        return {"l": (x, y + h / 2), "r": (x + w, y + h / 2), "t": (x + w / 2, y), "b": (x + w / 2, y + h)}[side]

    def link(self, a, b, label="", color=SLATE, dashed=False, label_dy=-7):
        """Orthogonal connector from a to b, chosen by relative position (boxes must not overlap)."""
        ax, ay, aw, ah = self.boxes[a]
        bx, by, bw, bh = self.boxes[b]
        marker = {SLATE: "arr", BLUE: "arr-blue", OK: "arr-green", OFF: "arr-red"}.get(color, "arr")
        dash = " stroke-dasharray='5 3'" if dashed else ""
        if bx >= ax + aw:                    # b to the right
            (x0, y0), (x1, y1) = self._edge(a, "r"), self._edge(b, "l")
        elif bx + bw <= ax:                  # b to the left
            (x0, y0), (x1, y1) = self._edge(a, "l"), self._edge(b, "r")
        elif by >= ay + ah:                  # b below
            (x0, y0), (x1, y1) = self._edge(a, "b"), self._edge(b, "t")
        else:                                # b above
            (x0, y0), (x1, y1) = self._edge(a, "t"), self._edge(b, "b")
        if abs(y0 - y1) < 1 or abs(x0 - x1) < 1:
            d = f"M{x0},{y0} L{x1},{y1}"
            lx, ly = (x0 + x1) / 2, (y0 + y1) / 2
        else:                                # elbow: horizontal first, then vertical
            xm = (x0 + x1) / 2
            d = f"M{x0},{y0} H{xm} V{y1} H{x1}"
            lx, ly = xm, (y0 + y1) / 2
        self.parts.append(f"<path d='{d}' fill='none' stroke='{color}' stroke-width='1.6'{dash} marker-end='url(#{marker})'/>")
        if label:
            if abs(y0 - y1) < 1:             # horizontal: label lines stacked above the line
                lines = label.split("\n")
                for j, s in enumerate(lines):
                    yy = ly + label_dy - (len(lines) - 1 - j) * 13
                    self.parts.append(f"<text x='{lx}' y='{yy}' text-anchor='middle' fill='{color}' "
                                      f"{font(10.5)}>{esc(s)}</text>")
            else:                            # vertical: label beside the line, on a light panel
                w = CHAR_W * len(label) + 10
                self.parts.append(f"<rect x='{lx + 8}' y='{ly - 9}' width='{w:.0f}' height='16' rx='3' fill='white' opacity='0.9'/>")
                self.parts.append(f"<text x='{lx + 13}' y='{ly + 3.5}' fill='{color}' {font(10.5)}>{esc(label)}</text>")

    def caption(self, y, s, color=INK):
        self.parts.append(f"<text x='24' y='{y}' fill='{color}' {font(11)}>{esc(s)}</text>")

    def band(self, x, y, w, h, title, sub):
        self.parts.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='8' fill='{NAVY}'/>")
        self.parts.append(f"<text x='{x + w/2}' y='{y + 20}' text-anchor='middle' fill='white' {font(12, bold=True)}>{esc(title)}</text>")
        self.parts.append(f"<text x='{x + w/2}' y='{y + 37}' text-anchor='middle' fill='{CYAN}' {font(10.5)}>{esc(sub)}</text>")

    def save(self, name, legend=None):
        leg = legend_svg(24, self.h - 10, legend) if legend else ""
        svg = (f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{self.h}' viewBox='0 0 {W} {self.h}'>"
               f"{defs()}<rect width='100%' height='100%' fill='white'/>"
               f"{header(self.title, self.subtitle)}{''.join(self.parts)}{leg}</svg>")
        with open(os.path.join(OUT, name), "w", encoding="utf-8") as fh:
            fh.write(svg)
        print("wrote", name)


LEGEND_BLOCKS = [(BLUE, "periférico / DPPI"), (OK, "caminho dos dados (EasyDMA)"), (OFF, "CPU / interrupção"),
                 (SLATE, "externo ao SoC")]


def _drain_side(d, Y1, Y2, Y3, BH, filt):
    """Right-hand side shared by the two block diagrams: SPIM, RAM ring, wrap IRQ, drain thread, queue,
    and the per-sample alternative (dashed)."""
    d.box("spim", 606, Y1, 172, BH, ["SPIM (CSN por HW)", "TASKS_START", "EasyDMA TX/RX"], OK, OK_TINT)
    d.box("ram", 838, Y1, 98, BH, ["RAM: anel", "APP_RING_SLOTS", "+ 8 de guarda"], OK, OK_TINT)
    d.box("wrap", 600, Y2, 190, BH, ["IRQ DMA.RX.READY", "(STARTED no nRF5340)", "1 por volta: PTR = slot 0"], OFF, OFF_TINT)
    d.box("drain", 826, Y2, 110, BH, ["Thread", "k_sleep(T)", "lê o head (PTR)", "arma o wrap 1×/volta"], OFF, OFF_TINT)
    d.box("msgq", 700, Y3, 236, BH, ["k_msgq (APP_QUEUE_DEPTH)",
                                     "put: filtra fresh, só novas" if filt else "put: todos os slots novos",
                                     "consumidor: k_msgq_get, em ordem"], INK, PANEL)
    d.box("persample", 400, Y3, 170, BH, ["IRQ END (por amostra)", "APP_PER_SAMPLE_IRQ:", "1 buffer, sem anel/wrap",
                                          "copia → fila; torn"], OFF, OFF_TINT)
    d.link("spim", "ram", "DMA", OK)
    d.link("spim", "wrap", "READY → IRQ (só quando armada)", OFF)
    d.link("ram", "drain", "slots novos", OK)
    d.link("drain", "wrap", "arma (head ≥ anel/2)", OFF)
    d.link("drain", "msgq", "k_msgq_put", OFF)
    # per-sample path, dashed: down the left of the wrap box so that no label lands on it
    d.parts.append(f"<path d='M630,{Y1+BH} V{Y1+BH+30} H485 V{Y3}' fill='none' stroke='{OFF}' stroke-width='1.6' "
                   f"stroke-dasharray='5 3' marker-end='url(#arr-red)'/>")
    d.parts.append(f"<text x='557' y='{Y1+BH+24}' text-anchor='middle' fill='{OFF}' {font(10.5)}>{esc('END (modo por amostra)')}</text>")
    d.link("persample", "msgq", "1 por amostra", OFF, dashed=True)


def b1_blocks_int():
    d = Blocks("Blocos — caso 1: data-ready do sensor → GPIOTE → DPPI → SPIM (gpiote_dppi_spim)",
               "Um canal DPPI; o EasyDMA enche o anel sozinho e a CPU entra uma vez por período de drenagem T (ou uma vez por amostra, tracejado)", h=560)
    Y1, Y2, Y3, BH = 96, 236, 360, 84
    d.box("sensor", 24, Y1, 156, BH, ["Acelerômetro", "ADXL362 / BMI270 / ADXL382", "INT = data-ready (nível)"], SLATE, PANEL)
    d.box("gpiote", 240, Y1, 136, BH, ["GPIOTE", "IN[n] event", "borda ↑ (ou ↓) do pino"])
    d.box("dppi", 436, Y1, 112, BH, ["DPPI", "canal único", "EEP → TEP"])
    _drain_side(d, Y1, Y2, Y3, BH, filt=False)
    d.link("sensor", "gpiote", "pino INT")
    d.link("gpiote", "dppi", "evento", BLUE)
    d.link("dppi", "spim", "tarefa\nSTART", BLUE)
    for i, s in enumerate(["Modo drenado, a cada T (APP_DRAIN_PERIOD_US, 10 ms):",
                           "1. acorda (k_sleep) e lê DMA.RX.PTR: head = próximo slot;",
                           "2. k_msgq_put dos slots [tail, head−1);",
                           "   (com ≤ 4 pendentes espera 1 transação e entrega o head−1);",
                           "3. se head ≥ anel/2, habilita a IRQ de READY uma vez:",
                           "   a ISR devolve PTR ao slot 0 logo após READY (wrap);",
                           "4. amostras a < 64 µs: espera o wrap acordada (≤ min(T/4, 8 períodos)).",
                           "IRQ/s = 1/T + 1 por volta do anel. Latência ≤ T real (≤ 4 pendentes); acima, a mais nova sai na drenagem seguinte (T real + 1 período).",
                           "Modo por amostra: IRQ de END copia cada rajada para a fila; latência de",
                           "uma ISR, uma IRQ por amostra (fórmula ≈ 36 / 41 µs; medido limpo até 25 k/s na TAG, 20 k/s na Thingy, M)."]):
        d.caption(Y2 + 14 + i * 17, s, INK if i in (0, 8) else SLATE)
    d.caption(478, "Partida: 1 START por software depois de ligar o DPPI (o data-ready já está alto; é nível, não pulso).")
    d.caption(498, "Nada além do GPIOTE do pino fica fora do domínio da SPIM: uma variante 100 % LP (GPIOTE30 + SPIM30) é só um overlay.")
    d.save("blocos_caso1_sensor_int.svg", LEGEND_BLOCKS)


def b2_blocks_timer():
    d = Blocks("Blocos — caso 2: TIMER → DPPI → SPIM (timer_dppi_spim)",
               "Igual ao caso 1 com o TIMER no lugar do GPIOTE; o sensor não participa do disparo", h=560)
    Y1, Y2, Y3, BH = 96, 236, 360, 84
    d.box("hfxo", 24, Y1, 156, BH, ["HFXO", "pedido via onoff", "(FLPR: hfxo_launcher)"], SLATE, PANEL)
    d.box("trig", 240, Y1, 136, BH, ["TIMER (disparo)", "1 MHz, CC0 = período", "short COMPARE0→CLEAR"])
    d.box("dppi", 436, Y1, 112, BH, ["DPPI", "canal único", "EEP → TEP"])
    _drain_side(d, Y1, Y2, Y3, BH, filt=True)
    d.link("hfxo", "trig", "clock\nexato")
    d.link("trig", "dppi", "COMPARE0", BLUE)
    d.link("dppi", "spim", "tarefa\nSTART", BLUE)
    for i, s in enumerate(["Mesma entrega do caso 1 (drenagem a cada T, ou IRQ de END).",
                           "Diferenças: os registradores do sensor guardam sempre",
                           "a última amostra, então ler em loop basta; a entrega lê",
                           "o bit de data-ready no STATUS da própria rajada e descarta",
                           "as repetidas (APP_QUEUE_FRESH_ONLY, skipped).",
                           "Timer acima do ODR: repetidas com DATA_READY = 0.",
                           "Timer abaixo do ODR real: perde amostras sem rastro —",
                           "manter 5–10 % acima do ODR nominal.",
                           "Bancada: APP_WRAP_LATENCY_STATS mede trigger → ISR."]):
        d.caption(Y2 + 14 + i * 17, s, INK if i == 0 else SLATE)
    d.caption(478, "Custo sobre o caso 1: TIMER de disparo + HFXO (155 µA, D) e as transações repetidas no barramento.")
    d.save("blocos_caso2_timer.svg", LEGEND_BLOCKS)


# --------------------------------------------------------------------------- timing figures
def d1_sensor_int():
    d = Diagram("Caso 1 — disparo pelo data-ready do sensor (INT → GPIOTE → DPPI → SPIM)",
                "BMI270 na TAG a 1 600 Hz: 1 transação por amostra nova, CPU fora do caminho; escala ≈ 625 µs por período (3 períodos no desenho)",
                6.0, ["amostra no sensor", "INT1 (data-ready)", "GPIOTE IN event", "DPPI ch", "SPIM START/END",
                      "CSN (hardware)", "EasyDMA → RAM", "CPU"], LEGEND_TIMING)
    for k, t in enumerate([0.4, 2.9, 5.4]):
        d.pulse("amostra no sensor", t, t + 0.05, SLATE, f"n+{k}")
        d.level("INT1 (data-ready)", t - 0.02 if k else 0, t, False)
        d.edge("INT1 (data-ready)", t, True)
        d.level("INT1 (data-ready)", t, t + 0.55, True)
        d.edge("INT1 (data-ready)", t + 0.55, False)
        d.level("INT1 (data-ready)", t + 0.55, min(t + 2.5, 6.0), False)
        d.pulse("GPIOTE IN event", t + 0.02, t + 0.06)
        d.pulse("DPPI ch", t + 0.05, t + 0.09)
        d.pulse("SPIM START/END", t + 0.08, t + 0.12, label="START")
        d.pulse("SPIM START/END", t + 0.52, t + 0.56, label="END")
        d.level("CSN (hardware)", (t - 2.5 + 0.56) if k else 0, t + 0.1, True)
        d.edge("CSN (hardware)", t + 0.1, False)
        d.level("CSN (hardware)", t + 0.1, t + 0.54, False)
        d.edge("CSN (hardware)", t + 0.54, True)
        d.level("CSN (hardware)", t + 0.54, min(t + 2.6, 6.0), True)
        d.block("EasyDMA → RAM", t + 0.12, t + 0.52, "17 B @ 8 MHz")
    d.level("CPU", 0, 6.0, False, OFF)
    d.note(3.0, "CPU", "modo drenado: dorme; acorda a cada T (10 ms) e uma vez por volta do anel. Modo por amostra: uma IRQ de END por transação", OFF)
    d.note(1.6, "INT1 (data-ready)", "nível: só desce quando os registradores de dados são lidos", WARN)
    d.arrow(0.42, "INT1 (data-ready)", 0.5, "SPIM START/END", "sem CPU")
    d.save("caso1_sensor_int.svg")


def d2_timer():
    d = Diagram("Caso 2 — disparo por TIMER (COMPARE → DPPI → SPIM), timer mais rápido que o ODR",
                "Timer mais rápido que o sensor (escala comprimida: 3 novas em 7 leituras; a 100 µs contra 400 Hz seria 1 em 25): as repetidas levam DATA_READY = 0",
                7.0, ["amostra no sensor", "TIMER COMPARE0", "DPPI ch", "SPIM START/END", "EasyDMA → RAM",
                      "STATUS.DATA_READY lido", "CPU"], LEGEND_TIMING)
    for k in range(7):
        t = 0.3 + k
        d.pulse("TIMER COMPARE0", t, t + 0.05)
        d.pulse("DPPI ch", t + 0.03, t + 0.08)
        d.pulse("SPIM START/END", t + 0.06, t + 0.1)
        d.block("EasyDMA → RAM", t + 0.1, t + 0.5, "")
    for t in [0.9, 3.4, 5.9]:
        d.pulse("amostra no sensor", t, t + 0.05, SLATE)
    fresh = {1: True, 3: True, 6: True}
    for k in range(7):
        t = 0.3 + k + 0.2
        f = fresh.get(k, False)
        d.pulse("STATUS.DATA_READY lido", t, t + 0.25, OK if f else SLATE,
                "1 (nova)" if f else "0 (repetida)", amp=22 if f else 8)
    d.level("CPU", 0, 7.0, False, OFF)
    d.note(3.5, "CPU", "sem ISR por transação; a drenagem (a cada T) descarta as repetidas (APP_QUEUE_FRESH_ONLY)", OFF)
    d.marker(0.3, "período do timer")
    d.marker(1.3, "")
    d.save("caso2_timer.svg")


def d3_drain():
    """Ring filled by the EasyDMA, drained every T by a thread; the wrap is armed once the head is past
    half of the ring and done by the READY IRQ."""
    d = Diagram("Anel, drenagem e wrap — a CPU entra uma vez por período de drenagem T, nunca por transação",
                "8 slots no desenho. A cada T a thread lê o head (DMA.RX.PTR), entrega os slots completos e, se head ≥ 4 (metade), arma a IRQ de READY, que devolve o PTR ao slot 0",
                13.0, ["SPIM START → READY", "DMA.RX.PTR (array list)", "thread de drenagem (a cada T)",
                       "IRQ READY (wrap)", "k_msgq"], LEGEND_TIMING)
    for k in range(13):
        t = 0.1 + k
        d.pulse("SPIM START → READY", t, t + 0.06, label=str(k) if k in (7, 8) else None)
    # transaction k writes slot k for k <= 7; armed at head 6 (READY of 6 cleared by
    # arm_wrap), the wrap fires at READY of 7 and sends 8 to slot 0
    slots = [0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4]
    laps = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1]
    for k, (s, lap) in enumerate(zip(slots, laps)):
        t = 0.1 + k
        col, tint = (OK, OK_TINT) if lap % 2 == 0 else (BLUE, BLUE_TINT)
        d.block("DMA.RX.PTR (array list)", t + 0.08, t + 0.9, f"slot {s}", col, tint)
    d.pulse("thread de drenagem (a cada T)", 2.5, 3.3, OFF, "head 3, 3 pendentes: espera 1 transação; o START da 3 prova que a 2 acabou → put 0..2; não arma (3 < 4)", label_side="right")
    d.pulse("thread de drenagem (a cada T)", 5.5, 6.3, OFF, "head 6: espera, START da 6 → put 3..5; arma (6 ≥ 4) e limpa o READY da 6", label_side="right")
    d.pulse("thread de drenagem (a cada T)", 8.5, 9.3, OFF, "head 1 da volta nova: put 6..7 (fim da volta antiga); espera → put 0", label_side="right")
    d.pulse("thread de drenagem (a cada T)", 11.5, 12.3, OFF, "head 4: espera → put 1..3; arma (4 ≥ 4)", label_side="right")
    d.pulse("IRQ READY (wrap)", 7.15, 7.4, OFF, "READY da 7 (head = 8): PTR = slot 0 → a transação 8 escreve o slot 0; wrap_last = 7", label_side="right")
    d.block("k_msgq", 3.3, 13.0, "consumidor: k_msgq_get, uma amostra por vez, em ordem; latência ≤ T real com ≤ 4 pendentes; com mais, o slot mais novo sai na drenagem seguinte", INK, PANEL)
    d.note(3.4, "IRQ READY (wrap)", "janela do datasheet: logo após READY, antes do próximo START (o hardware reescreve o PTR a cada START)", WARN)
    d.note(2.4, "DMA.RX.PTR (array list)", "sem CPU: o EasyDMA avança um slot por transação; a volta antiga fica meio anel à frente da nova. Com ≤ 4 pendentes a drenagem espera 1 transação (settle) e entrega também o head−1", OK)
    d.arrow(5.7, "thread de drenagem (a cada T)", 7.15, "IRQ READY (wrap)")
    d.arrow(7.13, "SPIM START → READY", 7.18, "IRQ READY (wrap)")
    d.marker(2.5, "T (APP_DRAIN_PERIOD_US)")
    d.marker(5.5, "")
    d.marker(8.5, "")
    d.marker(11.5, "")
    d.save("anel_drenagem.svg")


def d7_per_sample():
    """Per-sample mode: one buffer, the END IRQ copies the burst into the queue; the torn window."""
    d = Diagram("Modo por amostra (APP_PER_SAMPLE_IRQ) — um buffer, uma IRQ de END por transação",
                "Esquerda: período confortável, a cópia acaba antes do próximo START. Direita: período curto demais, o START seguinte atropela a cópia (torn)",
                8.0, ["START (data-ready / timer)", "SPIM: transação", "IRQ END: memcpy + k_msgq_put", "k_msgq", "torn"],
                LEGEND_TIMING)
    for k, t in enumerate([0.3, 2.3]):
        d.pulse("START (data-ready / timer)", t, t + 0.05, label=f"k+{k}")
        d.block("SPIM: transação", t + 0.05, t + 0.75, "rajada → buffer")
        d.block("IRQ END: memcpy + k_msgq_put", t + 0.8, t + 1.1, "ISR", OFF, OFF_TINT)
        d.pulse("k_msgq", t + 1.1, t + 1.15, OK, f"amostra k+{k}")
    d.note(1.3, "IRQ END: memcpy + k_msgq_put", "≈ 1,2 µs acordado (entrada média 0,3–6 µs conforme o intervalo); entrada máx. 15,5 µs no M33 do nRF54L15, 26,3 µs no nRF5340 (M). Um START antes de a ISR entrar não é detectado", OFF)
    d.marker(4.4, "")
    for k, t in enumerate([4.7, 5.55, 6.4]):
        d.pulse("START (data-ready / timer)", t, t + 0.05, WARN if k else BLUE, label=f"j+{k}")
        d.block("SPIM: transação", t + 0.05, t + 0.75, "", OK if k == 0 else WARN, OK_TINT if k == 0 else WARN_TINT)
    d.block("IRQ END: memcpy + k_msgq_put", 4.7 + 0.8, 4.7 + 1.1, "ISR", OFF, OFF_TINT)
    d.block("torn", 5.55, 5.8, "", WARN, WARN_TINT)
    d.note(6.3, "torn", "o START de j+1 chega durante a cópia de j: READY reaparece, a amostra é descartada (torn++)", WARN)
    d.note(6.3, "k_msgq", "vale enquanto período > transação + entrada máx. da ISR + 2 µs: ≈ 36 µs na TAG, ≈ 41 µs na Thingy (E); medido limpo até 40 µs nos dois (M)", OK)
    d.save("por_amostra.svg")


def d4_bus_limit():
    d = Diagram("Teto do barramento — quando o período do timer é menor que a transação",
                "Tag: 17 B a 8 MHz = 17 µs + START + CSNDUR ≈ 18,5 µs; a 19 µs cabe (52,6 k/s, comprovado), a 18 µs o START chega com a SPIM ocupada (não comprovado)",
                6.0, ["TIMER (19 µs)", "SPIM ocupada", "TIMER (18 µs)", "SPIM ocupada ", "SPIM ocupada  "], LEGEND_TIMING)
    for k in range(3):
        t = 0.3 + k * 1.9
        d.pulse("TIMER (19 µs)", t, t + 0.05)
        d.block("SPIM ocupada", t + 0.05, t + 1.85, "≈ 18,5 µs")
    d.note(3.0, "SPIM ocupada", "margem ≈ 0,5 µs entre o fim da transação e o próximo START: 52 623 transações/s, late_wraps = 0, Z variando", OK)
    for k in range(3):
        t = 0.3 + k * 1.8
        d.pulse("TIMER (18 µs)", t, t + 0.05, WARN if k else BLUE)
    d.block("SPIM ocupada ", 0.35, 2.15, "≈ 18,5 µs")
    d.note(3.4, "SPIM ocupada ", "START com a SPIM ocupada: o ponteiro segue avançando (xfers 55,6 k/s), o bit fresh sai do ODR (363/s) e a faixa de Z estreita (0,58–0,62);", OFF)
    d.note(3.4, "SPIM ocupada  ", "no nRF5340 o START reinicia a transação: nada passa pelo filtro (fresh 0) ou, numa captura anterior, Z congelado. O critério é o conteúdo, não a contagem", OFF)
    d.save("teto_barramento.svg")


def d5_oversampling():
    d = Diagram("Timer × ODR — a margem mínima que não perde amostra (Tag, ODR real 401,8/s)",
                "Dois relógios livres: abaixo do ODR perde em silêncio; ~1 % acima nunca perde e descarta as repetidas",
                10.0, ["amostras do sensor (402/s)", "timer 2500 µs (400/s)", "novas lidas", "timer 2475 µs (404/s)",
                       "novas lidas "], LEGEND_TIMING)
    for k in range(10):
        t = 0.2 + k * 0.995
        d.pulse("amostras do sensor (402/s)", t, t + 0.04, SLATE)
    for k in range(10):
        t = 0.6 + k * 1.0
        d.pulse("timer 2500 µs (400/s)", t, t + 0.04)
    for k in range(10):
        t = 0.6 + k * 1.0
        lost = k == 7
        d.pulse("novas lidas", t + 0.05, t + 0.3, OFF if lost else OK, "perdida" if lost else "", amp=8 if lost else 18)
    for k in range(10):
        t = 0.6 + k * 0.99
        d.pulse("timer 2475 µs (404/s)", t, t + 0.04)
    for k in range(10):
        t = 0.6 + k * 0.99
        rep = k == 6
        d.pulse("novas lidas ", t + 0.05, t + 0.3, SLATE if rep else OK, "repetida" if rep else "", amp=8 if rep else 18)
    d.note(5.0, "novas lidas", "fresh = 399,8/s < 401,8: ~2 amostras/s desaparecem sem deixar rastro no flag (skipped = 0)", OFF)
    d.note(5.0, "novas lidas ", "fresh = 401,6/s: nenhuma perdida; as repetidas (skipped, 19 em 9 s) são descartadas na drenagem", OK)
    d.save("timer_vs_odr.svg")


def d6_adxl382():
    """Expected timing for the ADXL382 at 64 kHz ODR, data-ready driven (case 1 at the maximum rate)."""
    P = 15.625  # us, 64 kHz
    d = Diagram("Caso ADXL382 a 64 kHz (INT0 → GPIOTE → DPPI → SPIM) — não testado em hardware, esperado",
                "Período 15,6 µs; 11 B por amostra: 12,5 µs a 8 MHz (80 %), 7 µs a 16 MHz (45 %, SPIM4 nRF5340), 4,25 µs a 32 MHz (27 %, SPIM00 nRF54L15)",
                3 * P + 4, ["amostra ADXL382 (64 kHz)", "INT0 (DATA_READY)", "GPIOTE IN → DPPI", "SPIM 8 MHz: 11 B",
                            "SPIM 16 MHz: 11 B", "SPIM 32 MHz: 11 B", "wrap (IRQ READY)"], LEGEND_TIMING)
    for k in range(3):
        t = 1.0 + k * P
        d.pulse("amostra ADXL382 (64 kHz)", t, t + 0.3, SLATE, f"n+{k}")
        d.edge("INT0 (DATA_READY)", t, True)
        d.level("INT0 (DATA_READY)", t, t + 0.5 + 12.5, True)
        d.edge("INT0 (DATA_READY)", t + 13.0, False)
        d.level("INT0 (DATA_READY)", t + 13.0, t + P, False)
        d.pulse("GPIOTE IN → DPPI", t + 0.1, t + 0.4)
        d.block("SPIM 8 MHz: 11 B", t + 0.5, t + 0.5 + 12.5, "12,5 µs (11 B + START + CSN)")
        d.block("SPIM 16 MHz: 11 B", t + 0.5, t + 0.5 + 7.0, "7 µs", BLUE, BLUE_TINT)
        d.block("SPIM 32 MHz: 11 B", t + 0.5, t + 0.5 + 4.25, "4,25 µs", BLUE, BLUE_TINT)
    d.note(P + 8.5, "SPIM 8 MHz: 11 B", "80 % do barramento, ≈ 3 µs até o próximo START (nRF5340 medido: 71,4 k/s válidos com 11 B a 14 µs)", OK)
    d.note(P + 8.5, "SPIM 16 MHz: 11 B", "45 %: margem para CSNDUR maior e para o jitter do ODR do sensor", BLUE)
    d.note(P + 8.5, "SPIM 32 MHz: 11 B", "27 %: só na SPIM00 (domínio MCU); a errata 8 não se aplica ao ADXL382 (1º byte 0x23, MSB 0)", BLUE)
    d.pulse("wrap (IRQ READY)", 1.0 + P + 0.5, 1.0 + P + 3.0, OFF,
            "1 wrap por volta do anel (≈ 138 amostras a T = 1 ms), na IRQ de READY com o core acordado: 1,1–2,2 µs na TAG, 1,8–2,3 µs na Thingy (M); prazo = um período", amp=14)
    d.note(P + 8.5, "wrap (IRQ READY)", "sem ZLI, sem RRAM standby: a drenagem espera acordada (período < 64 µs); nRF5340 medido a 14 µs: 0 late. Modo por amostra: fora da faixa (15,6 < 12,5 + 15,5 ou 26 + 2)", OFF)
    d.marker(1.0, "15,6 µs")
    d.marker(1.0 + P, "")
    d.save("caso_adxl382_64k.svg")


def c1_m33_vs_flpr():
    """Bar chart: trigger -> ISR latency of an IRQ from idle, Cortex-M33 vs FLPR on the nRF54L15."""
    h = 486
    p = [defs(), f"<rect width='100%' height='100%' fill='white'/>",
         header("nRF54L15 — Cortex-M33 × FLPR (RISC-V): latência de uma IRQ saindo de idle (trigger → ISR)",
                "nRF54L15 TAG, timer_dppi_spim com APP_WRAP_LATENCY_STATS: do COMPARE do trigger até a ISR de wrap; média e máximo (M)")]
    # ---- panel A: latency bars
    ax, ay, aw, ah = 60, 100, 540, 260
    ymax = 20.0
    p.append(f"<text x='{ax}' y='{ay-14}' fill='{INK}' {font(12, bold=True)}>Latência trigger → ISR de wrap (µs)</text>")
    for v in (0, 5, 10, 15, 20):
        y = ay + ah - ah * v / ymax
        p.append(f"<line x1='{ax}' y1='{y:.1f}' x2='{ax+aw}' y2='{y:.1f}' stroke='{LINE}' stroke-width='1' stroke-dasharray='2 4'/>")
        p.append(f"<text x='{ax-8}' y='{y+4:.1f}' text-anchor='end' fill='{SLATE}' {font(10.5)}>{v}</text>")
    bars = [("M33 padrão, idle\n(atual, 1 ms, com log)", 16.07, 16.31, OFF, OFF_TINT),
            ("M33 acordado\n(atual, ≤ 50 µs, com log)", 1.81, 2.18, WARN, WARN_TINT),
            ("M33 padrão ou constlat\n(anterior, sem log)", 16.8, 17.3, OFF, OFF_TINT),
            ("M33 + RRAM standby\n(anterior, sem log)", 2.75, 2.93, OK, OK_TINT),
            ("FLPR, RAM\n(anterior, sem log)", 2.43, 2.50, BLUE, BLUE_TINT)]
    bw = 62
    gap = (aw - bw * len(bars)) / (len(bars) + 1)
    for i, (name, avg, mx, col, tint) in enumerate(bars):
        x = ax + gap + i * (bw + gap)
        y_avg = ay + ah - ah * avg / ymax
        y_max = ay + ah - ah * mx / ymax
        p.append(f"<rect x='{x:.1f}' y='{y_max:.1f}' width='{bw}' height='{ah - (y_max - ay):.1f}' rx='3' fill='{tint}'/>")
        p.append(f"<rect x='{x:.1f}' y='{y_avg:.1f}' width='{bw}' height='{ah - (y_avg - ay):.1f}' rx='3' fill='{col}' filter='url(#shadow)'/>")
        p.append(f"<text x='{x + bw/2:.1f}' y='{y_max - 6:.1f}' text-anchor='middle' fill='{col}' {font(11, bold=True)}>{f'{avg:.1f}'.replace('.', ',')} / {f'{mx:.1f}'.replace('.', ',')}</text>")
        for k, line in enumerate(name.split("\n")):
            p.append(f"<text x='{x + bw/2:.1f}' y='{ay + ah + 16 + 13*k}' text-anchor='middle' fill='{INK}' {font(10.5)}>{esc(line)}</text>")
    p.append(f"<line x1='{ax}' y1='{ay+ah}' x2='{ax+aw}' y2='{ay+ah}' stroke='{SLATE}' stroke-width='1'/>")
    p.append(legend_svg(ax, ay + ah + 52, [(INK, "barra cheia = média"), (LINE, "barra clara = máximo"),
                                          (OFF, "atual = u_tag_wrap_latency.log (M); anterior = mecanismo de entrega anterior, log não incluído")]))
    # ---- panel B: notes
    bx, by = 640, 100
    box_h = 278
    p.append(f"<rect x='{bx}' y='{by}' width='{W-24-bx}' height='{box_h}' rx='8' fill='{PANEL}' stroke='{LINE}' filter='url(#shadow)'/>")
    lines = [
        (INK, True, "O que os 16 µs são"),
        (SLATE, False, "Core em idle: a RRAM fica em power-down e"),
        (SLATE, False, "a 1ª instrução da ISR espera 13 µs (tIDLE2CPU)."),
        (SLATE, False, "Constant latency sozinho não muda isso."),
        (SLATE, False, "RRAM em standby: 2,75 µs. FLPR (RAM): 2,43 µs."),
        (INK, True, "Quando importa"),
        (SLATE, False, "Para uma IRQ com prazo < ~18 µs que chegue com o"),
        (SLATE, False, "core dormindo, e para o custo de cada drenagem."),
        (SLATE, False, "O wrap não depende disso: com amostras a < 64 µs"),
        (SLATE, False, "a thread espera acordada e o wrap sai em 1,1–2,2 µs."),
        (INK, True, "Modo por amostra (ISR de END, M)"),
        (SLATE, False, "≈ 1,2 µs acordado, entrada até 15,5 µs de idle (média"),
        (SLATE, False, "0,3–6 µs); vale enquanto período > transação + 15,5 + 2 µs"),
        (SLATE, False, "(≈ 36 µs): limpo até 40 µs (25 k/s); marginal a 30–25 µs."),
    ]
    y = by + 22
    for col, bold, s in lines:
        p.append(f"<text x='{bx+14}' y='{y}' fill='{col}' {font(11, bold=bold)}>{esc(s)}</text>")
        y += 18 if bold else 17
    p.append(f"<text x='{W-24}' y='{h-10}' text-anchor='end' fill='{SLATE}' {font(10)}>{esc('Custo de corrente: low-power idle 2,9 µA (ION_IDLE8); constant latency 0,55 mA (ION_IDLE11); RRAM standby: não publicado, medir com PPK2')}</text>")
    svg = f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{h}' viewBox='0 0 {W} {h}'>{''.join(p)}</svg>"
    with open(os.path.join(OUT, "m33_vs_flpr_nrf54l15.svg"), "w", encoding="utf-8") as fh:
        fh.write(svg)
    print("wrote m33_vs_flpr_nrf54l15.svg")


def c3_consumo_modos():
    """Three panels: modelled SoC current for the default drain period, for the shortest one and for the
    per-sample mode, SPIM22 vs SPIM00, at 1 600 / 16 000 / 50 000 amostras/s."""
    import math
    h = 580
    rates = ["1 600", "16 000", "50 000"]
    inst = [("SPIM22", BLUE), ("SPIM00", OFF)]
    panels = [
        ("Drenado, T = 10 ms (1 600/s), 1 ms (16 k, 50 k/s)", [[48, 349], [288, 592], [702, 1016]]),
        ("Drenado, T = período (625 → ≈ 710 µs), 100 → ≈ 180 µs acima", [[176, 476], [675, 979], [952, 1265]]),
        ("Por amostra (APP_PER_SAMPLE_IRQ)", [[59, 359], [245, 549], None]),
    ]
    p = [defs(), "<rect width='100%' height='100%' fill='white'/>",
         header("nRF54L15 — corrente média do SoC por modo de entrega, instância de SPIM e taxa (modelo E, sem PPK2)",
                "Caso 1 (data-ready), 11 B por rajada, toda amostra vai para a fila; SPIM22 a 8 MHz, SPIM00 a 32 MHz; Cortex-M33 padrão, RRAM em power-down")]
    ymin, ymax = math.log10(20), math.log10(2000)
    ay, ah = 112, 250
    pw, gap, x0 = 270, 24, 54
    ticks = [20, 50, 100, 200, 500, 1000, 2000]

    def Y(v):
        return ay + ah - ah * (math.log10(v) - ymin) / (ymax - ymin)

    for i, (title, data) in enumerate(panels):
        ax = x0 + i * (pw + gap)
        p.append(f"<rect x='{ax}' y='{ay}' width='{pw}' height='{ah}' fill='{ZEBRA}' rx='4'/>")
        p.append(f"<text x='{ax + pw/2:.1f}' y='{ay-20}' text-anchor='middle' fill='{INK}' {font(11, bold=True)}>{esc(title)}</text>")
        for v in ticks:
            p.append(f"<line x1='{ax}' y1='{Y(v):.1f}' x2='{ax+pw}' y2='{Y(v):.1f}' stroke='{LINE}' stroke-width='1' stroke-dasharray='2 4'/>")
            if i == 0:
                p.append(f"<text x='{ax-6}' y='{Y(v)+4:.1f}' text-anchor='end' fill='{SLATE}' {font(10.5)}>{v} µA</text>")
        p.append(f"<line x1='{ax}' y1='{ay+ah}' x2='{ax+pw}' y2='{ay+ah}' stroke='{SLATE}' stroke-width='1'/>")
        gw = pw / len(rates)
        bw, nb = 28, 2
        for g, (rate, vals) in enumerate(zip(rates, data)):
            gx = ax + g * gw + (gw - nb * bw - (nb - 1) * 6) / 2
            if vals is None:
                p.append(f"<rect x='{gx:.1f}' y='{Y(60):.1f}' width='{nb*bw+6}' height='{ay + ah - Y(60):.1f}' rx='2' fill='url(#hatch)'/>")
                p.append(f"<text x='{ax + g * gw + gw/2:.1f}' y='{Y(60) - 18:.1f}' text-anchor='middle' fill='{SLATE}' {font(9.5)}>fora da</text>")
                p.append(f"<text x='{ax + g * gw + gw/2:.1f}' y='{Y(60) - 6:.1f}' text-anchor='middle' fill='{SLATE}' {font(9.5)}>faixa</text>")
            else:
                for k, ((name, col), v) in enumerate(zip(inst, vals)):
                    x = gx + k * (bw + 6)
                    y = Y(v)
                    p.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{bw}' height='{ay + ah - y:.1f}' rx='2' fill='{col}' opacity='0.9'/>")
                    p.append(f"<text x='{x + bw/2:.1f}' y='{y - 5:.1f}' text-anchor='middle' fill='{col}' {font(9.5, bold=True)}>{v}</text>")
            p.append(f"<text x='{ax + g * gw + gw/2:.1f}' y='{ay+ah+15}' text-anchor='middle' fill='{INK}' {font(10.5)}>{esc(rate)}</text>")
        p.append(f"<text x='{ax + pw/2:.1f}' y='{ay+ah+30}' text-anchor='middle' fill='{SLATE}' {font(10)}>amostras por segundo</text>")
    p.append(f"<text x='20' y='{ay-10}' fill='{SLATE}' {font(10)}>{esc('µA, log')}</text>")
    p.append(legend_svg(x0, ay + ah + 54, [(BLUE, "SPIM22 (PERI, P1, 8 MHz)"), (OFF, "SPIM00 (MCU, P2, 32 MHz)"),
                                            (LINE, "por amostra a 50 k/s: 20 µs < 12,5 de transação + 15,5 de entrada máx. da ISR + 2; FLPR: sem número")]))
    bx, by, bw2 = x0, ay + ah + 66, W - 24 - x0
    p.append(f"<rect x='{bx}' y='{by}' width='{bw2}' height='{94}' rx='8' fill='{PANEL}' stroke='{LINE}'/>")
    p.append(f"<text x='{bx+12}' y='{by+18}' fill='{INK}' {font(11, bold=True)}>Leitura</text>")
    p.append(f"<text x='{bx+12}' y='{by+35}' fill='{SLATE}' {font(10.5)}>{esc('T curto custa 3,6× (1,6 k/s), 2,3× (16 k/s) e 1,4× (50 k/s) o T longo: cada drenagem são ~21 µs de CPU (16,1 a RRAM acordando, média medida) + 15 µs de assentamento com ≤ 4 amostras; por amostra 3,5 µs.')}</text>")
    p.append(f"<text x='{bx+12}' y='{by+51}' fill='{SLATE}' {font(10.5)}>{esc('Por amostra: 7,5 µs de CPU por amostra a 1 600/s e 4,1 a 16 k/s (entrada média da ISR 3,8 e 0,4 µs, E: interpolada entre os passos medidos, + 1,2 de ISR + 2,5 de consumidor). Custa menos que T = período em toda a faixa (59 contra 176 µA a 1 600/s; 245 contra 675 a 16 k/s).')}</text>")
    p.append(f"<text x='{bx+12}' y='{by+67}' fill='{SLATE}' {font(10.5)}>{esc('T real = T arredondado ao tick de 32 µs + 1 tick + a drenagem: 100 µs → 176–191 µs (5 200–5 700 drenagens/s; 3 e 9 amostras de latência a 16 k e 50 k/s), 1 ms → 1,08 ms. A 16 k e 50 k/s o T = 10 ms pediria anel de 320 e 1 000 slots.')}</text>")
    p.append(f"<text x='{bx+12}' y='{by+83}' fill='{SLATE}' {font(10.5)}>{esc('SPIM00 custa ~+300 µA em qualquer taxa: só paga por barramento. Wrap: uma IRQ por volta do anel (anel/2 amostras); a < 64 µs de período a thread espera acordada (um período por wrap, ou o limite de 25 µs a 16 k/s com T = 100 µs).')}</text>")
    p.append(f"<text x='{x0}' y='{h-24}' fill='{SLATE}' {font(9.5)}>{esc('Premissas: base 2,9 µA; PERI ligado 20 µA (R, Academy); domínio MCU 300 µA (E, proxy TIMER00) só na SPIM00; SPIM ativa 0,25 mA (SPIM2x) / 0,8 mA (SPIM00); t = 12,5 µs (8 MHz) ou 4,25 µs (32 MHz);')}</text>")
    p.append(f"<text x='{x0}' y='{h-10}' fill='{SLATE}' {font(9.5)}>{esc('CPU 2,6 mA × [por drenagem: acordar médio medido (16,1 µs a ≥ 500 µs, 15,5 a 250, 9,0 a 100 µs) + 5 de trabalho (+ 15 de assentamento com ≤ 4 amostras); por wrap: acordar + 3 µs de idle ou um período + 3 µs acordado; por amostra: 1 µs de put + 2,5 de consumidor; modo por amostra: entrada média + 1,2 + 2,5 µs] (E).')}</text>")
    svg = f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{h}' viewBox='0 0 {W} {h}'>{''.join(p)}</svg>"
    with open(os.path.join(OUT, "consumo_modos_nrf54l15.svg"), "w", encoding="utf-8") as fh:
        fh.write(svg)
    print("wrote consumo_modos_nrf54l15.svg")


if __name__ == "__main__":
    c1_m33_vs_flpr()
    c3_consumo_modos()
    b1_blocks_int()
    b2_blocks_timer()
    d1_sensor_int()
    d2_timer()
    d3_drain()
    d7_per_sample()
    d4_bus_limit()
    d5_oversampling()
    d6_adxl382()
