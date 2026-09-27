"""Generate the timing diagrams (SVG) used in the READMEs. No dependencies.

python gen_diagrams.py   -> writes *.svg next to this file
"""
import os

OUT = os.path.dirname(os.path.abspath(__file__))

W = 960
LEFT = 150
FONT = "font-family='Segoe UI, Helvetica, Arial, sans-serif' font-size='13'"
MONO = "font-family='Consolas, Menlo, monospace' font-size='12'"
COL = {
    "sig": "#1f2937", "hw": "#2563eb", "cpu": "#dc2626", "dma": "#059669",
    "grid": "#e5e7eb", "muted": "#6b7280", "warn": "#d97706",
}


class Diagram:
    def __init__(self, title, subtitle, t_end, rows, height=None):
        self.title, self.subtitle, self.t_end = title, subtitle, t_end
        self.rows = rows  # list of row names
        self.h = height or (90 + 46 * len(rows) + 40)
        self.parts = []
        self.y0 = 70

    def x(self, t):
        return LEFT + (W - LEFT - 20) * t / self.t_end

    def yrow(self, name):
        return self.y0 + self.rows.index(name) * 46

    def text(self, x, y, s, color=COL["sig"], anchor="start", mono=False, size=None):
        f = MONO if mono else FONT
        if size:
            f = f.replace("font-size='13'", f"font-size='{size}'").replace("font-size='12'", f"font-size='{size}'")
        self.parts.append(f"<text x='{x:.1f}' y='{y:.1f}' fill='{color}' text-anchor='{anchor}' {f}>{s}</text>")

    def pulse(self, row, t0, t1, color=COL["hw"], label=None, amp=22, dashed=False):
        y = self.yrow(row)
        x0, x1 = self.x(t0), self.x(t1)
        dash = " stroke-dasharray='4 3'" if dashed else ""
        self.parts.append(
            f"<path d='M{x0:.1f},{y:.1f} V{y-amp:.1f} H{x1:.1f} V{y:.1f}' fill='none' stroke='{color}' stroke-width='2'{dash}/>")
        if label:
            self.text((x0 + x1) / 2, y - amp - 5, label, color, "middle", size=11)

    def level(self, row, t0, t1, high, color=COL["hw"]):
        y = self.yrow(row)
        yy = y - (22 if high else 0)
        self.parts.append(f"<line x1='{self.x(t0):.1f}' y1='{yy:.1f}' x2='{self.x(t1):.1f}' y2='{yy:.1f}' stroke='{color}' stroke-width='2'/>")

    def edge(self, row, t, up, color=COL["hw"]):
        y = self.yrow(row)
        self.parts.append(f"<line x1='{self.x(t):.1f}' y1='{y:.1f}' x2='{self.x(t):.1f}' y2='{y-22:.1f}' stroke='{color}' stroke-width='2'/>")

    def block(self, row, t0, t1, label, color=COL["dma"], fill="#d1fae5"):
        y = self.yrow(row)
        x0, x1 = self.x(t0), self.x(t1)
        self.parts.append(f"<rect x='{x0:.1f}' y='{y-24:.1f}' width='{max(x1-x0,2):.1f}' height='24' fill='{fill}' stroke='{color}' stroke-width='1.5'/>")
        if label:
            self.text((x0 + x1) / 2, y - 8, label, color, "middle", size=11)

    def arrow(self, t0, row0, t1, row1, label=None, color=COL["muted"]):
        x0, y0 = self.x(t0), self.yrow(row0) - 22
        x1, y1 = self.x(t1), self.yrow(row1) - 24
        self.parts.append(f"<line x1='{x0:.1f}' y1='{y0:.1f}' x2='{x1:.1f}' y2='{y1:.1f}' stroke='{color}' stroke-width='1.2' stroke-dasharray='3 3' marker-end='url(#arr)'/>")
        if label:
            self.text((x0 + x1) / 2 + 6, (y0 + y1) / 2, label, color, "start", size=11)

    def note(self, t, row, s, color=COL["warn"]):
        self.text(self.x(t), self.yrow(row) + 14, s, color, "middle", size=11)

    def marker(self, t, label, color=COL["muted"]):
        x = self.x(t)
        self.parts.append(f"<line x1='{x:.1f}' y1='{self.y0-40}' x2='{x:.1f}' y2='{self.h-30}' stroke='{color}' stroke-width='1' stroke-dasharray='2 4'/>")
        self.text(x, self.y0 - 44, label, color, "middle", size=11)

    def save(self, name):
        rows = []
        for r in self.rows:
            y = self.yrow(r)
            rows.append(f"<line x1='{LEFT}' y1='{y}' x2='{W-20}' y2='{y}' stroke='{COL['grid']}' stroke-width='1'/>")
            rows.append(f"<text x='{LEFT-8}' y='{y-6}' text-anchor='end' fill='{COL['sig']}' {FONT}>{r}</text>")
        svg = f"""<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{self.h}' viewBox='0 0 {W} {self.h}'>
<defs><marker id='arr' markerWidth='8' markerHeight='8' refX='6' refY='4' orient='auto'><path d='M0,0 L8,4 L0,8 z' fill='{COL['muted']}'/></marker></defs>
<rect width='100%' height='100%' fill='white'/>
<text x='20' y='26' fill='{COL['sig']}' font-weight='bold' {FONT.replace("13", "16")}>{self.title}</text>
<text x='20' y='44' fill='{COL['muted']}' {FONT}>{self.subtitle}</text>
{''.join(rows)}
{''.join(self.parts)}
<text x='{W-20}' y='{self.h-8}' text-anchor='end' fill='{COL['muted']}' {FONT.replace("13", "11")}>tempo &#8594;</text>
</svg>"""
        with open(os.path.join(OUT, name), "w", encoding="utf-8") as fh:
            fh.write(svg)
        print("wrote", name)


def d1_sensor_int():
    d = Diagram("Caso 1 — disparo pelo data-ready do sensor (INT → GPIOTE → DPPI → SPIM)",
                "BMI270 na Tag a 400 Hz: 1 transação por amostra nova, CPU fora do caminho; escala ≈ 2,5 ms por período",
                6.0, ["amostra no sensor", "INT1 (data-ready)", "GPIOTE IN event", "DPPI ch", "SPIM START/END", "CSN (hardware)", "EasyDMA → RAM", "CPU"])
    for k, t in enumerate([0.4, 2.9, 5.4]):
        d.pulse("amostra no sensor", t, t + 0.05, COL["muted"], f"n+{k}")
        # level: high from sample until data regs read (end of burst)
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
    d.level("CPU", 0, 6.0, False, COL["cpu"])
    d.note(3.0, "CPU", "dormindo o tempo todo (LATEST) — o relatório 1×/s é a única atividade", COL["cpu"])
    d.note(0.8, "INT1 (data-ready)", "nível: só desce quando os registradores de dados são lidos", COL["warn"])
    d.arrow(0.42, "INT1 (data-ready)", 0.5, "SPIM START/END", "sem CPU")
    d.save("caso1_sensor_int.svg")


def d2_timer():
    d = Diagram("Caso 2 — disparo por TIMER (COMPARE → DPPI → SPIM), timer mais rápido que o ODR",
                "Timer a 100 µs contra sensor a 400 Hz: leituras repetidas levam DATA_READY = 0 e são descartadas pelo bit de STATUS",
                7.0, ["amostra no sensor", "TIMER COMPARE0", "DPPI ch", "SPIM START/END", "EasyDMA → RAM", "STATUS.DATA_READY lido", "CPU"])
    for k in range(7):
        t = 0.3 + k
        d.pulse("TIMER COMPARE0", t, t + 0.05)
        d.pulse("DPPI ch", t + 0.03, t + 0.08)
        d.pulse("SPIM START/END", t + 0.06, t + 0.1)
        d.block("EasyDMA → RAM", t + 0.1, t + 0.5, "")
    for t in [0.9, 3.4, 5.9]:
        d.pulse("amostra no sensor", t, t + 0.05, COL["muted"])
    fresh = {1: True, 3: True, 6: True}
    for k in range(7):
        t = 0.3 + k + 0.2
        f = fresh.get(k, False)
        d.pulse("STATUS.DATA_READY lido", t, t + 0.25, COL["dma"] if f else COL["muted"], "1 (nova)" if f else "0 (repetida)", amp=22 if f else 8)
    d.level("CPU", 0, 7.0, False, COL["cpu"])
    d.note(3.5, "CPU", "sem ISR por transação; no modo QUEUE a ISR de bloco descarta as repetidas (APP_QUEUE_FRESH_ONLY)", COL["cpu"])
    d.marker(0.3, "período do timer")
    d.marker(1.3, "")
    d.save("caso2_timer.svg")


def d3_queue():
    d = Diagram("Modo QUEUE — EasyDMA array list em ping-pong, uma interrupção a cada N transações",
                "N = 4 para caber no desenho; 2N slots + N de folga. COMPARE1 rebobina o PTR (ISR zero-latency); EGU faz o trabalho da fila",
                12.0, ["SPIM END (→ contador)", "contador (COUNT)", "RXD.PTR (array list)", "COMPARE0 = N / COMPARE1 = 2N", "ISR zero-latency (wrap)", "EGU → ISR da fila", "k_msgq"])
    for k in range(11):
        t = 0.5 + k
        d.pulse("SPIM END (→ contador)", t, t + 0.06)
        d.text(d.x(t + 0.03), d.yrow("contador (COUNT)") - 8, str((k % 8) + 1), COL["hw"], "middle", size=11)
    slots = ["A0", "A1", "A2", "A3", "B0", "B1", "B2", "B3", "A0", "A1", "A2"]
    for k, s in enumerate(slots):
        t = 0.5 + k
        d.block("RXD.PTR (array list)", t + 0.08, t + 0.98, s, COL["dma"] if s.startswith("A") else COL["hw"], "#d1fae5" if s.startswith("A") else "#dbeafe")
    d.pulse("COMPARE0 = N / COMPARE1 = 2N", 4.5, 4.6, COL["warn"], "COMPARE0")
    d.pulse("COMPARE0 = N / COMPARE1 = 2N", 8.5, 8.6, COL["warn"], "COMPARE1 (+CLEAR)")
    d.pulse("ISR zero-latency (wrap)", 8.55, 8.8, COL["cpu"], "PTR = A0 antes do próximo START")
    d.pulse("EGU → ISR da fila", 4.6, 5.3, COL["cpu"], "bloco A → 4× k_msgq_put")
    d.pulse("EGU → ISR da fila", 8.6, 9.3, COL["cpu"], "bloco B → 4× k_msgq_put")
    d.block("k_msgq", 5.3, 12.0, "consumidor tira uma amostra por k_msgq_get, em ordem", COL["sig"], "#f3f4f6")
    d.note(9.6, "ISR zero-latency (wrap)", "prazo: 1 período do disparo (a 64 k/s ≈ 15 µs) — por isso zero-latency", COL["warn"])
    d.arrow(8.55, "COMPARE0 = N / COMPARE1 = 2N", 8.6, "ISR zero-latency (wrap)")
    d.arrow(4.55, "COMPARE0 = N / COMPARE1 = 2N", 4.65, "EGU → ISR da fila", "DPPI → EGU.TRIGGER0")
    d.save("modo_queue_pingpong.svg")


def d4_bus_limit():
    d = Diagram("Teto do barramento — quando o período do timer é menor que a transação",
                "Tag: 17 B a 8 MHz = 17 µs + CSNDUR; a 20 µs cabe (50 k/s), a 18 µs o START chega com a SPIM ocupada",
                6.0, ["TIMER (20 µs)", "SPIM ocupada", "TIMER (18 µs)", "SPIM ocupada "])
    for k in range(3):
        t = 0.3 + k * 2.0
        d.pulse("TIMER (20 µs)", t, t + 0.05)
        d.block("SPIM ocupada", t + 0.05, t + 1.75, "17 µs + CSN")
    d.note(3.0, "SPIM ocupada", "folga ≈ 2–3 µs entre END e o próximo START: 50 000 transações/s, late_wraps = 0", COL["dma"])
    for k in range(3):
        t = 0.3 + k * 1.8
        d.pulse("TIMER (18 µs)", t, t + 0.05, COL["warn"] if k else COL["hw"])
    d.block("SPIM ocupada ", 0.35, 2.05, "17 µs + CSN")
    d.note(3.4, "SPIM ocupada ", "START durante a transação: ignorado; medido 0 transações/s a 18 µs (a SPIM não retoma)", COL["cpu"])
    d.save("teto_barramento.svg")


def d5_oversampling():
    d = Diagram("Timer × ODR — a margem mínima que não perde amostra (Tag, ODR real 402/s)",
                "Dois relógios livres: abaixo do ODR perde em silêncio; ~2 % acima nunca perde e descarta as repetidas",
                10.0, ["amostras do sensor (402/s)", "timer 2500 µs (400/s)", "novas lidas", "timer 2450 µs (408/s)", "novas lidas "])
    for k in range(10):
        t = 0.2 + k * 0.995
        d.pulse("amostras do sensor (402/s)", t, t + 0.04, COL["muted"])
    for k in range(10):
        t = 0.6 + k * 1.0
        d.pulse("timer 2500 µs (400/s)", t, t + 0.04)
    for k in range(10):
        t = 0.6 + k * 1.0
        lost = k == 7
        d.pulse("novas lidas", t + 0.05, t + 0.3, COL["cpu"] if lost else COL["dma"], "perdida" if lost else "", amp=8 if lost else 18)
    for k in range(10):
        t = 0.6 + k * 0.98
        d.pulse("timer 2450 µs (408/s)", t, t + 0.04)
    for k in range(10):
        t = 0.6 + k * 0.98
        rep = k == 6
        d.pulse("novas lidas ", t + 0.05, t + 0.3, COL["muted"] if rep else COL["dma"], "repetida" if rep else "", amp=8 if rep else 18)
    d.note(5.0, "novas lidas", "fresh = 400,0/s < 402: ~2 amostras/s desaparecem sem deixar rastro no flag", COL["cpu"])
    d.note(5.0, "novas lidas ", "fresh = 402,6/s: nenhuma perdida; as repetidas (skipped) são descartadas na ISR", COL["dma"])
    d.save("timer_vs_odr.svg")


def d6_adxl382():
    """Expected timing for the ADXL382 at 64 kHz ODR, data-ready driven (case 1 at the maximum rate)."""
    P = 15.625  # us, 64 kHz
    d = Diagram("Caso ADXL382 — data-ready a 64 kHz (INT0 → GPIOTE → DPPI → SPIM), esperado",
                "Período 15,6 µs; rajada STATUS0..ZDATA_L = 1 + 10 bytes: 11 µs a 8 MHz (86 % do barramento) ou 5,5 µs a 16 MHz (SPIM4 do nRF5340)",
                3 * P + 4, ["amostra ADXL382 (64 kHz)", "INT0 (DATA_READY)", "GPIOTE IN → DPPI", "SPIM 8 MHz: 11 B", "SPIM 16 MHz: 11 B", "prazo do wrap (QUEUE)"])
    for k in range(3):
        t = 1.0 + k * P
        d.pulse("amostra ADXL382 (64 kHz)", t, t + 0.3, COL["muted"], f"n+{k}")
        # data-ready: high until the data registers are read (end of the burst)
        d.edge("INT0 (DATA_READY)", t, True)
        d.level("INT0 (DATA_READY)", t, t + 1.5 + 11.0, True)
        d.edge("INT0 (DATA_READY)", t + 12.5, False)
        d.level("INT0 (DATA_READY)", t + 12.5, t + P, False)
        d.pulse("GPIOTE IN → DPPI", t + 0.1, t + 0.4)
        d.block("SPIM 8 MHz: 11 B", t + 0.5, t + 1.5 + 11.0, "1,0 µs START + 11 µs")
        d.block("SPIM 16 MHz: 11 B", t + 0.5, t + 1.5 + 5.5, "1,0 + 5,5 µs", COL["hw"], "#dbeafe")
    d.note(P + 8.5, "SPIM 8 MHz: 11 B", "folga ≈ 2 µs até o próximo START (medido: 71,4 k/s com 11 B no nRF5340)", COL["dma"])
    d.note(P + 8.5, "SPIM 16 MHz: 11 B", "folga ≈ 8 µs: margem para CSNDUR maior e para o jitter do ODR do sensor", COL["hw"])
    d.pulse("prazo do wrap (QUEUE)", 1.0 + P + 12.5, 1.0 + 2 * P + 0.5, COL["cpu"], "END do slot 2N−1 → START seguinte: ≈ 3 µs a 8 MHz, ≈ 9 µs a 16 MHz", amp=14)
    d.note(P + 8.5, "prazo do wrap (QUEUE)", "por isso o wrap é ISR zero-latency (M33) e o anel tem N slots de folga; LATEST não tem prazo", COL["cpu"])
    d.marker(1.0, "15,6 µs")
    d.marker(1.0 + P, "")
    d.save("caso_adxl382_64k.svg")


class Blocks:
    """Block/connection diagram: boxes and labelled arrows, no time axis."""

    def __init__(self, title, subtitle, w=960, h=420):
        self.title, self.subtitle, self.w, self.h = title, subtitle, w, h
        self.parts = []
        self.boxes = {}

    def box(self, key, x, y, w, h, lines, color=COL["hw"], fill="#eff6ff"):
        self.boxes[key] = (x, y, w, h)
        self.parts.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='6' fill='{fill}' stroke='{color}' stroke-width='1.5'/>")
        n = len(lines)
        for i, s in enumerate(lines):
            yy = y + h / 2 + (i - (n - 1) / 2) * 16 + 5
            f = FONT if i == 0 else FONT.replace("font-size='13'", "font-size='11'")
            wgt = " font-weight='bold'" if i == 0 else ""
            self.parts.append(f"<text x='{x + w / 2}' y='{yy:.1f}' text-anchor='middle' fill='{color}'{wgt} {f}>{s}</text>")

    def link(self, a, b, label="", color=COL["muted"], dashed=False, side="h", dy=0):
        ax, ay, aw, ah = self.boxes[a]
        bx, by, bw, bh = self.boxes[b]
        if side == "h":
            x0, y0 = ax + aw, ay + ah / 2 + dy
            x1, y1 = bx, by + bh / 2 + dy
        else:
            x0, y0 = ax + aw / 2, ay + ah
            x1, y1 = bx + bw / 2, by
        dash = " stroke-dasharray='5 3'" if dashed else ""
        self.parts.append(f"<line x1='{x0}' y1='{y0}' x2='{x1}' y2='{y1}' stroke='{color}' stroke-width='1.6'{dash} marker-end='url(#arr)'/>")
        if label:
            self.parts.append(f"<text x='{(x0 + x1) / 2}' y='{(y0 + y1) / 2 - 6}' text-anchor='middle' fill='{color}' {FONT.replace('13', '11')}>{label}</text>")

    def legend(self, y, items):
        x = 20
        for color, text in items:
            self.parts.append(f"<rect x='{x}' y='{y - 10}' width='14' height='10' fill='{color}'/>")
            self.parts.append(f"<text x='{x + 20}' y='{y}' fill='{COL['sig']}' {FONT.replace('13', '11')}>{text}</text>")
            x += 26 + 6.5 * len(text)

    def save(self, name):
        svg = f"""<svg xmlns='http://www.w3.org/2000/svg' width='{self.w}' height='{self.h}' viewBox='0 0 {self.w} {self.h}'>
<defs><marker id='arr' markerWidth='8' markerHeight='8' refX='7' refY='4' orient='auto'><path d='M0,0 L8,4 L0,8 z' fill='{COL['muted']}'/></marker></defs>
<rect width='100%' height='100%' fill='white'/>
<text x='20' y='26' fill='{COL['sig']}' font-weight='bold' {FONT.replace("13", "16")}>{self.title}</text>
<text x='20' y='44' fill='{COL['muted']}' {FONT}>{self.subtitle}</text>
{''.join(self.parts)}
</svg>"""
        with open(os.path.join(OUT, name), "w", encoding="utf-8") as fh:
            fh.write(svg)
        print("wrote", name)


def b1_blocks_int():
    d = Blocks("Blocos — caso 1: data-ready do sensor → GPIOTE → DPPI → SPIM (gpiote_dppi_spim)",
               "Tudo em hardware; a CPU só configura no início e, no modo QUEUE, atende uma IRQ a cada N amostras")
    d.box("sensor", 20, 90, 150, 80, ["Acelerômetro", "ADXL362 / BMI270 / ADXL382", "INT = data-ready (nível)"], COL["muted"], "#f3f4f6")
    d.box("gpiote", 230, 90, 150, 80, ["GPIOTE", "IN[n] event", "borda ↑ (ou ↓) do pino"])
    d.box("dppi", 440, 90, 120, 80, ["DPPI", "canal 0", "EEP → TEP"])
    d.box("spim", 620, 90, 170, 80, ["SPIM (CSN por HW)", "TASKS_START", "EasyDMA TX/RX"], COL["dma"], "#d1fae5")
    d.box("ram", 840, 90, 100, 80, ["RAM", "rajada", "STATUS..Z"], COL["dma"], "#d1fae5")
    d.box("cnt", 620, 250, 170, 80, ["TIMER (contador)", "TASKS_COUNT ← END", "CC0 = N, CC1 = 2N"])
    d.box("egu", 440, 250, 120, 80, ["EGU", "TRIGGER0/1", "→ ISR da fila"], COL["cpu"], "#fee2e2")
    d.box("cpu", 230, 250, 150, 80, ["CPU", "ISR ZLI: wrap do PTR", "ISR EGU: k_msgq_put ×N"], COL["cpu"], "#fee2e2")
    d.link("sensor", "gpiote", "pino INT")
    d.link("gpiote", "dppi", "evento")
    d.link("dppi", "spim", "tarefa START")
    d.link("spim", "ram", "DMA")
    d.link("spim", "cnt", "EVENTS_END → DPPI ch1 → COUNT", side="v")
    d.link("cnt", "egu", "COMPARE0/1 → DPPI ch2/3")
    d.link("egu", "cpu", "IRQ (modo QUEUE)")
    d.parts.append(f"<text x='20' y='370' fill='{COL['sig']}' {FONT}>Modo LATEST: só o caminho de cima existe; a CPU lê o buffer quando quiser (leitura dupla comparada).</text>")
    d.parts.append(f"<text x='20' y='390' fill='{COL['sig']}' {FONT}>Partida: 1 START por software depois de ligar o DPPI (o data-ready já está alto; é nível, não pulso).</text>")
    d.legend(410, [(COL["hw"], "periférico"), (COL["dma"], "caminho dos dados"), (COL["cpu"], "CPU / interrupção"), (COL["muted"], "externo")])
    d.save("blocos_caso1_sensor_int.svg")


def b2_blocks_timer():
    d = Blocks("Blocos — caso 2: TIMER → DPPI → SPIM (timer_dppi_spim)",
               "Igual ao caso 1 com o TIMER no lugar do GPIOTE; o sensor não participa do disparo")
    d.box("hfxo", 20, 90, 150, 80, ["HFXO", "pedido via onoff", "(FLPR: hfxo_launcher)"], COL["muted"], "#f3f4f6")
    d.box("trig", 230, 90, 150, 80, ["TIMER (disparo)", "1 MHz, CC0 = período", "short COMPARE0→CLEAR"])
    d.box("dppi", 440, 90, 120, 80, ["DPPI", "canal 0", "EEP → TEP"])
    d.box("spim", 620, 90, 170, 80, ["SPIM (CSN por HW)", "TASKS_START", "EasyDMA TX/RX"], COL["dma"], "#d1fae5")
    d.box("ram", 840, 90, 100, 80, ["RAM", "rajada", "STATUS..Z"], COL["dma"], "#d1fae5")
    d.box("sensor", 840, 250, 100, 80, ["Sensor", "regs sempre", "com a última"], COL["muted"], "#f3f4f6")
    d.box("cnt", 620, 250, 170, 80, ["TIMER (contador)", "TASKS_COUNT ← END", "CC0 = N, CC1 = 2N"])
    d.box("egu", 440, 250, 120, 80, ["EGU", "TRIGGER0/1", "→ ISR da fila"], COL["cpu"], "#fee2e2")
    d.box("cpu", 230, 250, 150, 80, ["CPU", "ISR ZLI: wrap do PTR", "ISR EGU: filtra fresh, put"], COL["cpu"], "#fee2e2")
    d.link("hfxo", "trig", "clock exato")
    d.link("trig", "dppi", "COMPARE0")
    d.link("dppi", "spim", "tarefa START")
    d.link("spim", "ram", "DMA")
    d.link("spim", "cnt", "EVENTS_END → DPPI ch1 → COUNT", side="v")
    d.link("cnt", "egu", "COMPARE0/1 → DPPI ch2/3")
    d.link("egu", "cpu", "IRQ (modo QUEUE)")
    d.parts.append(f"<text x='20' y='370' fill='{COL['sig']}' {FONT}>Timer acima do ODR: amostras repetidas com STATUS.DATA_READY = 0, descartadas na ISR (APP_QUEUE_FRESH_ONLY).</text>")
    d.parts.append(f"<text x='20' y='390' fill='{COL['sig']}' {FONT}>Timer abaixo do ODR real: perde amostras sem rastro — manter 5–10 % acima do ODR nominal.</text>")
    d.legend(410, [(COL["hw"], "periférico"), (COL["dma"], "caminho dos dados"), (COL["cpu"], "CPU / interrupção"), (COL["muted"], "externo")])
    d.save("blocos_caso2_timer.svg")


if __name__ == "__main__":
    b1_blocks_int()
    b2_blocks_timer()
    d1_sensor_int()
    d2_timer()
    d3_queue()
    d4_bus_limit()
    d5_oversampling()
    d6_adxl382()
