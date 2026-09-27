# timer_dppi_spim — leitura de acelerômetro em taxa fixa por TIMER (opcional)

NCS v3.4.1 / nrfx 4.0. Um TIMER dispara a SPIM em período fixo por DPPI,
independente do sensor: a abordagem do experimento original de 2025. Ler os mesmos
registradores em loop basta, porque eles sempre têm a última amostra; o bit
de data-ready no `STATUS`, lido na mesma rajada, marca as amostras novas.

```
TIMER.COMPARE0 ──DPPI──▶ SPIM.TASKS_START ──▶ EasyDMA (rajada, CSN por hardware)
                          SPIM.EVENTS_END ──DPPI──▶ TIMER (contador) ──▶ [IRQ por bloco]
```

Diagramas: [`docs/caso2_timer.svg`](../docs/caso2_timer.svg),
[`docs/timer_vs_odr.svg`](../docs/timer_vs_odr.svg),
[`docs/teto_barramento.svg`](../docs/teto_barramento.svg) (inline no
[README da raiz](../README.md)).

Quando usar em vez do [`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md):
sensor sem pino de data-ready, ou taxa de leitura fixa desacoplada do
sensor. Custa um TIMER, o HFXO (para período exato) e leituras repetidas que
precisam ser filtradas. Também é o exemplo de **bancada**: varredura de
período para medir o teto do barramento e a margem timer × ODR.

| Alvo | Sensor | Barramento | Disparo / contador / EGU |
|---|---|---|---|
| `thingy53/nrf5340/cpuapp` | ADXL362 | SPIM4 (CSN HW), P0.29/28/26, CSN P0.22 | TIMER1 / TIMER2 / EGU0 |
| `nrf54l15tag/nrf54l15/cpuapp` | BMI270 | SPIM22 (P1, PERI), CSN P1.07 | TIMER20 / TIMER21 / EGU20 |
| `nrf54l15tag/nrf54l15/cpuflpr` | idem | idem | idem, HFXO pedido pelo `hfxo_launcher` |

## Kconfig

| Símbolo | Default | Função |
|---|---|---|
| `APP_SENSOR_{ADXL362,BMI270}` | pelo DT | backend do sensor |
| `APP_SENSOR_ODR_HZ` | 400 | ODR do sensor |
| `APP_SAMPLE_PERIOD_US` | 1000 | período do TIMER (10 µs..1 s) |
| `APP_SWEEP_PERIODS_US` / `APP_SWEEP_STEP_S` | "" / 10 | varredura de períodos (bancada); imprime `=== sweep result` por período |
| `APP_CONSUME_LATEST` / `APP_CONSUME_QUEUE` | LATEST | consumo |
| `APP_BLOCK_SAMPLES` / `APP_QUEUE_DEPTH` | 16 / 64 | N amostras por IRQ; profundidade da fila |
| `APP_QUEUE_FRESH_ONLY` | n | filtro na ISR: só amostras com data-ready entram na fila (`skipped` conta as repetidas) |
| `APP_SPI_FREQ_HZ`, `APP_SPI_CSN_DURATION`, `APP_SPI_RX_DELAY` | 4 MHz, 2, driver | timing da SPIM |
| `APP_REQUEST_HFXO` | y (se há clock control) | TIMER exato |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório |

```
nrfutil sdk-manager toolchain launch --ncs-version v3.4.1 --chdir C:\ncs\v3.4.1 -- ^
  west build -s <repo>\timer_dppi_spim -d <build> -b <alvo> -p always ^
    [-- "-Dtimer_dppi_spim_EXTRA_CONF_FILE=<repo>\timer_dppi_spim\bench\sweep-tag.conf"]
west flash -d <build> --dev-id <serial J-Link>
```

Cenários de bancada em `bench/*.conf` (caminho absoluto, argumento entre
aspas no PowerShell). Log por RTT: Thingy com `overlay-rtt.conf` como
`EXTRA_CONF_FILE`; a Tag já vem com RTT nos `boards/*.conf`. Scripts em
[`tools/`](../tools).

No FLPR, `sysbuild.cmake` troca o `vpr_launcher` padrão por `launcher/`
(`hfxo_launcher`): um app core mínimo que pede o HFXO e dorme, para o TIMER
do FLPR rodar do cristal (sem isso: 10016/s em vez de 10000).

## Como funciona (src/)

Igual ao `gpiote_dppi_spim` (mesmos `sensor_*.c`, mesmo engine), com estas
diferenças em `spim_dppi.c` / `main.c`:

- o disparo é `TIMER.COMPARE0` (1 MHz, short CLEAR) em vez do GPIOTE IN;
  `spim_dppi_set_period_us()` reprograma o período em tempo de execução;
- pede o HFXO (`onoff` do `CLOCK_CONTROL_NRF`) antes de configurar;
- no modo QUEUE a ISR de EGU pode descartar as repetidas
  (`APP_QUEUE_FRESH_ONLY`, bit de data-ready no STATUS);
- `main.c` tem o `run_sweep()` de bancada.

## Resultados (2026-09-27, log por RTT, `test-logs/`)

### 1 kHz e 10 kHz, sensor a 400 Hz

| Alvo | TIMER 1 kHz + LATEST | TIMER 10 kHz + QUEUE |
|---|---|---|
| Thingy:53 M33 (ADXL362) | 1000/s exato | 10000/s, fresh ≈ 380 |
| Tag M33 (BMI270) | 1000/s exato | 10000/s, fresh 402 |
| Tag FLPR (BMI270) | — | 10016/s do HFINT; **10001/s** com `hfxo_launcher` |

### Timer × ODR: taxa mínima sem perda (Tag M33, BMI270 a 402/s real, `bench/sweep-tag.conf`)

| Período | xfers/s | fresh/s na fila | Perde? |
|---|---|---|---|
| 2500 µs (400/s) | 400 | 400,0 | **sim, ~2/s, sem rastro** |
| 2475 µs (404/s) | 404 | 401,5 | limiar |
| 2450 µs (408/s) | 408 | 402,6 | não (skipped 55 em 10 s) |
| 2400 µs (416/s) | 416 | 401,4 | não |
| 2200 µs (454/s) | 454 | 402,4 | não |

Abaixo do ODR real perde em silêncio (o data-ready volta a subir antes da
próxima leitura). Regra: timer ≥ ODR nominal × 1,05 a 1,10.

### Teto do barramento (QUEUE, N = 64, fila 512, filtro na ISR)

Tag M33, BMI270, 17 bytes a 8 MHz (`bench/bus-max-tag.conf`):

| Período | xfers/s | late_wraps |
|---|---|---|
| 40 / 25 µs | 24995 / 39997 | 0 |
| **20 µs** | **49999** | 0 |
| ≤ 18 µs | 0 | START chega com a SPIM ocupada; ela para |

Thingy:53 M33, ADXL362, 11 bytes a 8 MHz (`bench/bus-64k-thingy.conf`):

| Período | xfers/s | late_wraps |
|---|---|---|
| 25 / 20 µs | 40011 / 50014 | 0 |
| 16 µs | 62520 | 48–64 por passo (0 no build unificado anterior) |
| 15 µs | 66672 | 2555 no build anterior; **neste build: MPU/BUS fault ~1,5 s após entrar** (2 de 2) |
| **14 µs** | **71430** | 0 (3 de 3 execuções) |
| 12 µs | 0 | — |

O regime entre 14 e 16 µs é marginal a 8 MHz: a transação de 11 B ocupa
12,3 µs e sobram 2–4 µs para o wrap. O comportamento muda entre builds
(alinhamento de fase entre END, atualização do `RXD.PTR` e a ISR), e a 15 µs
a corrupção do anel chega a derrubar o firmware. Para 64 k/s em produto:
**16 MHz na SPIM4** (transação de 7 µs) e `late_wraps = 0` como critério;
a bancada pula 15 µs (`bench/bus-64k-thingy.conf`).

Tag FLPR com `hfxo_launcher` (`bench/sweep-flpr.conf`): 10001 / 24994 /
39996 / 49989 por segundo, `late_wraps` 0, sem zero-latency IRQ.

## Achados específicos deste exemplo

1. **HFXO**: sem pedir, o TIMER roda do HFINT (~0,2 % fora). No FLPR não há
   clock control → `hfxo_launcher`.
2. **Wrap do anel tem prazo duro**: a 64 k/s a ISR tem ~14 µs para rebobinar
   `RXD.PTR`; ISR normal com `k_msgq_put` em loop perdia o prazo e o DMA
   escrevia fora do anel (hard fault). Solução: anel de 3N, wrap em ISR
   zero-latency (`IRQ_DIRECT_CONNECT`), fila via EGU, `late_wraps`.
3. **`fresh` acima do ODR** com leituras espaçadas < ~100 µs: o sensor leva
   um tempo para limpar o bit após a leitura (Thingy a 25 µs: 534 "fresh"/s
   para 380 reais). O filtro é heurística; a taxa real é o ODR.
4. **Log deferred trava na varredura**: bancada usa `LOG_MODE_IMMEDIATE` +
   `LOG_BACKEND_RTT_MODE_DROP` + pilha de log 2048.
5. **Sysbuild**: `-D<imagem>_CONFIG_X=y` só para símbolos Kconfig; Kconfig
   novo pede `-p always`; strings via `.conf`; trocar o `vpr_launcher` =
   `SB_CONFIG_VPR_LAUNCHER=n` + `ExternalZephyrProject_Add` no
   `sysbuild.cmake` do app.

Os achados de silício (errata 8, RXDELAY, tempestade de IRQ, EasyDMA em RAM)
estão no README do `gpiote_dppi_spim`.
