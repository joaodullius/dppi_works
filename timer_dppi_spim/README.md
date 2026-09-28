# timer_dppi_spim: acelerômetro lido em taxa fixa via TIMER → DPPI → SPIM

Leitura periódica disparada por TIMER, para sensor sem pino de data-ready
ou taxa fixa; é também a bancada do repositório.

## Visão geral

Um TIMER gera `COMPARE0` a cada período; por DPPI ele aciona
`SPIM.TASKS_START`, e o EasyDMA grava a rajada com CSN por hardware. Ler os
registradores em loop basta: o bit de data-ready no `STATUS` (`fresh`)
separa novas de repetidas, descartadas pelo filtro `APP_QUEUE_FRESH_ONLY`
(`skipped`). Transações/s é a taxa do timer. Entrega como no
[`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md); custo extra: TIMER,
HFXO e repetidas. Conceitos no [README da raiz](../README.md).

![Blocos do caso 2](../docs/blocos_caso2_timer.svg)

![Timing do caso 2](../docs/caso2_timer.svg)

## Requisitos

| Alvo (`-b`) | Sensor | Barramento | TIMER |
|---|---|---|---|
| `thingy53/nrf5340/cpuapp` | ADXL362 | SPIM4, P0.29/28/26, CSN P0.22 | TIMER1 |
| `nrf54l15tag/nrf54l15/cpuapp` | BMI270 | SPIM22 (P1, PERI), CSN P1.07 | TIMER20 |
| `nrf54l15tag/nrf54l15/cpuflpr` | BMI270 | SPIM22 | TIMER20; HFXO pelo `hfxo_launcher` |

nRF Connect SDK v3.4.1 (`nrfutil sdk-manager`); J-Link e log por RTT.

## Configuração

| Kconfig | Default | Função |
|---|---|---|
| `APP_SENSOR_*`, `APP_SENSOR_ODR_HZ` | devicetree, 400 | backend e ODR |
| `APP_SAMPLE_PERIOD_US` | 1000 | período do TIMER (10 µs a 1 s) |
| `APP_SWEEP_PERIODS_US`, `APP_SWEEP_STEP_S` | "", 10 | varredura: períodos e segundos por passo |
| `APP_PER_SAMPLE_IRQ`, `APP_DRAIN_PERIOD_US`, `APP_RING_SLOTS`, `APP_WRAP_AWAKE_BELOW_US`, `APP_QUEUE_DEPTH` | n, 10000, 256, 64, 256 | como no `gpiote_dppi_spim` |
| `APP_QUEUE_FRESH_ONLY` | n | filtro na entrega (drenagem ou ISR de `END`); `skipped` conta as repetidas |
| `APP_SPI_FREQ_HZ`, `APP_SPI_CSN_DURATION`, `APP_SPI_RX_DELAY` | 4 MHz, 2, −1 | SPIM (8 MHz e `RXDELAY` 1 na TAG) |
| `APP_REQUEST_HFXO`, `APP_REPORT_PERIOD_MS` | y, 1000 | TIMER exato; relatório |
| `APP_WRAP_LATENCY_STATS` | n | bancada: TIMER a 16 MHz e captura, na ISR de wrap ou de `END`, do tempo desde o `COMPARE` |
| `APP_WRAP_ON_STARTED` | n | bancada: wrap em `STARTED` em vez de `DMA.RX.READY` (nRF54L) |
| `APP_RRAM_STANDBY` | n | bancada: RRAM em standby (`RRAMC.POWER.LOWPOWERCONFIG.MODE`); ≈ 14 µs a menos por acordar (16,8 → 2,75 µs, anterior, sem log) |

| `chosen` | Uso |
|---|---|
| `app,accel` | acelerômetro; `cs-gpios` = CSN |
| `app,timer-trigger` | TIMER de disparo |

Bancada (`bench/*.conf` como `EXTRA_CONF_FILE`; taxa alta com T = 1 ms,
anel 512, fila 1024):

| Arquivo | Cenário |
|---|---|
| `sweep-tag.conf` | timer × ODR na TAG: 2500 a 2200 µs, filtro, T = 10 ms |
| `bus-max-tag.conf` / `bus-64k-thingy.conf` | teto: 40 a 16 µs / 100 a 10 µs, 8 MHz, latência do wrap |
| `per-sample-tag.conf` / `per-sample-thingy.conf` | modo por amostra: 1000 a 19 µs / 1000 a 14 µs, 8 MHz, latência trigger → ISR de `END`, filtro |
| `wrap-latency-tag.conf` (+ `-started`) | latência do wrap: 1000 a 20 µs, T = 1 ms; variante `APP_WRAP_ON_STARTED` |
| `sweep-flpr.conf` | FLPR com `hfxo_launcher`: 100 a 20 µs (não remedido) |
| `constlat.conf`, `rram-standby.conf` | sobre `wrap-latency-tag.conf`: constant latency (`CONFIG_SOC_NRF_FORCE_CONSTLAT`, única configuração com `CONFIG_NRF_SYS_EVENT`) / `APP_RRAM_STANDBY` |

## Compilação e gravação

```
nrfutil sdk-manager toolchain launch --ncs-version v3.4.1 --chdir C:\ncs\v3.4.1 -- ^
  west build -s <repo>\timer_dppi_spim -d <build> -b <alvo> -p always ^
    [-- "-Dtimer_dppi_spim_EXTRA_CONF_FILE=<repo>\timer_dppi_spim\bench\sweep-tag.conf"]
west flash -d <build> --dev-id <serial J-Link>
```

`EXTRA_CONF_FILE` absoluto, entre aspas, vários com `;`; na Thingy:53
acrescentar `overlay-rtt.conf`. Captura: `tools\flash_and_capture.ps1
-Build <build> -Out <log> -Seconds <s> -Serial <serial> -Device
<dispositivo> -Elf <zephyr.elf>`. FLPR: o `sysbuild.cmake` troca o
`vpr_launcher` pelo `hfxo_launcher` (`launcher/`); o build precisa de
`-DSB_CONFIG_VPR_LAUNCHER=n`. Sem HFXO o TIMER roda do HFINT, ≈ 0,2 % fora
(M, log não incluído).

## Teste

Relatório do `gpiote_dppi_spim` mais `skipped`; `queued` segue o timer (sem
filtro) ou o ODR real (com filtro), `fresh` o ODR real (heurístico
< ~100 µs). Na varredura cada passo descarta o primeiro segundo e imprime
totais (diferença no passo); o relatório por segundo só sai na janela de
acomodação de cada passo, e dela vêm as faixas de Z. A latência
(`APP_WRAP_LATENCY_STATS`) é módulo o período. Teste bom: `late_wraps =
overflows = torn = 0`, `fresh/s` no ODR real, Z variando.

## Saída de exemplo

TAG, `bench/bus-max-tag.conf` (`test-logs/u_tag_busmax.log`):

```
<inf> app: timer_dppi_spim: BMI270, trigger=timer 25 us, drain every 1000 us fresh-only
<inf> spim_dppi: DPPI connected, burst 17 bytes, ring 512 slots, drain every 1000 us, wrap on DMA.RX.READY
<inf> app: === sweep: period 40 us (25000.0 Hz) for 8 s
<inf> app: t=1000 ms xfers=25014 queued=403 fresh=403 skipped=24609 dropped=0 late=0 ovf=0 torn=0 Z avg=0.59 min=0.55 max=0.65 m/s^2
<inf> app: === sweep result: period 40 us: xfers/s=24994 queued/s=401 fresh/s=401.8 skipped=172162 dropped=0 late_wraps=0 overflows=0 torn=0
<inf> app: === latency (trigger -> wrap ISR): min=1.06 avg=1.67 max=2.06 us
```

## Resultados

M, `test-logs/`, 8 MHz, mecanismo atual, `late = ovf = 0` em todos os
passos salvo indicação.

**Timer × ODR** (TAG, BMI270 a 401,8/s, `sweep-tag.conf`, `u_tag_timer_vs_odr.log`):

| Timer | Transações/s | `fresh`/s | `skipped` em 9 s | Perde? |
|---|---|---|---|---|
| 2500 µs | 400 | 399,8 | 0 | **sim, ≈ 2/s, sem rastro** |
| 2475 µs | 404 | 401,6 | 19 | não |
| 2450 / 2425 / 2400 µs | 408 / 412 / 416 | 402,1 / 401,8 / 402,0 | 56 / 94 / 132 | não |
| 2350 / 2300 / 2200 µs | 425 / 434 / 454 | 401,8 / 401,7 / 402,1 | 213 / 296 / 474 | não |

Abaixo do ODR real perde em silêncio; 0,5 % acima já não. Regra: timer 5 a
10 % acima do ODR nominal.

**Teto do barramento** (drenado, T = 1 ms, anel 512, fila 1024, filtro;
`xfers` avança mesmo acima do teto: validade pelo conteúdo):

| TAG M33, 17 B (`u_tag_busmax.log`) | Transações/s | `fresh`/s | Wrap (mín / média / máx) | Z na janela |
|---|---|---|---|---|
| 40 / 25 / 20 µs | 24 994 / 39 993 / 49 999 | 401,8 / 401,8 / 402,0 | 1,06 / 1,13–1,67 / 1,93–2,06 µs | 0,54–0,65 m/s² |
| **19 µs** | **52 623** | 402,0 | 1,06 / 1,35 / 2,06 | 0,55–0,64 |
| 18 / 17 / 16 µs | 55 557 / 58 826 / 62 490 | 363 / 454 / 453 | 1,1–1,7 / 2,06 | **não comprovados**: `fresh` fora do ODR, Z 0,58–0,62 |

| Thingy:53, 11 B (`u_thingy_bus64k.log`) | Transações/s | `fresh`/s | Wrap (mín / média / máx) | Z na janela |
|---|---|---|---|---|
| 100 µs | 9 998 | 372 | 1,81 / 2,71 / 24,37 µs (IRQ de idle) | −9,16 a −6,30 |
| 50 µs | 19 997 | 366 | 1,81 / 1,87 / 2,25 (espera acordada) | −9,18 a −6,26 |
| 25 / 20 / 16 µs | 40 003 / 50 009 / 62 489 | 544 / 394 / 483 (bit lido 2×) | 1,81 / 1,85–1,86 / 2,25 | −10,78 a −4,57 |
| 15 µs | 66 658 | 391 | 1,81 / 1,85 / 2,25 | −9,82 a −4,74 |
| **14 µs** | **71 461** | 406 | 1,81 / 1,85 / 2,25 | −9,93 a −5,81 |
| 12 / 11 / 10 µs | 83 367 / 90 966 / 100 010 | 0 | 1,81 / 1,85–1,86 / 2,25 | **inválidos** (captura anterior: Z congelado −8,44 / −6,89) |

Tetos 52,6 k/s (1/t previa 54 k) e 71,4 k/s (previa 80 k; real entre 71,4
e 83 k/s). A 100 µs na Thingy a IRQ de idle mostra o wake-up do nRF5340
(24,37 máx; captura anterior 10,87). `queued = fresh` exato.

**Modo por amostra** (`APP_PER_SAMPLE_IRQ`, filtro, fila 1024; latência =
trigger → ISR de `END`, inclui a transação):

| TAG M33, 17 B (`u_tag_persample_sweep.log`) | Transações/s | `fresh`/s | `torn` | Latência (mín / média / máx) | Leitura |
|---|---|---|---|---|---|
| 1000 µs | 1 000 | 402 | 0 | 19,68 / 25,64 / 35,18 µs | 18,5 + 1,2 de ISR; entrada até 15,5 (máx − mín), 6,0 média |
| 500 / 250 / 100 µs | 2 000 / 4 000 / 9 999 | 402 | 0 | 19,62–19,68 / 22,66–20,28 / 34,75–34,81 | entrada média 3,0 / 1,5 / 0,66 |
| 50 / 40 µs | 19 999 / 24 997 | 402 | 0 | 19,62 / 19,98–19,92 / 34,50–34,56 | **limpo até 40 µs (25 k/s)** |
| 30 / 25 µs | 33 329 / 39 997 | 402 | 1 / 0 | 19,62 / 19,77–19,69 / 29,18–22,37 | marginal (captura anterior: mínimos 0,06 e 9,68) |
| 20 µs | 49 997 | 2,6 | 248 267 em 5 s | 0,00 / 19,59 / 19,93 | cópia atropelada |
| 19 µs | 52 631 | 402 | 0 | 0,62 / 0,75 / 9,37 | **falso limpo**: ISR 0,6–0,8 µs após o `START` seguinte |

| Thingy:53, 11 B (`u_thingy_persample_sweep.log`) | Transações/s | `fresh`/s | `torn` em 5 s | Latência (mín / média / máx) | Leitura |
|---|---|---|---|---|---|
| 1000 / 250 / 100 µs | 1 000 / 4 000 / 9 998 | 372,0 / 372,2 / 373,0 | 0 | 14,12–14,00 / 14,2 / 14,81, 14,37, 29,87 | limpo; 12,5 + ≈ 1,5 |
| 50 µs | 20 006 | 365,8 | 0 | 13,87 / 14,34 / 40,18 | −1,7 %, igual ao drenado (366,2): bit do sensor; entrada máx 26,3 |
| 40 µs | 25 007 | 354,8 | 0 | 13,81 / 14,55 / 36,87 | sem ISR atrasada, −5 % sem contraparte drenada: **25 k/s com ressalva; sem ressalva até 50 µs (20 k/s)** |
| 30 / 25 µs | 33 327 / 39 993 | 537 / 536 | 5 / 27 | mín 0,06 / 0,12 | ISRs após o `START` seguinte |
| 20 µs | 49 774 | 241 | 0 | 0,75 / 14,49 / 17,93 | −35 % sem `torn` |
| 16 / 15 / 14 µs | 62 143 / 66 083 / 70 820 | 146 / 202 / 244 | 134 287 / 189 899 / 119 | mín 0,00 | 43 %, 57 %, 0,03 % atropeladas |

Regra: garantia se período > transação + entrada máxima (15,5 µs M33
nRF54L15, 26,3 µs nRF5340) + 2 µs ≈ 36 µs (27 k/s) / 41 µs (24 k/s);
medido limpo 40 µs (25 k/s) nos dois. O primeiro sintoma de excesso é a
latência mínima abaixo da transação.

**Latência de uma IRQ saindo de idle** (M33, `wrap-latency-tag.conf`,
drenado, T = 1 ms, sem filtro, `u_tag_wrap_latency.log`, `queued/s ≈ xfers/s`):

| Período | Caminho | Latência (mín / média / máx) |
|---|---|---|
| 1000 / 500 µs | IRQ de idle | 16,06 / 16,07–16,06 / 16,31 µs |
| 250 µs | IRQ de idle | 1,43 / 15,50 / 16,31 |
| 100 µs | IRQ de idle (core às vezes acordado) | 1,18 / 8,98 / 16,31 |
| 50 µs | espera acordada | 1,75 / 1,80 / 2,00 |
| 40 / 30 / 25 / 20 µs | espera acordada | 1,18–1,75 / 1,22–1,81 / 2,00–2,18 |

16,3 µs = RRAM em power-down (`tIDLE2CPU` 13 µs, D, + ~2); as médias são o
acordar do modelo de consumo. Mecanismo anterior (sem log): 16,8 µs no M33
(igual com constant latency), 2,75 com RRAM standby, 2,43 no FLPR.

## Achados

- Wake-up do nRF5340 também passa do período: a drenagem que dormia após
  armar o wrap deu dezenas a centenas de `late_wraps` por passo entre 25 e
  14 µs (M, sem log); com a espera acordada, zero até 14 µs.
- ≈ 16 µs do M33 em idle = RRAM em power-down; constant latency não
  resolve; RRAM standby e FLPR dão 2,4–2,8 µs (anterior).
- Wrap após `STARTED`/`DMA.RX.READY`, nunca após `END`: a versão anterior
  corrompia o ponteiro (MPU/BUS fault a 15 µs, `late_wraps` variando por
  build).
- Modo por amostra: `START` antes da ISR mistura duas transações sem
  sintoma; o sinal é a latência mínima abaixo da transação.
- `fresh` acima do ODR < ~100 µs (Thingy a 25 µs: 543,5 para ≈ 372): acima
  de ~10 k/s o caso 2 não garante "só amostras novas".
- HFINT ≈ 0,2 % fora; FLPR sem clock control no NCS 3.4.1 (`hfxo_launcher`);
  log deferred trava a varredura (`LOG_MODE_IMMEDIATE`, RTT DROP); sysbuild:
  `-D<imagem>_CONFIG_X` só para Kconfig, `-p always` para símbolo novo,
  `SB_CONFIG_VPR_LAUNCHER=n` + `ExternalZephyrProject_Add` para o launcher.

## Dependências

nrfx 4.0: `nrfx_timer` (COMPARE0 + short `CLEAR`, 1 ou 16 MHz),
`nrfx_spim`, `nrfx_gppi`, HALs `nrf_spim`, `nrf_timer`, `nrf_rramc`.
Zephyr: `clock_control` (`onoff` do HFXO), `pinctrl`, `k_msgq`, thread de
drenagem, log por RTT; sysbuild com `hfxo_launcher` no FLPR. Engine e
backends do `gpiote_dppi_spim`, mais filtro, `spim_dppi_set_period_us()` e
`run_sweep()`.
