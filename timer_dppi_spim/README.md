# timer_dppi_spim: acelerômetro lido em taxa fixa via TIMER → DPPI → SPIM

## Visão geral

Este exemplo lê um acelerômetro SPI em período fixo, sem CPU no caminho da
aquisição. Um TIMER gera um evento `COMPARE0` a cada período. Esse evento,
por DPPI, aciona o `TASKS_START` da SPIM. A SPIM controla o chip select por
hardware e o EasyDMA entrega a rajada em RAM. Como os registradores de dados
do sensor sempre guardam a última amostra, ler os mesmos registradores em
loop basta. O bit de data-ready no `STATUS`, lido na mesma rajada, marca as
amostras novas.

```
TIMER.COMPARE0 ──DPPI──▶ SPIM.TASKS_START ──▶ EasyDMA (rajada, CSN por hardware)
                          SPIM.EVENTS_END ──DPPI──▶ TIMER (contador) ──▶ [IRQ por bloco]
```

Um segundo TIMER, em modo contador, conta os eventos `END` da SPIM e, no modo
QUEUE, gera uma interrupção a cada N amostras. O consumo das amostras é
escolhido por Kconfig (modo LATEST ou modo QUEUE), como no
[`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md).

Use este exemplo quando o sensor não tem pino de data-ready ou quando a taxa
de leitura deve ser fixa e independente do sensor. O custo é um TIMER, o
HFXO (para período exato) e leituras repetidas, que precisam ser filtradas.
Este exemplo também é a bancada do repositório: varre períodos para medir o
teto do barramento e a margem entre timer e ODR. Os diagramas estão no
[README da raiz](../README.md).

## Requisitos

| Alvo (`-b`) | Sensor | Barramento | Disparo / contador / EGU |
|---|---|---|---|
| `thingy53/nrf5340/cpuapp` | ADXL362 | SPIM4 (CSN por hardware), P0.29/28/26, CSN P0.22 | TIMER1 / TIMER2 / EGU0 |
| `nrf54l15tag/nrf54l15/cpuapp` | BMI270 | SPIM22 (P1, domínio PERI), CSN P1.07 | TIMER20 / TIMER21 / EGU20 |
| `nrf54l15tag/nrf54l15/cpuflpr` | BMI270 | SPIM22, CSN P1.07 | TIMER20 / TIMER21 / EGU20; HFXO pedido pelo `hfxo_launcher` |

Ferramentas:

- nRF Connect SDK v3.4.1 com a toolchain instalada pelo `nrfutil sdk-manager`.
- J-Link para gravação e log por RTT.

## Configuração

### Kconfig

| Símbolo | Default | Função |
|---|---|---|
| `APP_SENSOR_ADXL362` / `APP_SENSOR_BMI270` | pelo devicetree | backend do sensor |
| `APP_SENSOR_ODR_HZ` | 400 | ODR do sensor |
| `APP_SAMPLE_PERIOD_US` | 1000 | período do TIMER (10 µs a 1 s) |
| `APP_SWEEP_PERIODS_US` | "" | lista de períodos para varrer (bancada); imprime `=== sweep result` por período |
| `APP_SWEEP_STEP_S` | 10 | segundos por passo da varredura |
| `APP_CONSUME_LATEST` / `APP_CONSUME_QUEUE` | LATEST | modo de consumo |
| `APP_BLOCK_SAMPLES` | 16 | N amostras por interrupção (modo QUEUE) |
| `APP_QUEUE_DEPTH` | 64 | profundidade da `k_msgq` em amostras |
| `APP_QUEUE_FRESH_ONLY` | n | filtro na ISR: só amostras com data-ready entram na fila; `skipped` conta as repetidas |
| `APP_SPI_FREQ_HZ` | 4 MHz | clock da SPIM (8 MHz na TAG) |
| `APP_SPI_CSN_DURATION` | 2 | `IFTIMING.CSNDUR` |
| `APP_SPI_RX_DELAY` | −1 (driver) | `IFTIMING.RXDELAY` (1 na TAG) |
| `APP_REQUEST_HFXO` | y (se há clock control) | TIMER com período exato |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório no log |

### Devicetree

| `chosen` | Uso |
|---|---|
| `app,accel` | nó do acelerômetro; o barramento é o pai do nó e `cs-gpios` dá o pino de CSN |
| `app,timer-trigger` | TIMER que dispara as transações |
| `app,timer-count` | TIMER usado como contador de `END` |
| `app,egu` | EGU que transforma os eventos de bloco em interrupção (modo QUEUE) |

Os overlays são os mesmos do `gpiote_dppi_spim`, mais o `app,timer-trigger`.

### Cenários de bancada

Os arquivos em `bench/` são passados como `EXTRA_CONF_FILE`:

| Arquivo | Cenário |
|---|---|
| `sweep-tag.conf` | margem timer × ODR na TAG: 2500 a 2200 µs, filtro na ISR |
| `bus-max-tag.conf` | teto do barramento na TAG: 40 a 15 µs, N = 64, fila 512 |
| `bus-64k-thingy.conf` | teto do barramento na Thingy:53: 25 a 12 µs, N = 64, fila 512 |
| `sweep-flpr.conf` | FLPR com `hfxo_launcher`: 100 a 20 µs |

## Compilação e gravação

```
nrfutil sdk-manager toolchain launch --ncs-version v3.4.1 --chdir C:\ncs\v3.4.1 -- ^
  west build -s <repo>\timer_dppi_spim -d <build> -b <alvo> -p always ^
    [-- "-Dtimer_dppi_spim_EXTRA_CONF_FILE=<repo>\timer_dppi_spim\bench\sweep-tag.conf"]
west flash -d <build> --dev-id <serial J-Link>
```

Os símbolos Kconfig são passados ao sysbuild com o prefixo da imagem
(`-Dtimer_dppi_spim_CONFIG_...`). Os `EXTRA_CONF_FILE` usam caminho absoluto
e vão entre aspas no PowerShell; vários arquivos são separados por `;`.

Log por RTT: na Thingy:53 acrescentar `overlay-rtt.conf` ao
`EXTRA_CONF_FILE`; na TAG já está em `boards/*.conf`. Scripts em
[`tools/`](../tools).

No FLPR, o `sysbuild.cmake` substitui o `vpr_launcher` padrão pela imagem
`hfxo_launcher` (`launcher/`), um app core mínimo que pede o HFXO e dorme.
Sem isso o TIMER do FLPR roda do HFINT (10016/s em vez de 10000).

## Teste

Modo LATEST (TAG, timer de 1 kHz, BMI270 a 400 Hz):

```
<inf> app: t=25000 ms xfers=25006 latest X=-0.74 Y=9.59 Z=-0.23 m/s^2
<inf> app: t=26000 ms xfers=26007 latest X=-0.75 Y=9.58 Z=-0.23 m/s^2
```

`xfers` avança na taxa do timer. O sufixo `(fresh)` aparece só quando a
última leitura trouxe uma amostra nova.

Modo QUEUE (TAG, timer de 10 kHz, N = 16, sem filtro):

```
<inf> app: t=25096 ms xfers=249968 queued=10000 fresh=402 dropped=0 Z avg=-0.23 min=-0.27 max=-0.19 m/s^2
<inf> app: t=26097 ms xfers=259968 queued=10000 fresh=401 dropped=0 Z avg=-0.23 min=-0.29 max=-0.19 m/s^2
```

`queued` segue o timer e `fresh` segue o ODR real do sensor. Com
`APP_QUEUE_FRESH_ONLY=y` a fila recebe só as amostras novas e as repetidas
aparecem em `skipped`.

Varredura (TAG, `bench/sweep-tag.conf`):

```
<inf> app: timer_dppi_spim: BMI270, trigger=timer 2500 us, consume=queue N=16 fresh-only
<inf> app: === sweep: period 2500 us (400.0 Hz) for 10 s
<inf> app: === sweep result: period 2500 us: xfers/s=400 queued/s=400 fresh/s=400.0 skipped=0 dropped=0 late_wraps=0
<inf> app: === sweep: period 2475 us (404.0 Hz) for 10 s
<inf> app: === sweep result: period 2475 us: xfers/s=404 queued/s=401 fresh/s=401.4 skipped=19 dropped=0 late_wraps=0
```

Cada passo descarta o primeiro segundo e imprime os totais dos seguintes.
`late_wraps` diferente de zero indica que o wrap do anel rodou depois do
prazo.

## Resultados

Medidos em 2026-09-27 com log por RTT; os logs estão em `test-logs/`.

### Timer de 1 kHz e 10 kHz, sensor a 400 Hz

| Alvo | TIMER 1 kHz, modo LATEST | TIMER 10 kHz, modo QUEUE |
|---|---|---|
| Thingy:53 M33 (ADXL362) | 1000/s exato | 10000/s, fresh ≈ 380 |
| Tag M33 (BMI270) | 1000/s exato | 10000/s, fresh 402 |
| Tag FLPR (BMI270) | — | 10016/s do HFINT; **10001/s** com `hfxo_launcher` |

### Timer × ODR: taxa mínima sem perda

Tag M33, BMI270 a 402/s real, `bench/sweep-tag.conf`:

| Período | xfers/s | fresh/s na fila | Perde amostras? |
|---|---|---|---|
| 2500 µs (400/s) | 400 | 400,0 | **sim, cerca de 2/s, sem rastro** |
| 2475 µs (404/s) | 404 | 401,5 | limiar |
| 2450 µs (408/s) | 408 | 402,6 | não (skipped 55 em 10 s) |
| 2400 µs (416/s) | 416 | 401,4 | não |
| 2200 µs (454/s) | 454 | 402,4 | não |

Abaixo do ODR real o timer perde amostras em silêncio: o data-ready volta a
subir antes da próxima leitura. Regra prática: timer ≥ ODR nominal × 1,05 a
1,10.

### Teto do barramento

Modo QUEUE, N = 64, fila 512, filtro na ISR.

Tag M33, BMI270, 17 bytes a 8 MHz (`bench/bus-max-tag.conf`):

| Período | xfers/s | late_wraps |
|---|---|---|
| 40 / 25 / 20 µs | 24995 / 40006 / 49999 | 0 |
| **19 µs** | **52632** | 0 |
| ≤ 18 µs | 0 | o `START` chega com a SPIM ocupada (17 µs + START + CSN ≈ 18,5 µs) e ela para |

Thingy:53 M33, ADXL362, 11 bytes a 8 MHz (`bench/bus-64k-thingy.conf`):

| Período | xfers/s | late_wraps |
|---|---|---|
| 25 / 20 / 16 µs | 40013 / 50005 / 62493 | 0 |
| 15 µs | 66667 | 0 (duas passagens) |
| **14 µs** | **71451** | 0 (duas passagens); dados válidos (Z varia de −11,9 a −4,3 m/s²) |
| 12 / 11 / 10 µs | 83335 / 90904 / 99991 STARTs | 0, mas **dados inválidos**: Z fixo em −11,10 (a transação de 11 bytes não termina; no nRF5340 o `START` durante a transação reinicia a SPIM) |

`xfers` conta `STARTED`, não transações completas: acima do teto a contagem
continua subindo enquanto o EasyDMA nunca entrega uma rajada inteira. O
teto real é o barramento (11 bytes + `START` + CSN ≈ 12,3 µs → 14 µs é o
último período com dados válidos, 71,4 k/s). O wrap do anel não limita:
a ISR escreve o `RXD.PTR` logo após o início da última transação do ciclo
e tem a transação inteira de margem (ver Achados).

Tag FLPR com `hfxo_launcher` (`bench/sweep-flpr.conf`): 10001 / 24994 /
39996 / 49989 transações por segundo, `late_wraps` 0, sem zero-latency IRQ.

## Detalhes de implementação

O engine (`src/spim_dppi.c`) e os backends de sensor são os mesmos do
`gpiote_dppi_spim`. As diferenças:

- o disparo é o `COMPARE0` de um TIMER a 1 MHz com short `CLEAR`, em vez do
  GPIOTE IN; `spim_dppi_set_period_us()` reprograma o período em tempo de
  execução;
- o HFXO é pedido pelo `onoff` do `CLOCK_CONTROL_NRF` antes da configuração;
- no modo QUEUE a ISR de EGU pode descartar as amostras repetidas
  (`APP_QUEUE_FRESH_ONLY`, bit de data-ready no `STATUS`);
- `src/main.c` traz o `run_sweep()` da bancada.

## Achados

1. **HFXO**: sem o pedido, o TIMER roda do HFINT, cerca de 0,2 % fora. O
   FLPR não tem clock control no NCS 3.4.1, por isso o `hfxo_launcher`.
2. **O wrap do anel é feito logo após `STARTED`, não após `END`**. O
   `RXD.PTR` é double-buffered e o hardware o reescreve a cada `START`; o
   datasheet só garante a escrita pela CPU "imediatamente após STARTED".
   A primeira versão contava `END` e escrevia o ponteiro entre o `END` e o
   `START` seguinte: com 2 a 4 µs de janela (15 µs de período a 8 MHz) a
   escrita coincidia às vezes com a atualização do hardware, o ponteiro
   ficava corrompido e o EasyDMA escrevia fora do anel (MPU/BUS fault em
   cerca de 1,5 s, reproduzível; `late_wraps` de 2555 num build e fault
   noutro só pela fase da ISR). Contando `STARTED`, a escrita tem a
   transação inteira de margem: zero `late_wraps` até 100 k STARTs/s.
   Histórico da versão anterior: a 64 k/s a ISR tinha cerca de 14 µs para
   reposicionar `RXD.PTR`. Uma ISR comum com `k_msgq_put` em loop perdia o
   prazo e o EasyDMA escrevia fora do anel (hard fault). Solução: anel de
   3N, wrap em ISR zero-latency (`IRQ_DIRECT_CONNECT`), fila via EGU e o
   contador `late_wraps`.
3. **`fresh` acima do ODR** com leituras espaçadas menos de cerca de 100 µs:
   o sensor demora a limpar o bit depois da leitura (Thingy a 25 µs: 534
   "fresh"/s para 380 reais). O filtro é uma heurística; a taxa real é o
   ODR.
4. **Log deferred trava na varredura**: a bancada usa `LOG_MODE_IMMEDIATE`,
   `LOG_BACKEND_RTT_MODE_DROP` e pilha do log em 2048 bytes.
5. **Sysbuild**: `-D<imagem>_CONFIG_X=y` só para símbolos Kconfig; um Kconfig
   novo pede `-p always`; strings via `.conf`. Trocar o `vpr_launcher` exige
   `SB_CONFIG_VPR_LAUNCHER=n` e `ExternalZephyrProject_Add` no
   `sysbuild.cmake` do app.

Os achados de silício (errata 8, `RXDELAY`, tempestade de IRQ, EasyDMA só em
RAM) estão no README do [`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md#achados).
