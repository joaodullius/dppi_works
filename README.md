# dppi_works — leitura de acelerômetro SPI sem CPU (DPPI + SPIM), nRF5340 e nRF54L15

Experimentos que respondem a uma pergunta de campo (jul/2025, ADXL382 a 64 k
amostras/s no nRF5340): como tirar a CPU do caminho entre "amostra pronta" e
"dados em RAM". Tudo em nRF Connect SDK v3.4.1 / nrfx 4.0, com CS por
hardware, validado em bancada na Thingy:53 (nRF5340 + ADXL362) e na
nRF54L15 TAG (BMI270), nos cores M33 e FLPR.

| Diretório | O que é |
|---|---|
| [`gpiote_dppi_spim/`](gpiote_dppi_spim/README.md) | **Recomendado.** Disparo pelo pino de data-ready do sensor (INT → GPIOTE → DPPI → SPIM). Uma transação por amostra, sem timer, sem HFXO, sem repetidas. |
| [`timer_dppi_spim/`](timer_dppi_spim/README.md) | Opcional. Disparo por TIMER em taxa fixa. Também é o exemplo de bancada: varredura de período, teto do barramento, margem timer × ODR. |
| [`docs/`](docs) | Diagramas de blocos e de timing (`gen_diagrams.py` → SVG, sem dependências) e [análise de consumo](docs/POWER.md). |
| [`tools/`](tools) | Scripts de flash + captura RTT usados nos testes. |
| `adxl382_spim/`, `17_adxl362_dt/`, `gpiote_dppi_gpiote/`, `timer_gppi_gpiote/` | Experimentos de jul/2025 (NCS v2.7.0, tag `ncs-v2.7.0`). |

Os dois exemplos compartilham o mesmo engine (`src/spim_dppi.c`), os mesmos
backends de sensor e os mesmos overlays; cada um traz só o seu disparo. O
consumo das amostras é escolhido por Kconfig nos dois: **LATEST** (buffer
único, sem interrupção) ou **QUEUE** (cada amostra numa `k_msgq`).

Cada caso tem dois diagramas: **blocos** (quem liga em quem, por DPPI) e
**timing** (uma linha por sinal, tempo na horizontal).

## Caso 1 — disparo pelo data-ready do sensor (recomendado)

O pino INT do sensor vira um evento GPIOTE que, por DPPI, dispara o
`TASKS_START` da SPIM; o CSN é do hardware e o EasyDMA entrega a rajada em
RAM. Uma transação por amostra nova, nenhuma instrução de CPU no caminho.

![Blocos do caso 1](docs/blocos_caso1_sensor_int.svg)

![Timing do caso 1](docs/caso1_sensor_int.svg)

Detalhe que travou o experimento original: o data-ready é **nível** — fica
alto até os registradores de dados serem lidos. Se ninguém lê depois de
ligar a medição, a borda de subida nunca vem; um `START` por software após
ligar o DPPI resolve.

Testado no ODR máximo de cada sensor, com fila e zero perdas:

| Alvo | ODR máx. do sensor | Medido (QUEUE, N = 16) |
|---|---|---|
| Thingy:53 M33, ADXL362 | 400 Hz | 384/s (ODR real do sensor −5 %), queued = fresh |
| Tag M33, BMI270 | 1600 Hz | 1601–1616/s, queued = fresh, 0 perdas |
| Tag FLPR, BMI270 | 1600 Hz | 1601–1616/s, 0 perdas |

## Caso 2 — disparo por TIMER (opcional)

Um TIMER dispara a SPIM em período fixo, independente do sensor. Ler os
mesmos registradores em loop basta, porque eles sempre têm a última amostra;
o bit de data-ready no `STATUS`, lido na mesma rajada, marca as amostras
novas e permite descartar as repetidas.

![Blocos do caso 2](docs/blocos_caso2_timer.svg)

![Timing do caso 2](docs/caso2_timer.svg)

Medido na Tag (ODR real 402/s): timer a 400/s perde ~2 amostras/s **sem
deixar rastro**; a partir de ~2 % acima do ODR (408/s) não perde nenhuma.

![Timer × ODR](docs/timer_vs_odr.svg)

## Consumo das amostras: último valor ou fila

No modo LATEST o DMA reescreve sempre o mesmo buffer e a CPU olha quando
quer. No modo QUEUE, o EasyDMA em *array list* enche um ping-pong de 2N
amostras (mais N de folga); um TIMER em modo contador conta os END da SPIM
e, a cada N, gera a interrupção: o wrap do ponteiro é feito numa ISR
zero-latency e o trabalho da fila (`k_msgq`) numa ISR de EGU acionada por
DPPI.

![Modo fila](docs/modo_queue_pingpong.svg)

## Teto do barramento

A 8 MHz, 17 bytes (BMI270) ocupam 17 µs + 1 µs de START + CSN: 50 k
transações/s cabem; a 18 µs de período a SPIM para. Com 11 bytes (ADXL362)
o nRF5340 chegou a 71,4 k/s.

![Teto do barramento](docs/teto_barramento.svg)

| Alvo | TIMER 1 kHz | TIMER 10 kHz + QUEUE | Teto (QUEUE, filtro na ISR) |
|---|---|---|---|
| Thingy:53 M33 (ADXL362, 11 B) | 1000/s exato | 10000/s, fresh ≈ 380 | **71,4 k/s**, `late_wraps` 0 (14 µs); 15–16 µs marginal a 8 MHz (wraps atrasados, e a 15 µs fault) |
| Tag M33 (BMI270, 17 B) | 1000/s exato | 10000/s, fresh 402 | **50 k/s**, `late_wraps` 0 (20 µs) |
| Tag FLPR (BMI270) | 10001/s (HFXO pelo app core) | — | **50 k/s**, `late_wraps` 0 sem ZLI |

## ADXL382 a 64 k amostras/s

O ADXL382 gera data-ready a até 64 kHz (16/32/64 kHz selecionáveis). O
caminho do caso 1 serve sem mudança de arquitetura: o que muda é o backend
do sensor e a folga do barramento.

![ADXL382 a 64 kHz](docs/caso_adxl382_64k.svg)

### O que trocar no `gpiote_dppi_spim`

1. **Backend `src/sensor_adxl382.c`** (novo item na `choice APP_SENSOR`,
   `target_sources_ifdef` no CMake). Diferenças para o ADXL362:
   - protocolo SPI: o primeiro byte é `(endereço << 1) | 1` para leitura,
     com auto-incremento; não existe o comando `0x0B`. Rajada:
     `burst_tx = { (0x11 << 1) | 1 }`, `burst_tx_len = 1`,
     `SENSOR_BURST_LEN = 11` (1 comando + `STATUS0..ZDATA_L`, 0x11..0x1A);
   - `fresh_offset = 1`, `fresh_mask = 0x01` (`STATUS0.DATA_READY`);
   - dados **big-endian** em `rx[5..10]` (`XDATA_H, XDATA_L, …`), ao
     contrário do ADXL362; `decode()` faz `(rx[5] << 8) | rx[6]` etc.;
   - `mg_per_lsb` pela faixa (±15 g: 2000 LSB/g → 0,5 mg/LSB);
   - `init()`: `DEVID_AD` (0x00) = 0xAD; standby → modo de alto desempenho
     com ODR 64 kHz no registrador `OP_MODE` (0x26; o código do ODR e a
     necessidade de faixa/filtro para 64 kHz devem ser confirmados no
     datasheet, que não está no repo);
   - `enable_drdy_int()`: mapear `DATA_READY` no INT0 (`INT0_MAP0`), polaridade
     ativa-alta por padrão.
2. **Overlay**: nó `adxl382@0` sob a `spi4` (nRF5340) com `int1-gpios` no pino
   ligado ao INT0 e `spi-max-frequency = <16000000>`; não há binding
   `adi,adxl382` no Zephyr do NCS 3.4.1 — acrescentar
   `dts/bindings/adi,adxl382.yaml` no app (compatível `adi,adxl382`, `include:
   [spi-device.yaml]`, propriedade `int1-gpios`) e apontar o `DT_COMPAT` no
   Kconfig.
3. **`APP_SPI_FREQ_HZ = 16000000`** na SPIM4 do nRF5340: a 8 MHz a rajada de
   11 B ocupa 12,5 dos 15,6 µs (80 %; medido 71,4 k/s, funciona, mas sem
   margem para jitter do ODR ou `CSNDUR` maior). A 16 MHz caem para 7 µs
   (45 %). No nRF54L15 as SPIM2x param em 8 MHz; 16/32 MHz só na SPIM00
   (domínio MCU), o que exige atravessar o PPIB entre DPPIC20 (GPIOTE20) e
   DPPIC00 — a GPPI resolve, mas confirmar latência.
4. **Modo QUEUE**: `APP_BLOCK_SAMPLES = 64` (1000 IRQ/s), `APP_QUEUE_DEPTH ≥
   512`, `CONFIG_ZERO_LATENCY_IRQS=y` (o wrap tem ~3 µs de folga a 8 MHz,
   ~9 µs a 16 MHz); `late_wraps` no log confirma se alguma ficou para trás.
   Modo LATEST não tem prazo nenhum.
5. **Verificação**: `xfers` no log deve dar 64 000/s ± tolerância do
   oscilador do sensor e `fresh = queued`; o `late_wraps` deve ficar em 0.

### Consumo estimado a 64 k amostras/s (só o SoC; detalhes em [docs/POWER.md](docs/POWER.md))

| SoC / modo | 8 MHz (80 % do barramento) | 16 MHz (45 %) |
|---|---|---|
| nRF5340, INT + LATEST | ≈ 2,4 mA | ≈ 1,8 mA |
| nRF5340, INT + QUEUE (N = 64) | ≈ 3,2 mA | ≈ 2,6 mA |
| nRF54L15 (SPIM2x), INT + LATEST | ≈ 0,3–0,9 mA | — (SPIM00 necessária) |
| nRF54L15 (SPIM2x), INT + QUEUE (N = 64) | ≈ 0,9–1,5 mA | — |

No nRF5340 quem manda é a SPIM (1,9–2,1 mA enquanto transfere) e o TIMER
contador (0,67 mA); a CPU do modo QUEUE acrescenta ~0,8 mA. No nRF54L15 a
faixa vem da incógnita de quanto o GPIOTE IN mantém ligado no domínio PERI
(2,9 µA a 0,55 mA) e da corrente da SPIM não publicada (estimada em
0,25 mA). O ADXL382 em si consome à parte.

## Achados

Os achados de silício e de bancada estão nos READMEs dos exemplos:
[`gpiote_dppi_spim`](gpiote_dppi_spim/README.md#achados) (errata 8 da SPIM
no nRF54L15, `RXDELAY` em ciclos de 16 MHz, tempestade de IRQ da SPIM,
data-ready nível, GPIOTE compartilhado) e
[`timer_dppi_spim`](timer_dppi_spim/README.md#achados-específicos-deste-exemplo)
(HFXO no FLPR, prazo do wrap, `fresh` acima do ODR, log deferred).
