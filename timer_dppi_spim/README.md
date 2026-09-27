# timer_dppi_spim: acelerômetro lido em taxa fixa via TIMER → DPPI → SPIM

## Visão geral

**TL;DR: para sensor sem pino de data-ready ou taxa fixa. Timer 5 a 10 %
acima do ODR nominal e filtro de repetidas; abaixo do ODR real perde amostras
sem aviso. É também a bancada do repositório.**

Este exemplo lê um acelerômetro SPI em período fixo, sem CPU no caminho da
aquisição. Um TIMER gera um evento `COMPARE0` a cada período. Esse evento,
por DPPI, aciona o `TASKS_START` da SPIM. A SPIM controla o chip select por
hardware e o EasyDMA entrega a rajada em RAM. Como os registradores de dados
do sensor sempre guardam a última amostra, ler os mesmos registradores em
loop basta. O bit de data-ready no `STATUS`, lido na mesma rajada (`fresh`),
marca as amostras novas; aqui transações/s é a taxa do timer, não a de
amostras.

```
TIMER.COMPARE0 ──DPPI──▶ SPIM.TASKS_START ──▶ EasyDMA (rajada, CSN por hardware) ──▶ anel em RAM
                                                                                      │
                               thread de drenagem (a cada T): lê o head, filtra fresh, k_msgq_put, arma o wrap ◀┘
                               IRQ DMA.RX.READY / STARTED (1 por T): PTR = slot 0
```

O anel, a drenagem e o wrap são os do
[`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md): o EasyDMA em array
list enche o anel sozinho, uma thread acorda a cada `APP_DRAIN_PERIOD_US`,
entrega os slots completos a uma `k_msgq` e habilita uma vez a interrupção
de `DMA.RX.READY` (`STARTED` no nRF5340), cuja ISR devolve o ponteiro ao
slot 0. Com o filtro de repetidas (`APP_QUEUE_FRESH_ONLY`) a drenagem só
põe na fila as amostras com o bit de data-ready ativo; as outras contam em
`skipped`.

Use este exemplo quando o sensor não tem pino de data-ready ou quando a taxa
de leitura deve ser fixa e independente do sensor. O custo é um TIMER de
disparo, o HFXO (para período exato) e leituras repetidas, que precisam ser
filtradas. Este exemplo também é a bancada do repositório: varre períodos
para medir o teto do barramento, a margem entre timer e ODR e a latência da
ISR de wrap. Os termos, os diagramas e a interpretação estão no
[README da raiz](../README.md).

## Requisitos

| Alvo (`-b`) | Sensor | Barramento | TIMER de disparo |
|---|---|---|---|
| `thingy53/nrf5340/cpuapp` | ADXL362 | SPIM4 (CSN por hardware), P0.29/28/26, CSN P0.22 | TIMER1 |
| `nrf54l15tag/nrf54l15/cpuapp` | BMI270 | SPIM22 (P1, domínio PERI), CSN P1.07 | TIMER20 |
| `nrf54l15tag/nrf54l15/cpuflpr` | BMI270 | SPIM22, CSN P1.07 | TIMER20; HFXO pedido pelo `hfxo_launcher` |

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
| `APP_DRAIN_PERIOD_US` | 10000 | T, período de drenagem (100 µs a 1 s): latência de entrega; 2/T interrupções por segundo |
| `APP_RING_SLOTS` | 256 | slots do anel (8 a 4096), mais 8 de guarda; ≥ 2 × transações por drenagem |
| `APP_QUEUE_DEPTH` | 256 | profundidade da `k_msgq` em amostras |
| `APP_QUEUE_FRESH_ONLY` | n | filtro na drenagem: só amostras com data-ready entram na fila; `skipped` conta as repetidas |
| `APP_SPI_FREQ_HZ` | 4 MHz | clock da SPIM (8 MHz na TAG) |
| `APP_SPI_CSN_DURATION` | 2 | `IFTIMING.CSNDUR` |
| `APP_SPI_RX_DELAY` | −1 (driver) | `IFTIMING.RXDELAY` (1 na TAG) |
| `APP_REQUEST_HFXO` | y (se há clock control) | TIMER com período exato |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório no log |
| `APP_WRAP_LATENCY_STATS` | n | bancada: TIMER de disparo a 16 MHz e captura, na ISR de wrap, do tempo desde o `COMPARE` que iniciou a transação (min/avg/max) |
| `APP_COUNT_STARTED` | n | bancada: no nRF54L faz o wrap na IRQ de `STARTED` em vez de `DMA.RX.READY` |
| `APP_RRAM_STANDBY` | n | nRF54L, bancada: RRAM em standby em idle (`POWER.LOWPOWERCONFIG`) em vez de power-down; wake-up rápido de qualquer ISR. Não é necessário para o wrap |

### Devicetree

| `chosen` | Uso |
|---|---|
| `app,accel` | nó do acelerômetro; o barramento é o pai do nó e `cs-gpios` dá o pino de CSN |
| `app,timer-trigger` | TIMER que dispara as transações |

Os overlays são os mesmos do `gpiote_dppi_spim`, mais o `app,timer-trigger`.

### Cenários de bancada

Os arquivos em `bench/` são passados como `EXTRA_CONF_FILE`. Os de taxa
alta usam T = 1 ms, anel de 512 slots e fila de 1024:

| Arquivo | Cenário |
|---|---|
| `sweep-tag.conf` | margem timer × ODR na TAG: 2500 a 2200 µs, filtro na drenagem |
| `bus-max-tag.conf` | teto do barramento na TAG: 40 a 16 µs, T = 1 ms, anel 512, fila 1024, latência do wrap |
| `bus-64k-thingy.conf` | teto do barramento na Thingy:53: 25 a 10 µs, T = 1 ms, anel 512, fila 1024 |
| `sweep-flpr.conf` | FLPR com `hfxo_launcher`: 100 a 20 µs, T = 1 ms, latência do wrap |
| `n1-tag.conf` | latência do wrap e da IRQ de idle: 1000 a 20 µs, T = 1 ms; de 1000 a 40 µs a IRQ de wrap vem de idle, abaixo a drenagem espera acordada; M33 × FLPR |
| `n1-tag-started.conf` | sobre o anterior: wrap na IRQ de `STARTED` em vez de `DMA.RX.READY` |
| `constlat.conf` | sobre o anterior: M33 em constant latency (`CONFIG_SOC_NRF_FORCE_CONSTLAT`) |
| `rram-standby.conf` | sobre o anterior: RRAM em standby em idle (`APP_RRAM_STANDBY`) |

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
Sem isso o TIMER do FLPR roda do HFINT (10016/s em vez de 10000, M).

## Teste

Banner (TAG, `bench/bus-max-tag.conf`, timer de 25 µs, T = 1 ms, filtro
na drenagem, `test-logs/u_tag_busmax.log`):

```
<inf> app: timer_dppi_spim: BMI270, trigger=timer 25 us, drain every 1000 us fresh-only
<inf> spim_dppi: DPPI connected, burst 17 bytes, ring 512 slots, drain every 1000 us, wrap on DMA.RX.READY
```

Fora da varredura o relatório por segundo tem o formato do
`gpiote_dppi_spim` mais o campo `skipped`: `xfers` avança na taxa do timer,
`queued` segue o timer (sem filtro) ou o ODR real (com filtro) e `fresh`
segue o ODR real do sensor. Com `APP_QUEUE_FRESH_ONLY=y` a fila recebe só
as amostras novas e as repetidas aparecem em `skipped`.

Varredura (mesma bancada):

```
<inf> app: === sweep: period 40 us (25000.0 Hz) for 8 s
<inf> app: === sweep result: period 40 us: xfers/s=24998 queued/s=398 fresh/s=398.4 skipped=172213 dropped=0 late_wraps=0 overflows=0
<inf> app: === wrap latency (trigger -> wrap ISR): min=1.50 avg=12.76 max=16.68 us
<inf> app: === sweep: period 25 us (40000.0 Hz) for 8 s
<inf> app: === sweep result: period 25 us: xfers/s=39999 queued/s=402 fresh/s=402.0 skipped=277200 dropped=0 late_wraps=0 overflows=0
<inf> app: === wrap latency (trigger -> wrap ISR): min=1.50 avg=1.77 max=2.37 us
```

Cada passo descarta o primeiro segundo e imprime os totais dos seguintes.
Campos: `xfers` conta transações iniciadas; `late` no relatório por
segundo e `late_wraps` no resultado da varredura são o mesmo contador, e
`ovf` é `overflows`. `late_wraps` diferente de zero indica que o wrap do
anel rodou depois do `START` seguinte: a transação que já tinha começado
usou o slot seguinte ao último (a folga) e não entra na fila (uma amostra
perdida, ordem preservada, sem corrupção). `overflows` diferente de zero
indica que uma drenagem encontrou o head além do anel: T longo demais para
`APP_RING_SLOTS`. A linha de latência só aparece com
`APP_WRAP_LATENCY_STATS`.

## Resultados

Medidos (M) em 2026-09-27 com log por RTT; os logs que estão em
`test-logs/` são os das bancadas de teto (`u_tag_busmax.log`,
`u_thingy_bus64k.log`), feitos com o mecanismo de drenagem atual. Os
resultados de timer × ODR e de latência de IRQ foram medidos com o
mecanismo de entrega anterior (uma IRQ por amostra ou por bloco); os
números valem, os logs não estão incluídos. SCK: 8 MHz na TAG; na
Thingy:53, 8 MHz no bench de 64 k (`bench/bus-64k-thingy.conf`).

### Timer × ODR: taxa mínima sem perda

TAG M33, BMI270 a 402/s reais, `bench/sweep-tag.conf`, filtro ligado (a
coluna "amostras novas/s" é o `fresh/s`, que com o filtro é igual ao
`queued/s`); mecanismo anterior, log não incluído:

| Período do timer | Transações/s | Amostras novas/s | Perde amostras? |
|---|---|---|---|
| 2500 µs (400/s) | 400 | 400,0 | **sim, cerca de 2/s, sem rastro** |
| 2475 µs (404/s) | 404 | 401,5 | não (18 repetidas em 10 s) |
| 2450 µs (408/s) | 408 | 402,6 | não (55 repetidas em 10 s) |
| 2400 µs (416/s) | 416 | 401,4 | não |
| 2200 µs (454/s) | 454 | 402,4 | não |

Abaixo do ODR real o timer perde amostras em silêncio: o data-ready volta a
subir antes da próxima leitura. Os 401,5 (2475 µs) e 401,4 (2400 µs) não
são perda: a taxa real do sensor oscila ±1/s entre janelas de 1 s. O
critério de perda é outro: `skipped = 0` com amostras novas/s abaixo do
ODR real significa que o timer nunca leu uma repetida, logo perdeu
amostras; a partir de 2450 µs aparecem repetidas (`skipped > 0`), prova de
que o timer está à frente do sensor. O medido é 1,5 % acima do ODR real
(2 % do nominal) numa unidade; a regra de projeto é timer 5 a 10 % acima
do ODR nominal, para cobrir a tolerância do oscilador do sensor. O
resultado não depende do mecanismo de entrega.

### Teto do barramento

T = 1 ms, anel de 512 slots, fila de 1024, filtro na drenagem. As
transações/s são `xfers/s`; acima do teto a contagem pode continuar subindo
com dados congelados, então a validade é julgada pelo conteúdo (`fresh` e
Z variando). Estes são tetos de barramento, não contagens de amostras
novas: a 19 µs de espaçamento o filtro `fresh` já não é confiável
(Achado 5), então `fresh` aqui não mede o ODR. 1/t prevê 54 k/s (17 B) e
80 k/s (11 B); o medido é até 10 % menor.

TAG M33, BMI270, 17 bytes a 8 MHz (`bench/bus-max-tag.conf`,
`test-logs/u_tag_busmax.log`):

| Período | Transações/s | fresh/s | late_wraps / overflows | Latência do wrap (mín. / média / máx.) | Dados |
|---|---|---|---|---|---|
| 40 µs | 24 998 | 398 | 0 / 0 | 1,50 / 12,76 / 16,68 µs (25 por drenagem: IRQ de idle) | válidos |
| 25 µs | 39 999 | 402 | 0 / 0 | 1,50 / 1,77 / 2,37 µs (espera acordada) | válidos |
| 20 µs | 49 999 | 400 | 0 / 0 | 1,50 / 2,03 / 2,43 µs | válidos |
| **19 µs** | **52 620** | 400 | 0 / 0 | 1,50 / 1,89 / 2,37 µs | válidos |
| 18 / 17 / 16 µs | 55 545 / 58 830 / 62 489 | **0** | 0 / 0 | 1,3–2,3 µs | congelados: o `START` chega com a SPIM ocupada, 17 µs + START + CSN ≈ 18,5 µs; a contagem de `DMA.RX.READY` continua (com `APP_COUNT_STARTED` ela para em 0) |

Thingy:53 M33, ADXL362, 11 bytes a 8 MHz (`bench/bus-64k-thingy.conf`,
`test-logs/u_thingy_bus64k.log`):

| Período | Transações/s | fresh/s | late_wraps / overflows | Dados |
|---|---|---|---|---|
| 25 / 20 / 16 µs | 40 009 / 50 006 / 62 523 | 533 / 399 / 478 | 0 / 0 | válidos (`fresh` acima do ODR real ≈ 380: o bit não é confiável neste espaçamento) |
| 15 µs | 66 688 | 385 | 0 / 0 | válidos |
| **14 µs** | **71 456** | 404 | 0 / 0 | válidos (Z variando) |
| 12 / 11 / 10 µs | 83 370 / 90 919 / 100 010 | **0** | 0 / 0 | **congelados**: o `START` durante a transação reinicia a SPIM e a rajada de 11 bytes nunca termina |

O teto real é o barramento: 11 bytes + `START` + CSN ≈ 12,5 µs, e 14 µs é o
último período com dados válidos (71,4 k/s). O wrap do anel não limita: a
ISR escreve o ponteiro logo após `READY`/`STARTED` e tem um período de
prazo, até o próximo `START` (ver Achados). Nestas bancadas chegam 25 a
100 transações por drenagem de 1 ms: de 25 µs para baixo (≥ 32) a thread
espera o wrap acordada, e a latência cai de 16,7 µs (IRQ de idle, a 40 µs)
para 2,4 µs de máximo. Antes de a espera acordada existir, a mesma bancada
na Thingy dava 58 a 352 `late_wraps` por passo de 7 s entre 25 e 14 µs
(Achado 3). O teto no FLPR não foi remedido com o mecanismo atual.

### M33 × FLPR: latência de uma IRQ saindo de idle

`bench/n1-tag.conf` com `APP_WRAP_LATENCY_STATS`: latência do `COMPARE` do
trigger até a ISR de wrap (inclui DPPI → `START` → `DMA.RX.READY` → IRQ).
Medido com o mecanismo anterior, que fazia uma IRQ por amostra pela mesma
cadeia; log não incluído. A linha do mecanismo atual vem de
`u_tag_busmax.log`.

| Configuração | Latência, core em idle (média / máx.) | Latência, core acordado |
|---|---|---|
| M33 padrão | 16,8 / 17,3 µs | 1,7–2,6 µs |
| M33 com wrap em `STARTED` | 17,3 / 17,8 µs | 2,3–2,7 µs |
| M33 + constant latency | 16,8 / 17,1 µs (não resolve) | 1,7 µs |
| M33 + RRAM standby | **2,75 / 2,93 µs** | 1,7–2,3 µs |
| FLPR | **2,43 / 2,50 µs** | 2,43 µs, constante |
| M33 padrão, mecanismo atual | 12,8 / 16,7 µs (a 40 µs, IRQ de idle) | 1,6–2,0 / 2,4 µs (≤ 25 µs, espera acordada) |

Naquela bancada o M33 sustentou uma IRQ por amostra até 50 k/s (máximo
varrido) e o FLPR até 40 k/s (a 50 k/s ≈ 40 % dos wraps atrasavam, core
saturado). Conclusão: os 17 µs são a RRAM em power-down (Achado 2); só
importam para uma IRQ com prazo abaixo de ~18 µs que chegue com o core
dormindo, e o wrap deste engine não está nesse caso. Interpretação
completa no [README da raiz](../README.md#prazo-do-wrap--wake-up-do-core).

## Detalhes de implementação

O engine (`src/spim_dppi.c`) e os backends de sensor são os mesmos do
`gpiote_dppi_spim`. As diferenças:

- o disparo é o `COMPARE0` de um TIMER a 1 MHz com short `CLEAR`, em vez do
  GPIOTE IN; `spim_dppi_set_period_us()` reprograma o período em tempo de
  execução;
- o HFXO é pedido pelo `onoff` do `CLOCK_CONTROL_NRF` antes da configuração;
- a drenagem pode descartar as amostras repetidas
  (`APP_QUEUE_FRESH_ONLY`, bit de data-ready no `STATUS`);
- com `APP_WRAP_LATENCY_STATS` o TIMER roda a 16 MHz e a ISR de wrap
  captura o tempo desde o `COMPARE` (`CC1`); com `APP_COUNT_STARTED` o
  wrap usa a IRQ de `STARTED`; com `APP_RRAM_STANDBY` o `RRAMC` fica em
  standby;
- `src/main.c` traz o `run_sweep()` da bancada.

## Achados

1. **HFXO**: sem o pedido, o TIMER roda do HFINT, cerca de 0,2 % fora (M). O
   FLPR não tem clock control no NCS 3.4.1, por isso o `hfxo_launcher`.
2. **Latência de 17 µs do M33 em idle = wake-up da RRAM.** Com o core em
   idle a RRAM entra em power-down (padrão do `RRAMC`) e a primeira
   instrução da ISR espera `tIDLE2CPU` = 13 µs (D). Medido: 16,8 µs até a
   ISR de wrap (M) = 13 µs de RRAM (D) + ~4 µs de DPPI, IRQ e entrada da
   ISR (M); com o core acordado, 1,7–2,6 µs. Constant latency sozinho não
   resolve; RRAM em standby (`APP_RRAM_STANDBY`) dá 2,75 µs constantes. O
   FLPR roda da RAM: 2,43 µs sem configurar nada. Só importa para uma ISR
   com prazo abaixo de ~18 µs que chegue com o core dormindo; o engine
   espera o wrap acordado nas taxas altas, e nas taxas medidas não houve
   `late_wraps`.
3. **O wake-up do nRF5340 também passa do período nas taxas altas.** Sem
   RRAM, o nRF5340 ainda assim atrasou o wrap quando a IRQ de
   `STARTED` chegava com o core em idle: a primeira versão da drenagem
   (que dormia depois de armar o wrap) deu 58 a 352 `late_wraps` por passo
   de 7 s entre 25 e 14 µs de período na Thingy:53. Com a espera acordada
   (≥ 32 transações por drenagem) foram zero em todos os passos, dados
   válidos até 14 µs (M, `u_thingy_bus64k.log`). A latência de wake-up do
   nRF5340 não foi medida em separado.
4. **O wrap do anel é feito logo após `STARTED` (nRF5340) ou `DMA.RX.READY`
   (nRF54L), nunca após `END`**. No nRF54L o `DMA.RX.READY` é o evento que o
   datasheet define para isso ("EasyDMA armazenou .PTR e .MAXCNT, permitindo
   escrevê-los para a próxima sequência"); a nrfx o chama de `RXSTARTED`. O
   ponteiro é double-buffered e o hardware o reescreve a cada `START`; o
   datasheet só garante a escrita pela CPU "imediatamente após STARTED".
   Uma versão anterior contava `END` e escrevia o ponteiro entre o `END` e
   o `START` seguinte, com 2 a 4 µs de janela a 15 µs de período: a escrita
   coincidia às vezes com a atualização do hardware, o ponteiro ficava
   corrompido e o EasyDMA escrevia fora do anel (MPU/BUS fault reproduzível
   no nRF5340; 2555 `late_wraps` num build e fault noutro, só pela fase da
   ISR). O desenho atual escreve na ISR de `READY`/`STARTED`, armada uma
   vez por drenagem, com um período de prazo, 8 slots de guarda e os
   contadores `late_wraps` e `overflows`: zero em ambos até 71,4 k/s no
   nRF5340 e 52,6 k/s no nRF54L15 (M), sem ISR zero-latency.
5. **`fresh` acima do ODR** com leituras espaçadas menos de cerca de 100 µs:
   o sensor demora a limpar o bit depois da leitura (Thingy a 25 µs: 533
   "fresh"/s para ≈ 380 reais). O filtro é uma heurística; a taxa real é o
   ODR. Com `APP_QUEUE_FRESH_ONLY` a 10 kHz (100 µs) o filtro ainda acerta
   (fresh 402 na TAG); abaixo disso a fila recebe repetidas marcadas como
   novas. Consequência: acima de ~10 k transações/s o caso 2 não garante
   "só amostras novas", e um sensor sem data-ready acima disso não tem
   estratégia limpa aqui (aceitar repetidas ou usar a FIFO do sensor).
6. **Log deferred trava na varredura**: a bancada usa `LOG_MODE_IMMEDIATE`,
   `LOG_BACKEND_RTT_MODE_DROP` e pilha do log em 2048 bytes.
7. **Sysbuild**: `-D<imagem>_CONFIG_X=y` só para símbolos Kconfig; um Kconfig
   novo pede `-p always`; strings via `.conf`. Trocar o `vpr_launcher` exige
   `SB_CONFIG_VPR_LAUNCHER=n` e `ExternalZephyrProject_Add` no
   `sysbuild.cmake` do app.

Os achados de silício (errata 8, `RXDELAY`, tempestade de IRQ, EasyDMA só em
RAM) estão no README do [`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md#achados).
