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
                               IRQ DMA.RX.READY / STARTED (1 por volta do anel): PTR = slot 0

   ou (APP_PER_SAMPLE_IRQ):    ──▶ EasyDMA ──▶ um buffer ──▶ IRQ END: filtra fresh, copia para a k_msgq
```

A entrega é a do [`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md): no
modo drenado o EasyDMA em array list enche o anel sozinho, uma thread
acorda a cada `APP_DRAIN_PERIOD_US`, entrega os slots completos a uma
`k_msgq` e, quando o head passou da metade do anel, habilita uma vez a
interrupção de `DMA.RX.READY` (`STARTED` no nRF5340), cuja ISR devolve o
ponteiro ao slot 0; no modo por amostra (`APP_PER_SAMPLE_IRQ`) a IRQ de
`END` copia cada rajada de um buffer único para a fila. Com o filtro de
repetidas (`APP_QUEUE_FRESH_ONLY`) só as amostras com o bit de data-ready
ativo entram na fila; as outras contam em `skipped`.

Use este exemplo quando o sensor não tem pino de data-ready ou quando a taxa
de leitura deve ser fixa e independente do sensor. O custo é um TIMER de
disparo, o HFXO (para período exato) e leituras repetidas, que precisam ser
filtradas. Este exemplo também é a bancada do repositório: varre períodos
para medir o teto do barramento, a margem entre timer e ODR, a latência
das ISRs e o limite do modo por amostra. Os termos, os diagramas e a
interpretação estão no [README da raiz](../README.md).

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
| `APP_PER_SAMPLE_IRQ` | n | modo por amostra: um buffer, IRQ de `END` por transação (ver `gpiote_dppi_spim`); os três símbolos seguintes não têm efeito |
| `APP_DRAIN_PERIOD_US` | 10000 | T, período de drenagem (100 µs a 1 s); o período real é T arredondado ao tick (32 µs) mais um tick mais a drenagem, e a latência de entrega ≤ esse T real em taxa baixa |
| `APP_RING_SLOTS` | 256 | slots do anel (8 a 4096), mais 8 de guarda; ≥ 2 × transações por drenagem (com o T real). A guarda só acusa o estouro; além dela a RAM corrompe |
| `APP_WRAP_AWAKE_BELOW_US` | 64 | abaixo desse espaçamento entre transações a thread espera o wrap acordada (no máximo min(T/4, 8 períodos + 8 µs)); 0 desliga |
| `APP_QUEUE_DEPTH` | 256 | profundidade da `k_msgq` em amostras |
| `APP_QUEUE_FRESH_ONLY` | n | filtro na entrega (drenagem ou ISR de `END`): só amostras com data-ready entram na fila; `skipped` conta as repetidas |
| `APP_SPI_FREQ_HZ` | 4 MHz | clock da SPIM (8 MHz na TAG) |
| `APP_SPI_CSN_DURATION` | 2 | `IFTIMING.CSNDUR` |
| `APP_SPI_RX_DELAY` | −1 (driver) | `IFTIMING.RXDELAY` (1 na TAG) |
| `APP_REQUEST_HFXO` | y (se há clock control) | TIMER com período exato |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório no log |
| `APP_WRAP_LATENCY_STATS` | n | bancada: TIMER de disparo a 16 MHz e captura, na ISR de wrap (ou na ISR de `END` no modo por amostra), do tempo desde o `COMPARE` que iniciou a transação (min/avg/max) |
| `APP_WRAP_ON_STARTED` | n | bancada: no nRF54L arma o wrap na IRQ de `STARTED` em vez de `DMA.RX.READY` |
| `APP_RRAM_STANDBY` | n | nRF54L, bancada: RRAM em standby em idle (`RRAMC.POWER.LOWPOWERCONFIG.MODE`) em vez de power-down; wake-up rápido (16–17 µs a menos) de qualquer ISR. Não é necessário para o wrap |

### Devicetree

| `chosen` | Uso |
|---|---|
| `app,accel` | nó do acelerômetro; o barramento é o pai do nó e `cs-gpios` dá o pino de CSN |
| `app,timer-trigger` | TIMER que dispara as transações |

Os overlays são os mesmos do `gpiote_dppi_spim`, mais o `app,timer-trigger`.

### Cenários de bancada

Os arquivos em `bench/` são passados como `EXTRA_CONF_FILE`. Os de taxa
alta no modo drenado usam T = 1 ms, anel de 512 slots e fila de 1024:

| Arquivo | Cenário |
|---|---|
| `sweep-tag.conf` | timer × ODR na TAG: 2500 a 2200 µs, filtro ligado, T = 10 ms |
| `bus-max-tag.conf` | teto do barramento na TAG: 40 a 16 µs, 8 MHz, T = 1 ms, anel 512, fila 1024, latência do wrap |
| `bus-64k-thingy.conf` | teto do barramento na Thingy:53: 100 a 10 µs, 8 MHz, T = 1 ms, anel 512, fila 1024, latência do wrap |
| `per-sample-tag.conf` | modo por amostra na TAG: 1000 a 19 µs, 8 MHz, latência trigger → ISR de `END`, filtro ligado |
| `per-sample-thingy.conf` | modo por amostra na Thingy:53: 1000 a 14 µs, 8 MHz, latência trigger → ISR de `END`, filtro ligado |
| `wrap-latency-tag.conf` | latência do wrap na TAG: 1000 a 20 µs, T = 1 ms; de 1000 a 100 µs a IRQ de wrap vem de idle, de 50 µs para baixo a drenagem espera acordada |
| `wrap-latency-tag-started.conf` | sobre o anterior: wrap na IRQ de `STARTED` em vez de `DMA.RX.READY` (`APP_WRAP_ON_STARTED`) |
| `sweep-flpr.conf` | FLPR com `hfxo_launcher`: 100 a 20 µs, T = 1 ms, latência do wrap (não remedido com o mecanismo atual) |
| `constlat.conf` | sobre `wrap-latency-tag.conf`: M33 em constant latency (`CONFIG_SOC_NRF_FORCE_CONSTLAT`, que exige `CONFIG_NRF_SYS_EVENT`; é a única configuração do repositório que o liga) |
| `rram-standby.conf` | sobre `wrap-latency-tag.conf`: RRAM em standby em idle (`APP_RRAM_STANDBY`) |

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
O build do FLPR precisa de `-DSB_CONFIG_VPR_LAUNCHER=n` na linha de
comando, para o sysbuild não acrescentar também o `vpr_launcher` padrão
(ver o Achado 8). Sem o `hfxo_launcher` o TIMER do FLPR roda do HFINT,
cerca de 0,2 % fora (M, log não incluído).

## Teste

Banner (TAG, `bench/bus-max-tag.conf`, timer de 25 µs, T = 1 ms, filtro
ligado, `test-logs/u_tag_busmax.log`):

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
<inf> app: === sweep result: period 40 us: xfers/s=25005 queued/s=402 fresh/s=402.0 skipped=172234 dropped=0 late_wraps=0 overflows=0 torn=0
<inf> app: === latency (trigger -> wrap ISR): min=1.12 avg=1.18 max=1.93 us
<inf> app: === sweep: period 25 us (40000.0 Hz) for 8 s
<inf> app: === sweep result: period 25 us: xfers/s=40004 queued/s=402 fresh/s=402.0 skipped=277238 dropped=0 late_wraps=0 overflows=0 torn=0
<inf> app: === latency (trigger -> wrap ISR): min=1.12 avg=1.18 max=1.93 us
```

Cada passo descarta o primeiro segundo e imprime os totais dos seguintes.
Campos: `xfers` conta transações iniciadas; `late` no relatório por
segundo e `late_wraps` no resultado da varredura são o mesmo contador, e
`ovf` é `overflows`. `late_wraps` diferente de zero indica que um `START`
entrou entre a limpeza do evento e a escrita do wrap: aquela transação
usou o slot seguinte ao último, que não é entregue (uma amostra perdida
por wrap tardio; nunca dado antigo na fila). `overflows` diferente de zero
indica que o EasyDMA chegou aos slots de guarda antes do wrap: T longo
demais para `APP_RING_SLOTS` (além da guarda a RAM corrompe sem aviso).
`torn` (modo por amostra) conta amostras atropeladas pelo `START`
seguinte durante a cópia. A linha de latência só aparece com
`APP_WRAP_LATENCY_STATS`; no modo por amostra ela é `trigger -> sample
ISR`.

## Resultados

Medidos (M) em 2026-09-27 com log por RTT, todos com o mecanismo de
entrega atual; os logs estão em `test-logs/`. SCK: 8 MHz na TAG; na
Thingy:53, 8 MHz nos benches (`bench/*.conf`).

### Timer × ODR: taxa mínima sem perda

TAG M33, BMI270 a 401,8/s reais, `bench/sweep-tag.conf`, filtro ligado,
modo drenado com T = 10 ms (a coluna "amostras novas/s" é o `fresh/s`,
que com o filtro é igual ao `queued/s`); `test-logs/u_tag_timer_vs_odr.log`:

| Período do timer | Transações/s | Amostras novas/s | Repetidas em 9 s (`skipped`) | Perde amostras? |
|---|---|---|---|---|
| 2500 µs (400/s) | 400 | 399,7 | 0 | **sim, cerca de 2/s, sem rastro** |
| 2475 µs (404/s) | 404 | 401,8 | 19 | não |
| 2450 µs (408/s) | 408 | 402,1 | 56 | não |
| 2425 µs (412/s) | 412 | 401,7 | 94 | não |
| 2400 µs (416/s) | 416 | 401,7 | 133 | não |
| 2350 µs (425/s) | 425 | 402,0 | 213 | não |
| 2300 µs (434/s) | 434 | 402,0 | 296 | não |
| 2200 µs (454/s) | 454 | 401,8 | 474 | não |

Abaixo do ODR real o timer perde amostras em silêncio: o data-ready volta a
subir antes da próxima leitura. O critério de perda é `skipped = 0` com
amostras novas/s abaixo do ODR real: o timer nunca leu uma repetida, logo
perdeu amostras. A partir de 2475 µs aparecem repetidas (`skipped > 0`),
prova de que o timer está à frente do sensor, e o `fresh` fica no ODR
real (401,7–402,1/s, oscilando ±0,2 pela janela). O medido é 0,5 % acima
do ODR real (1 % do nominal) numa unidade; a regra de projeto é timer 5 a
10 % acima do ODR nominal, para cobrir a tolerância do oscilador do
sensor. O resultado não depende do mecanismo de entrega.

### Teto do barramento

Modo drenado, T = 1 ms, anel de 512 slots, fila de 1024, filtro ligado. As
transações/s são `xfers/s`, que vem do ponteiro do EasyDMA e avança a cada
`START`; acima do teto a contagem continua subindo com dados congelados e
o bit `fresh` não é confiável, então a validade é julgada pelo conteúdo
(Z variando). Estes são tetos de barramento, não contagens de amostras
novas. 1/t prevê 54 k/s (17 B) e 80 k/s (11 B); o medido é até 10 %
menor, e no nRF5340 a varredura não tem passo entre 14 e 12 µs.

TAG M33, BMI270, 17 bytes a 8 MHz (`bench/bus-max-tag.conf`,
`test-logs/u_tag_busmax.log`):

| Período | Transações/s | fresh/s | late_wraps / overflows | Latência do wrap (mín. / média / máx.) | Dados (Z mín.–máx. na janela do log) |
|---|---|---|---|---|---|
| 40 µs | 25 005 | 402 (= queued) | 0 / 0 | 1,12 / 1,18 / 1,93 µs (espera acordada) | válidos, 0,55–0,65 m/s² |
| 25 µs | 40 004 | 402 | 0 / 0 | 1,12 / 1,18 / 1,93 µs | válidos, 0,54–0,63 |
| 20 µs | 49 991 | 401,8 | 0 / 0 | 1,68 / 1,68 / 1,93 µs | válidos, 0,55–0,64 |
| **19 µs** | **52 632** | 402 | 0 / 0 | 1,12 / 1,29 / 1,93 µs | válidos, 0,54–0,66 |
| 18 / 17 / 16 µs | 55 549 / 58 816 / 62 490 | 364 / 454 / 453 (fora do ODR) | 0 / 0 | 1,1–1,9 µs | **não comprovados**: `fresh` sai do ODR e a faixa de Z estreita para 0,57–0,61 em todas as janelas, mas não congela de todo; o `START` chega com a SPIM ocupada (17 µs + START + CSN ≈ 18,5 µs) |

Thingy:53 M33, ADXL362, 11 bytes a 8 MHz (`bench/bus-64k-thingy.conf`,
`test-logs/u_thingy_bus64k.log`):

| Período | Transações/s | fresh/s | late_wraps / overflows | Latência do wrap (mín. / média / máx.) | Dados (Z mín.–máx. na janela do log) |
|---|---|---|---|---|---|
| 100 µs | 10 003 | 372 | 0 / 0 | 1,75 / 2,10 / 10,87 µs (IRQ de idle: período ≥ 64 µs) | válidos (ODR real ≈ 372), −9,63 a −6,35 m/s² |
| 50 µs | 20 004 | 367 | 0 / 0 | 1,56 / 1,70 / 2,18 µs (espera acordada) | válidos, −9,36 a −6,44 |
| 25 / 20 / 16 µs | 40 006 / 50 010 / 62 504 | 544 / 407 / 487 | 0 / 0 | 1,56 / 1,69–1,71 / 2,06–2,12 µs | válidos (`fresh` acima do ODR real: o bit é lido duas vezes neste espaçamento), −10,69 a −4,88 |
| 15 µs | 66 694 | 400 | 0 / 0 | 1,56 / 1,72 / 2,12 µs | válidos, −9,68 a −6,02 |
| **14 µs** | **71 426** | 388 | 0 / 0 | 1,56 / 1,72 / 2,12 µs | válidos, −9,32 a −5,97 |
| 12 / 11 / 10 µs | 83 324 / 90 927 / 99 989 | 607 / 607 / 607 (bit sem sentido) | 0 / 0 | 1,56 / 1,71 / 2,06–2,12 µs | **congelados**: Z mín. −8,44 e máx. −6,89 idênticos em todas as janelas dos três passos; o `START` durante a transação reinicia a SPIM e a rajada de 11 bytes nunca termina |

O teto real é o barramento: 11 bytes + `START` + CSN ≈ 12,5 µs, e 14 µs é o
último período com dados válidos (71,4 k/s; o teto real está entre 71,4 e
83 k/s). O wrap do anel não limita: a ISR escreve o ponteiro logo após
`READY`/`STARTED` e tem um período de prazo, até o próximo `START` (ver
Achados). No passo de 100 µs a IRQ de wrap vem de idle e mostra o wake-up
do nRF5340: 10,87 µs de máximo (M). Nos passos de 50 µs para baixo a
thread espera o wrap acordada e a latência fica em 1,56–2,18 µs. Com
`queued/s = fresh/s` exato em todos os passos (antes da correção do
Achado 8 do `gpiote_dppi_spim`, `queued` passava `fresh` em 1 a 5/s). O
teto no FLPR não foi medido com o mecanismo atual.

### Modo por amostra: até onde vale

`APP_PER_SAMPLE_IRQ`, filtro ligado, fila de 1024. A ISR de `END` copia
cada rajada para a fila; `torn` conta as cópias atropeladas pelo `START`
seguinte.

TAG M33, BMI270, 17 bytes a 8 MHz (`bench/per-sample-tag.conf`,
`test-logs/u_tag_persample_sweep.log`; latência = trigger → ISR de `END`,
que inclui os 18,5 µs da transação):

| Período | Transações/s | fresh/s (= queued) | torn | Latência (mín. / média / máx.) | Leitura |
|---|---|---|---|---|---|
| 1000 µs | 1 000 | 402 | 0 | 19,68 / 25,74 / 35,00 µs | core dorme entre amostras: a ISR paga a RRAM (até 16,5 µs) |
| 500 / 250 / 100 µs | 2 000 / 4 000 / 10 001 | 402 | 0 | 19,68 / 22,71–20,29 / 34,75–34,81 µs | idem, média cai porque o core dorme menos |
| 50 / 40 µs | 20 002 / 25 002 | 402 | 0 | 19,68 / 19,99–19,92 / 34,75 µs | ainda há idle entre amostras (máx. 34,75); **último período limpo comprovado: 40 µs (25 k/s)** |
| 30 / 25 µs | 33 337 / 40 006 | 402 | 0 | 0,06 / 19,77 / 27,37 e 9,68 / 19,59 / 22,81 µs | **falso limpo**: mínimos abaixo dos 18,5 µs da transação só são possíveis módulo o período, ou seja, algumas ISRs entraram depois do `START` seguinte; contadores limpos, dados possivelmente misturados |
| 20 µs | 49 994 | 2,8 | **247 716 em 5 s** | 0,00 / 19,58 / 19,93 µs | a cópia é atropelada pelo `START` seguinte (1,5 µs de folga) |
| 19 µs | 52 639 | 402 | 0 | 0,68 / 0,76 / 9,81 µs (módulo o período) | **falso limpo**: a ISR entra 0,7 µs depois do `START` seguinte em todas; a rajada copiada mistura duas transações e o bit `fresh` é o da nova. Não detectável pelo engine |

Thingy:53 M33, ADXL362, 11 bytes a 8 MHz (`bench/per-sample-thingy.conf`,
`test-logs/u_thingy_persample_sweep.log`; latência = trigger → ISR de
`END`, que inclui os 12,5 µs da transação):

| Período | Transações/s | fresh/s | torn (em 5 s) | Latência (mín. / média / máx.) | Leitura |
|---|---|---|---|---|---|
| 1000 / 250 / 100 µs | 1 000 / 4 001 / 9 999 | 371,6 / 371,6 / 372,0 | 0 | 13,9–14,1 / 14,2 / 14,8, 14,4 e 25,3 µs | limpo (ODR real ≈ 372); 12,5 de transação + ≈ 1,5 de ISR; a 100 µs aparece o wake-up (25,3) |
| 50 µs | 19 997 | 367 | 0 | 13,87 / 14,34 / 37,12 µs | −1,3 % de amostras novas: a ISR entra às vezes depois do `START` seguinte (12,5 + até 23 µs de wake-up + cópia), não detectável |
| 40 µs | 25 005 | 352 | 0 | 13,62 / 14,55 / 36,18 µs | −5 %, mesmo mecanismo |
| 30 / 25 µs | 33 331 / 40 002 | 536 / 546 | 3 / 10 | 0,06 e 0,12 de mínimo (módulo o período) | `fresh` acima do ODR (bit lido duas vezes) e as primeiras cópias atropeladas |
| 20 µs | 49 792 | 259 | 0 | 0,25 / 14,47 / 17,56 µs | metade das amostras novas some sem `torn`: a ISR entra depois do `START` seguinte |
| 16 / 15 / 14 µs | 62 222 / 66 135 / 70 819 | 94 / 171 / 269 | 151 764 / 193 766 / 154 | 0,00 de mínimo | 61 %, 59 % e 0,04 % das cópias atropeladas; a 14 µs a ISR quase sempre entra já na transação seguinte |

Regra que sai daqui, em três números: (1) garantia: período > transação +
wake-up máximo de idle (≈ 16,5 µs no M33 do nRF54L15, ≈ 23 µs no nRF5340)
+ 2 µs, ou seja, ≈ 37 µs (27 k/s) na TAG e ≈ 38 µs (26 k/s) na Thingy;
(2) medido limpo: 40 µs (25 k/s) na TAG e 100 µs (10 k/s) na Thingy, com
−1 % a 50 µs; (3) consumo: custa menos que o modo drenado com T = período
até ≈ 12 k/s (E, `docs/POWER.md`). O primeiro sintoma de excesso não é
`torn`, é a latência mínima abaixo da transação. Acima do limite, modo
drenado.

### Latência de uma IRQ saindo de idle (M33)

`bench/wrap-latency-tag.conf` com `APP_WRAP_LATENCY_STATS`, modo drenado
com T = 1 ms, sem filtro: latência do `COMPARE` do trigger até a ISR de
wrap (inclui DPPI → `START` → `DMA.RX.READY` → IRQ), uma vez por volta do
anel (`test-logs/u_tag_wrap_latency.log`):

| Período | Caminho | Latência (mín. / média / máx.) |
|---|---|---|
| 1000 µs | IRQ de idle | 16,12 / 16,14 / 16,50 µs |
| 500 µs | IRQ de idle | 3,37 / 15,56 / 16,50 µs |
| 250 µs | IRQ de idle | 7,43 / 15,81 / 16,50 µs |
| 100 µs | IRQ de idle | 1,25 / 9,50 / 16,18 µs (às vezes o core ainda está acordado) |
| 50 µs | espera acordada (< 64 µs) | 1,81 / 1,87 / 2,12 µs |
| 40 / 30 / 25 / 20 µs | espera acordada | 1,25–1,81 / 1,28–1,90 / 2,06–2,12 µs |

`late_wraps = 0` e `overflows = 0` em todos os passos, `queued/s ≈
xfers/s` (sem filtro; diferem por poucas unidades pela borda da janela).
Os 16,5 µs de máximo são a RRAM em power-down (Achado 2). Com o mecanismo
anterior (uma IRQ por amostra, log não incluído) mediram-se 16,8 µs no M33
padrão (o mesmo com constant latency), 2,75 µs com RRAM em standby e
2,43 µs no FLPR, pela mesma cadeia; esses não foram remedidos.
Interpretação completa no
[README da raiz](../README.md#prazo-do-wrap--wake-up-do-core).

## Detalhes de implementação

O engine (`src/spim_dppi.c`) e os backends de sensor são os mesmos do
`gpiote_dppi_spim`. As diferenças:

- o disparo é o `COMPARE0` de um TIMER a 1 MHz com short `CLEAR`, em vez do
  GPIOTE IN; `spim_dppi_set_period_us()` reprograma o período em tempo de
  execução;
- o HFXO é pedido pelo `onoff` do `CLOCK_CONTROL_NRF` antes da configuração;
- a entrega pode descartar as amostras repetidas
  (`APP_QUEUE_FRESH_ONLY`, bit de data-ready no `STATUS`), na drenagem ou
  na ISR de `END`;
- com `APP_WRAP_LATENCY_STATS` o TIMER roda a 16 MHz e a ISR de wrap (ou
  a de `END`, no modo por amostra) captura o tempo desde o `COMPARE`
  (`CC1`); com `APP_WRAP_ON_STARTED` o wrap usa a IRQ de `STARTED`; com
  `APP_RRAM_STANDBY` o `RRAMC` fica em standby;
- `src/main.c` traz o `run_sweep()` da bancada.

## Achados

1. **HFXO**: sem o pedido, o TIMER roda do HFINT, cerca de 0,2 % fora (M,
   log não incluído). O FLPR não tem clock control no NCS 3.4.1, por isso
   o `hfxo_launcher`.
2. **Latência de ≈ 16 µs do M33 em idle = wake-up da RRAM.** Com o core
   em idle a RRAM entra em power-down (padrão do `RRAMC`) e a primeira
   instrução da ISR espera `tIDLE2CPU` = 13 µs (D). Medido: 16,1 µs de
   média e 16,5 de máximo até a ISR de wrap a 1 000 µs de período (M,
   `u_tag_wrap_latency.log`) = 13 µs de RRAM (D) + ~3 µs de DPPI, IRQ e
   entrada da ISR; com o core acordado, 1,25–2,12 µs. Com o mecanismo
   anterior: constant latency sozinho não resolve; RRAM em standby
   (`APP_RRAM_STANDBY`) dá 2,75 µs constantes; o FLPR roda da RAM, 2,43 µs
   sem configurar nada (M, log não incluído). Importa para qualquer ISR
   com prazo abaixo de ~18 µs que chegue com o core dormindo, e para o
   custo de acordar a drenagem; o engine espera o wrap acordado nas taxas
   altas, e nas taxas medidas não houve `late_wraps`.
3. **O wake-up do nRF5340 também passa do período nas taxas altas.** Sem
   RRAM, o nRF5340 ainda assim atrasou o wrap quando a IRQ de
   `STARTED` chegava com o core em idle: a primeira versão da drenagem
   (que dormia depois de armar o wrap) deu dezenas a centenas de
   `late_wraps` por passo entre 25 e 14 µs de período na Thingy:53 (M,
   execução anterior, log não incluído). Com a espera acordada
   (`APP_WRAP_AWAKE_BELOW_US`) foram zero em todos os passos, dados
   válidos até 14 µs (M, `u_thingy_bus64k.log`). Medido agora com
   `APP_WRAP_LATENCY_STATS`: a IRQ de wrap vinda de idle (passo de
   100 µs) entra em 10,87 µs no máximo, e a ISR de `END` do modo por
   amostra em até 37,12 µs a 50 µs de período, ou seja, ≈ 23 µs além da
   transação (M, `u_thingy_persample_sweep.log`). É esse wake-up que
   limita o modo por amostra no nRF5340 a ~10 k/s.
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
   no nRF5340, e `late_wraps` que variavam de build para build só pela
   fase da ISR). O desenho atual escreve na ISR de `READY`/`STARTED`,
   armada uma vez por volta do anel, com um período de prazo, 8 slots de
   guarda e os contadores `late_wraps` e `overflows`: zero em ambos até
   71,4 k/s no nRF5340 e 52,6 k/s no nRF54L15 (M).
5. **`fresh` acima do ODR** com leituras espaçadas menos de cerca de 100 µs:
   o sensor demora a limpar o bit depois da leitura (Thingy a 25 µs: 547
   "fresh"/s para ≈ 372 reais). O filtro é uma heurística; a taxa real é o
   ODR. Com `APP_QUEUE_FRESH_ONLY` a 10 kHz (100 µs) o filtro ainda acerta
   (fresh 372 na Thingy, 402 na TAG); abaixo disso a fila recebe repetidas
   marcadas como novas. Consequência: acima de ~10 k transações/s o caso 2
   não garante "só amostras novas", e um sensor sem data-ready acima disso
   não tem estratégia limpa aqui (aceitar repetidas ou usar a FIFO do
   sensor).
6. **O modo por amostra tem um limite invisível.** A ISR de `END` detecta
   um `START` que chega durante a cópia (`torn`), mas não um que chega
   antes de ela entrar: a rajada copiada mistura duas transações e o bit
   `fresh` é o da nova. Na TAG isso aparece já a 30 e 25 µs (latência
   mínima de 0,06 e 9,7 µs, abaixo da transação, com contadores limpos) e
   de forma completa a 19 µs (`torn` 0, `fresh` 402, latência de 0,7 µs
   módulo o período); na Thingy a 50 e 40 µs (`fresh` 367 e 352 para
   372). O critério é a latência máxima da ISR mais a transação caberem no
   período, e o sintoma é a latência mínima, não o contador.
7. **Log deferred trava na varredura**: a bancada usa `LOG_MODE_IMMEDIATE`,
   `LOG_BACKEND_RTT_MODE_DROP` e pilha do log em 2048 bytes.
8. **Sysbuild**: `-D<imagem>_CONFIG_X=y` só para símbolos Kconfig; um Kconfig
   novo pede `-p always`; strings via `.conf`. Trocar o `vpr_launcher` exige
   `SB_CONFIG_VPR_LAUNCHER=n` e `ExternalZephyrProject_Add` no
   `sysbuild.cmake` do app.

Os achados de silício (errata 8, `RXDELAY`, tempestade de IRQ, EasyDMA só em
RAM) e o do wrap a cada drenagem estão no README do
[`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md#achados).
