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
| `APP_DRAIN_PERIOD_US` | 10000 | T, período de drenagem (100 µs a 1 s): latência de entrega ≤ T + tick + drenagem |
| `APP_RING_SLOTS` | 256 | slots do anel (8 a 4096), mais 8 de guarda; ≥ 2 × transações por drenagem |
| `APP_WRAP_AWAKE_BELOW_US` | 64 | abaixo desse espaçamento entre transações a thread espera o wrap acordada; 0 desliga |
| `APP_QUEUE_DEPTH` | 256 | profundidade da `k_msgq` em amostras |
| `APP_QUEUE_FRESH_ONLY` | n | filtro na entrega (drenagem ou ISR de `END`): só amostras com data-ready entram na fila; `skipped` conta as repetidas |
| `APP_SPI_FREQ_HZ` | 4 MHz | clock da SPIM (8 MHz na TAG) |
| `APP_SPI_CSN_DURATION` | 2 | `IFTIMING.CSNDUR` |
| `APP_SPI_RX_DELAY` | −1 (driver) | `IFTIMING.RXDELAY` (1 na TAG) |
| `APP_REQUEST_HFXO` | y (se há clock control) | TIMER com período exato |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório no log |
| `APP_WRAP_LATENCY_STATS` | n | bancada: TIMER de disparo a 16 MHz e captura, na ISR de wrap (ou na ISR de `END` no modo por amostra), do tempo desde o `COMPARE` que iniciou a transação (min/avg/max) |
| `APP_COUNT_STARTED` | n | bancada: no nRF54L arma o wrap na IRQ de `STARTED` em vez de `DMA.RX.READY` |
| `APP_RRAM_STANDBY` | n | nRF54L, bancada: RRAM em standby em idle (`RRAMC.POWER.LOWPOWERCONFIG.MODE`) em vez de power-down; wake-up rápido de qualquer ISR. Não é necessário para o wrap |

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
| `bus-max-tag.conf` | teto do barramento na TAG: 40 a 16 µs, T = 1 ms, anel 512, fila 1024, latência do wrap |
| `bus-64k-thingy.conf` | teto do barramento na Thingy:53: 100 a 10 µs, T = 1 ms, anel 512, fila 1024 |
| `per-sample-tag.conf` | modo por amostra na TAG: 1000 a 19 µs, latência trigger → ISR de `END`, filtro ligado |
| `per-sample-thingy.conf` | modo por amostra na Thingy:53: 1000 a 14 µs, filtro ligado |
| `n1-tag.conf` | latência do wrap na TAG: 1000 a 20 µs, T = 1 ms; de 1000 a 100 µs a IRQ de wrap vem de idle, de 50 µs para baixo a drenagem espera acordada |
| `n1-tag-started.conf` | sobre o anterior: wrap na IRQ de `STARTED` em vez de `DMA.RX.READY` |
| `sweep-flpr.conf` | FLPR com `hfxo_launcher`: 100 a 20 µs, T = 1 ms, latência do wrap (não remedido com o mecanismo atual) |
| `constlat.conf` | sobre `n1-tag.conf`: M33 em constant latency (`CONFIG_SOC_NRF_FORCE_CONSTLAT`) |
| `rram-standby.conf` | sobre `n1-tag.conf`: RRAM em standby em idle (`APP_RRAM_STANDBY`) |

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
Sem isso o TIMER do FLPR roda do HFINT, cerca de 0,2 % fora (M, log não
incluído).

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
<inf> app: === sweep result: period 40 us: xfers/s=25001 queued/s=402 fresh/s=402.0 skipped=172207 dropped=0 late_wraps=0 overflows=0 torn=0
<inf> app: === latency (trigger -> wrap ISR): min=1.25 avg=1.62 max=2.12 us
<inf> app: === sweep: period 25 us (40000.0 Hz) for 8 s
<inf> app: === sweep result: period 25 us: xfers/s=39992 queued/s=401 fresh/s=401.8 skipped=277152 dropped=0 late_wraps=0 overflows=0 torn=0
<inf> app: === latency (trigger -> wrap ISR): min=1.25 avg=1.59 max=2.12 us
```

Cada passo descarta o primeiro segundo e imprime os totais dos seguintes.
Campos: `xfers` conta transações iniciadas; `late` no relatório por
segundo e `late_wraps` no resultado da varredura são o mesmo contador, e
`ovf` é `overflows`. `late_wraps` diferente de zero indica que um `START`
entrou entre a limpeza do evento e a escrita do wrap: aquela transação
usou o slot seguinte ao último e nada se perdeu (diagnóstico de wrap
tardio). `overflows` diferente de zero indica que o EasyDMA chegou aos
slots de guarda antes do wrap: T longo demais para `APP_RING_SLOTS`.
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
| 2500 µs (400/s) | 400 | 400,1 | 0 | **sim, cerca de 2/s, sem rastro** |
| 2475 µs (404/s) | 404 | 401,8 | 19 | não |
| 2450 µs (408/s) | 408 | 401,8 | 56 | não |
| 2425 µs (412/s) | 412 | 401,8 | 94 | não |
| 2400 µs (416/s) | 416 | 401,7 | 133 | não |
| 2350 µs (425/s) | 425 | 401,8 | 212 | não |
| 2300 µs (434/s) | 434 | 401,7 | 296 | não |
| 2200 µs (454/s) | 454 | 401,8 | 474 | não |

Abaixo do ODR real o timer perde amostras em silêncio: o data-ready volta a
subir antes da próxima leitura. O critério de perda é `skipped = 0` com
amostras novas/s abaixo do ODR real: o timer nunca leu uma repetida, logo
perdeu amostras. A partir de 2475 µs aparecem repetidas (`skipped > 0`),
prova de que o timer está à frente do sensor, e o `fresh` fica no ODR
real (401,7–401,8/s, oscilando ±0,1 pela janela). O medido é 0,5 % acima
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

| Período | Transações/s | fresh/s | late_wraps / overflows | Latência do wrap (mín. / média / máx.) | Dados |
|---|---|---|---|---|---|
| 40 µs | 25 001 | 402 (= queued) | 0 / 0 | 1,25 / 1,62 / 2,12 µs (espera acordada) | válidos, Z 0,55–0,64 m/s² |
| 25 µs | 39 992 | 402 | 0 / 0 | 1,25 / 1,59 / 2,12 µs | válidos |
| 20 µs | 49 991 | 402 | 0 / 0 | 1,81 / 1,81 / 2,12 µs | válidos |
| **19 µs** | **52 620** | 402 | 0 / 0 | 1,25 / 1,66 / 2,12 µs | válidos |
| 18 / 17 / 16 µs | 55 549 / 58 826 / 62 496 | 546 / 227 / 453 (bit sem sentido) | 0 / 0 | 1,3–2,1 µs | **congelados**: Z mín. 0,55 = máx. 0,58; o `START` chega com a SPIM ocupada (17 µs + START + CSN ≈ 18,5 µs) |

Thingy:53 M33, ADXL362, 11 bytes a 8 MHz (`bench/bus-64k-thingy.conf`,
`test-logs/u_thingy_bus64k.log`):

| Período | Transações/s | fresh/s | late_wraps / overflows | Dados |
|---|---|---|---|---|
| 100 / 50 µs | 10 003 / 20 003 | 372 / 366 | 0 / 0 | válidos (ODR real ≈ 372) |
| 25 / 20 / 16 µs | 40 013 / 50 026 / 62 513 | 547 / 393 / 481 | 0 / 0 | válidos (`fresh` acima do ODR real: o bit não é confiável neste espaçamento) |
| 15 µs | 66 676 | 402 | 0 / 0 | válidos |
| **14 µs** | **71 441** | 416 | 0 / 0 | válidos (Z variando) |
| 12 / 11 / 10 µs | 83 330 / 90 934 / 100 035 | 306 / 303 / 303 (bit sem sentido) | 0 / 0 | **congelados**: Z = −7,29 fixo; o `START` durante a transação reinicia a SPIM e a rajada de 11 bytes nunca termina |

O teto real é o barramento: 11 bytes + `START` + CSN ≈ 12,5 µs, e 14 µs é o
último período com dados válidos (71,4 k/s; o teto real está entre 71,4 e
83 k/s). O wrap do anel não limita: a ISR escreve o ponteiro logo após
`READY`/`STARTED` e tem um período de prazo, até o próximo `START` (ver
Achados). Nestes benches todas as transações estão a menos de 64 µs, então
a thread espera o wrap acordada em todos os passos e a latência fica em
1,25–2,12 µs. Com `queued/s = fresh/s` exato em todos os passos (antes da
correção do Achado 8 do `gpiote_dppi_spim`, `queued` passava `fresh` em
1 a 5/s). O teto no FLPR não foi medido com o mecanismo atual.

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
| 500 / 250 / 100 µs | 2 000 / 4 000 / 10 002 | 402 | 0 | 19,68 / 22,71–20,29 / 34,8 µs | idem, média cai porque o core dorme menos |
| 50 / 40 µs | 19 993 / 25 002 | 401,8 / 402 | 0 | 19,68 / 19,99–19,92 / 34,75 µs | ainda há idle entre amostras (máx. 34,75) |
| 30 / 25 µs | 33 335 / 40 000 | 402 | 0 | ≈ 19,6–19,8 µs de média, 27,4 / 22,8 máx. | o core não chega a dormir; ISR ≈ 1,2 µs |
| 20 µs | 49 994 | 3 | **247 706 em 5 s** | — | a cópia é atropelada pelo `START` seguinte (1,5 µs de folga) |
| 19 µs | 52 636 | 402 | 0 | 0,68 / 0,76 / 9,81 µs (módulo o período) | **falso limpo**: a ISR entra 0,7 µs depois do `START` seguinte; a rajada copiada mistura duas transações e o bit `fresh` é o da nova. Não detectável pelo engine |

Thingy:53 M33, ADXL362, 11 bytes a 8 MHz (`bench/per-sample-thingy.conf`,
`test-logs/u_thingy_persample_sweep.log`):

| Período | Transações/s | fresh/s | torn | Leitura |
|---|---|---|---|---|
| 1000 / 100 µs | 1 000 / 9 999 | 371,6 / 372,8 | 0 | limpo (ODR real ≈ 372) |
| 50 µs | 20 005 | 302 | 0 | perde ~19 % das amostras novas: a ISR entra depois do `START` seguinte (12,5 µs de transação + 14–25 µs de wake-up + cópia > 50 µs às vezes), não detectável |
| 25 µs | 40 011 | 103,8 | 132 833 | 60 % das cópias atropeladas |
| 20 µs e abaixo | 0 | 0 | — | o core não acompanha; o engine para |

Regra que sai daqui: o modo por amostra vale enquanto período > transação
+ wake-up de idle (≈ 16,5 µs no M33 do nRF54L15, > 25 µs no nRF5340) +
2 µs. Na TAG isso dá ≈ 37 µs (27 k/s) com margem, e o medido foi limpo
até 25 µs (40 k/s) porque a essa taxa o core nunca dorme; no nRF5340 dá ≈
40 µs e o medido foi limpo até 100 µs (10 k/s) e com perdas a 50 µs. Acima
disso, modo drenado.

### Latência de uma IRQ saindo de idle (M33)

`bench/n1-tag.conf` com `APP_WRAP_LATENCY_STATS`, modo drenado com T =
1 ms, sem filtro: latência do `COMPARE` do trigger até a ISR de wrap
(inclui DPPI → `START` → `DMA.RX.READY` → IRQ), uma vez por volta do anel
(`test-logs/u_tag_wrap_latency.log`):

| Período | Caminho | Latência (mín. / média / máx.) |
|---|---|---|
| 1000 µs | IRQ de idle | 1,31 / 15,44 / 16,25 µs |
| 500 µs | IRQ de idle | 1,31 / 14,41 / 16,25 µs |
| 250 µs | IRQ de idle | 1,31 / 14,53 / 16,25 µs |
| 100 µs | IRQ de idle | 1,25 / 8,51 / 16,25 µs (às vezes o core ainda está acordado) |
| 50 µs | espera acordada (< 64 µs) | 1,62 / 1,81 / 1,87 µs |
| 40 / 30 / 25 / 20 µs | espera acordada | 1,25–1,81 / 1,27–1,81 / 1,81–1,87 µs |

`late_wraps = 0` e `overflows = 0` em todos os passos, `queued/s =
xfers/s` (sem filtro). Os 16,25 µs de máximo são a RRAM em power-down
(Achado 2). Com o mecanismo anterior (uma IRQ por amostra, log não
incluído) mediram-se 16,8 µs no M33 padrão, 2,75 µs com RRAM em standby e
2,43 µs no FLPR, pela mesma cadeia; esses dois últimos não foram remedidos.
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
  (`CC1`); com `APP_COUNT_STARTED` o wrap usa a IRQ de `STARTED`; com
  `APP_RRAM_STANDBY` o `RRAMC` fica em standby;
- `src/main.c` traz o `run_sweep()` da bancada.

## Achados

1. **HFXO**: sem o pedido, o TIMER roda do HFINT, cerca de 0,2 % fora (M,
   log não incluído). O FLPR não tem clock control no NCS 3.4.1, por isso
   o `hfxo_launcher`.
2. **Latência de 14–16 µs do M33 em idle = wake-up da RRAM.** Com o core
   em idle a RRAM entra em power-down (padrão do `RRAMC`) e a primeira
   instrução da ISR espera `tIDLE2CPU` = 13 µs (D). Medido: 14,4–15,4 µs
   de média e 16,25 de máximo até a ISR de wrap (M,
   `u_tag_wrap_latency.log`) = 13 µs de RRAM (D) + ~2 µs de DPPI, IRQ e
   entrada da ISR; com o core acordado, 1,25–1,87 µs. Com o mecanismo
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
   válidos até 14 µs (M, `u_thingy_bus64k.log`). O mesmo wake-up limita o
   modo por amostra no nRF5340 a ~10 k/s (M, `u_thingy_persample_sweep.log`).
   A latência de wake-up do nRF5340 não foi medida em separado.
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
   `fresh` é o da nova. Na TAG isso aparece a 19 µs (`torn` 0, `fresh`
   402, latência de 0,7 µs módulo o período); na Thingy a 50 µs (`fresh`
   302 para 372). O critério é a latência máxima da ISR mais a transação
   caberem no período, não o contador.
7. **Log deferred trava na varredura**: a bancada usa `LOG_MODE_IMMEDIATE`,
   `LOG_BACKEND_RTT_MODE_DROP` e pilha do log em 2048 bytes.
8. **Sysbuild**: `-D<imagem>_CONFIG_X=y` só para símbolos Kconfig; um Kconfig
   novo pede `-p always`; strings via `.conf`. Trocar o `vpr_launcher` exige
   `SB_CONFIG_VPR_LAUNCHER=n` e `ExternalZephyrProject_Add` no
   `sysbuild.cmake` do app.

Os achados de silício (errata 8, `RXDELAY`, tempestade de IRQ, EasyDMA só em
RAM) e o do wrap a cada drenagem estão no README do
[`gpiote_dppi_spim`](../gpiote_dppi_spim/README.md#achados).
