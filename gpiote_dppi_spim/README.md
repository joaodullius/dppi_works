# gpiote_dppi_spim: acelerômetro lido no data-ready via GPIOTE → DPPI → SPIM

## Visão geral

**TL;DR: exemplo recomendado. Uma transação por amostra nova, sem TIMER de
disparo e sem HFXO; o EasyDMA enche um anel e uma thread o drena para uma
`k_msgq` a cada T (10 ms por padrão). Medido até 1600 Hz com zero perdas
no Cortex-M33 e no FLPR.**

Este exemplo lê um acelerômetro SPI sem CPU no caminho da aquisição. O pino
de data-ready do sensor é ligado a um evento GPIOTE IN. Esse evento, por
DPPI, aciona o `TASKS_START` da SPIM. A SPIM controla o chip select por
hardware e o EasyDMA entrega a rajada em RAM. Cada amostra nova gera
exatamente uma transação, então transações/s = amostras/s.

```
INT (data-ready) ──GPIOTE IN──DPPI──▶ SPIM.TASKS_START ──▶ EasyDMA (rajada, CSN por hardware) ──▶ anel em RAM
                                                                                                   │
                                            thread de drenagem (a cada T): lê o head, k_msgq_put, arma o wrap ◀┘
                                            IRQ DMA.RX.READY / STARTED (1 por T): PTR = slot 0
```

É o único canal DPPI do exemplo. O EasyDMA em *array list* avança um slot
por transação sozinho. Uma thread acorda a cada `APP_DRAIN_PERIOD_US`, lê
no ponteiro do EasyDMA quantos slots chegaram, empurra os completos para a
`k_msgq` e habilita uma vez a interrupção de `DMA.RX.READY` (`STARTED` no
nRF5340); essa ISR devolve o ponteiro ao slot 0. O consumidor retira as
amostras uma a uma, em ordem. T é a escolha de produto principal: T ≈
período do sensor para latência de uma amostra, T = 10 ms (default) para
menor CPU e consumo; interrupções por segundo = 2/T, qualquer que seja a
taxa. A instância da SPIM e o core entram só por barramento e arquitetura
(ver o README da raiz).

Este é o exemplo recomendado: não usa TIMER de disparo nem HFXO, não lê
amostras repetidas, e a taxa de transações é exatamente o ODR do sensor. O
exemplo [`timer_dppi_spim`](../timer_dppi_spim/README.md) é a alternativa
para sensores sem pino de data-ready ou para taxa fixa independente do
sensor. Os termos (anel, drenagem, wrap, `late_wraps`, `overflows`,
`fresh`), os diagramas de blocos e de timing e o guia "Como escolher" estão
no [README da raiz](../README.md). O engine `src/spim_dppi.c` é o mesmo do
`timer_dppi_spim`, que lhe acrescenta o filtro de repetidas e as opções de
bancada.

## Requisitos

| Alvo (`-b`) | Sensor | Barramento | Data-ready |
|---|---|---|---|
| `thingy53/nrf5340/cpuapp` | ADXL362 | SPIM4 (única com CSN por hardware no nRF5340), P0.29/28/26, CSN P0.22 | INT1, P0.19 |
| `nrf54l15tag/nrf54l15/cpuapp` | BMI270 | SPIM22 (P1, domínio PERI), CSN P1.07 | INT, P1.04 (GPIOTE20) |
| `nrf54l15tag/nrf54l15/cpuflpr` | BMI270 | SPIM22, CSN P1.07 (mesmo código, no RISC-V) | INT, P1.04 (GPIOTE20) |

Ferramentas:

- nRF Connect SDK v3.4.1 com a toolchain instalada pelo `nrfutil sdk-manager`.
- J-Link para gravação e log por RTT. A Thingy:53 é gravada pelo conector de
  debug de uma DK; a TAG não tem UART, então o log é sempre por RTT.

## Configuração

### Kconfig

| Símbolo | Default | Função |
|---|---|---|
| `APP_SENSOR_ADXL362` / `APP_SENSOR_BMI270` | pelo devicetree (`dt_compat_enabled`) | backend do sensor (`src/sensor_*.c`) |
| `APP_SENSOR_ODR_HZ` | 400 | ODR do sensor, que é também a taxa de transações (ADXL362 até 400 Hz, BMI270 até 1600 Hz) |
| `APP_DRAIN_PERIOD_US` | 10000 | T, período de drenagem (100 µs a 1 s): latência de entrega; 2/T interrupções por segundo |
| `APP_RING_SLOTS` | 256 | slots do anel (8 a 4096); mais 8 slots de guarda fixos. Deve caber mais de uma drenagem de amostras (≥ 2 × taxa × T) |
| `APP_QUEUE_DEPTH` | 256 | profundidade da `k_msgq` em amostras (pelo menos uma drenagem mais o atraso do consumidor) |
| `APP_SPI_FREQ_HZ` | 4 MHz | clock da SPIM (8 MHz na TAG, ver Achados) |
| `APP_SPI_CSN_DURATION` | 2 | `IFTIMING.CSNDUR` |
| `APP_SPI_RX_DELAY` | −1 (driver) | `IFTIMING.RXDELAY` (1 na TAG, ver Achados) |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório no log |

RAM do engine = (`APP_RING_SLOTS` + 8) × bytes da rajada + `APP_QUEUE_DEPTH`
× bytes da rajada: 8,8 KB com os defaults e 17 B (BMI270), 5,7 KB com 11 B
(ADXL362).

### Devicetree

O overlay da placa define tudo o que é específico do hardware por um nó
`chosen`:

| `chosen` | Uso |
|---|---|
| `app,accel` | nó do acelerômetro. O barramento é o pai do nó; `cs-gpios` do barramento dá o pino de CSN; `int1-gpios` ou `irq-gpios` dá o pino de data-ready e a instância GPIOTE do port |

Na Thingy:53 o overlay move o ADXL362 da `spi3` para a `spi4` e acrescenta
`NRF_PSEL(SPIM_CSN, 0, 22)` ao grupo de pinos. Na TAG o overlay acrescenta o
CSN ao `spi22_default` e desliga os outros sensores do barramento. Nenhum
TIMER ou EGU é usado.

## Compilação e gravação

```
nrfutil sdk-manager toolchain launch --ncs-version v3.4.1 --chdir C:\ncs\v3.4.1 -- ^
  west build -s <repo>\gpiote_dppi_spim -d <build> -b <alvo> -p always ^
    [-- "-Dgpiote_dppi_spim_CONFIG_APP_SENSOR_ODR_HZ=1600" "-Dgpiote_dppi_spim_CONFIG_APP_DRAIN_PERIOD_US=625"]
west flash -d <build> --dev-id <serial J-Link>
```

Os símbolos Kconfig são passados ao sysbuild com o prefixo da imagem
(`-Dgpiote_dppi_spim_CONFIG_...`). No PowerShell, cada argumento `-D` vai
entre aspas.

Log por RTT:

- Thingy:53: acrescentar
  `"-Dgpiote_dppi_spim_EXTRA_CONF_FILE=<repo>\gpiote_dppi_spim\overlay-rtt.conf"`
  (desliga o console USB CDC e liga o RTT).
- TAG: já configurado em `boards/nrf54l15tag_*.conf`.
- Scripts de gravação e captura em [`tools/`](../tools).

No FLPR o sysbuild usa o `vpr_launcher` padrão; este exemplo não precisa do
HFXO.

## Teste

Depois de gravar, o log mostra a inicialização do sensor, o disparo e a
conexão DPPI, e em seguida um relatório por segundo (TAG, BMI270 a
1600 Hz, T = 10 ms, `test-logs/u_tag_int_drain10ms_1600.log`):

```
<inf> app: gpiote_dppi_spim: BMI270, trigger=data-ready pin, drain every 10000 us
<inf> spim_dppi: DPPI connected, burst 17 bytes, ring 256 slots, drain every 10000 us, wrap on DMA.RX.READY
<inf> app: t=1000 ms xfers=1616 queued=1606 fresh=1606 dropped=0 late=0 ovf=0 Z avg=-0.51 min=-0.62 max=-0.40 m/s^2
<inf> app: t=2000 ms xfers=3225 queued=1609 fresh=1609 dropped=0 late=0 ovf=0 Z avg=-0.52 min=-0.63 max=-0.41 m/s^2
```

`xfers` é o total de transações iniciadas (laps completos do anel mais o
head lido no ponteiro do EasyDMA) e avança no ODR do sensor. `fresh` indica
que o bit de data-ready estava ativo na rajada (no caso 1 é sempre
verdadeiro; serve de verificação). `queued` é o número de amostras que
passaram pela fila no período: postas pela drenagem e retiradas pelo
consumidor, iguais quando `dropped = 0`. Um teste bem sucedido tem
`queued = fresh`, `dropped = 0`, `late = 0` e `ovf = 0`. `dropped` conta
amostras que não couberam na fila. `late` é o contador `late_wraps`: wraps
do anel escritos depois de o `START` seguinte já ter ocorrido; a transação
que já tinha começado usou o slot seguinte ao último (a folga) e não entra
na fila (uma amostra perdida, ordem preservada, sem corrupção). `ovf` é
`overflows`: drenagens que encontraram o head além do anel (T longo demais
para `APP_RING_SLOTS`; os 8 slots de guarda absorvem a escrita). Se uma
borda de data-ready se perder, a aquisição para com o pino alto (o
data-ready é nível); um watchdog que dispare `START` por software quando
`xfers` não avança não está implementado.

## Resultados

Medidos (M) em 2026-09-27 com log por RTT, T = 10 ms, anel de 256 slots,
fila de 256; os logs estão em `test-logs/`. Todos com `dropped = 0`,
`late = 0` e `ovf = 0`. SCK: 4 MHz na Thingy:53 (default do Kconfig), 8 MHz
na TAG (`boards/*.conf`).

| Alvo | ODR | Transações/s (= amostras/s) | Log |
|---|---|---|---|
| Thingy:53 M33 (ADXL362) | 400 Hz (máximo do sensor; real ≈ 380/s) | 372–374/s, queued = fresh | `u_thingy_int_drain10ms.log` |
| TAG M33 (BMI270) | **1600 Hz (máximo do sensor)** | **1607–1609/s, queued = fresh, 0 perdas** | `u_tag_int_drain10ms_1600.log` |
| TAG FLPR (BMI270) | **1600 Hz** | **1607–1622/s (alternando: granularidade do relógio de uptime do FLPR), queued = fresh, 0 perdas** | `u_tag_flpr_int_drain10ms_1600.log` |

Imagens: TAG M33 50 168 B de flash e 27 616 B de RAM; TAG FLPR 29 292 B de
código e 5 768 B de dados, tudo em RAM (M). Com T = 10 ms a 1600 Hz chegam
16 amostras por drenagem e a CPU atende 200 interrupções por segundo; a
latência de entrega é 10 ms. Para latência de uma amostra,
`APP_DRAIN_PERIOD_US=625` (não medido separadamente; o mecanismo é o
mesmo). O custo de CPU de cada T está modelado em
[`docs/POWER.md`](../docs/POWER.md).

Acima do ODR dos sensores disponíveis o limite deste caminho é o barramento,
não o disparo. Os tetos foram medidos com o exemplo de TIMER, porque nenhum
sensor da bancada gera data-ready além de 1600 Hz: 52,6 k/s na TAG com
17 bytes e 71,4 k/s na Thingy:53 com 11 bytes, com `late = 0` e `ovf = 0`
(ver [`timer_dppi_spim`](../timer_dppi_spim/README.md)). O caso do ADXL382
a 64 kHz, não testado, está no [README da raiz](../README.md).

## Detalhes de implementação

- `src/app_dt.h`: deriva tudo do `chosen app,accel`. Barramento por
  `DT_BUS`, instância nrfx por `DT_REG_ADDR`, IRQ por `DT_IRQN`, CSN por
  `cs-gpios`, pino de data-ready por `int1-gpios`/`irq-gpios` e instância
  GPIOTE pelo `gpiote-instance` do port (compartilhada com o `gpio_nrfx`
  via `gpiote_nrfx.h`).
- `src/sensor.h`, `src/sensor_adxl362.c`, `src/sensor_bmi270.c`: init
  bloqueante do sensor, mapeamento do data-ready no pino
  (`enable_drdy_int`), descritor da rajada e `decode()`. A rajada começa no
  registrador `STATUS`, cujo bit de data-ready vira o campo `fresh`.
- `src/spim_dppi.c`: engine. A SPIM é armada em modo repetido
  (`HOLD_XFER | REPEATED_XFER | NO_XFER_EVT_HANDLER | RX_POSTINC`); o driver
  nrfx só é usado no init, e depois todas as interrupções da SPIM são
  desligadas. O GPIOTE IN é configurado sem handler (só evento). A única
  ligação DPPI é feita com `nrfx_gppi_conn_alloc` e `nrfx_gppi_conn_enable`:
  evento GPIOTE IN → `TASKS_START`. O anel tem `APP_RING_SLOTS` + 8 slots.
  A thread `drain_thread` (prioridade cooperativa −1) faz `k_sleep(T)` e
  chama `drain()`: lê o head em `DMA.RX.PTR` (`RXD.PTR` no nRF5340),
  entrega `[tail, head − 1)` à fila, e se ainda não há wrap pendente
  habilita a interrupção de `DMA.RX.READY` (`RXSTARTED` na nrfx; `STARTED`
  no nRF5340). `wrap_isr()` escreve `PTR = ring[0]`, verifica se um segundo
  `READY` chegou no meio (`late_wraps++`), guarda `wrap_last` e desabilita
  a IRQ; a drenagem seguinte entrega primeiro o lap antigo até `wrap_last`.
  Se chegaram ≥ 32 amostras na drenagem (`SPIN_WRAP_MIN_ARRIVED`) a thread
  espera o wrap acordada, no máximo T/4, para que a IRQ não pague o
  wake-up de idle. `overflows` conta o head além do anel. Como o data-ready
  é um nível já ativo quando o DPPI é ligado, um `START` por software lê e
  limpa a primeira amostra.
- `src/main.c`: só relata. Consome a fila com `k_msgq_get`, uma amostra por
  vez, e imprime as estatísticas de cada período.

## Achados

1. **O data-ready é um nível** (ADXL362 e BMI270). Fica alto até os
   registradores de dados serem lidos, então sem uma primeira leitura a borda
   nunca acontece. Um `START` por software depois de ligar o DPPI resolve.
2. **nRF54L15, errata 8 da SPIM**: com CPHA = 0, `PRESCALER > 2` e primeiro
   bit em 1 (o `0x83` do BMI270), o MOSI sai errado. O workaround da nrfx
   exige uma escrita por transação, impossível com disparo por DPPI. A saída
   é 8 MHz (`PRESCALER = 2`). A errata só atinge sensores cujo primeiro byte
   tem o bit mais significativo em 1: o `0x0B` do ADXL362 e o `0x23` do
   ADXL382 não são afetados, e para eles a SPIM00 a 32 MHz fica livre.
3. **`IFTIMING.RXDELAY` no nRF54L é em ciclos de 16 MHz**, não em 1/64 MHz
   como no nRF5340. O valor de reset (2) equivale a um bit inteiro a 8 MHz
   e amostra o bit seguinte. `APP_SPI_RX_DELAY=1` corrige.
4. **Tempestade de interrupções da SPIM** ao armar o modo repetido no
   nRF54L: a nrfx deixa a IRQ de `STARTED` habilitada. O exemplo desliga
   todas as interrupções da SPIM depois de armar e só religa a de `READY`,
   uma vez por drenagem, para o wrap.
5. **O EasyDMA só lê RAM**: o prefixo TX do backend é copiado para RAM antes
   de armar a transferência.
6. **GPIOTE compartilhado**: com `CONFIG_GPIO=y` o `gpio_nrfx` já é dono da
   instância. Usar `GPIOTE_NRFX_INST_BY_NODE` e `nrfx_gpiote_channel_alloc`
   em vez de inicializar de novo.
7. **O wrap do anel é feito logo após `STARTED` (nRF5340) ou `DMA.RX.READY`
   (nRF54L), nunca após `END`**. O `DMA.RX.READY` do nRF54L é, pela definição
   do datasheet, "gerado quando o EasyDMA armazenou os registradores .PTR e
   .MAXCNT, permitindo escrevê-los para a próxima sequência"; a nrfx o chama
   de `RXSTARTED` e o exemplo o seleciona com `NRF_SPIM_HAS_DMA_REG`. O
   ponteiro é double-buffered e o hardware o reescreve (`PTR += MAXCNT`) a
   cada `START`; o datasheet diz que o registrador pode ser atualizado
   "imediatamente após o evento STARTED". Uma primeira versão contava `END`
   e escrevia o ponteiro entre o `END` e o `START` seguinte: a 15 µs de
   período a escrita da CPU coincidia às vezes com a atualização do
   hardware, o ponteiro ficava corrompido e o EasyDMA escrevia fora do anel
   (MPU/BUS fault reproduzível no nRF5340). Escrevendo na ISR de
   `READY`/`STARTED`, a escrita tem um período de prazo, até o próximo
   `START`: zero `late_wraps` até 71,4 k/s no nRF5340 e 52,6 k/s no
   nRF54L15 (M), sem ISR zero-latency. A 1600 Hz nada disso é crítico, mas
   o mecanismo é o mesmo.
8. **O wake-up do core de idle é maior que um período nas taxas altas**,
   nos dois SoCs (17 µs no M33 do nRF54L15 pela RRAM; no nRF5340 também
   acima de 14–25 µs, ver os Achados do `timer_dppi_spim`). Por isso a
   drenagem espera o wrap acordada quando chegam ≥ 32 amostras por T. Em
   taxas baixas, como 1600 Hz com T = 10 ms (16 por drenagem), a IRQ de
   wrap vem de idle e o core dorme entre drenagens.
9. **RTT**: o bloco de controle do firmware anterior fica na RAM, então o
   logger precisa do endereço de `_SEGGER_RTT` do ELF (`-RTTAddress`). O
   FLPR é lido pela conexão M33. Resetar antes de anexar.
10. **Os sensores mantêm estado entre resets** (a alimentação não cai). O
    init sempre escreve ODR e modo.
