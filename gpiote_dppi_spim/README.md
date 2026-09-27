# gpiote_dppi_spim: acelerômetro lido no data-ready via GPIOTE → DPPI → SPIM

## Visão geral

**TL;DR: exemplo recomendado. Uma transação por amostra nova, sem TIMER de
disparo e sem HFXO; toda amostra vai para uma `k_msgq` em blocos de N.
Medido até 1600 Hz com zero perdas.**

Este exemplo lê um acelerômetro SPI sem CPU no caminho da aquisição. O pino
de data-ready do sensor é ligado a um evento GPIOTE IN. Esse evento, por
DPPI, aciona o `TASKS_START` da SPIM. A SPIM controla o chip select por
hardware e o EasyDMA entrega a rajada em RAM. Cada amostra nova gera
exatamente uma transação, então transações/s = amostras/s.

```
INT (data-ready) ──GPIOTE IN──DPPI──▶ SPIM.TASKS_START ──▶ EasyDMA (rajada, CSN por hardware)
                                      SPIM.STARTED / DMA.RX.READY ──DPPI──▶ TIMER (contador) ──▶ [IRQ por bloco]
```

Um TIMER em modo contador conta os inícios de transação (`STARTED` no
nRF5340, `DMA.RX.READY` no nRF54L); seus `COMPARE` geram a interrupção a
cada N amostras. O EasyDMA em *array list* preenche um anel de 3N slots; a
cada N amostras a ISR de bloco empurra o bloco para uma `k_msgq`, e o
consumidor as retira uma a uma, em ordem. N é a única escolha de produto:
N = 1 para latência de uma amostra, N = 64 (default) para menor CPU e
consumo.

Este é o exemplo recomendado: não usa TIMER de disparo nem HFXO, não lê
amostras repetidas, e a taxa de transações é exatamente o ODR do sensor. O
exemplo [`timer_dppi_spim`](../timer_dppi_spim/README.md) é a alternativa
para sensores sem pino de data-ready ou para taxa fixa independente do
sensor. Os termos (anel, wrap, `late_wraps`, `fresh`), os diagramas de
blocos e de timing e o guia "Como escolher" estão no
[README da raiz](../README.md). O engine `src/spim_dppi.c` é uma cópia
idêntica do usado pelo `timer_dppi_spim`.

## Requisitos

| Alvo (`-b`) | Sensor | Barramento | Data-ready | Contador / EGU |
|---|---|---|---|---|
| `thingy53/nrf5340/cpuapp` | ADXL362 | SPIM4 (única com CSN por hardware no nRF5340), P0.29/28/26, CSN P0.22 | INT1, P0.19 | TIMER2 / EGU0 |
| `nrf54l15tag/nrf54l15/cpuapp` | BMI270 | SPIM22 (P1, domínio PERI), CSN P1.07 | INT, P1.04 | TIMER21 / EGU20 |
| `nrf54l15tag/nrf54l15/cpuflpr` | BMI270 | SPIM22, CSN P1.07 (mesmo código, no RISC-V) | INT, P1.04 | TIMER21 / EGU20 |

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
| `APP_BLOCK_SAMPLES` | 64 | N, amostras por interrupção (1 para latência de uma amostra) |
| `APP_QUEUE_DEPTH` | 256 | profundidade da `k_msgq` em amostras (pelo menos dois blocos mais o atraso do consumidor) |
| `APP_SPI_FREQ_HZ` | 4 MHz | clock da SPIM (8 MHz na TAG, ver Achados) |
| `APP_SPI_CSN_DURATION` | 2 | `IFTIMING.CSNDUR` |
| `APP_SPI_RX_DELAY` | −1 (driver) | `IFTIMING.RXDELAY` (1 na TAG, ver Achados) |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório no log |

### Devicetree

O overlay da placa define tudo o que é específico do hardware por nós
`chosen`:

| `chosen` | Uso |
|---|---|
| `app,accel` | nó do acelerômetro. O barramento é o pai do nó; `cs-gpios` do barramento dá o pino de CSN; `int1-gpios` ou `irq-gpios` dá o pino de data-ready e a instância GPIOTE do port |
| `app,timer-count` | TIMER usado como contador de inícios de transação (`STARTED` no nRF5340, `DMA.RX.READY` no nRF54L) |
| `app,egu` | EGU que transforma os eventos de bloco em interrupção |

Na Thingy:53 o overlay move o ADXL362 da `spi3` para a `spi4` e acrescenta
`NRF_PSEL(SPIM_CSN, 0, 22)` ao grupo de pinos. Na TAG o overlay acrescenta o
CSN ao `spi22_default` e desliga os outros sensores do barramento.

## Compilação e gravação

```
nrfutil sdk-manager toolchain launch --ncs-version v3.4.1 --chdir C:\ncs\v3.4.1 -- ^
  west build -s <repo>\gpiote_dppi_spim -d <build> -b <alvo> -p always ^
    [-- "-Dgpiote_dppi_spim_CONFIG_APP_SENSOR_ODR_HZ=1600" "-Dgpiote_dppi_spim_CONFIG_APP_BLOCK_SAMPLES=1"]
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
1600 Hz, N = 16):

```
<inf> app: t=25000 ms xfers=40240 queued=1615 fresh=1615 dropped=0 late=0 Z avg=-0.18 min=-0.30 max=-0.07 m/s^2
<inf> app: t=26000 ms xfers=41848 queued=1601 fresh=1601 dropped=0 late=0 Z avg=-0.18 min=-0.28 max=-0.06 m/s^2
```

`xfers` é o contador de inícios de transação em hardware e avança no ODR do
sensor. `fresh` indica que o bit de data-ready estava ativo na rajada (no
caso 1 é sempre verdadeiro; serve de verificação). `queued` é o número de
amostras que passaram pela fila no período: postas pela ISR de bloco (EGU)
e retiradas pelo consumidor, iguais quando `dropped = 0`. Um teste bem
sucedido tem `queued = fresh`, `dropped = 0` e `late = 0`. `dropped` conta
amostras que não couberam na fila. `late` é o contador `late_wraps`: wraps
do anel feitos depois de o `START` seguinte já ter ocorrido; a transação
que já tinha começado foi para a folga do anel e não entra na fila (uma
amostra perdida, ordem preservada, não se acumula de um ciclo para o
outro). Não corrompe memória, e o contador é a forma de detectar.

## Resultados

Medidos (M) em 2026-09-27 com log por RTT; os logs estão em `test-logs/`.
Todos com `dropped = 0` e `late = 0`. SCK: 4 MHz na Thingy:53 (default do
Kconfig), 8 MHz na TAG (`boards/*.conf`).

| Alvo | ODR | Transações/s (N = 16) |
|---|---|---|
| Thingy:53 M33 (ADXL362) | 400 Hz (máximo do sensor; real ≈ 380/s, 368–384 entre janelas) | ≈ 380/s, queued = fresh |
| TAG M33 (BMI270) | 400 Hz | 400/s, queued = fresh |
| TAG M33 (BMI270) | **1600 Hz (máximo do sensor)** | **1601–1616/s, queued = fresh, 0 perdas** |
| TAG FLPR (BMI270) | 400 Hz | 400/s, queued = fresh |
| TAG FLPR (BMI270) | **1600 Hz** | **1601–1616/s, 0 perdas** |

N = 1 e N = 64 usam o mesmo caminho; o custo de CPU de cada N está medido
no bench N = 1 do [`timer_dppi_spim`](../timer_dppi_spim/README.md) e
modelado em [`docs/POWER.md`](../docs/POWER.md).

Acima do ODR dos sensores disponíveis o limite deste caminho é o barramento,
não o disparo. Os tetos foram medidos com o exemplo de TIMER, porque nenhum
sensor da bancada gera data-ready além de 1600 Hz: 52,6 k/s na TAG com
17 bytes e 71,4 k/s na Thingy:53 com 11 bytes (ver
[`timer_dppi_spim`](../timer_dppi_spim/README.md)). O caso do ADXL382 a
64 kHz, não testado, está no [README da raiz](../README.md).

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
  (`HOLD_XFER | REPEATED_XFER | NO_XFER_EVT_HANDLER`); o driver nrfx só é
  usado no init. O GPIOTE IN é configurado sem handler (só evento). As
  ligações são feitas com `nrfx_gppi_conn_alloc` e `nrfx_gppi_conn_enable`.
  O EasyDMA usa `RX_POSTINC` (array list) sobre um anel de 3N slots (bloco
  A, bloco B, folga). O contador conta inícios de transação
  (`STARTED` no nRF5340, `DMA.RX.READY` no nRF54L, que a nrfx chama
  `RXSTARTED`); quando a transação k começa o contador vale k+1, daí:
  `COMPARE0 = N+1` (a transação N começou, o bloco A está completo) e
  `COMPARE2 = 1` (a primeira transação do ciclo seguinte começou, o bloco B
  está completo; ignorado no primeiro ciclo) vão por DPPI para a EGU, cuja
  ISR faz o trabalho da fila; `COMPARE1 = 2N` (a última transação do ciclo
  começou, short `CLEAR`) dispara a ISR zero-latency que só devolve
  `RXD.PTR` ao slot 0, com um período de prazo, até o próximo `START`.
  `late_wraps` conta os wraps feitos depois desse `START`. Como o data-ready
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
   todas as interrupções da SPIM depois de armar.
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
   `RXD.PTR` é double-buffered e o hardware o reescreve (`PTR += MAXCNT`) a
   cada `START`; o datasheet diz que o registrador pode ser atualizado
   "imediatamente após o evento STARTED". Uma primeira versão contava `END`
   e escrevia o ponteiro entre o `END` e o `START` seguinte: a 15 µs de
   período a escrita da CPU coincidia às vezes com a atualização do
   hardware, o ponteiro ficava corrompido e o EasyDMA escrevia fora do anel
   (MPU/BUS fault reproduzível no nRF5340). Contando inícios, a escrita tem um
   período de prazo, até o próximo `START`: zero `late_wraps` até 71,4 k/s
   no nRF5340 e 52,6 k/s no nRF54L15 (M). A ISR de wrap é zero-latency
   (`IRQ_DIRECT_CONNECT`) no M33 e o trabalho da fila vai para a EGU. A
   1600 Hz nada disso é crítico, mas o mecanismo é o mesmo.
8. **RTT**: o bloco de controle do firmware anterior fica na RAM, então o
   logger precisa do endereço de `_SEGGER_RTT` do ELF (`-RTTAddress`). O
   FLPR é lido pela conexão M33. Resetar antes de anexar.
9. **Os sensores mantêm estado entre resets** (a alimentação não cai). O init
   sempre escreve ODR e modo.
