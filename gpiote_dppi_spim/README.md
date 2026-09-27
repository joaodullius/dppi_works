# gpiote_dppi_spim: acelerômetro lido no data-ready via GPIOTE → DPPI → SPIM

## Visão geral

**TL;DR: exemplo recomendado. Uma transação por amostra nova, sem TIMER de
disparo e sem HFXO; o EasyDMA enche um anel e uma thread o drena para uma
`k_msgq` a cada T (10 ms por padrão), ou, com `APP_PER_SAMPLE_IRQ`, a IRQ
de `END` da SPIM copia cada amostra para a fila. Medido até 1600 Hz com
zero perdas no Cortex-M33 e no FLPR, nos dois modos.**

Este exemplo lê um acelerômetro SPI sem CPU no caminho da aquisição. O pino
de data-ready do sensor é ligado a um evento GPIOTE IN. Esse evento, por
DPPI, aciona o `TASKS_START` da SPIM. A SPIM controla o chip select por
hardware e o EasyDMA entrega a rajada em RAM. Cada amostra nova gera
exatamente uma transação, então transações/s = amostras/s.

```
INT (data-ready) ──GPIOTE IN──DPPI──▶ SPIM.TASKS_START ──▶ EasyDMA (rajada, CSN por hardware) ──▶ anel em RAM
                                                                                                   │
                                            thread de drenagem (a cada T): lê o head, k_msgq_put, arma o wrap ◀┘
                                            IRQ DMA.RX.READY / STARTED (1 por volta do anel): PTR = slot 0

   ou (APP_PER_SAMPLE_IRQ):                 ──▶ EasyDMA ──▶ um buffer ──▶ IRQ END: copia para a k_msgq
```

É o único canal DPPI do exemplo. No modo drenado (padrão) o EasyDMA em
*array list* avança um slot por transação sozinho; uma thread acorda a
cada `APP_DRAIN_PERIOD_US`, lê no ponteiro do EasyDMA quantos slots
chegaram, empurra os completos para a `k_msgq` e, quando o head passou da
metade do anel, habilita uma vez a interrupção de `DMA.RX.READY`
(`STARTED` no nRF5340); essa ISR devolve o ponteiro ao slot 0. No modo por
amostra (`APP_PER_SAMPLE_IRQ`) há um buffer só, sem anel nem wrap, e a
interrupção `END` da SPIM copia cada rajada para a `k_msgq` assim que a
transação termina. O consumidor retira as amostras uma a uma, em ordem. A
escolha de produto principal é essa: T = 10 ms (default) para menor CPU e
consumo, T ≈ período do sensor para latência de uma amostra sem prazo
duro, ou o modo por amostra para latência de uma ISR. A instância da SPIM
e o core entram só por barramento e arquitetura (ver o README da raiz).

Este é o exemplo recomendado: não usa TIMER de disparo nem HFXO, não lê
amostras repetidas, e a taxa de transações é exatamente o ODR do sensor. O
exemplo [`timer_dppi_spim`](../timer_dppi_spim/README.md) é a alternativa
para sensores sem pino de data-ready ou para taxa fixa independente do
sensor. Os termos (anel, drenagem, wrap, `late_wraps`, `overflows`,
`torn`, `fresh`), os diagramas de blocos e de timing e o guia "Como
escolher" estão no [README da raiz](../README.md). O engine
`src/spim_dppi.c` tem o mesmo desenho do `timer_dppi_spim`; o de lá tem a
mais o filtro de repetidas, o pedido do HFXO, a captura de latência e as
opções de bancada.

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
| `APP_PER_SAMPLE_IRQ` | n | modo por amostra: um buffer e a IRQ de `END` copia cada amostra para a fila. Uma interrupção por amostra (≈ 1,2 µs de ISR acordado, até 16,5 µs saindo de idle no M33 do nRF54L15, até 23 µs no nRF5340); vale enquanto o período for maior que a transação mais essa latência mais a cópia. Um `START` durante a cópia é detectado (`torn`); um antes de a ISR entrar não. Com ele, os três símbolos seguintes não têm efeito |
| `APP_DRAIN_PERIOD_US` | 10000 | T, período de drenagem (100 µs a 1 s). O período real é T arredondado ao tick do kernel mais um tick (32 µs) mais a drenagem; latência de entrega ≤ esse T real em taxa baixa; 1/T acordares por segundo mais uma IRQ de wrap por volta do anel |
| `APP_RING_SLOTS` | 256 | slots do anel (8 a 4096); mais 8 slots de guarda fixos. O wrap é armado quando o head passa da metade, então o anel deve caber duas drenagens de amostras (≥ 2 × taxa × T real). A guarda só acusa o estouro (`ovf`); além dela a RAM corrompe |
| `APP_WRAP_AWAKE_BELOW_US` | 64 | com amostras mais próximas que isso, a thread espera o wrap acordada (no máximo min(T/4, 8 períodos + 8 µs)) em vez de deixar a IRQ vir de idle; 0 desliga |
| `APP_QUEUE_DEPTH` | 256 | profundidade da `k_msgq` em amostras (pelo menos uma drenagem mais o atraso do consumidor) |
| `APP_SPI_FREQ_HZ` | 4 MHz | clock da SPIM (8 MHz na TAG, ver Achados) |
| `APP_SPI_CSN_DURATION` | 2 | `IFTIMING.CSNDUR` |
| `APP_SPI_RX_DELAY` | −1 (driver) | `IFTIMING.RXDELAY` (1 na TAG, ver Achados) |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório no log |

RAM do engine no modo drenado = (`APP_RING_SLOTS` + 8) × bytes da rajada +
`APP_QUEUE_DEPTH` × bytes da rajada: 8,8 KB com os defaults e 17 B
(BMI270), 5,7 KB com 11 B (ADXL362). No modo por amostra, uma rajada mais
a fila.

### Devicetree

O overlay da placa define tudo o que é específico do hardware por um nó
`chosen`:

| `chosen` | Uso |
|---|---|
| `app,accel` | nó do acelerômetro. O barramento é o pai do nó; `cs-gpios` do barramento dá o pino de CSN; `int1-gpios` ou `irq-gpios` dá o pino de data-ready e a instância GPIOTE do port |

Na Thingy:53 o overlay move o ADXL362 da `spi3` para a `spi4` e acrescenta
`NRF_PSEL(SPIM_CSN, 0, 22)` ao grupo de pinos. Na TAG o overlay acrescenta o
CSN ao `spi22_default` e desliga os outros sensores do barramento. Nenhum
outro periférico é usado.

## Compilação e gravação

```
nrfutil sdk-manager toolchain launch --ncs-version v3.4.1 --chdir C:\ncs\v3.4.1 -- ^
  west build -s <repo>\gpiote_dppi_spim -d <build> -b <alvo> -p always ^
    [-- "-Dgpiote_dppi_spim_CONFIG_APP_SENSOR_ODR_HZ=1600" "-Dgpiote_dppi_spim_CONFIG_APP_PER_SAMPLE_IRQ=y"]
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
<inf> spim_dppi: SPIM @0x500c8000, hardware CSN on pin 39, 8000000 Hz, CSNDUR 2, RXDELAY 1
<inf> bmi270: config upload: 328 bytes in 11 chunks, 23 ms; INIT_ADDR readback 0x0A00
<inf> bmi270: ACC_CONF 0xAC (+/-2 g, ODR 1600 Hz)
<inf> spim_dppi: DPPI connected, burst 17 bytes, ring 256 slots, drain every 10000 us, wrap on DMA.RX.READY
<inf> app: t=3000 ms xfers=4832 queued=1608 fresh=1608 dropped=0 late=0 ovf=0 torn=0 Z avg=0.59 min=0.49 max=0.72 m/s^2
<inf> app: t=4000 ms xfers=6441 queued=1609 fresh=1609 dropped=0 late=0 ovf=0 torn=0 Z avg=0.59 min=0.45 max=0.71 m/s^2
```

No modo por amostra o banner diz `one interrupt per sample` e a conexão
`burst 17 bytes, one buffer, one END interrupt per sample`
(`test-logs/u_tag_int_persample_1600.log`).

`xfers` é o total de transações iniciadas (no modo drenado, voltas
completas do anel mais o head lido no ponteiro do EasyDMA; no modo por
amostra, interrupções `END` atendidas) e avança no ODR do sensor. `fresh`
indica que o bit de data-ready estava ativo na rajada (no caso 1 é sempre
verdadeiro; serve de verificação). `queued` é o número de amostras que
passaram pela fila no período: postas pelo engine e retiradas pelo
consumidor, iguais quando `dropped = 0`. Um teste bem sucedido tem
`queued = fresh`, `dropped = 0`, `late = 0`, `ovf = 0` e `torn = 0`.
`dropped` conta amostras que não couberam na fila. `late` é o contador
`late_wraps`: wraps em que um `START` entrou entre a limpeza do evento e a
escrita do ponteiro; aquela transação usou o slot seguinte ao último, que
não é entregue (a ISR não sabe se o `START` veio antes ou depois de ler o
head): uma amostra perdida por wrap tardio, nunca dado antigo na fila.
`ovf` é `overflows`: voltas em que o EasyDMA chegou aos 8 slots de guarda
antes do wrap (T longo demais para `APP_RING_SLOTS`; além da guarda a RAM
corrompe sem aviso). `torn` (modo por amostra) conta amostras cuja cópia
foi atropelada pelo `START` seguinte, descartadas. Se uma borda de data-ready se perder, a aquisição
para com o pino alto (o data-ready é nível); um watchdog que dispare
`START` por software quando `xfers` não avança não está implementado.

## Resultados

Medidos (M) em 2026-09-27 com log por RTT; os logs estão em `test-logs/`.
Modo drenado com T = 10 ms, anel de 256 slots, fila de 256; modo por
amostra com fila de 256. Todos com `dropped = 0`, `late = 0`, `ovf = 0` e
`torn = 0`. SCK: 4 MHz na Thingy:53 (default do Kconfig), 8 MHz na TAG
(`boards/*.conf`).

| Alvo | ODR | Modo | Transações/s (= amostras/s) | Log |
|---|---|---|---|---|
| Thingy:53 M33 (ADXL362) | 400 Hz (máximo do sensor; real ≈ 372/s) | drenado | 371–373/s, queued = fresh | `u_thingy_int_drain10ms.log` |
| Thingy:53 M33 (ADXL362) | 400 Hz | por amostra | 372/s, queued = fresh | `u_thingy_int_persample.log` |
| TAG M33 (BMI270) | **1600 Hz (máximo do sensor)** | drenado | **1608–1609/s, queued = fresh, 0 perdas** | `u_tag_int_drain10ms_1600.log` |
| TAG M33 (BMI270) | 1600 Hz | por amostra | **1607–1608/s, queued = fresh, 0 perdas** | `u_tag_int_persample_1600.log` |
| TAG FLPR (BMI270) | **1600 Hz** | drenado | **≈ 1607/s**: `xfers` avança 1613–1614 por relatório, mas os relatórios saem a cada ≈ 1004 ms (1,102 → 2,106 → 3,110 s no log), e `queued` alterna 1607/1621 pela mesma janela | `u_tag_flpr_int_drain10ms_1600.log` |
| TAG FLPR (BMI270) | 1600 Hz | por amostra | **1607–1608/s, queued = fresh, 0 perdas** | `u_tag_flpr_int_persample_1600.log` |

Imagens (saída do build): TAG M33 50 208 B de flash no modo drenado e
49 696 B no modo por amostra; TAG FLPR 29 292 B, tudo em RAM. Com T = 10 ms
a 1600 Hz chegam 16 amostras por drenagem, o wrap acontece a cada 8
drenagens (volta de 128 slots) e a CPU atende 112 interrupções por
segundo; a latência de entrega é até o T real, ≈ 10,05 ms (T arredondado
ao tick de 32 µs mais um tick, mais a drenagem). Para latência de uma
amostra sem prazo duro, `APP_DRAIN_PERIOD_US=625` (672 µs reais; não
medido em separado, o mecanismo é o mesmo); para latência de uma ISR,
`APP_PER_SAMPLE_IRQ=y` (medido acima). O custo de CPU de cada opção está modelado em
[`docs/POWER.md`](../docs/POWER.md).

Acima do ODR dos sensores disponíveis o limite deste caminho é o barramento,
não o disparo. Os tetos foram medidos com o exemplo de TIMER, porque nenhum
sensor da bancada gera data-ready além de 1600 Hz: 52,6 k/s na TAG com
17 bytes e 71,4 k/s na Thingy:53 com 11 bytes, com `late = 0` e `ovf = 0`;
o modo por amostra foi medido limpo até 25 k/s na TAG (falso limpo de 33 a
40 k/s) e 10 k/s na Thingy (ver
[`timer_dppi_spim`](../timer_dppi_spim/README.md)). O caso do ADXL382
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
  (`HOLD_XFER | REPEATED_XFER | NO_XFER_EVT_HANDLER`, mais `RX_POSTINC` no
  modo drenado); o driver nrfx só é usado no init, e depois todas as
  interrupções da SPIM são desligadas (no modo por amostra, religa-se a de
  `END`). O GPIOTE IN é configurado sem handler (só evento). A única
  ligação DPPI é feita com `nrfx_gppi_conn_alloc` e `nrfx_gppi_conn_enable`:
  evento GPIOTE IN → `TASKS_START`. Modo drenado: o anel tem
  `APP_RING_SLOTS` + 8 slots; a thread `drain_thread` (prioridade
  cooperativa −1) faz `k_sleep(T)` e chama `drain()`, que lê o head em
  `DMA.RX.PTR` (`RXD.PTR` no nRF5340) e entrega `[tail, head − 1)` à
  fila; com até 4 slots pendentes (`SETTLE_MAX_PENDING`) ela espera um
  tempo de transação (`XFER_SETTLE_US`, `k_busy_wait`) e, se o head não
  mudou (`settled()`), entrega também o slot head − 1. Se o head passou de
  `APP_RING_SLOTS / 2` e não há wrap pendente, habilita a interrupção de
  `DMA.RX.READY` (`RXSTARTED` na nrfx; `STARTED` no nRF5340).
  `wrap_isr()` limpa o evento, lê o head, escreve `PTR = ring[0]`,
  verifica se um `START` entrou no meio (`late_wraps++`; esse slot não é
  entregue), guarda `wrap_last = head − 1` e desabilita a IRQ; a drenagem
  seguinte entrega primeiro a volta antiga até `wrap_last`. Se as amostras
  da drenagem estavam mais próximas que `APP_WRAP_AWAKE_BELOW_US`, a
  thread espera o wrap acordada (no máximo min(T/4, 8 períodos + 8 µs)),
  para que a IRQ não pague o wake-up de idle.
  `overflows` conta, uma vez por volta, o head além do anel. Modo por
  amostra: `sample_isr()` roda na IRQ de `END`, limpa o evento `READY`,
  copia o buffer para uma variável local, confere se `READY` reapareceu
  (`torn++` e descarta) e faz `k_msgq_put`. Como o data-ready é um nível
  já ativo quando o DPPI é ligado, um `START` por software lê e limpa a
  primeira amostra nos dois modos.
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
   todas as interrupções da SPIM depois de armar e só religa a de `READY`
   (uma vez por volta do anel, para o wrap) ou a de `END` (modo por
   amostra).
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
   nRF54L15 (M). A 1600 Hz nada disso é crítico, mas o mecanismo é o mesmo.
8. **O wrap a cada drenagem sobrescrevia a volta anterior.** A primeira
   versão da drenagem armava o wrap em toda drenagem, então cada volta do
   anel durava cerca de um T, e os dois últimos slots da volta antiga (só
   entregues na drenagem seguinte) eram alcançados pela volta nova mais ou
   menos no mesmo instante. O sintoma era `queued/s` maior que `fresh/s`
   em 1 a 5 amostras nas bancadas com filtro, onde isso é impossível: a
   amostra entregue era outra, mais nova. Uma revisão cega do repositório
   achou pelo log. Correção: armar o wrap só quando o head passa da metade
   do anel, o que deixa a volta antiga meio anel à frente da nova e
   transforma a regra "anel ≥ 2 × taxa × T real" em garantia; depois disso
   `queued = fresh` exato em todas as bancadas (M,
   `timer_dppi_spim/test-logs/`). A mesma revisão apontou que um wrap
   tardio podia entregar um slot antigo como amostra; agora esse slot é
   pulado e contado.
9. **O wake-up do core de idle é maior que um período nas taxas altas**,
   nos dois SoCs (≈ 16 µs no M33 do nRF54L15 pela RRAM; ≈ 11 µs na IRQ de
   wrap e até 23 µs na ISR de `END` no nRF5340, M, ver os Achados do
   `timer_dppi_spim`). Por isso a drenagem espera o wrap acordada quando
   as amostras estão mais próximas que `APP_WRAP_AWAKE_BELOW_US` (64 µs).
   Em taxas baixas, como 1600 Hz com T = 10 ms, a IRQ de wrap vem de idle
   e o core dorme entre drenagens; o acordar da própria drenagem paga a
   mesma RRAM.
10. **RTT**: o bloco de controle do firmware anterior fica na RAM, então o
    logger precisa do endereço de `_SEGGER_RTT` do ELF (`-RTTAddress`). O
    FLPR é lido pela conexão M33. Resetar antes de anexar.
11. **Os sensores mantêm estado entre resets** (a alimentação não cai). O
    init sempre escreve ODR e modo.
