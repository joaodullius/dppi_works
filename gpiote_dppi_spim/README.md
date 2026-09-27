# gpiote_dppi_spim — leitura de acelerômetro disparada pelo data-ready do sensor (recomendado)

NCS v3.4.1 / nrfx 4.0. O pino de *data-ready* do acelerômetro vira um evento
GPIOTE que, por DPPI, dispara a SPIM; o CSN é do hardware e o EasyDMA
entrega a rajada em RAM. **Uma transação por amostra nova, nenhuma instrução
de CPU no caminho da aquisição.**

```
INT (data-ready) ──GPIOTE IN──DPPI──▶ SPIM.TASKS_START ──▶ EasyDMA (rajada, CSN por hardware)
                                      SPIM.EVENTS_END ──DPPI──▶ TIMER (contador) ──▶ [IRQ por bloco]
```

Diagrama: [`docs/caso1_sensor_int.svg`](../docs/caso1_sensor_int.svg)
(inline no [README da raiz](../README.md)). O caso a 64 kHz do ADXL382 está
descrito lá também.

Por que é o exemplo recomendado: não há timer de disparo, não precisa de HFXO,
não lê amostras repetidas, e a taxa é exatamente o ODR do sensor. O
[`timer_dppi_spim`](../timer_dppi_spim/README.md) é a alternativa quando o
sensor não tem pino de data-ready ou quando se quer uma taxa fixa
independente dele.

| Alvo | Sensor | Barramento | Data-ready | Contador / EGU |
|---|---|---|---|---|
| `thingy53/nrf5340/cpuapp` | ADXL362 | SPIM4 (única com CSN HW no nRF5340), P0.29/28/26, CSN P0.22 | INT1 P0.19 | TIMER2 / EGU0 |
| `nrf54l15tag/nrf54l15/cpuapp` | BMI270 | SPIM22 (P1, domínio PERI), CSN P1.07 | INT P1.04 | TIMER21 / EGU20 |
| `nrf54l15tag/nrf54l15/cpuflpr` | idem | idem (mesmo código, no RISC-V) | idem | idem |

## Kconfig

| Símbolo | Default | Função |
|---|---|---|
| `APP_SENSOR_{ADXL362,BMI270}` | pelo DT (`dt_compat_enabled`) | backend do sensor (`src/sensor_*.c`) |
| `APP_SENSOR_ODR_HZ` | 400 | ODR do sensor = taxa de transações (ADXL362 até 400, BMI270 até 1600) |
| `APP_CONSUME_LATEST` / `APP_CONSUME_QUEUE` | LATEST | último valor num buffer único, ou cada amostra numa `k_msgq` |
| `APP_BLOCK_SAMPLES` / `APP_QUEUE_DEPTH` | 16 / 64 | N amostras por IRQ; profundidade da fila |
| `APP_SPI_FREQ_HZ`, `APP_SPI_CSN_DURATION`, `APP_SPI_RX_DELAY` | 4 MHz, 2, driver | timing da SPIM (ver achados) |
| `APP_REPORT_PERIOD_MS` | 1000 | período do relatório |

```
nrfutil sdk-manager toolchain launch --ncs-version v3.4.1 --chdir C:\ncs\v3.4.1 -- ^
  west build -s <repo>\gpiote_dppi_spim -d <build> -b <alvo> -p always ^
    [-- "-Dgpiote_dppi_spim_CONFIG_APP_CONSUME_QUEUE=y" "-Dgpiote_dppi_spim_CONFIG_APP_SENSOR_ODR_HZ=1600"]
west flash -d <build> --dev-id <serial J-Link>
```

Log de bancada por RTT: Thingy com
`"-Dgpiote_dppi_spim_EXTRA_CONF_FILE=<repo>\gpiote_dppi_spim\overlay-rtt.conf"`;
a Tag já vem com RTT em `boards/nrf54l15tag_*.conf` (não tem UART). Scripts
de flash + captura em [`tools/`](../tools). No FLPR o sysbuild usa o
`vpr_launcher` padrão (aqui não há timer que precise do HFXO).

## Como funciona (src/)

- `app_dt.h` — tudo vem do `chosen app,accel`: barramento = pai do sensor
  (`DT_BUS`), instância nrfx pelo `DT_REG_ADDR`, IRQ pelo `DT_IRQN`, CSN por
  `cs-gpios`, pino de data-ready por `int1-gpios`/`irq-gpios`, instância
  GPIOTE pelo `gpiote-instance` do port (compartilhada com o `gpio_nrfx` via
  `gpiote_nrfx.h`). `chosen app,timer-count` e `app,egu` dão contador e EGU.
- `sensor.h` + `sensor_adxl362.c` / `sensor_bmi270.c` — init bloqueante,
  mapeamento do data-ready no pino (`enable_drdy_int`), descritor da rajada
  e `decode()`. A rajada começa no `STATUS` do sensor: o bit de data-ready
  (`fresh`) serve de verificação (deve ser sempre 1 neste exemplo).
- `spim_dppi.c` — SPIM em modo repetido (`HOLD_XFER | REPEATED_XFER |
  NO_XFER_EVT_HANDLER`, o driver nrfx só é usado no init), GPIOTE IN sem
  handler (só evento), GPPI (`nrfx_gppi_conn_alloc/conn_enable`), TIMER
  contador de END. No modo QUEUE: `RX_POSTINC` (array list) sobre um anel de
  3N slots, `COMPARE0 = N`, `COMPARE1 = 2N` (short CLEAR); a ISR do contador
  é zero-latency e só rebobina `RXD.PTR` (conta `late_wraps` se o DMA já
  passou); o trabalho de fila roda numa ISR de EGU acionada por DPPI.
  **O data-ready é nível**: já está alto quando o DPPI é ligado, então um
  `START` por software lê (e limpa) a primeira amostra; daí em diante cada
  amostra nova gera a borda.
- `main.c` — só relata: LATEST imprime o último valor 1×/s; QUEUE consome a
  fila uma amostra por `k_msgq_get` e imprime estatísticas por segundo.

## Resultados (2026-09-27, log por RTT, `test-logs/`)

Contagem = contador em hardware de `SPIM.END`. `fresh` = amostras com
data-ready no STATUS. Todos com `dropped = 0`, `late = 0`.

| Alvo | ODR | LATEST | QUEUE (N = 16) |
|---|---|---|---|
| Thingy:53 M33 (ADXL362) | 400 Hz (máx. do sensor; real ≈ 380) | 380/s, fresh | 384/s, queued = fresh |
| Tag M33 (BMI270) | 400 Hz | 402/s, fresh | 400/s, queued = fresh |
| Tag M33 (BMI270) | **1600 Hz (máx. do sensor)** | — | **1601–1616/s, queued = fresh, 0 perdas** |
| Tag FLPR (BMI270) | 400 Hz | 402/s | 400/s |
| Tag FLPR (BMI270) | **1600 Hz** | — | **1601–1616/s, 0 perdas** |

O caminho por INT aguenta o que o barramento aguenta (71 k/s medidos no
nRF5340 com rajada de 11 B, 50 k/s na Tag com 17 B — ver o exemplo timer);
o limite é o ODR do sensor. Para o ADXL382 a 64 kHz ver o README da raiz.

## Achados

1. **Data-ready é nível** (ADXL362 e BMI270): fica alto até os registradores
   de dados serem lidos. Foi o que travou o experimento original: sem uma
   primeira leitura, a borda nunca vem. Um `START` por software após ligar o
   DPPI resolve.
2. **nRF54L15 errata 8 (SPIM)**: com CPHA=0, `PRESCALER > 2` e primeiro bit
   1 (`0x83` do BMI270) o MOSI sai errado; o workaround da nrfx precisa de
   uma escrita por transação — impossível com disparo por DPPI. Saída: 8 MHz
   (`PRESCALER = 2`).
3. **`IFTIMING.RXDELAY` no nRF54L é em ciclos de 16 MHz**, não 1/64 MHz como
   no nRF5340: o reset (2) = um bit inteiro a 8 MHz → `APP_SPI_RX_DELAY=1`.
4. **Tempestade de IRQ da SPIM** ao armar o modo repetido no nRF54L (IRQ de
   `STARTED` deixada pela nrfx): todas as interrupções da SPIM são
   desligadas depois de armar.
5. **EasyDMA só lê RAM**: o prefixo TX do backend é copiado para RAM.
6. **GPIOTE compartilhado**: com `CONFIG_GPIO=y` o `gpio_nrfx` já é dono da
   instância; usar `GPIOTE_NRFX_INST_BY_NODE` e `nrfx_gpiote_channel_alloc`
   em vez de inicializar de novo.
7. **Wrap do anel tem prazo de um período**: acima de ~50 k/s a ISR precisa
   ser zero-latency (`IRQ_DIRECT_CONNECT`) e o trabalho de fila vai para a
   EGU; a 1600 Hz nada disso é crítico, mas o mecanismo é o mesmo.
8. **RTT**: passar `-RTTAddress` do `_SEGGER_RTT` do ELF (o bloco do firmware
   anterior fica na RAM); FLPR lido pela conexão M33; resetar antes de anexar.
9. **Sensores mantêm estado entre resets** (rail não cai): o init sempre
   escreve ODR/modo.
