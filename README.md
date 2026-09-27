# dppi_works — aquisição SPI sem CPU com xPPI (DPPI/PPI) no nRF5340 e nRF54L15

Exemplos de interligação de periféricos por xPPI: um evento de hardware
dispara a SPIM, o EasyDMA entrega a rajada em RAM e a CPU só entra para
consumir os dados. O caso concreto é um acelerômetro SPI lido no seu
data-ready ou em taxa fixa por TIMER. Os exemplos rodam em nRF Connect SDK
v3.4.1 (nrfx 4.0), com chip select por hardware, e foram validados na
Thingy:53 (nRF5340 + ADXL362) e na nRF54L15 TAG (BMI270), nos cores Cortex-M33
e FLPR.

| Diretório | Conteúdo |
|---|---|
| [`gpiote_dppi_spim/`](gpiote_dppi_spim/README.md) | **Exemplo recomendado.** O pino de data-ready do sensor dispara a SPIM (INT → GPIOTE → DPPI → SPIM). Uma transação por amostra, sem timer, sem HFXO, sem leituras repetidas. |
| [`timer_dppi_spim/`](timer_dppi_spim/README.md) | Exemplo opcional. Um TIMER dispara a SPIM em taxa fixa. Inclui a bancada: varredura de período, teto do barramento e margem timer × ODR. |
| [`docs/`](docs) | Diagramas de blocos e de timing (`gen_diagrams.py` gera os SVG, sem dependências) e [análise de consumo](docs/POWER.md). |
| [`tools/`](tools) | Scripts de gravação e captura de log por RTT usados nos testes. |
| `adxl382_spim/`, `17_adxl362_dt/`, `gpiote_dppi_gpiote/`, `timer_gppi_gpiote/` | Experimentos anteriores (NCS v2.7.0, tag `ncs-v2.7.0`). |

Os dois exemplos compartilham o mesmo engine (`src/spim_dppi.c`), os mesmos
backends de sensor e os mesmos overlays. Cada um traz só o seu disparo. O
consumo das amostras é escolhido por Kconfig nos dois:

- **Modo LATEST**: o EasyDMA reescreve sempre o mesmo buffer. A CPU lê o
  último valor quando quiser, sem interrupções.
- **Modo QUEUE**: o EasyDMA em *array list* enche um anel em ping-pong. A
  cada N transações uma interrupção empurra o bloco para uma `k_msgq`.

## Os dois casos

Cada caso tem um diagrama de **blocos** (quem liga em quem, por DPPI) e um de
**timing** (uma linha por sinal, tempo na horizontal).

### Caso 1: disparo pelo data-ready do sensor (recomendado)

O pino INT do sensor vira um evento GPIOTE que, por DPPI, aciona o
`TASKS_START` da SPIM. O CSN é do hardware e o EasyDMA entrega a rajada em
RAM. Uma transação por amostra nova, nenhuma instrução de CPU no caminho.

![Blocos do caso 1](docs/blocos_caso1_sensor_int.svg)

![Timing do caso 1](docs/caso1_sensor_int.svg)

O data-ready é um **nível**: fica alto até os registradores de dados serem
lidos. Sem uma primeira leitura depois de ligar a medição, a borda de subida
nunca acontece. O exemplo dispara um `START` por software logo após ligar o
DPPI; daí em diante cada amostra nova gera a borda.

Resultado no ODR máximo de cada sensor, modo QUEUE com N = 16, zero perdas:

| Alvo | ODR máximo do sensor | Medido |
|---|---|---|
| Thingy:53 M33, ADXL362 | 400 Hz | 384/s (ODR real do sensor −5 %), queued = fresh |
| Tag M33, BMI270 | 1600 Hz | 1601–1616/s, queued = fresh, 0 perdas |
| Tag FLPR, BMI270 | 1600 Hz | 1601–1616/s, 0 perdas |

### Caso 2: disparo por TIMER (opcional)

Um TIMER dispara a SPIM em período fixo, independente do sensor. Ler os
mesmos registradores em loop basta, porque eles sempre guardam a última
amostra. O bit de data-ready no `STATUS`, lido na mesma rajada, marca as
amostras novas e permite descartar as repetidas.

![Blocos do caso 2](docs/blocos_caso2_timer.svg)

![Timing do caso 2](docs/caso2_timer.svg)

Medido na Tag (ODR real 402/s): um timer a 400/s perde cerca de 2 amostras
por segundo **sem deixar rastro**. A partir de 2 % acima do ODR (408/s) não
perde nenhuma.

![Timer × ODR](docs/timer_vs_odr.svg)

## Consumo das amostras: modo LATEST ou modo QUEUE

No modo LATEST o EasyDMA reescreve sempre o mesmo buffer e a CPU lê uma cópia
coerente quando precisa. No modo QUEUE o EasyDMA em *array list* enche um
anel em ping-pong de 2N amostras, com mais N de folga. Um TIMER em modo
contador conta os `STARTED` da SPIM. No início da última transação do ciclo
uma ISR zero-latency reposiciona o ponteiro (wrap) com a transação inteira
de margem, que é a janela segura do datasheet para escrever o `RXD.PTR`; a
cada bloco completo a EGU, acionada por DPPI, roda a ISR que alimenta a
fila (`k_msgq`).

![Modo QUEUE](docs/modo_queue_pingpong.svg)

## Teto do barramento

A 8 MHz, uma rajada de 17 bytes (BMI270) ocupa 17 µs mais 1 µs de `START` e
o tempo de CSN, cerca de 18,5 µs. Cabem 52,6 k transações por segundo
(19 µs). Com período de 18 µs o `START` chega com a SPIM ocupada e, no
nRF54L15, ela para. Com rajada de 11 bytes (ADXL362) o nRF5340 chegou a
71,4 k/s (14 µs); abaixo disso o `START` reinicia a transação em curso e os
dados deixam de ser válidos, embora os `STARTED` continuem a ser contados.
O wrap do anel do modo QUEUE não limita: tem a transação inteira de margem.

![Teto do barramento](docs/teto_barramento.svg)

| Alvo | TIMER 1 kHz | TIMER 10 kHz + QUEUE | Teto (QUEUE, filtro na ISR) |
|---|---|---|---|
| Thingy:53 M33 (ADXL362, 11 B) | 1000/s exato | 10000/s, fresh ≈ 380 | **71,4 k/s** (14 µs), `late_wraps` 0 de 25 a 14 µs |
| Tag M33 (BMI270, 17 B) | 1000/s exato | 10000/s, fresh 402 | **52,6 k/s** (19 µs), `late_wraps` 0; a 18 µs a SPIM para |
| Tag FLPR (BMI270) | 10001/s (HFXO pedido pelo app core) | — | **50 k/s** (20 µs), `late_wraps` 0 sem ZLI |

## Caso de alta taxa: ADXL382 a 64 kHz

O ADXL382 gera data-ready a até 64 kHz (16, 32 ou 64 kHz selecionáveis). O
caminho do caso 1 serve sem mudança de arquitetura. O que muda é o backend
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
     datasheet, que não está no repositório);
   - `enable_drdy_int()`: mapear `DATA_READY` no INT0 (`INT0_MAP0`),
     polaridade ativa-alta por padrão.
2. **Overlay**: nó `adxl382@0` sob a `spi4` (nRF5340) com `int1-gpios` no
   pino ligado ao INT0 e `spi-max-frequency = <16000000>`. Não há binding
   `adi,adxl382` no Zephyr do NCS 3.4.1: acrescentar
   `dts/bindings/adi,adxl382.yaml` no app (compatível `adi,adxl382`,
   `include: [spi-device.yaml]`, propriedade `int1-gpios`) e apontar o
   `DT_COMPAT` no Kconfig.
3. **`APP_SPI_FREQ_HZ = 16000000`** na SPIM4 do nRF5340. A 8 MHz a rajada de
   11 bytes ocupa 12,5 dos 15,6 µs (80 %; medido 71,4 k/s, funciona, mas sem
   margem para jitter do ODR ou `CSNDUR` maior). A 16 MHz caem para 7 µs
   (45 %). No nRF54L15 as SPIM2x param em 8 MHz; 16 e 32 MHz só na SPIM00
   (domínio MCU), o que exige atravessar o PPIB entre DPPIC20 (GPIOTE20) e
   DPPIC00. A GPPI resolve a ligação; a latência deve ser confirmada.
4. **Modo QUEUE**: `APP_BLOCK_SAMPLES = 64` (1000 IRQ/s), `APP_QUEUE_DEPTH ≥
   512`, `CONFIG_ZERO_LATENCY_IRQS=y`. O wrap é feito logo após o `STARTED`
   da última transação do ciclo e tem a transação inteira de margem (12 µs
   a 8 MHz, 7 µs a 16 MHz). O contador `late_wraps` no log mostra se algum
   wrap ficou para trás. O modo LATEST não tem prazo.
5. **Verificação**: `xfers` no log deve dar 64 000/s mais ou menos a
   tolerância do oscilador do sensor, com `fresh = queued` e `late_wraps`
   em 0.

### Consumo estimado a 64 k amostras/s

Só o SoC; detalhes em [docs/POWER.md](docs/POWER.md).

| SoC / modo | 8 MHz (80 % do barramento) | 16 MHz (45 %) |
|---|---|---|
| nRF5340, INT + LATEST | ≈ 2,4 mA | ≈ 1,8 mA |
| nRF5340, INT + QUEUE (N = 64) | ≈ 3,2 mA | ≈ 2,6 mA |
| nRF54L15 (SPIM2x), INT + LATEST | ≈ 0,3–0,9 mA | — (SPIM00 necessária) |
| nRF54L15 (SPIM2x), INT + QUEUE (N = 64) | ≈ 0,9–1,5 mA | — |

No nRF5340 dominam a SPIM (1,9–2,1 mA enquanto transfere) e o TIMER contador
(0,67 mA); a CPU do modo QUEUE acrescenta cerca de 0,8 mA. No nRF54L15 a
faixa vem de duas incógnitas: quanto o GPIOTE IN mantém ligado no domínio
PERI (2,9 µA a 0,55 mA) e a corrente da SPIM, não publicada (estimada em
0,25 mA). O consumo do ADXL382 não está incluído.

## Achados

Os achados de silício e de bancada estão nos READMEs dos exemplos:
[`gpiote_dppi_spim`](gpiote_dppi_spim/README.md#achados) (errata 8 da SPIM
no nRF54L15, `RXDELAY` em ciclos de 16 MHz, tempestade de IRQ da SPIM,
data-ready em nível, GPIOTE compartilhado) e
[`timer_dppi_spim`](timer_dppi_spim/README.md#achados) (HFXO no FLPR, prazo
do wrap, `fresh` acima do ODR, log deferred).
