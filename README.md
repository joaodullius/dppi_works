# dppi_works — aquisição SPI sem CPU com DPPI (nRF5340, nRF54L15)

Leitura de acelerômetro SPI em taxa alta sem CPU no caminho: um evento de
hardware (data-ready do sensor ou TIMER) dispara a SPIM por DPPI e o
EasyDMA grava cada rajada em RAM; a CPU só entrega as amostras a uma fila.
nRF Connect SDK v3.4.1 (nrfx 4.0), chip select por hardware. Validado na
Thingy:53 (nRF5340 + ADXL362) e na nRF54L15 TAG (Zephyr `nrf54l15tag`,
BMI270), Cortex-M33 e FLPR.

| Resultado (M) | Valor | Onde |
|---|---|---|
| Data-ready a 1 600 Hz (BMI270), M33 e FLPR, dois modos de entrega | 1 607–1 609 amostras/s, zero perdas | `gpiote_dppi_spim` |
| Data-ready a 400 Hz (ADXL362) | 372–374/s, zero perdas | `gpiote_dppi_spim` |
| Teto do barramento, nRF54L15 SPIM22, 17 B a 8 MHz | 52,6 k transações/s | `timer_dppi_spim` |
| Teto do barramento, nRF5340 SPIM4, 11 B a 8 MHz | 71,4 k transações/s | `timer_dppi_spim` |
| Modo por amostra, limpo pelo critério da latência | até 40 µs de período (25 k/s) nos dois SoCs | `timer_dppi_spim` |

Índice: [Visão geral](#visão-geral) · [Exemplos](#exemplos) ·
[Requisitos](#requisitos) · [Como funciona](#como-funciona) ·
[Limites medidos](#limites-medidos) · [Como escolher](#como-escolher) ·
[Consumo](#consumo) · [Qual SPIM](#nrf54l15-spim22-spim00-spim30) ·
[Medido × estimado](#o-que-está-medido-e-o-que-é-estimado) ·
[Achados](#achados) · [Glossário](#glossário) · [Referências](#referências)

## Visão geral

Um driver convencional faz uma transação SPI por chamada e a CPU acorda a
cada amostra; a 64 k/s isso é a CPU inteira. Aqui o evento "amostra
pronta" aciona `SPIM.START` pelo DPPI sem instrução nenhuma, e o EasyDMA
em *array list* escreve uma transação atrás da outra em RAM. A CPU entra
só para esvaziar o anel: a cada período de drenagem T (modo drenado,
padrão) ou na interrupção `END` de cada transação (modo por amostra). Vale
a partir de ~1 k amostras/s e é indispensável acima de ~10 k/s; abaixo de
~1 k/s use o subsistema de sensores do Zephyr. Vale com PPI no nRF52, não
testado.

![Blocos do caso 1](docs/blocos_caso1_sensor_int.svg)

![Timing do caso 1](docs/caso1_sensor_int.svg)

Marcação: **M** = medido (log em `*/test-logs/`, salvo "log não
incluído"), **E** = estimado, **D** = datasheet, **R** = relato (Nordic
Academy, DevZone).

## Exemplos

| Exemplo | Disparo | Quando usar |
|---|---|---|
| [`gpiote_dppi_spim`](gpiote_dppi_spim/README.md) (caso 1, recomendado) | data-ready → GPIOTE IN → DPPI → `SPIM.START` | sensor com pino: uma transação por amostra, sem TIMER, HFXO nem repetidas |
| [`timer_dppi_spim`](timer_dppi_spim/README.md) (caso 2) | `TIMER.COMPARE0` → DPPI → `SPIM.START` | sensor sem pino ou taxa fixa; é a bancada |
| `docs/` | — | figuras (`gen_diagrams.py`) e [modelo de consumo](docs/POWER.md) |
| `tools/` | — | gravação e captura de log por RTT |

Mesmo engine (`src/spim_dppi.c`), backends e overlays nos dois; o do TIMER
tem a mais o filtro de repetidas, o HFXO, a captura de latência e as opções
de bancada.

## Requisitos

| Item | Valor |
|---|---|
| SDK | nRF Connect SDK v3.4.1, toolchain do `nrfutil sdk-manager` |
| Placas | Thingy:53 (ADXL362 na SPIM4); nRF54L15 TAG (BMI270 na SPIM22), `cpuapp` e `cpuflpr` |
| Depuração | J-Link, log por RTT (a TAG não tem UART; a Thingy é gravada pelo debug de uma DK) |
| Corrente | não medida: consumo é modelo (E) |

## Como funciona

### Disparo

| | Caso 1: data-ready | Caso 2: TIMER |
|---|---|---|
| Evento | GPIOTE IN do pino INT | `COMPARE0` (short `CLEAR`) |
| Taxa de transações | o ODR do sensor | a do timer |
| Repetidas | não | sim; filtro `APP_QUEUE_FRESH_ONLY` (bit de data-ready no `STATUS`) |
| Custo extra | domínio do GPIOTE em idle | TIMER + HFXO |
| Risco | data-ready é nível: borda perdida para a aquisição (`START` por software na partida; watchdog em produto, não implementado) | abaixo do ODR real perde sem aviso; `fresh` não confiável < ~100 µs entre leituras |
| Regra | pino com uma borda por amostra | timer 5 a 10 % acima do ODR nominal |

![Blocos do caso 2](docs/blocos_caso2_timer.svg)

![Timing do caso 2](docs/caso2_timer.svg)

![Timer × ODR](docs/timer_vs_odr.svg)

*TAG, BMI270 a 401,8/s reais (M): timer a 400/s perde ≈ 2/s sem rastro
(`fresh` 399,8, `skipped` 0); a 404/s não perde (`fresh` 401,6, 19 repetidas
em 9 s); de 408 a 454/s `fresh` fica em 401,7–402,1.*

Sensor sem data-ready acima de ~10 k/s: aceitar repetidas ou FIFO do
sensor com *watermark* (uma transação por bloco; não coberta).

### Entrega

| | Modo drenado (padrão) | Modo por amostra (`APP_PER_SAMPLE_IRQ`) |
|---|---|---|
| Mecanismo | anel em array list; thread a cada T entrega os slots completos à `k_msgq` e arma o wrap uma vez por volta | um buffer; a IRQ de `END` copia cada rajada para a `k_msgq` |
| Latência | ≤ T real com até 4 amostras por drenagem; acima, a mais nova sai na seguinte (T real + um período ≈ 10,7 ms a 1 600 Hz, T = 10 ms) | entrada da ISR + cópia: ≈ 1,2 µs acordado; até 15,5 µs de idle no M33 nRF54L15, 26,3 µs no nRF5340 (M) |
| Interrupções | 1/T + 1 wrap por volta (anel/2 amostras) | uma por amostra |
| RAM | (`APP_RING_SLOTS` + 8 + fila) × B: 8,8 KB (17 B), 5,7 KB (11 B) | uma rajada + fila |
| Limite de taxa | teto do barramento | período > transação + entrada máx + 2 µs: ≈ 36 µs TAG, ≈ 41 µs Thingy (E); medido limpo até 40 µs (25 k/s) |
| Falha detectável | `late` (wrap tardio, limite superior de perdas), `ovf` (anel pequeno) | `torn` (START durante a cópia); START **antes** da ISR não é detectável |
| Consumo a 1 600/s, SPIM22 (E) | 48 µA (T = 10 ms), 176 µA (T = 625 µs) | 59 µA |
| Quando | bloco; qualquer taxa até o teto | latência de uma ISR, timestamp na ISR, ≤ 25 k/s |

Cada amostra passa sozinha pela fila (`k_msgq_put` + `k_msgq_get` + decode
≈ 3,5 µs, E): 22 % da CPU a 64 k/s; entrega em bloco não implementada.

![Anel, drenagem e wrap](docs/anel_drenagem.svg)

![Modo por amostra](docs/por_amostra.svg)

### Anel e wrap (modo drenado)

O ponteiro do EasyDMA conta transações iniciadas, não terminadas: a
drenagem entrega `[tail, head − 1)` e, com até 4 pendentes, espera um tempo
de transação (`XFER_SETTLE_US` ≈ 21 µs para 17 B, 15 µs para 11 B) e
entrega também o slot head − 1. Quando o head passa da metade do anel, a
drenagem habilita uma vez a IRQ de `DMA.RX.READY` (nRF54L) / `STARTED`
(nRF5340); a ISR escreve `PTR = slot 0` na janela do datasheet
("imediatamente após STARTED"), com um período de prazo. Um `START` entre a
limpeza do evento e a escrita usa o slot k + 1: `late` conta e o slot é
entregue ou pulado, nunca dado antigo. Nas taxas altas a thread espera o
wrap acordada (`APP_WRAP_AWAKE_BELOW_US`, 64 µs; ≤ min(T/4, 8 períodos +
8 µs) ≈ 520 µs, bloqueando as threads preemptíveis). Wrap durante o
assentamento: a drenagem entrega só até o head lido (`wrap_done` é lido
antes do head).

| Regra | Valor |
|---|---|
| T real | ⌈T/tick⌉ · tick + 1 tick + drenagem: 10 ms → ≈ 10,07 ms, 1 ms → ≈ 1,08 ms, 625 µs → ≈ 710 µs, 100 µs → ≈ 180–190 µs (tick 32 µs nRF54L15, 30,5 µs nRF5340) |
| Anel | `APP_RING_SLOTS` ≥ 2 × taxa × T real (máx 4 096); voltas entre anel/2 e anel + 8 |
| Guarda | 8 slots: só acusam o estouro (`ovf`); além deles o EasyDMA corrompe a RAM |
| Fila | `APP_QUEUE_DEPTH` ≥ amostras por drenagem + atraso do consumidor |
| Acordar da drenagem | paga a RRAM como qualquer IRQ (`CONFIG_NRF_SYS_EVENT` só em `constlat.conf`): 16,1 µs de média a ≥ 500 µs, 9,0 a 100 µs (M, proxy pela ISR de wrap) |

## Limites medidos

### Teto do barramento

t = bytes × 8 / SCK + 1,5 µs (E; 1,5 inferido do teto); 1/t é otimista em
até 10 %.

| Rajada, SCK | Transação (E) | Ocupação a 64 k/s |
|---|---|---|
| 11 B, 8 / 16 / 32 MHz | 12,5 / 7,0 / 4,25 µs | 80 / 45 / 27 % |
| 17 B, 8 / 32 MHz | 18,5 / 5,75 µs | > 100 % (teto 52,6 k/s) / 37 % |

| Alvo | Último período válido | Transações/s | Acima do teto | Fonte |
|---|---|---|---|---|
| TAG M33, SPIM22, 17 B | 19 µs | **52,6 k** | ponteiro segue (55,6–62,5 k/s), `fresh` sai do ODR (363 / 454 / 453 contra 402), Z estreita (0,58–0,62 contra 0,54–0,65): 18–16 µs não comprovados | M, `u_tag_busmax.log` |
| Thingy:53, SPIM4, 11 B | 14 µs | **71,4 k** | `xfers` segue (83–100 k/s), nada passa pelo filtro; teto real entre 71,4 e 83 k/s | M, `u_thingy_bus64k.log` |

![Teto do barramento](docs/teto_barramento.svg)

Acima do teto nada no log acusa por si: o critério é o conteúdo. SPIM2x
com 11 B: 80 k/s pela fórmula, ≈ 71–80 k/s (E); FLPR não medido.

### Latência de uma IRQ saindo de idle

| Caminho | M33 nRF54L15 | nRF5340 | Fonte |
|---|---|---|---|
| IRQ de wrap, core em idle | 16,1 µs média, 16,3 máx (RRAM: `tIDLE2CPU` 13 µs, D, + ~2) | 2,7 média, 24,4 máx | M, `u_tag_wrap_latency.log`, `u_thingy_bus64k.log` |
| IRQ de wrap, espera acordada (< 64 µs) | 1,22–1,81 média, 2,18 máx | 1,85–1,87 média, 2,25 máx | M, idem |
| Entrada da ISR de `END` (máx − mín do mesmo passo) | 15,5 µs | 26,3 µs | M, `u_*_persample_sweep.log` |
| Mecanismo anterior, log não incluído | 16,8 (máx 17,3; igual com constant latency); 2,75 com RRAM standby; FLPR 2,43 | — | M |

![M33 × FLPR](docs/m33_vs_flpr_nrf54l15.svg)

O wrap não exige ZLI, RRAM standby nem FLPR (0 `late_wraps` até os tetos);
esses ficam para ISRs do produto e para livrar o M33. FLPR sem consumo
medido (`vpr_offloading` 146 → 125 µA, R; DevZone +0,5 mA de idle do VPR, R).

### Limite do modo por amostra

| | TAG (17 B) | Thingy:53 (11 B) |
|---|---|---|
| Fórmula (E) | 18,5 + 15,5 + 2 ≈ 36 µs (27 k/s) | 12,5 + 26,3 + 2 ≈ 41 µs (24 k/s) |
| Medido limpo (mínimo da latência acima da transação, `torn` 0) | 40 µs (25 k/s); 30–25 µs marginal (numa captura, ISRs após o `START` seguinte) | 40 µs sem ISR atrasada mas −5 % de novas (em aberto); sem ressalva até 50 µs (20 k/s) |
| Falha | 20 µs: `torn` em quase todas; 19 µs: falso limpo | 30 µs: mínimo 0,06 µs e `torn`; 20 µs: −35 % sem `torn` |

## Como escolher

Entradas: ODR, B (bytes com o comando), SCK máximo, latência aceitável, SoC.

1. **Disparo.** Pino de data-ready → caso 1. Sem pino → caso 2, período =
   1/(ODR × 1,05..1,10), `APP_QUEUE_FRESH_ONLY=y`.
2. **Barramento.** t_trans = B × 8 / SCK + 1,5 µs; sobra = t_per − t_trans
   ≥ 10 % de t_per ou ≥ 2 µs → ok (nRF5340 validou 89 % de ocupação). Senão
   subir o SCK: SPIM4 a 16/32 MHz (nRF5340); só SPIM00 a 32 MHz no nRF54L15
   (não testado; errata 8 se o comando tiver MSB 1).
3. **Entrega.**

   | Requisito | Configuração | Latência | Consumo (E, 1 600/s) |
   |---|---|---|---|
   | alguns ms | drenado, T = 10 ms (1 ms acima de ~12 k/s) | ≤ T real (+ 1 período com > 4 por drenagem) | 48 µA |
   | uma amostra, sem prazo duro | drenado, T ≈ t_per (mín 100 µs) | ≤ T real (625 → ≈ 710 µs) | 176 µA (2,3–3,6× até 16 k/s) |
   | uma ISR, determinística | `APP_PER_SAMPLE_IRQ=y`, ≤ 25 k/s | entrada da ISR + cópia | 59 µA |

   Anel ≥ 2 × taxa × T real; fila ≥ amostras por drenagem + atraso do
   consumidor. Referências: 1 600/s, T = 10 ms → 16 por drenagem, anel 256,
   112 IRQ/s; 64 k/s, T = 1 ms → 69 por drenagem, anel 256 (512 na bancada),
   ≈ 1 390 IRQ/s; 50 k/s com T = 10 ms pediria anel de 1 000. Timestamp no
   modo drenado: índice × período ou contagem do data-ready.
4. **Onde roda.** SPIM2x no nRF54L15 (M); SPIM00 só pelo passo 2 (E); FLPR
   para livrar o M33. O prazo do wrap é resolvido pelo engine.

| Exemplo resolvido | Configuração | Consumo (E) | Notas |
|---|---|---|---|
| BMI270, 1 600 Hz, 17 B, nRF54L15 | caso 1, drenado T = 10 ms, SPIM22 | ≈ 51 µA; T = 625 µs ≈ 197 µA; por amostra ≈ 61 µA | 3 % de ocupação; medido 1 608–1 609/s M33, ≈ 1 607/s FLPR |
| ADXL382, 64 kHz, 11 B, latência 1 ms, nRF54L15 | caso 1, drenado T = 1 ms, SPIM22 a 8 MHz (80 %) ou SPIM00 a 32 MHz (27 %) | ≈ 0,88 / ≈ 1,20 mA | por amostra fora da faixa (15,6 < 12,5 + 15,5 + 2); errata 8 não atinge `0x23`; não testado |
| ADXL382, 64 kHz, 11 B, nRF5340 | caso 1, drenado T = 1 ms, SPIM4 a 8 MHz (80 %) ou 16 MHz (45 %) | ≈ 2,2 / ≈ 1,7 mA | SCK máximo e pinos a confirmar; perfil medido a 71,4 k/s com o ADXL362 |
| Sem data-ready, 16 kHz, 11 B, nRF54L15 | caso 2, timer 59,5 µs (21 %), drenado T = 1 ms | ≈ 0,43 mA | `fresh` não confiável a 59,5 µs (ADXL362 a 25 µs: 543,5 novas/s para ≈ 372) |

![ADXL382 a 64 kHz](docs/caso_adxl382_64k.svg)

ADXL382 no `gpiote_dppi_spim`: backend `sensor_adxl382.c` (comando
`(0x11 << 1) | 1` = `0x23`, 11 B `STATUS0..ZDATA_L`, `fresh_mask 0x01`,
big-endian, `DEVID_AD` 0xAD, `OP_MODE` 0x26 com ODR a confirmar,
`DATA_READY` no INT0), binding `adi,adxl382.yaml`, overlay `adxl382@0`,
`APP_SPI_FREQ_HZ = 16000000` na SPIM4, T = 1 ms. Verificar `xfers` ≈
64 000/s, `fresh = queued`, `late = ovf = 0`, Z variando.

## Consumo

Modelo (E, sem PPK2): base 2,9 µA; PERI 20 µA (R); MCU 300 µA só na
SPIM00; SPIM 0,25 mA (SPIM2x) ou 0,8 mA (SPIM00) × ocupação; CPU 2,6 mA ×
fração, com o acordar médio medido (16,1 µs a ≥ 500 µs, 9,0 a 100 µs, 1,2
acordado), 5 µs por drenagem, 3,5 µs por amostra e, por amostra, a entrada
média da ISR (3,8 µs a 625 µs, 0,4 a 62,5 µs, interpolado) + 1,2 + 2,5 µs.
Caso 1, 11 B, M33, µA:

| Entrega | Instância | 1 600/s | 16 k/s | 50 k/s |
|---|---|---|---|---|
| drenado, T = 10 ms (1 600/s) e 1 ms (16 k, 50 k) | SPIM22 (8 MHz) | **48** | **288** | **702** |
| drenado, T = 10 ms, 1 ms | SPIM00 (32 MHz) | 349 | 592 | 1 016 |
| drenado, T = período (625 µs) e 100 µs (176–191 µs reais) | SPIM22 | 176 | 675 | 952 |
| drenado, T = período, 100 µs | SPIM00 | 476 | 979 | 1 265 |
| por amostra | SPIM22 | 59 | 245 | fora da faixa |
| por amostra | SPIM00 | 359 | 549 | fora da faixa |
| qualquer, FLPR | SPIM22 | sem número | sem número | sem número |

![Consumo por modo de entrega, instância e taxa](docs/consumo_modos_nrf54l15.svg)

- T curto custa 3,6× / 2,3× / 1,4× o T longo (1,6 k / 16 k / 50 k/s): as
  drenagens (~21 µs, 16,1 de RRAM) mais 15 µs de assentamento.
- Por amostra custa menos que T = período em toda a faixa e, a 16 k/s,
  menos que T = 1 ms (245 contra 288): a ISR acorda em 0,4–4 µs de média.
- SPIM00: +300 µA; só por barramento. 17 B: + 0,25 mA × taxa × 6 µs
  (SPIM22) ou 0,8 mA × taxa × 1,5 µs (SPIM00).
- nRF5340 (SPIM4, 11 B): ≈ 0,11 mA a 1 600/s (T = 10 ms), 0,22 (625 µs),
  0,11 (por amostra); 64 k/s ≈ 2,2 mA a 8 MHz, 1,7 a 16 MHz (T = 1 ms),
  2,4 / 1,9 com T = 100 µs.

Regra: data-ready + drenado T = 10 ms na SPIM22; por amostra para latência
de uma amostra (≤ 25 k/s); SPIM00 só por barramento. Detalhes em
[`docs/POWER.md`](docs/POWER.md).

## nRF54L15: SPIM22, SPIM00, SPIM30

| Instância | Domínio | Core / SCK máx | Pinos | DPPI | Estado | Quando |
|---|---|---|---|---|---|---|
| SPIM20/21/22 | PERI | 16 MHz / 8 MHz (`PRESCALER` 2..126) | P1 (20/21 também P2) | DPPIC20, 16 canais; sem PPIB com o GPIOTE20 | M | caso geral |
| SPIM00 | MCU | 128 MHz / 32 MHz (`PRESCALER` 4..126) | P2 dedicados, drive E0/E1 | DPPIC00, 8 canais; disparo pelo PPIB | E | rajada longa a 64 k/s, > 71–80 k/s, SCK > 8 MHz, MCU já ligado; +300 µA; errata 8 com MSB 1 (`0x83` sim; `0x0B`, `0x23` não; BMI270 aceita modo 3) |
| SPIM30 | LP | 16 MHz / 8 MHz | P0 | DPPIC30, 4 canais | E | variante 100 % LP com GPIOTE30: PERI dorme, ≈ −20 µA (R); é um overlay |

Na TAG só a SPIM22 alcança o sensor (P1.05/06/08, CSN P1.07, INT P1.04).
Teste no nRF54L15 DK: SPIM00 em P2.06/08/09/10; SPIM30 em P0.00–P0.03 com a
UART0 solta no Board Configurator. Só o overlay muda; a GPPI resolve o PPIB.

## O que está medido e o que é estimado

| Instância / core | Disparo | Drenado | Por amostra | Teto | Consumo |
|---|---|---|---|---|---|
| nRF54L15 SPIM22, M33 | data-ready e TIMER | M até 52,6 k/s | M: limpo até 25 k/s, marginal 33–40 k/s, falso limpo a 52,6 k/s | M, 52,6 k/s | E |
| nRF54L15 SPIM22, FLPR | data-ready | M, 1 600 Hz | M, 1 600 Hz | não medido | não medido |
| nRF54L15 SPIM00, SPIM30 | — | E (overlay) | E | E | E |
| nRF5340 SPIM4, M33 | data-ready e TIMER | M até 71,4 k/s | M: sem ressalva até 20 k/s; 25 k/s com −5 % em aberto; atropeladas de 33 k/s | M, 71,4 k/s | E |
| ADXL382 | — | E | fora da faixa | E (perfil de 11 B medido com o ADXL362) | E |

Latências de IRQ do mecanismo atual: M com log; RRAM standby, FLPR e
constant latency: M anterior, sem log. Correntes: nenhuma medida.

## Achados

- Data-ready dos dois sensores é nível: sem uma primeira leitura a borda
  nunca vem; `START` por software na partida.
- nRF54L15, errata 8 da SPIM: CPHA = 0, `PRESCALER > 2` e MSB do comando
  em 1 corrompem o MOSI; workaround incompatível com DPPI; 8 MHz nas SPIM2x.
- `IFTIMING.RXDELAY` no nRF54L é em ciclos de 16 MHz: reset (2) amostra o
  bit seguinte a 8 MHz; usar 1.
- Wrap logo após `STARTED`/`DMA.RX.READY`, nunca após `END`: a versão
  anterior colidia com o hardware (MPU/BUS fault a 15 µs no nRF5340).
- Wrap a cada drenagem deixava a volta nova alcançar a anterior (`queued` >
  `fresh` em 1–5/s); armar em anel/2 corrigiu (revisão cega do log).
- Wake-up de idle maior que o período nas taxas altas nos dois SoCs
  (16,3 µs pela RRAM; 24,4 µs no nRF5340): a espera acordada resolve.
- Modo por amostra: `START` antes de a ISR entrar mistura duas transações
  sem sintoma; o sinal é a latência mínima abaixo da transação.
- `fresh` acima do ODR com leituras < ~100 µs; acima de ~10 k/s o caso 2
  não garante "só amostras novas".
- A nrfx deixa a IRQ de `STARTED` ligada ao armar o modo repetido no
  nRF54L: desligar todas depois de armar.
- Histórico: contador em TIMER, EGU e ISR zero-latency não existem mais
  (121 µA no nRF54L15, 475 µA no nRF5340 saíram do modelo).

## Glossário

| Termo | Significado |
|---|---|
| transação, rajada | `START` → bytes → `END`; comando + `STATUS` + dados (11 B ADXL362, 17 B BMI270); no caso 1 = uma amostra |
| amostra nova, `fresh` | bit de data-ready no `STATUS` da rajada (heurístico < ~100 µs) |
| período | entre dois `START` (1/ODR no caso 1, do timer no caso 2) |
| ODR nominal / real | configurado / medido (401,8/s no BMI270, ≈ 372/s no ADXL362): tolerância do oscilador do sensor |
| data-ready | pino "amostra nova": nível (ADXL362, BMI270 *non-latched*), pulso ou *latched*; push-pull ativo alto |
| primeiro byte | endereço + R/W (+ auto-incremento): BMI270 `0x83`, ADXL362 `0x0B`, ADXL382 `0x23`; errata 8 só com MSB 1 |
| byte dummy | BMI270: `0x83` + dummy + `STATUS` (0x03) + 8 B auxiliares + 6 B XYZ (0x0C–0x11) = 17 B; ADXL362: `0x0B` + endereço + `STATUS` + `FIFO_ENTRIES` L/H (0x0C/0x0D, não usados) + 6 B XYZ = 11 B |
| leitura dummy, `adv_pwr_save`, *config file* | BMI270: a primeira leitura só troca I²C → SPI; o init desliga o modo de economia (1 ms entre escritas); blob `max_fifo` do Zephyr, 328 bytes em 11 blocos, 23 ms (M) |
| FIFO, *watermark* | buffer do sensor com interrupção por nível; não coberto |
| modo repetido | `HOLD_XFER` + `REPEATED_XFER` + `NO_XFER_EVT_HANDLER` (+ `RX_POSTINC`): a SPIM se repete a cada `START` |
| ZLI | zero-latency interrupt do Zephyr; não usada |
| mg/LSB | ±2 g: BMI270 16 384 LSB/g, ADXL362 1 mg/LSB; `decode()` dá m/s² |
| CPOL/CPHA | modo 0 nos dois (BMI270 aceita modo 3); a errata 8 depende de CPHA |
| `CSNDUR`, `RXDELAY`, `PRESCALER` | `IFTIMING` em ciclos do core da SPIM (16 MHz SPIM2x/30, 128 MHz SPIM00; 64 MHz nRF5340, D); `RXDELAY` = atraso do MISO; `PRESCALER` 16/2 = 8 MHz, 128/4 = 32 MHz |
| errata 8 | nRF54L: CPHA = 0, `PRESCALER > 2`, MSB 1 → MOSI errado |
| 1,5 µs | custo fixo por transação, inferido do teto (E) |
| modo drenado / por amostra | anel + thread a cada T / um buffer + IRQ de `END` (≤ 25 k/s) |
| T, T real | `APP_DRAIN_PERIOD_US` (10 ms; mín 100 µs); real = ⌈T/tick⌉ · tick + 1 tick + drenagem |
| drenagem, assentamento | leitura do head, entrega, wrap armado em anel/2; com ≤ 4 pendentes espera `XFER_SETTLE_US` |
| slot, anel, volta, guarda | espaço de uma rajada; `APP_RING_SLOTS` + 8; passagem do slot 0 ao wrap; os 8 slots extras |
| head, tail | próximo slot do EasyDMA (`DMA.RX.PTR`/`RXD.PTR`); próximo a entregar |
| array list | ponteiro avança um slot por transação (`RX_POSTINC`) |
| wrap, prazo | ponteiro ao slot 0 na ISR de `DMA.RX.READY`/`STARTED`; prazo = um período |
| espera acordada | `APP_WRAP_AWAKE_BELOW_US` (64 µs): thread espera o wrap, ≤ min(T/4, 8 períodos + 8 µs) |
| `late`, `ovf`, `torn` | wraps tardios (limite superior de perdas); voltas até a guarda; cópias atropeladas |
| falso limpo, módulo o período | contadores zerados com latência mínima abaixo da transação = ISR após o `START` seguinte; a captura lê o TIMER zerado a cada `COMPARE` (0,06 µs a 30 µs ≈ 30,06 µs) |
| janela | período do relatório; nas varreduras só a janela de acomodação de cada passo |
| `xfers`, `queued`, `dropped`, `skipped` | transações iniciadas; amostras pela fila; fila cheia; repetidas descartadas |
| `STARTED` / `DMA.RX.READY` | início de transação (nRF5340 / nRF54L, `RXSTARTED` na nrfx): ponteiro liberado |
| DPPI, GPPI, PPIB | evento publica num canal, tarefa assina; camada da nrfx; ponte entre domínios do nRF54L15 |
| MCU / PERI / LP | domínios do nRF54L15: SPIM00, TIMER00 / SPIM2x, TIMER2x, GPIOTE20 / SPIM30, GPIOTE30, GRTC (relógio do Zephyr) |
| RRAM standby | `APP_RRAM_STANDBY`: `RRAMC.POWER.LOWPOWERCONFIG.MODE` em vez do power-down (`tIDLE2CPU` 13 µs, D) |
| FLPR / VPR, `hfxo_launcher` | RISC-V do nRF54L15 em RAM / seu bloco; imagem do app core que o sobe e pede o HFXO |
| HFXO / HFINT, constant latency | cristal / RC (~0,2 % fora, M sem log); recursos ligados em idle, 0,55 mA (D), não corrige a latência |
| `ION_IDLE`*n*, `ITIMER`*n*, `ISPIM`*n*, `IAPPCPU`*n* | parâmetros "Current consumption" dos datasheets (*LowLatency* = idle com GPIOTE IN no nRF5340) |
| drive E0/E1, CSN | classes de corrente do nRF54L15 para a SPIM00 a 32 MHz; chip select da SPIM (`PSEL.CSN`) |
| TAG, PPK2, Board Configurator | nRF54L15 TAG; Power Profiler Kit II (não usado); ferramenta de pinos das DKs |

## Referências

- nRF5340 e nRF54L15 Product Specification (MCP da Nordic): SPIM (EasyDMA
  double-buffered, `DMA.RX.READY`, instâncias, AHB), EasyDMA array list,
  errata 8, `RRAMC.POWER.LOWPOWERCONFIG`, `tIDLE2CPU`, DPPI/PPIB, "Current
  consumption".
- nRF Connect SDK v3.4.1: nrfx 4.0 (`nrfx_gppi`, `nrfx_spim`, `nrfx_gpiote`,
  `nrfx_timer`), driver BMI270 (`max_fifo`), amostra `vpr_offloading` (R).
- Nordic Academy e DevZone: correntes de domínio e do VPR (R).
