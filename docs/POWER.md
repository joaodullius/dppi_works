# Consumo — modelo de corrente média (nRF54L15 e nRF5340)

**TL;DR: nada aqui foi medido com PPK2. É um modelo de corrente média do SoC
montado com os números que os datasheets publicam (D) e, onde eles não
publicam, com estimativas marcadas (E) ou relatos (R). Só o SoC: o sensor
não entra. A tabela que importa é a de
[T × instância × taxa](#nrf54l15-t--instância--taxa-caso-1-11-b).**

Marcação: **M** = medido em bancada (log em `*/test-logs/`), **E** =
estimado, **D** = datasheet (`ps_nrf54l15` e `ps_nrf5340`, tabelas "Current
consumption", típicos a 3 V, 25 °C, DC/DC, consultados pelo MCP da Nordic),
**R** = relato (Nordic Academy, DevZone), não é especificação.

Este modelo cobre o uso do repositório: taxa alta (a partir de ~1 k
amostras/s, indispensável acima de ~10 k/s), toda amostra na fila, o anel
drenado por uma thread a cada T. Abaixo de ~1 k/s um produto usa o
subsistema de sensores do Zephyr, e o consumo é outro assunto.

## Premissas (um conjunto só, usado em todas as tabelas)

| Termo | nRF54L15 | nRF5340 | Fonte |
|---|---|---|---|
| Base, System ON idle, RAM retida | 2,9 µA (`ION_IDLE8`) | 1,5 µA (`ION_IDLE7`) | D |
| Domínio mantido ligado pelo GPIOTE IN | PERI 20 µA (Academy mediu +17 µA numa DK, arredondado) | 48 µA (`ION_IDLE4`, GPIOTE IN event, LowLatency) | 54L: R; 5340: D |
| Domínio MCU ligado por um periférico dele (SPIM00) | 300 µA (`ITIMER0` TIMER00 450 µA − `ITIMER1` TIMER20 142 µA; a Academy mostra que o TIMER00 a 1 MHz custa quase o mesmo que a 128 MHz: o custo é o domínio) | — | E, proxy D + R |
| SPIM ativa | 0,25 mA (SPIM2x; proxy: TIMER20 a 16 MHz 142 µA, TWIM ~250 µA na Academy); 0,8 mA (SPIM00, razão TIMER00/TIMER20) | 1,7 mA a 8 Mbps (`ISPIM2`, HFINT); 1,9 mA a 16 Mbps (E, entre `ISPIM2` e `ISPIM4` 2,1 mA a 32 Mbps) | 54L: E; 5340: D |
| TIMER de disparo (caso 2) | 121 µA (`ITIMER2`, TIMER20 a 1 MHz) + HFXO 34 µA (`ISTBY_X32M_X2`) | 670 µA (`ITIMER1`, HFXO64M) + HFXO 135 µA | D |
| CPU ativa | 2,6 mA (`IAPPCPU0`, 128 MHz) | 3,3 mA (`IAPPCPU5`, 64 MHz, HFINT) | D |
| CPU por drenagem | 5 µs para acordar a thread e ler o head + 3 µs da IRQ de wrap; mais um período do sensor de espera acordada quando chegam ≥ 32 amostras por drenagem (a thread espera pelo wrap sem dormir, no máximo T/4) | idem | E, ordem de grandeza de bancada |
| CPU por amostra | 1 µs de `k_msgq_put` na drenagem + 2,5 µs de `k_msgq_get` e decode no consumidor = 3,5 µs | idem | E |
| CPU acordada | 2,6 mA × fração do tempo ocupado (o modelo não usa 2,6 mA contínuos) | 3,3 mA × fração | E |
| Wake-up da RRAM (13 µs, `tIDLE2CPU`) | não contado: os acordares da thread são agendados e o Zephyr acorda a RRAM antes (`NRF_SYS_EVENT_IRQ_LATENCY`); na IRQ de wrap vinda de idle o core fica parado esperando a RRAM, não ativo | — | E; `tIDLE2CPU` D |
| Corrente ativa do FLPR | não publicada | — | — |
| RRAM em standby, custo de idle | não publicado; não é mais necessária para o wrap | — | — |
| Constant latency em idle | 0,55 mA (`ION_IDLE11`); medido, não corrige a latência sozinha | — | D, M |
| FLPR | sem número: a amostra `vpr_offloading` da Nordic mediu 146 → 125 µA no nRF54L15 DK (app core 3,0 % de CPU contra FLPR 0,1 %, uma transferência SPI por ms) porque o FLPR roda da RAM e não acorda a RRAM; um relato de DevZone dá ≈ +0,5 mA de idle do VPR noutra configuração, confirmado por um engenheiro da Nordic como "corrente de sono padrão do VPR". As duas fontes conflitam para este uso | — | R (NCS docs, DevZone) |
| Transação | t = bytes × 8 / SCK + 1,5 µs: 11 B a 8 MHz 12,5 µs; 17 B a 8 MHz 18,5 µs; 11 B a 16 MHz 7,0 µs; 11 B a 32 MHz 4,25 µs | idem | E, coerente com M (17 B → 18,5 µs medido) |

PERI 20 µA (R) é a premissa de menor confiança; ela desloca todas as
linhas do nRF54L15 por igual e não muda nenhuma comparação.

Modelo: `I = base + domínio(s) ligado(s) + SPIM ativa × (taxa × t) + TIMER
de disparo + HFXO + CPU ativa × [(1/T) × (8 µs + espera) + taxa × 3,5 µs]`.

- Caso 1 (INT): sem TIMER de disparo, sem HFXO. Caso 2 (TIMER): + TIMER de
  disparo + HFXO, e o GPIOTE não é usado.
- Espera = um período do sensor quando taxa × T ≥ 32 (a drenagem espera
  acordada pelo wrap), zero abaixo disso.
- Não há mais contador TIMER (121 µA no nRF54L15, 475 µA no nRF5340) nem
  EGU: o número de transações vem do ponteiro do EasyDMA.
- Rajada de 17 bytes em vez de 11: somar 0,25 mA × taxa × 6 µs na SPIM22 (8
  MHz) ou 0,8 mA × taxa × 1,5 µs na SPIM00 (32 MHz), só onde o barramento
  ainda cabe (17 B a 50 k/s na SPIM22 são 92 % de ocupação, fora do
  critério de sobra).

## nRF54L15: T × instância × taxa (caso 1, 11 B)

![Consumo por T, instância e taxa](consumo_modos_nrf54l15.svg)

*Corrente média do SoC contra amostras/s para o T padrão (10 ms; 1 ms a
16 k e 50 k/s) e para o T mais curto (o período do sensor a 1 600/s; o
mínimo de 100 µs acima), SPIM22 e SPIM00 (E).*

Caso 1 (data-ready), rajada de 11 bytes, toda amostra na fila, Cortex-M33
padrão. SPIM22 a 8 MHz (12,5 µs por transação), SPIM00 a 32 MHz (4,25 µs).
Corrente média do SoC em µA (E).

| T | Instância (SCK) | 1 600/s | 16 k/s | 50 k/s |
|---|---|---|---|---|
| 10 ms (1 600/s), 1 ms (16 k e 50 k/s) | SPIM22 (8 MHz) | **45** | **239** | **707** |
| 10 ms, 1 ms | SPIM00 (32 MHz) | 345 | 544 | 1 021 |
| período (625 µs), 100 µs (mínimo) | SPIM22 | 76 | 427 | 842 |
| período, 100 µs | SPIM00 | 376 | 731 | 1 156 |
| qualquer, FLPR | SPIM22 | sem número (E) | sem número (E) | sem número (E) |

Termos de cada linha:

- **Fixo** = 2,9 + PERI 20 + domínio MCU 300 só na SPIM00.
- **Barramento** = SPIM ativa × (taxa × t): a 1 600/s, 5 µA (SPIM22, 2 %)
  ou 5,4 µA (SPIM00, 0,7 %); a 16 k/s, 50 µA (20 %) ou 54 µA (6,8 %); a
  50 k/s, 156 µA (62,5 %) ou 170 µA (21 %).
- **CPU, T longo** = 2,6 mA × fração: a 1 600/s e T = 10 ms, 100 drenagens
  × 8 µs + 1 600 × 3,5 µs = 6 400 µs/s = 0,64 % → 17 µA; a 16 k/s e T =
  1 ms, 8 000 + 56 000 = 6,4 % → 166 µA; a 50 k/s e T = 1 ms, 50 amostras
  por drenagem ≥ 32, então 1 000 × (8 + 20 de espera) + 175 000 = 20,3 % →
  528 µA.
- **CPU, T curto** = a 1 600/s e T = 625 µs, 1 600 × 8 + 5 600 = 1,84 % →
  48 µA; a 16 k/s e T = 100 µs, 10 000 × 8 + 56 000 = 13,6 % → 354 µA; a
  50 k/s e T = 100 µs, 80 000 + 175 000 = 25,5 % → 663 µA.
- **Por que T = 1 ms a 16 k e 50 k/s**: com T = 10 ms chegariam 160 e 500
  amostras por drenagem, e o anel precisaria de 320 e 1 000 slots (2×);
  com T = 1 ms são 16 e 50, e o anel padrão de 256 serve. É também o T da
  bancada (`bench/*.conf`).
- **Por que 100 µs e não o período**: `APP_DRAIN_PERIOD_US` vai de 100 µs
  a 1 s. A 16 k/s isso é 1,6 amostras de latência; a 50 k/s, 5 amostras,
  abaixo do limiar de 32 da espera acordada: a IRQ de wrap viria de idle
  (17 µs) contra um período de 20 µs, margem não medida. Para 50 k/s use
  T ≥ 1 ms. Se o Kconfig deixasse T = 62,5 µs a 16 k/s, o modelo daria
  551 µA na SPIM22 (E).
- **FLPR**: sem fórmula. Roda da RAM; o custo de idle do bloco VPR vai de
  "economia" (`vpr_offloading`, R) a +0,5 mA (DevZone, R) conforme a
  configuração, e a corrente ativa do FLPR não é publicada. Só o PPK2
  decide.

Três leituras:

1. **T curto custa 1,7× (1,6 k/s), 1,8× (16 k/s) e 1,2× (50 k/s) o T
   longo** (76 contra 45 µA; 427 contra 239; 842 contra 707). A diferença
   é só o número de drenagens, ~8 µs de CPU cada; o custo por amostra é o
   mesmo. A 50 k/s a espera acordada do T = 1 ms (20 µs por drenagem, 2 %
   de CPU) aproxima as duas linhas.
2. **A SPIM00 custa ~300 µA a mais em qualquer taxa** (o domínio MCU
   ligado). Só paga pela sobra de barramento: SCK acima de 8 MHz, rajada
   longa a 64 k/s, ou taxa acima de ≈ 71–80 k/s (E). Nunca por consumo.
3. **Sem contador nem EGU, o custo fixo é o domínio PERI.** A 1 600/s são
   45 µA: 20 de PERI (R), 17 de CPU, 5 de SPIM, 2,9 de base. Uma variante
   100 % LP (GPIOTE30 + SPIM30, não testada) tiraria os 20 µA do PERI (E).

Regra que sai da tabela: data-ready + T = 10 ms na SPIM22, salvo se a
latência de uma amostra for requisito (então T ≈ período, até 10 k/s) ou o
barramento não couber a 8 MHz (então SPIM00). Próximo passo: medir com
PPK2 FLPR contra M33 no caso 1.

## nRF5340 (Thingy:53, SPIM4, caso 1, 11 B)

Termos: base 1,5 + GPIOTE 48 + SPIM × (taxa × t) + CPU 3,3 mA × fração.
Sem HFXO no caso 1, sem contador (E).

| Taxa | T longo | T curto | Observação |
|---|---|---|---|
| 1 600/s, 8 MHz | ≈ 0,10 mA (T = 10 ms) | ≈ 0,14 mA (T = 625 µs) | SPIM 1,7 mA × 2 % = 34 µA; CPU 21 ou 61 µA; o GPIOTE (48 µA, D) é o maior termo fixo |
| 64 k/s, 8 MHz | ≈ 2,2 mA (T = 1 ms) | ≈ 2,4 mA (T = 100 µs) | SPIM 1,7 mA × 80 % = 1,36 mA; CPU 1 000 × (8 + 15,6 de espera) + 64 000 × 3,5 = 24,8 % → 0,82 mA; com T = 100 µs, 30,4 % → 1,0 mA |
| 64 k/s, 16 MHz | ≈ 1,7 mA (T = 1 ms) | ≈ 1,9 mA (T = 100 µs) | SPIM 1,9 mA × 45 % = 0,85 mA |

Os testes da Thingy a 400 Hz rodaram a 4 MHz (default do Kconfig; transação
de 23,5 µs); o bench de 64 k rodou a 8 MHz (`bench/bus-64k-thingy.conf`).

Leituras: no nRF5340 a SPIM ativa custa 7× a do nRF54L15 (1,7 mA contra
~0,25 mA) e a CPU 3,3 contra 2,6 mA; a 64 k/s com T = 1 ms o nRF5340 gasta
2,0 a 2,6× o nRF54L15 (1,7–2,2 mA contra 0,87 mA), e a 1 600/s 2,3× (0,10
contra 0,045 mA), porque ali o GPIOTE de 48 µA domina.

## Caso 2 (TIMER) no nRF54L15

SPIM22, 17 B (18,5 µs por transação, BMI270), TIMER de disparo 121 µA + HFXO
34 µA em vez do GPIOTE (o TIMER em PERI já mantém o domínio ligado). CPU com
filtro: 0,5 µs por transação (checar o bit na drenagem) + 3,5 µs por amostra
nova (put, get, decode); sem filtro, 3,5 µs por transação; mais 8 µs por
drenagem (E).

| Taxa do timer | T | Com filtro | Sem filtro | Termos |
|---|---|---|---|---|
| 408/s (sensor a 402/s) | 10 ms | ≈ 166 µA | ≈ 166 µA | 2,9 + 155 + 0,25 mA × 0,75 % + ≈ 6 de CPU |
| 10 k/s (sensor a 400 Hz) | 1 ms | ≈ 242 µA | ≈ 316 µA | 2,9 + 155 + 0,25 mA × 18,5 % + 37 ou 112 de CPU |
| 50 k/s (20 µs; o teto é 52,6 k/s a 19 µs) | 1 ms | ≈ 530 µA | ≈ 0,92 mA | 2,9 + 155 + 0,25 mA × 92 % + 141 ou 528 de CPU (a espera acordada de 20 µs por drenagem incluída) |

O TIMER de disparo mais o HFXO custam 155 µA fixos, contra 20 µA do
GPIOTE: é a razão de o caso 1 ser o recomendado quando o pino existe. No
FLPR o custo de idle do VPR não tem número (ver premissas).

## ADXL382 a 64 k amostras/s (caso 1, 11 B, não testado)

| SoC / instância | T = 1 ms | T = 100 µs (mínimo) | Termos do T = 1 ms |
|---|---|---|---|
| nRF5340 SPIM4, 8 MHz | ≈ 2,2 mA | ≈ 2,4 mA | 1,5 + 48 + 1,7 mA × 80 % + 3,3 mA × 24,8 % |
| nRF5340 SPIM4, 16 MHz | ≈ 1,7 mA | ≈ 1,9 mA | idem com 1,9 mA × 45 % |
| nRF54L15 SPIM22, 8 MHz | ≈ 0,87 mA | ≈ 1,01 mA | 2,9 + 20 + 0,25 mA × 80 % + 2,6 mA × 24,8 % (644 µA) |
| nRF54L15 SPIM00, 32 MHz | ≈ 1,18 mA | ≈ 1,33 mA | 2,9 + 20 + 300 + 0,8 mA × 27 % + 644 |

A 64 k/s com T = 1 ms chegam 64 amostras por drenagem: a thread espera
acordada pelo wrap (um período, 15,6 µs, por drenagem = 1,6 % de CPU) e o
custo de CPU é 24,8 % do tempo, dominado pelos 3,5 µs por amostra (22,4 %).
Com T = 100 µs (6,4 amostras de latência) as drenagens custam 8 % e a
espera acordada não entra (6 amostras por drenagem < 32; período de
15,6 µs contra wake-up de 17 µs no M33: não use T = 100 µs a 64 k/s no
nRF54L15, ou baixe `SPIN_WRAP_MIN_ARRIVED`). No nRF54L15 a SPIM2x atende
64 k/s com 11 B a 80 % do barramento, sem margem; a SPIM00 a 27 % dá
margem. A errata 8 não se aplica ao ADXL382 (primeiro byte 0x23, bit mais
significativo 0). O ADXL382 em si (não incluído) consome na casa de 1 mA
em alto desempenho, conferir no datasheet do sensor.

## Quando a SPIM00 faz sentido

1. Rajadas maiores que 11 bytes a 64 k/s (STATUS + XYZ + temperatura, ou
   FIFO): com 8 MHz o teto é 52,6 k/s para 17 bytes (M); a 32 MHz cabem até
   ~45 bytes por transação a 64 k/s (E).
2. Taxas acima do que 8 MHz alcança com rajada curta (≈ 71–80 k/s para
   11 B, E, não medido no nRF54L; 1/t é otimista em até 10 %).
3. Sensor que exige SCK > 8 MHz, ou `CSNDUR` maior sem perder taxa.
4. Sobra de barramento: 27 % contra 80 % de ocupação a 64 k/s, para
   absorver jitter do ODR ou um segundo dispositivo no barramento.
5. Domínio MCU já ligado por outro motivo (CPU ativa o tempo todo, flash
   externa na SPIM00, constant latency): os 300 µA já estão pagos.

Contra: pinos dedicados do P2 com drive E0/E1, errata 8 quando o primeiro
byte do comando tem o bit mais significativo em 1 (CPHA = 1 ou trocar o
comando; não é o caso do ADXL382; o BMI270 aceita modo 3, não testado) e
disparo em PERI atravessando o PPIB (latência não especificada). O wrap
não muda de uma instância para outra.

## O que reduziria em produto

1. Disparo por INT quando o pino existe: sem HFXO, sem TIMER de disparo, sem
   repetidas.
2. T = 10 ms sempre que a latência de 10 ms for aceitável; T ≈ período do
   sensor só quando a latência de uma amostra for requisito (custa 1,2 a
   1,8×).
3. Medir com PPK2 o que o modelo assume: custo do domínio PERI mantido pelo
   GPIOTE IN, corrente da SPIM do nRF54L15, custo real de uma drenagem, e
   FLPR contra M33 no caso 1.
4. Filtro de repetidas na drenagem sempre que o timer for mais rápido que
   o ODR.
5. Variante 100 % LP (GPIOTE30 + SPIM30 no nRF54L15 DK): tira os 20 µA do
   PERI; não testada.
6. O limite de T é RAM (anel de (slots + 8) rajadas mais a fila: a 64 kHz
   com T = 1 ms, anel de 256 e fila de 256, 2,9 KB + 2,8 KB) e a latência
   de entrega (T).
