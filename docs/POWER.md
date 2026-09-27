# Consumo — modelo de corrente média (nRF54L15 e nRF5340)

**TL;DR: nada aqui foi medido com PPK2. É um modelo de corrente média do SoC
montado com os números que os datasheets publicam (D) e, onde eles não
publicam, com estimativas marcadas (E) ou relatos (R). Só o SoC: o sensor
não entra. A tabela que importa é a de
[entrega × instância × taxa](#nrf54l15-entrega--instância--taxa-caso-1-11-b).**

Marcação: **M** = medido em bancada (log em `*/test-logs/`), **E** =
estimado, **D** = datasheet (`ps_nrf54l15` e `ps_nrf5340`, tabelas "Current
consumption", típicos a 3 V, 25 °C, DC/DC, consultados pelo MCP da Nordic),
**R** = relato (Nordic Academy, DevZone), não é especificação.

Este modelo cobre o uso do repositório: taxa alta (a partir de ~1 k
amostras/s, indispensável acima de ~10 k/s), toda amostra na fila, entregue
por drenagem do anel a cada T (padrão) ou por uma interrupção `END` por
amostra (`APP_PER_SAMPLE_IRQ`). Abaixo de ~1 k/s um produto usa o
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
| Acordar de idle | 17 µs com o core parado esperando a RRAM (power-down em idle, `tIDLE2CPU` 13 µs, D; medido 14,4–15,4 µs de média e 16,25 de máximo do disparo à ISR, `u_tag_wrap_latency.log`, M), contados como CPU ativa: clocks ligados, sem executar | não modelado (sem RRAM; a latência de 14–25 µs é de clock e regulador, com o core em baixa corrente) | E |
| CPU por drenagem | 17 µs de acordar + 5 µs de trabalho (ler o head, entregar, armar) = 22 µs | 8 µs | E, ordem de grandeza de bancada |
| CPU por wrap (uma vez por volta do anel, ≥ anel/2 amostras) | 17 + 3 µs se a IRQ vem de idle (período ≥ 64 µs); um período + 3 µs se a thread espera acordada (período < 64 µs, `APP_WRAP_AWAKE_BELOW_US`) | 3 µs, ou um período + 3 µs | E |
| CPU por amostra, modo drenado | 1 µs de `k_msgq_put` na drenagem + 2,5 µs de `k_msgq_get` e decode no consumidor = 3,5 µs | idem | E |
| CPU por amostra, modo por amostra | 17 µs de acordar (o core dorme entre amostras em todas as taxas em que o modo vale; medido até 34,75 µs de máximo a 50 µs de período, M) + 3 µs de ISR e cópia + 2,5 µs de consumidor = 22,5 µs | 3 + 2,5 = 5,5 µs | E |
| CPU acordada | 2,6 mA × fração do tempo ocupado (o modelo não usa 2,6 mA contínuos) | 3,3 mA × fração | E |
| Corrente ativa do FLPR | não publicada | — | — |
| RRAM em standby, custo de idle | não publicado; tiraria os 17 µs de acordar de cada drenagem e de cada ISR | — | — |
| Constant latency em idle | 0,55 mA (`ION_IDLE11`); medido com o mecanismo anterior, não corrige a latência sozinha | — | D, M |
| FLPR | sem número: a amostra `vpr_offloading` da Nordic mediu 146 → 125 µA no nRF54L15 DK (app core 3,0 % de CPU contra FLPR 0,1 %, uma transferência SPI por ms) porque o FLPR roda da RAM e não acorda a RRAM; um relato de DevZone dá ≈ +0,5 mA de idle do VPR noutra configuração, confirmado por um engenheiro da Nordic como "corrente de sono padrão do VPR". As duas fontes conflitam para este uso | — | R (NCS docs, DevZone) |
| Transação | t = bytes × 8 / SCK + 1,5 µs: 11 B a 8 MHz 12,5 µs; 17 B a 8 MHz 18,5 µs; 11 B a 16 MHz 7,0 µs; 11 B a 32 MHz 4,25 µs; 17 B a 32 MHz 5,75 µs | idem | E; o 1,5 µs é inferido do teto medido (19 µs passa, 18 falha, com 17 µs de bits) |

PERI 20 µA (R) é a premissa de menor confiança; ela desloca todas as
linhas do nRF54L15 por igual e não muda nenhuma comparação. A segunda é o
custo de acordar (17 µs contados como CPU ativa): ele domina o custo por
drenagem e por interrupção, e é o que o PPK2 mais precisa confirmar.

Modelo: `I = base + domínio(s) ligado(s) + SPIM ativa × (taxa × t) + TIMER
de disparo + HFXO + CPU ativa × fração`, com

- fração (modo drenado) = (1/T) × 22 µs + wraps/s × (20 µs ou período +
  3 µs) + taxa × 3,5 µs;
- fração (modo por amostra) = taxa × 22,5 µs;
- wraps/s = taxa / comprimento da volta; a volta é a primeira drenagem em
  que o head passa de anel/2, ou seja, ⌈(anel/2) / (amostras por drenagem)⌉
  × amostras por drenagem, com o anel padrão de 256 slots.
- Caso 1 (INT): sem TIMER de disparo, sem HFXO. Caso 2 (TIMER): + TIMER de
  disparo + HFXO, e o GPIOTE não é usado.
- Rajada de 17 bytes em vez de 11: somar 0,25 mA × taxa × 6 µs na SPIM22 (8
  MHz) ou 0,8 mA × taxa × 1,5 µs na SPIM00 (32 MHz), só onde o barramento
  ainda cabe (17 B a 50 k/s na SPIM22 são 92 % de ocupação, fora do
  critério de sobra).

## nRF54L15: entrega × instância × taxa (caso 1, 11 B)

![Consumo por modo de entrega, instância e taxa](consumo_modos_nrf54l15.svg)

*Corrente média do SoC contra amostras/s para o T padrão (10 ms; 1 ms a
16 k e 50 k/s), para o T mais curto (o período do sensor a 1 600/s; o
mínimo de 100 µs acima) e para o modo por amostra, SPIM22 e SPIM00 (E).*

Caso 1 (data-ready), rajada de 11 bytes, toda amostra na fila, Cortex-M33
padrão. SPIM22 a 8 MHz (12,5 µs por transação), SPIM00 a 32 MHz (4,25 µs).
Corrente média do SoC em µA (E).

| Entrega | Instância (SCK) | 1 600/s | 16 k/s | 50 k/s |
|---|---|---|---|---|
| drenado, T = 10 ms (1 600/s) e 1 ms (16 k e 50 k/s) | SPIM22 (8 MHz) | **49** | **297** | **711** |
| drenado, T = 10 ms, 1 ms | SPIM00 (32 MHz) | 349 | 601 | 1 025 |
| drenado, T = período (625 µs) e 100 µs (mínimo) | SPIM22 | 135 | 812 | 1 229 |
| drenado, T = período, 100 µs | SPIM00 | 435 | 1 116 | 1 543 |
| por amostra (`APP_PER_SAMPLE_IRQ`) | SPIM22 | 122 | 1 009 | fora da faixa |
| por amostra | SPIM00 | 422 | 1 313 | fora da faixa |
| qualquer, FLPR | SPIM22 | sem número (E) | sem número (E) | sem número (E) |

Termos de cada linha:

- **Fixo** = 2,9 + PERI 20 + domínio MCU 300 só na SPIM00.
- **Barramento** = SPIM ativa × (taxa × t): a 1 600/s, 5 µA (SPIM22, 2 %)
  ou 5,4 µA (SPIM00, 0,7 %); a 16 k/s, 50 µA (20 %) ou 54 µA (6,8 %); a
  50 k/s, 156 µA (62,5 %) ou 170 µA (21 %).
- **Wraps por segundo** (anel 256): a 1 600/s e T = 10 ms, 16 por
  drenagem, volta de 128, 12,5 wraps/s pela IRQ de idle (período 625 µs ≥
  64), 20 µs cada; a 16 k/s e T = 1 ms, 16 por drenagem, volta de 128,
  125 wraps/s com espera acordada (período 62,5 µs < 64), 65,5 µs cada;
  a 50 k/s e T = 1 ms, 50 por drenagem, volta de 150, 333 wraps/s, 23 µs
  cada. Com T = 100 µs: 12,5, 125 e 385 wraps/s.
- **CPU, T longo** = 2,6 mA × fração: a 1 600/s e T = 10 ms, 100
  drenagens × 22 µs + 12,5 × 20 + 1 600 × 3,5 = 8 050 µs/s = 0,81 % →
  21 µA; a 16 k/s e T = 1 ms, 22 000 + 8 188 + 56 000 = 8,6 % → 224 µA; a
  50 k/s e T = 1 ms, 22 000 + 7 667 + 175 000 = 20,5 % → 532 µA.
- **CPU, T curto** = a 1 600/s e T = 625 µs, 1 600 × 22 + 250 + 5 600 =
  4,1 % → 107 µA; a 16 k/s e T = 100 µs, 220 000 + 8 188 + 56 000 =
  28,4 % → 739 µA; a 50 k/s e T = 100 µs, 220 000 + 8 846 + 175 000 =
  40,4 % → 1 050 µA. As 10 000 drenagens/s custam 17 % de CPU só em RRAM
  acordando.
- **CPU, por amostra** = 22,5 µs × taxa: a 1 600/s, 3,6 % → 94 µA; a
  16 k/s, 36 % → 936 µA. A 50 k/s o modo não vale: 20 µs de período
  contra 12,5 de transação mais 17 de acordar (medido na TAG com 17 B:
  limpo até 25 µs, `torn` a 20 µs).
- **Por que T = 1 ms a 16 k e 50 k/s**: com T = 10 ms chegariam 160 e 500
  amostras por drenagem, e o anel precisaria de 320 e 1 000 slots (2×);
  com T = 1 ms são 16 e 50, e o anel padrão de 256 serve. É também o T da
  bancada (`bench/*.conf`).
- **Por que 100 µs e não o período**: `APP_DRAIN_PERIOD_US` vai de 100 µs
  a 1 s. A 16 k/s isso é 1,6 amostras de latência; a 50 k/s, 5 amostras.
  A espera acordada cobre os dois (período < 64 µs). Se o Kconfig
  deixasse T = 62,5 µs a 16 k/s, o modelo daria 1 155 µA na SPIM22 (E):
  a essa altura o modo por amostra (1 009 µA) é o caminho.
- **FLPR**: sem fórmula. Roda da RAM; o custo de idle do bloco VPR vai de
  "economia" (`vpr_offloading`, R) a +0,5 mA (DevZone, R) conforme a
  configuração, e a corrente ativa do FLPR não é publicada. Só o PPK2
  decide.

Três leituras:

1. **T curto custa 2,8× (1,6 k/s), 2,7× (16 k/s) e 1,7× (50 k/s) o T
   longo** (135 contra 49 µA; 812 contra 297; 1 229 contra 711). A
   diferença é só o número de drenagens, ~22 µs de CPU cada, 17 dos quais
   são a RRAM acordando; o custo por amostra é o mesmo. O modo por amostra
   fica entre os dois a 1 600/s (122 µA: uma interrupção custa menos que
   uma drenagem mais uma amostra) e acima dos dois a 16 k/s (1 009 µA).
2. **A SPIM00 custa ~300 µA a mais em qualquer taxa** (o domínio MCU
   ligado). Só paga pela sobra de barramento: SCK acima de 8 MHz, rajada
   longa a 64 k/s, ou taxa acima de ≈ 71–80 k/s (E). Nunca por consumo.
3. **O custo fixo é o domínio PERI.** A 1 600/s são 49 µA: 20 de PERI (R),
   21 de CPU, 5 de SPIM, 2,9 de base. Uma variante 100 % LP (GPIOTE30 +
   SPIM30, não testada) tiraria os 20 µA do PERI (E). RRAM em standby
   tiraria 17 µs de cada drenagem e de cada ISR (a 1 600/s e T = 10 ms,
   4,4 µA; por amostra, 71 µA) ao custo de uma corrente de idle não
   publicada.

Regra que sai da tabela: data-ready + modo drenado com T = 10 ms na
SPIM22, salvo se a latência de uma amostra for requisito (então o modo por
amostra até ~5 k/s, ou T ≈ período até 10 k/s) ou o barramento não couber
a 8 MHz (então SPIM00). Próximo passo: medir com PPK2 FLPR contra M33 no
caso 1, e o custo real de uma drenagem.

## nRF5340 (Thingy:53, SPIM4, caso 1, 11 B)

Termos: base 1,5 + GPIOTE 48 + SPIM × (taxa × t) + CPU 3,3 mA × fração,
com 8 µs por drenagem, 3 µs por wrap (um período + 3 na espera acordada),
3,5 µs por amostra, 5,5 µs por amostra no modo por amostra; o acordar não
é modelado (E). Sem HFXO no caso 1.

| Taxa | Drenado, T longo | Drenado, T curto | Por amostra | Observação |
|---|---|---|---|---|
| 1 600/s, 8 MHz | ≈ 0,10 mA (T = 10 ms) | ≈ 0,14 mA (T = 625 µs) | ≈ 0,11 mA | SPIM 1,7 mA × 2 % = 34 µA; CPU 21, 61 ou 29 µA; o GPIOTE (48 µA, D) é o maior termo fixo |
| 64 k/s, 8 MHz | ≈ 2,2 mA (T = 1 ms) | ≈ 2,4 mA (T = 100 µs) | fora da faixa | SPIM 1,7 mA × 80 % = 1,36 mA; CPU 1 000 × 8 + 500 × 18,6 + 64 000 × 3,5 = 24,1 % → 0,80 mA; com T = 100 µs, 31,3 % → 1,03 mA |
| 64 k/s, 16 MHz | ≈ 1,7 mA (T = 1 ms) | ≈ 1,9 mA (T = 100 µs) | fora da faixa | SPIM 1,9 mA × 45 % = 0,85 mA |

Os testes da Thingy a 400 Hz rodaram a 4 MHz (default do Kconfig; transação
de 23,5 µs); os benches de 64 k e por amostra rodaram a 8 MHz
(`bench/bus-64k-thingy.conf`, `bench/per-sample-thingy.conf`). O modo por
amostra no nRF5340 foi medido limpo até 10 k/s e com perdas a 20 k/s
(M): o wake-up de 14–25 µs mais a transação de 12,5 µs não cabem em 50 µs
com folga.

Leituras: no nRF5340 a SPIM ativa custa 7× a do nRF54L15 (1,7 mA contra
~0,25 mA) e a CPU 3,3 contra 2,6 mA; a 64 k/s com T = 1 ms o nRF5340 gasta
1,9 a 2,5× o nRF54L15 (1,7–2,2 mA contra 0,89 mA), e a 1 600/s 2,1× (0,10
contra 0,049 mA), porque ali o GPIOTE de 48 µA domina.

## Caso 2 (TIMER) no nRF54L15

SPIM22, 17 B (18,5 µs por transação, BMI270), TIMER de disparo 121 µA + HFXO
34 µA em vez do GPIOTE (o TIMER em PERI já mantém o domínio ligado). CPU com
filtro: 0,5 µs por transação (checar o bit na drenagem) + 3,5 µs por amostra
nova (put, get, decode); sem filtro, 3,5 µs por transação; mais 22 µs por
drenagem e os wraps (E).

| Taxa do timer | T | Com filtro | Sem filtro | Termos |
|---|---|---|---|---|
| 408/s (sensor a 402/s) | 10 ms | ≈ 170 µA | ≈ 170 µA | 2,9 + 155 + 0,25 mA × 0,75 % + ≈ 10 de CPU (0,39 %) |
| 10 k/s (sensor a 400 Hz) | 1 ms | ≈ 282 µA | ≈ 356 µA | 2,9 + 155 + 0,25 mA × 18,5 % (46) + 78 ou 152 de CPU (3,0 ou 5,9 %; 77 wraps/s pela IRQ de idle) |
| 50 k/s (20 µs; o teto é 52,6 k/s a 19 µs) | 1 ms | ≈ 535 µA | ≈ 0,92 mA | 2,9 + 155 + 0,25 mA × 92 % (231) + 146 ou 532 de CPU (5,6 ou 20,5 %; 333 wraps/s com espera acordada de 20 µs) |

O TIMER de disparo mais o HFXO custam 155 µA fixos, contra 20 µA do
GPIOTE: é a razão de o caso 1 ser o recomendado quando o pino existe. No
FLPR o custo de idle do VPR não tem número (ver premissas).

## ADXL382 a 64 k amostras/s (caso 1, 11 B, não testado)

| SoC / instância | T = 1 ms | T = 100 µs (mínimo) | Termos do T = 1 ms |
|---|---|---|---|
| nRF5340 SPIM4, 8 MHz | ≈ 2,2 mA | ≈ 2,4 mA | 1,5 + 48 + 1,7 mA × 80 % + 3,3 mA × 24,1 % |
| nRF5340 SPIM4, 16 MHz | ≈ 1,7 mA | ≈ 1,9 mA | idem com 1,9 mA × 45 % |
| nRF54L15 SPIM22, 8 MHz | ≈ 0,89 mA | ≈ 1,40 mA | 2,9 + 20 + 0,25 mA × 80 % + 2,6 mA × 25,5 % (664 µA) |
| nRF54L15 SPIM00, 32 MHz | ≈ 1,20 mA | ≈ 1,72 mA | 2,9 + 20 + 300 + 0,8 mA × 27 % + 664 |

A 64 k/s com T = 1 ms chegam 64 amostras por drenagem: 1 000 drenagens ×
22 µs, 500 wraps/s com espera acordada (um período, 15,6 µs, + 3) e
64 000 × 3,5 µs dão 25,5 % de CPU no nRF54L15, dominados pelos 3,5 µs por
amostra (22,4 %). Com T = 100 µs (6,4 amostras de latência) as 10 000
drenagens custam 22 % sozinhas: 45,3 % de CPU, 1 179 µA. O modo por
amostra não serve (15,6 µs de período contra 12,5 de transação mais o
wake-up). No nRF54L15 a SPIM2x atende 64 k/s com 11 B a 80 % do
barramento, sem margem; a SPIM00 a 27 % dá margem. A errata 8 não se
aplica ao ADXL382 (primeiro byte 0x23, bit mais significativo 0). O
ADXL382 em si (não incluído) consome na casa de 1 mA em alto desempenho,
conferir no datasheet do sensor.

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
2. T = 10 ms sempre que a latência de 10 ms for aceitável; o modo por
   amostra (até ~5 k/s) ou T ≈ período do sensor só quando a latência de
   uma amostra for requisito (custa 2,5 a 2,8×).
3. Medir com PPK2 o que o modelo assume: custo do domínio PERI mantido pelo
   GPIOTE IN, corrente da SPIM do nRF54L15, custo real de acordar (os
   17 µs de RRAM) e de uma drenagem, e FLPR contra M33 no caso 1.
4. Filtro de repetidas na drenagem sempre que o timer for mais rápido que
   o ODR.
5. Variante 100 % LP (GPIOTE30 + SPIM30 no nRF54L15 DK): tira os 20 µA do
   PERI; não testada.
6. RRAM em standby (`APP_RRAM_STANDBY`) se a corrente de idle que ela
   custa (não publicada) for menor que os 17 µs × 2,6 mA por acordar que
   ela economiza: a 1 600/s e T = 10 ms são 4,4 µA, pouco; no modo por
   amostra a 1 600/s, 71 µA, muito.
7. O limite de T é RAM (anel de (slots + 8) rajadas mais a fila: a 64 kHz
   com T = 1 ms, anel de 256 e fila de 256, 2,9 KB + 2,8 KB) e a latência
   de entrega (T mais o tick do kernel mais a drenagem).
