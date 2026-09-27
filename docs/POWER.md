# Consumo — modelo de corrente média (nRF54L15 e nRF5340)

**TL;DR: nada aqui foi medido com PPK2. É um modelo de corrente média do SoC
montado com os números que os datasheets publicam (D) e, onde eles não
publicam, com estimativas marcadas (E). Só o SoC: o sensor não entra.**

Marcação: **M** = medido em bancada, **E** = estimado, **D** = datasheet
(fonte: MCP da Nordic, `ps_nrf54l15` e `ps_nrf5340`, tabelas "Current
consumption", típicos a 3 V, 25 °C, DC/DC).

## Premissas (um conjunto só, usado em todas as tabelas)

| Termo | nRF54L15 | nRF5340 | Fonte |
|---|---|---|---|
| Base, System ON idle, RAM retida | 2,9 µA (`ION_IDLE8`) | 1,5 µA (`ION_IDLE7`) | D |
| Domínio mantido ligado pelo GPIOTE IN | PERI 20 µA (Academy mediu +17 µA numa DK); LP 5 µA | 48 µA (`ION_IDLE4`, GPIOTE IN event, LowLatency) | PERI: Academy; LP: E; 5340: D |
| Domínio MCU ligado por um periférico dele (SPIM00) | 300 µA (`ITIMER0` TIMER00 450 µA − `ITIMER1` TIMER20 142 µA; a Academy mostra que o TIMER00 a 1 MHz já custa quase o mesmo que a 128 MHz, o custo é o domínio) | — | E, proxy D |
| SPIM ativa | 0,25 mA (SPIM2x/30, proxy: TIMER20 a 16 MHz 142 µA, TWIM ~250 µA na Academy); 0,8 mA (SPIM00, razão TIMER00/TIMER20) | 1,7 mA a 8 Mbps (`ISPIM2`); 1,9 mA a 16 Mbps (E, interpolado entre `ISPIM2` e `ISPIM4` 2,1 mA a 32 Mbps) | 54L: E; 5340: D |
| TIMER contador (1 MHz) | 121 µA (`ITIMER2`) | 670 µA (`ITIMER1`, HFXO64M) ou 475 µA (`ITIMER0`, HFINT) | D |
| TIMER de disparo (caso 2) | 121 µA + HFXO 34 µA (`ISTBY_X32M_X2`) | 670 µA + HFXO 135 µA | D |
| CPU ativa | 2,6 mA (`IAPPCPU0`, 128 MHz) | 3,6 mA (`IAPPCPU5`, 64 MHz, HFXO64M) | D |
| CPU por amostra, QUEUE N = 64 | 3,6 µs (ISR de bloco ≈ 69 µs por 64 amostras + wrap 2 µs a cada 128 + `k_msgq_get` e decode 2,5 µs) | idem | E, ordem de grandeza de bancada |
| CPU por amostra, QUEUE N = 1 | 21 µs no M33 padrão (8 µs de trabalho + 13 µs de wake-up da RRAM, `tIDLE2CPU`, enquanto o core dorme entre amostras, até ~25 k/s); 8 µs com RRAM em standby, no FLPR, ou com o core acordado | 8 µs (não há RRAM) | E; `tIDLE2CPU` D |
| RRAM em standby, custo de idle | não publicado | — | — |
| Constant latency em idle | 0,55 mA (`ION_IDLE11`); medido, não corrige a latência sozinha | — | D, M |
| Bloco VPR (FLPR) ligado | ≈ +0,5 mA | — | relato de DevZone, não é datasheet |
| Transação | t = bytes × 8 / SCK + 1,5 µs: 11 B a 8 MHz 12,5 µs; 17 B a 8 MHz 18,5 µs; 11 B a 16 MHz 7,0 µs; 11 B a 32 MHz 4,25 µs | idem | E, coerente com M (17 B → 18,5 µs medido) |

Modelo: `I = base + domínio(s) ligado(s) + SPIM ativa × (taxa × t) + contador
+ TIMER de disparo + HFXO + CPU ativa × (taxa × µs por amostra)`.

- Modo LATEST: sem contador, sem CPU (os exemplos ligam o contador só para
  relatar a taxa; em produto ele sai).
- Modo QUEUE: contador sempre; na SPIM30 o contador em PERI acorda PERI, +20 µA.
- Caso 1 (INT): sem TIMER de disparo, sem HFXO. Caso 2 (TIMER): + TIMER de
  disparo + HFXO, e o GPIOTE não é usado.

## nRF54L15: modo × instância × taxa (caso 1, 11 B)

![Consumo por modo, instância e taxa](consumo_modos_nrf54l15.svg)

*Corrente média do SoC contra amostras/s: LATEST, QUEUE N = 64 e QUEUE
N = 1 nas três instâncias (E).*

Tudo por INT, rajada de 11 bytes, M33 padrão, corrente média do SoC em µA
(E). Colunas por instância: SPIM30 / SPIM22 / SPIM00.

| Amostras/s | LATEST 30 / 22 / 00 | QUEUE N = 64 30 / 22 / 00 | QUEUE N = 1 30 / 22 / 00 |
|---|---|---|---|
| 400 | 9 / 24 / 324 | 154 / 149 / 449 | 172 / 167 / 467 |
| 1 500 | 13 / 28 / 327 | 168 / 163 / 462 | 236 / 231 / 530 |
| 16 000 | 57 / 72 / 366 | 348 / 343 / 637 | 1 072 / 1 067 / 1 361 |
| 50 000 | 162 / 177 / 459 | 771 / 766 / 1 048 | 1 343 / 1 338 / 1 620 |

Dentro de cada número:

- **LATEST** = 2,9 + domínio do GPIOTE (LP 5 na SPIM30, PERI 20 nas outras)
  + domínio MCU 300 só na SPIM00 + SPIM ativa × ocupação. Sem TIMER, sem CPU.
  (A ocupação desta tabela foi calculada com 12,3 µs por transação de 11 B;
  a fórmula arredondada dá 12,5 µs, diferença abaixo de 2 µA em 50 k/s.)
- **QUEUE N = 64** = LATEST + contador TIMER21 121 (na SPIM30 mais 20, porque
  acorda PERI) + CPU de 3,6 µs por amostra a 2,6 mA.
- **QUEUE N = 1** = LATEST + contador + CPU de 21 µs por amostra até ~25 k/s
  (8 µs de trabalho + 13 µs de wake-up da RRAM); a 50 k/s o core não dorme e
  sobra só o trabalho, 8 µs.

Quatro leituras:

1. **LATEST é outro produto.** Entrega só o valor atual. É a linha mais baixa
   em todas as taxas porque não tem contador nem CPU, mas não serve se cada
   amostra importa.
2. **Entre as instâncias, a diferença é constante, não depende da taxa.**
   SPIM30 economiza 15 µA sobre a SPIM22 só em LATEST; em QUEUE o contador
   acorda PERI e a SPIM30 fica 5 µA pior. SPIM00 custa 300 µA a mais em
   qualquer linha. A instância nunca é a decisão principal; o modo de consumo é.
3. **N = 1 contra N = 64 é a maior alavanca acima de 1 k/s.** A 16 k/s são
   1,07 mA contra 0,34 mA, e a 50 k/s 1,34 contra 0,77. Dois terços do custo
   de N = 1 até 25 k/s é a RRAM acordando a cada amostra; com RRAM em standby
   N = 1 cai para 0,53 mA a 16 k/s, mas o custo de idle desse modo não é
   publicado.
4. **Abaixo de 2 k/s quem manda é o contador**, 121 µA, igual em N = 1 e
   N = 64. Ali a única forma de descer é o QUEUE por tempo, sem TIMER, que
   ficaria em 13 a 30 µA e ainda não foi implementado.

Regra que sai da tabela: valor atual → LATEST na SPIM30 ou SPIM22; todas as
amostras com latência tolerável → N = 64 na SPIM22; latência de uma amostra →
N = 1, e a partir de alguns k/s só com RRAM em standby ou FLPR; SPIM00 só por
barramento, nunca por consumo.

**QUEUE por tempo (não implementado).** Sem TIMER contador: a CPU acorda por
GRTC a cada T ms, lê `RXD.PTR` para saber quantas amostras chegaram, empurra
o bloco e, antes de devolver o ponteiro ao slot 0, espera o flag
`DMA.RX.READY` da transação em curso; anel dimensionado para mais de um T. É
o desenho do notificador por `k_timer` da biblioteca PPI Sequencer do NCS
mais novo. Custo = LATEST + CPU por amostra + ~1 µA de wake-ups: 9 / 24 µA a
100/s, 13 / 28 µA a 400/s, 28 / 43 µA a 1600/s, 88 / 103 µA a 6400/s (SPIM30
/ SPIM22, E). O limite de ~10 k/s é um julgamento: abaixo dele os 121 µA do
contador dominam; acima, o prazo do wrap (um período) pede o contador em
hardware.

## Caso 2 (TIMER) no nRF54L15

Mesmo modelo, SPIM22, 17 B (BMI270), acrescentando TIMER de disparo 121 µA e
HFXO 34 µA e trocando o GPIOTE por nada (E):

| Taxa | LATEST | QUEUE N = 16 com filtro | QUEUE sem filtro |
|---|---|---|---|
| 400/s (timer a 408/s) | ≈ 283 µA | ≈ 408 µA | ≈ 410 µA |
| 10 k/s (timer, ODR 400 Hz) | ≈ 325 µA | ≈ 545 µA | ≈ 545 µA (o filtro poupa só o consumidor) |
| 50 k/s (teto, 19 µs) | ≈ 435 µA | ≈ 1,0 mA | ≈ 1,1 mA |

O TIMER de disparo mais o HFXO custam 155 µA fixos: é a razão de o caso 1
ser o recomendado quando o pino existe. No FLPR somar ≈ 0,5 mA do bloco VPR.

## nRF5340 (Thingy:53, SPIM4, caso 1, 11 B)

| Taxa | LATEST | QUEUE N = 64 | QUEUE N = 1 | Observação |
|---|---|---|---|---|
| 400/s, 8 MHz | ≈ 58 µA | ≈ 733 µA | ≈ 740 µA | contador de 670 µA domina o QUEUE; sem RRAM, N = 1 custa quase o mesmo que N = 64 |
| 64 k/s, 8 MHz | ≈ 1,4 mA | ≈ 2,9 mA | ≈ 3,9 mA | SPIM 1,7 mA × 80 % |
| 64 k/s, 16 MHz | ≈ 0,9 mA | ≈ 2,4 mA | ≈ 3,4 mA | SPIM 1,9 mA × 45 % |

Termos: base 1,5 + GPIOTE 48 + SPIM × ocupação (+ contador 670 + CPU 3,6 mA ×
taxa × 3,6 µs ou 8 µs no QUEUE). Os testes da Thingy a 400 Hz rodaram a
4 MHz (default do Kconfig; transação de 23,5 µs, ≈ +8 µA sobre o valor a
8 MHz); o bench de 64 k rodou a 8 MHz (`bench/bus-64k-thingy.conf`).

Leituras:

- No nRF5340 o TIMER contador (670 µA) custa mais que todo o resto a 400/s;
  em produto o modo QUEUE de baixa taxa pede o mesmo "QUEUE por tempo" do
  nRF54L15, ou contagem por software na ISR de bloco.
- No teto o nRF5340 gasta 3 a 4× o nRF54L15 pelo mesmo trabalho: SPIM
  1,7 mA contra ~0,25 mA, TIMER 670 contra 121 µA.

## ADXL382 a 64 k amostras/s (caso 1, 11 B, não testado)

| SoC / instância | LATEST | QUEUE N = 64 | Termos do QUEUE |
|---|---|---|---|
| nRF5340 SPIM4, 8 MHz | ≈ 1,4 mA | ≈ 2,9 mA | 1,5 + 48 + 1,7 mA × 80 % + 670 + 3,6 mA × 23 % |
| nRF5340 SPIM4, 16 MHz | ≈ 0,9 mA | ≈ 2,4 mA | idem com 1,9 mA × 45 % |
| nRF54L15 SPIM22, 8 MHz | ≈ 0,22 mA | ≈ 0,94 mA | 2,9 + 20 + 0,25 mA × 80 % + 121 + 2,6 mA × 23 % |
| nRF54L15 SPIM30, 8 MHz | ≈ 0,21 mA | ≈ 0,95 mA | idem com LP 5 e +20 de PERI pelo contador |
| nRF54L15 SPIM00, 32 MHz | ≈ 0,54 mA | ≈ 1,26 mA | 2,9 + 20 + 300 + 0,8 mA × 27 % + 121 + 600 |

QUEUE N = 1 a 64 k/s (core acordado, 8 µs por amostra = 51 % de CPU): somar
≈ 0,73 mA às colunas QUEUE do nRF54L15 e ≈ 1,0 mA às do nRF5340. Os três
caminhos do nRF54L15 atendem 64 k/s com 11 B (E); SPIM22 e SPIM30 a 80 % do
barramento, SPIM00 a 27 %. O ADXL382 em si (não incluído) consome na casa de
1 mA em alto desempenho, conferir no datasheet do sensor.

## Quando a SPIM00 faz sentido

1. Rajadas maiores que 11 bytes a 64 k/s (STATUS + XYZ + temperatura, ou
   FIFO): com 8 MHz o teto é 52,6 k/s para 17 bytes (M); a 32 MHz cabem até
   ~45 bytes por transação a 64 k/s (E).
2. Taxas acima do que 8 MHz alcança com rajada curta (~80 k/s para 11 B, E,
   não medido no nRF54L).
3. Sensor que exige SCK > 8 MHz, ou `CSNDUR` maior sem perder taxa.
4. Margem de barramento: 27 % contra 80 % a 64 k/s, para absorver jitter do
   ODR ou um segundo dispositivo no barramento.
5. Domínio MCU já ligado por outro motivo (CPU ativa o tempo todo, flash
   externa na SPIM00, constant latency): os 300 µA já estão pagos.

Contra: pinos dedicados do P2 com drive E0/E1, errata 8 sempre ativa (CPHA =
1 ou primeiro bit 0), disparo em PERI atravessando o PPIB e, em QUEUE, a mesma
exigência de RRAM em standby ou FLPR das outras abaixo de 18 µs de período.

## O que reduziria em produto

1. Tirar o contador quando o modo for LATEST; em QUEUE de baixa taxa,
   contar por software ou drenar por tempo.
2. Disparo por INT quando o pino existe: sem HFXO, sem TIMER de disparo, sem
   repetidas.
3. Medir com PPK2 o que o modelo assume: custo do domínio LP e PERI mantidos
   pelo GPIOTE IN, corrente da SPIM do nRF54L15, custo da RRAM em standby.
4. Filtro de repetidas na ISR sempre que o timer for mais rápido que o ODR.
5. N grande reduz interrupções e a RRAM acordando; o limite é RAM e a
   latência de entrega (N períodos).
