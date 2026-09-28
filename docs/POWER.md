# Modelo de consumo (nRF54L15 e nRF5340)

Corrente média do SoC, sem PPK2: datasheets (D), latências médias de ISR
medidas (M), estimativas (E), relatos (R). Só o SoC. Cobre taxa alta (≥ ~1 k
amostras/s), toda amostra na fila, por drenagem a cada T ou por IRQ `END`
(`APP_PER_SAMPLE_IRQ`).

## Premissas

| Termo | nRF54L15 | nRF5340 | Fonte |
|---|---|---|---|
| Base (idle, RAM retida) | 2,9 µA (`ION_IDLE8`) | 1,5 µA (`ION_IDLE7`) | D |
| Domínio do GPIOTE IN | PERI 20 µA (Academy +17 µA) | 48 µA (`ION_IDLE4`, *LowLatency*) | R / D |
| Domínio MCU (SPIM00) | 300 µA (`ITIMER0` 450 − `ITIMER1` 142) | — | E |
| SPIM ativa | 0,25 mA SPIM2x (proxy TIMER20 142 µA, TWIM ~250); 0,8 mA SPIM00 | 1,7 mA a 8 Mbps (`ISPIM2`); 1,9 a 16 Mbps (E) | E / D |
| TIMER + HFXO (caso 2) | 121 (`ITIMER2`) + 34 µA (`ISTBY_X32M_X2`) | 670 (`ITIMER1`) + 135 µA | D |
| CPU ativa | 2,6 mA (`IAPPCPU0`) | 3,3 mA (`IAPPCPU5`) | D |
| Acordar de idle | proxy: média da latência disparo → ISR de wrap de idle no mesmo intervalo (`u_tag_wrap_latency.log`, inclui ≈ 1,1 µs de DPPI): 16,1 µs a ≥ 500 µs (máx 16,31: RRAM, `tIDLE2CPU` 13 µs), 15,5 a 250, 9,0 a 100, 1,2 acordado (≤ 50 µs); 12,3–12,9 a 176–191 µs interpolado | 2,7 µs média (máx 24,4, só nos prazos) | M, E |
| CPU por drenagem | acordar + 5 µs (21,1 a ≥ 500 µs); + assentamento (`XFER_SETTLE_US` 15 µs 11 B, 21 µs 17 B) com ≤ 4 amostras | 2,7 + 8 µs, + 15 de assentamento | E |
| T real | ⌈T/32 µs⌉ · 32 + 1 tick + drenagem: 10 ms → 10 068 µs; 1 ms → 1 076; 625 µs → 708 (11 B) / 714 (17 B); 100 µs → 176–191 | tick 30,5 µs: 10 051; 1 048; 697; 163 | D, E |
| CPU por wrap (1 por volta ≥ anel/2) | acordar + 3 µs (IRQ de idle, período ≥ 64 µs); um período + 3 na espera acordada; se o limite min(T/4, 8 períodos + 8 µs) expira (T = 100 µs a 16 k/s: 25 < 62,5), limite + 3,1 + 3 | idem com 2,7 | E |
| CPU por amostra, drenado | 1 µs `k_msgq_put` + 2,5 µs `k_msgq_get`/decode = 3,5 µs | idem | E |
| CPU por amostra, modo por amostra | entrada média da ISR (média − mín, `u_tag_persample_sweep.log`): 6,0 µs a 1000 µs, 3,0 a 500, 1,5 a 250, 0,66 a 100, 0,36 a 50; 3,8 a 625 e 0,43 a 62,5 interpolados; + 1,2 + 2,5 = 7,5 µs a 1 600/s, 4,1 a 16 k/s (máx 15,5 só no limite de taxa) | 0,5 + 1,5 + 2,5 = 4,5 µs (máx 26,3) | M, E |
| Transação | bytes × 8 / SCK + 1,5 µs: 11 B → 12,5 / 7,0 / 4,25 µs (8 / 16 / 32 MHz); 17 B → 18,5 / 5,75 | idem | E |
| Sem número | FLPR (`vpr_offloading` 146 → 125 µA, R; DevZone +0,5 mA de idle do VPR, R); RRAM standby (idle não publicado; ≈ 14 µs a menos por acordar); constant latency 0,55 mA (`ION_IDLE11`) não corrige a latência | — | R, D |

PERI 20 µA (R) é a premissa de menor confiança e desloca todas as linhas
por igual; o acordar domina o custo por drenagem. Usar o máximo (16,5 µs)
em vez da média inflava o modo por amostra em 3×.

## Modelo

`I = base + domínios + SPIM × (taxa × t) + TIMER + HFXO + CPU × fração`

- fração (drenado) = drenagens/s × (acordar + 5 + 15 se ≤ 4 por drenagem)
  + wraps/s × custo por wrap + taxa × 3,5 µs;
- fração (por amostra) = taxa × (entrada média + 1,2 + 2,5 µs);
- drenagens/s = 1 / T real; volta = ⌈(anel/2) / (amostras por drenagem)⌉ ×
  amostras por drenagem (anel 256); wraps/s = taxa / volta;
- caso 1 sem TIMER/HFXO; caso 2 com TIMER + HFXO, sem GPIOTE;
- 17 B: + 0,25 mA × taxa × 6 µs (SPIM22) ou 0,8 mA × taxa × 1,5 µs
  (SPIM00), só onde cabe (17 B a 50 k/s na SPIM22 são 92 %); assentamento
  21 µs.

## nRF54L15 (caso 1, 11 B, M33, µA)

![Consumo por modo de entrega, instância e taxa](consumo_modos_nrf54l15.svg)

| Entrega | Instância | 1 600/s | 16 k/s | 50 k/s |
|---|---|---|---|---|
| drenado, T = 10 ms (1 600/s) e 1 ms (16 k, 50 k) | SPIM22 (8 MHz) | **48** | **288** | **702** |
| drenado, T = 10 ms, 1 ms | SPIM00 (32 MHz) | 349 | 592 | 1 016 |
| drenado, T = período (625 µs) e 100 µs | SPIM22 | 176 | 675 | 952 |
| drenado, T = período, 100 µs | SPIM00 | 476 | 979 | 1 265 |
| por amostra | SPIM22 | 59 | 245 | fora da faixa |
| por amostra | SPIM00 | 359 | 549 | fora da faixa |
| FLPR | SPIM22 | sem número | sem número | sem número |

| Termo | 1 600/s | 16 k/s | 50 k/s |
|---|---|---|---|
| T real (drenagens/s) | 10 068 µs (99); 708 µs (1 413) | 1 076 µs (929); 191 µs (5 222) | 1 076 (929); 176 (5 666) |
| Fixo | 2,9 + 20 (+ 300 SPIM00) | idem | idem |
| Barramento | 5 µA SPIM22 (2 %), 5,4 SPIM00 (0,7 %) | 50 (20 %), 54 (6,8 %) | 156 (62,5 %), 170 (21 %) |
| Wraps/s (anel 256) | 12,4 (volta 129, IRQ de idle, 18,5 µs) | 116 (volta 138, acordada, 65,5 µs); 124 com T = 100 µs | 310 (volta 161, 23 µs); 378 com T = 100 µs |
| CPU, T longo | 99 × 21,1 + 12,4 × 19,1 + 1 600 × 3,5 = 0,80 % → 20 µA | 19 040 + 7 600 + 56 000 = 8,3 % → 215 | 19 040 + 7 120 + 175 000 = 20,1 % → 523 |
| CPU, T curto | 1 413 × 36,1 + 12,4 × 19,1 + 5 600 = 5,7 % → 148 | 5 222 × 32,9 + 124 × 31,1 + 56 000 = 23,2 % → 602 | 5 666 × 20,0 + 378 × 23 + 175 000 = 29,7 % → 772 |
| CPU, por amostra | 7,5 × 1 600 = 1,2 % → 31 | 4,1 × 16 000 = 6,6 % → 172 | fora da faixa (20 < 12,5 + 15,5 + 2) |

T = 10 ms a 16 k e 50 k/s pediria anel de 320 e 1 000 slots, por isso T =
1 ms (o da bancada). 100 µs é o mínimo do Kconfig (176–191 µs reais: 3 e 9
amostras de latência); T = 62,5 µs a 16 k/s daria 816 µA. As 5 222–5 666
drenagens/s custam 6,7–6,9 % só em acordar.

## nRF5340 (Thingy:53, SPIM4, caso 1, 11 B)

Base 1,5 + GPIOTE 48 + SPIM × ocupação + 3,3 mA × fração (2,7 + 8 µs por
drenagem, + 15 com ≤ 4 amostras; 2,7 + 3 por wrap de idle ou período + 3
acordado; 3,5 µs por amostra; 4,5 no modo por amostra); sem HFXO.

| Taxa | T longo | T curto | Por amostra | Termos |
|---|---|---|---|---|
| 1 600/s, 8 MHz | ≈ 0,11 mA (T = 10 ms) | ≈ 0,22 mA (625 µs) | ≈ 0,11 mA | SPIM 34 µA (2 %); CPU 23 / 141 / 24 µA; GPIOTE 48 µA domina |
| 64 k/s, 8 MHz | ≈ 2,2 mA (T = 1 ms) | ≈ 2,4 mA (100 µs) | fora da faixa | SPIM 1,36 mA; CPU 954 × 10,7 + 477 × 18,6 + 64 000 × 3,5 = 24,3 % → 0,80 mA; T = 100 µs (163 µs reais) 29,8 % → 0,98 mA |
| 64 k/s, 16 MHz | ≈ 1,7 mA | ≈ 1,9 mA | fora da faixa | SPIM 1,9 mA × 45 % = 0,85 mA |

A 64 k/s o nRF5340 gasta 1,9–2,5× o nRF54L15 (SPIM 1,7 contra ~0,25 mA);
a 1 600/s 2,2× (0,11 contra 0,048). Testes a 400 Hz na Thingy a 4 MHz
(23,5 µs por transação); benches a 8 MHz.

## ADXL382 a 64 k/s (caso 1, 11 B, não testado)

| SoC / instância | T = 1 ms (1 076 µs reais) | T = 100 µs (176 µs reais, ≈ 11 amostras de latência) | Termos (T = 1 ms) |
|---|---|---|---|
| nRF5340 SPIM4, 8 MHz | ≈ 2,2 mA | ≈ 2,4 mA | 1,5 + 48 + 1,7 mA × 80 % + 3,3 mA × 24,3 % |
| nRF5340 SPIM4, 16 MHz | ≈ 1,7 mA | ≈ 1,9 mA | idem com 1,9 mA × 45 % |
| nRF54L15 SPIM22, 8 MHz | ≈ 0,88 mA | ≈ 1,12 mA | 2,9 + 20 + 0,25 mA × 80 % + 2,6 mA × 25,3 % (654 µA) |
| nRF54L15 SPIM00, 32 MHz | ≈ 1,20 mA | ≈ 1,44 mA | 2,9 + 20 + 300 + 0,8 mA × 27 % + 654 |

nRF54L15, T = 1 ms: 68,9 por drenagem, 929 × 21,1 + 464 wraps × (15,6 + 3)
+ 64 000 × 3,5 µs = 25,3 % de CPU (22,4 % são os 3,5 µs por amostra). T =
100 µs: 5 666 drenagens × 19,9 µs (12,3 de acordar) = 11,3 % sozinhas,
34,6 %, 900 µA. Por amostra fora da faixa (15,6 < 12,5 + 15,5 ou 26 + 2).
Errata 8 não se aplica (0x23). O ADXL382 (~1 mA, conferir) não está
incluído.

## Caso 2 (TIMER, nRF54L15, SPIM22, 17 B)

TIMER 121 + HFXO 34 µA no lugar do GPIOTE; com filtro 0,5 µs por transação
+ 3,5 por amostra nova; sem filtro 3,5 por transação; ≈ 20 µs por drenagem
e os wraps.

| Timer | T | Com filtro | Sem filtro | Termos |
|---|---|---|---|---|
| 408/s (sensor 402/s) | 10 ms | ≈ 169 µA | ≈ 169 µA | 2,9 + 155 + 0,25 mA × 0,75 % + ≈ 10 (0,37 %) |
| 10 k/s | 1 ms | ≈ 274 µA | ≈ 348 µA | 2,9 + 155 + 46 (18,5 %) + 70 ou 144 (2,7 / 5,6 %; 77 wraps/s de idle) |
| 50 k/s (20 µs) | 1 ms | ≈ 526 µA | ≈ 0,91 mA | 2,9 + 155 + 231 (92 %) + 137 ou 523 (5,3 / 20,1 %; 310 wraps/s acordados) |

TIMER + HFXO custam 155 µA fixos contra 20 µA do GPIOTE. Exemplo "sem
data-ready a 16 kHz": ≈ 0,43 mA.

## Leituras

- T curto custa 3,6× / 2,3× / 1,4× o T longo (176 / 48; 675 / 288; 952 /
  702): drenagens de ~21 µs (16,1 de RRAM) mais 15 µs de assentamento.
- Por amostra custa menos que T = período em toda a faixa (59 / 176; 245 /
  675) e, a 16 k/s, menos que T = 1 ms (245 / 288); o limite é o prazo
  (≈ 25 k/s).
- SPIM00: +300 µA; só por barramento (rajada > 11 B a 64 k/s, até ~45 B a
  32 MHz; > 71–80 k/s; SCK > 8 MHz; sobra 27 % contra 80 %; MCU já ligado).
  Contra: pinos P2 drive E0/E1, errata 8 com MSB 1, disparo pelo PPIB.
- A 1 600/s o fixo é o PERI: 48 µA = 20 PERI + 20 CPU + 5 SPIM + 2,9; a
  variante 100 % LP (GPIOTE30 + SPIM30, não testada) tiraria os 20 µA.
- RRAM standby: ≈ 14 µs a menos por acordar = 3,7 µA a 1 600/s (T = 10 ms),
  ≈ 15 µA por amostra, contra um idle não publicado.
- Regra: data-ready + drenado T = 10 ms na SPIM22; por amostra para latência
  de uma amostra (≤ 25 k/s); T = 100 µs só acima; SPIM00 só por barramento.

## Limitações

- Nenhuma corrente medida (PPK2: PERI do GPIOTE, SPIM do nRF54L15, acordar,
  drenagem, FLPR × M33).
- Acordar é proxy pela ISR de wrap; interpolado entre 100 e 250 µs.
- FLPR e RRAM standby sem número.
- Limite de T: RAM ((slots + 8 + fila) × rajada: 2,9 + 2,8 KB a 64 kHz com
  T = 1 ms) e latência (T real; em taxa alta a mais nova sai na drenagem
  seguinte); além da guarda o EasyDMA corrompe a RAM.
- Valores D típicos a 3 V, 25 °C, DC/DC; sensor não incluído.
