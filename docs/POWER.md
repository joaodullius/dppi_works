# Consumo — modelo de corrente média (nRF54L15 e nRF5340)

**TL;DR: nada aqui foi medido com PPK2. É um modelo de corrente média do SoC
montado com os números que os datasheets publicam (D), com as latências
médias de ISR medidas nos logs (M) e, onde nada disso existe, com
estimativas marcadas (E) ou relatos (R). Só o SoC: o sensor não entra. A
tabela que importa é a de
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
| Base, System ON idle, RAM retida | 2,9 µA (`ION_IDLE8`: corrente de idle com toda a RAM retida, tabela "Current consumption" do datasheet) | 1,5 µA (`ION_IDLE7`) | D |
| Domínio mantido ligado pelo GPIOTE IN | PERI 20 µA (Academy mediu +17 µA numa DK, arredondado) | 48 µA (`ION_IDLE4`: idle com um GPIOTE IN event configurado, o modo que o datasheet chama de *LowLatency*) | 54L: R; 5340: D |
| Domínio MCU ligado por um periférico dele (SPIM00) | 300 µA (`ITIMER0` TIMER00 450 µA − `ITIMER1` TIMER20 142 µA; a Academy mostra que o TIMER00 a 1 MHz custa quase o mesmo que a 128 MHz: o custo é o domínio) | — | E, proxy D + R |
| SPIM ativa | 0,25 mA (SPIM2x; proxy: TIMER20 a 16 MHz 142 µA, TWIM ~250 µA na Academy); 0,8 mA (SPIM00, razão TIMER00/TIMER20) | 1,7 mA a 8 Mbps (`ISPIM2`, HFINT); 1,9 mA a 16 Mbps (E, entre `ISPIM2` e `ISPIM4` 2,1 mA a 32 Mbps) | 54L: E; 5340: D |
| TIMER de disparo (caso 2) | 121 µA (`ITIMER2`, TIMER20 a 1 MHz) + HFXO 34 µA (`ISTBY_X32M_X2`) | 670 µA (`ITIMER1`, HFXO64M) + HFXO 135 µA | D |
| CPU ativa | 2,6 mA (`IAPPCPU0`, 128 MHz) | 3,3 mA (`IAPPCPU5`, 64 MHz, HFINT) | D |
| Acordar de idle (IRQ ou `k_sleep`) | proxy: a média medida da latência disparo → ISR de wrap saindo de idle no mesmo intervalo entre eventos (`u_tag_wrap_latency.log`, M), que inclui ≈ 1,1 µs de cadeia DPPI/START/READY: 16,1 µs com intervalos ≥ 500 µs (16,06–16,07 de média, 16,31 de máximo: a RRAM está em power-down, `tIDLE2CPU` 13 µs, D), 15,5 µs a 250 µs, 9,0 µs a 100 µs, 1,2 µs quando o core não chegou a dormir (≤ 50 µs); entre 100 e 250 µs, interpolação linear (E): 12,3 µs a 176 µs, 12,9 µs a 191 µs. Contado como CPU ativa: clocks ligados, sem executar. O máximo (16,4 µs) só entra nos prazos, não no consumo | a média medida na IRQ de wrap a 100 µs de período (`u_thingy_bus64k.log`, M): 2,7 µs (máximo 24,4 µs, que só entra nos prazos); sem RRAM, é clock e regulador | M, E entre pontos |
| CPU por drenagem | acordar (acima) + 5 µs de trabalho (ler o head, entregar, armar) = 21,1 µs com intervalos ≥ 500 µs (20,5 a 250 µs); mais a espera de assentamento (`XFER_SETTLE_US` = bytes × 8 / SCK + 4 µs: 15 µs para 11 B, 21 µs para 17 B) quando chegam até 4 amostras por drenagem | 2,7 + 8 µs, mais os mesmos 15 µs de assentamento | E, ordem de grandeza de bancada |
| Período real de drenagem (T real) | T arredondado para cima ao tick de 32 µs mais um tick (`k_sleep`), mais a própria drenagem (o `k_sleep` começa depois do trabalho): 10 ms → 10 048 + 20 = 10 068 µs; 1 ms → 1 056 + 20 = 1 076 µs; 625 µs → 672 + 36 = 708 µs (11 B) ou 714 µs (17 B); 100 µs → 160 + 16 a 32 = 176 a 191 µs | tick de 30,5 µs: 10 ms → 10 040 + 11 = 10 051 µs; 1 ms → 1 038 + 11 = 1 048 µs; 625 µs → 671 + 26 = 697 µs; 100 µs → 153 + 11 = 163 µs | D (kernel), E |
| CPU por wrap (uma vez por volta do anel, ≥ anel/2 amostras) | acordar (pelo intervalo entre wraps) + 3 µs se a IRQ vem de idle (período ≥ 64 µs); um período + 3 µs se a thread espera acordada e o wrap chega dentro do limite min(T/4, 8 períodos + 8 µs); se o limite expira antes do período (T = 100 µs a 16 k/s: 25 µs < 62,5 µs), o limite + acordar a 62,5 µs (3,1 µs, E) + 3 µs | idem, com 2,7 µs de acordar | E |
| CPU por amostra, modo drenado | 1 µs de `k_msgq_put` na drenagem + 2,5 µs de `k_msgq_get` e decode no consumidor = 3,5 µs | idem | E |
| CPU por amostra, modo por amostra | entrada média da ISR de `END` no mesmo período (média − mínimo em `u_tag_persample_sweep.log`, M): 6,0 µs a 1 000 µs, 3,0 a 500, 1,5 a 250, 0,66 a 100, 0,36 a 50 (o core nem sempre chega a dormir entre amostras); 3,8 µs a 625 µs e 0,43 a 62,5 µs por interpolação (E); + 1,2 µs de ISR e cópia + 2,5 µs de consumidor = 7,5 µs a 1 600/s, 4,1 µs a 16 k/s. Os máximos (15,5 µs de entrada) só entram no limite de taxa | entrada média 0,1–0,7 µs (`u_thingy_persample_sweep.log`, M; máximo 26,3): 0,5 + 1,5 + 2,5 = 4,5 µs | M, E |
| CPU acordada | 2,6 mA × fração do tempo ocupado (o modelo não usa 2,6 mA contínuos) | 3,3 mA × fração | E |
| Corrente ativa do FLPR | não publicada | — | — |
| RRAM em standby, custo de idle | não publicado; tiraria ≈ 14 µs de cada acordar de idle (16,8 → 2,75 µs, mecanismo anterior, log não incluído) | — | — |
| Constant latency em idle | 0,55 mA (`ION_IDLE11`); medido com o mecanismo anterior, não corrige a latência sozinha | — | D, M |
| FLPR | sem número: a amostra `vpr_offloading` da Nordic mediu 146 → 125 µA no nRF54L15 DK (app core 3,0 % de CPU contra FLPR 0,1 %, uma transferência SPI por ms) porque o FLPR roda da RAM e não acorda a RRAM; um relato de DevZone dá ≈ +0,5 mA de idle do VPR noutra configuração, descrito ali como a corrente de sono padrão do VPR. As duas fontes conflitam para este uso | — | R (NCS docs, DevZone) |
| Transação | t = bytes × 8 / SCK + 1,5 µs: 11 B a 8 MHz 12,5 µs; 17 B a 8 MHz 18,5 µs; 11 B a 16 MHz 7,0 µs; 11 B a 32 MHz 4,25 µs; 17 B a 32 MHz 5,75 µs | idem | E; o 1,5 µs é inferido do teto medido (19 µs passa, 18 não é comprovado, com 17 µs de bits), não medido em separado |

PERI 20 µA (R) é a premissa de menor confiança; ela desloca todas as
linhas do nRF54L15 por igual e não muda nenhuma comparação. A segunda é o
custo de acordar (a média medida, contada como CPU ativa): ele domina o
custo por drenagem, e é o que o PPK2 mais precisa confirmar. Usar o
**máximo** medido em vez da média (como uma versão anterior deste modelo
fazia) inflava o modo por amostra em 3× e invertia a comparação com T =
período.

Modelo: `I = base + domínio(s) ligado(s) + SPIM ativa × (taxa × t) + TIMER
de disparo + HFXO + CPU ativa × fração`, com

- fração (modo drenado) = drenagens/s × (acordar + 5 µs + 15 µs de
  assentamento se amostras por drenagem ≤ 4) + wraps/s × (custo por wrap) +
  taxa × 3,5 µs;
- fração (modo por amostra) = taxa × (entrada média + 1,2 + 2,5 µs);
- drenagens/s = 1 / T real; amostras por drenagem = taxa × T real; wraps/s =
  taxa / comprimento da volta; a volta é a primeira drenagem em que o head
  passa de anel/2, ou seja, ⌈(anel/2) / (amostras por drenagem)⌉ × amostras
  por drenagem, com o anel padrão de 256 slots.
- Caso 1 (INT): sem TIMER de disparo, sem HFXO. Caso 2 (TIMER): + TIMER de
  disparo + HFXO, e o GPIOTE não é usado.
- Rajada de 17 bytes em vez de 11: somar 0,25 mA × taxa × 6 µs na SPIM22 (8
  MHz) ou 0,8 mA × taxa × 1,5 µs na SPIM00 (32 MHz), só onde o barramento
  ainda cabe (17 B a 50 k/s na SPIM22 são 92 % de ocupação, fora do
  critério de sobra); e 21 µs de assentamento em vez de 15.

## nRF54L15: entrega × instância × taxa (caso 1, 11 B)

![Consumo por modo de entrega, instância e taxa](consumo_modos_nrf54l15.svg)

*Corrente média do SoC contra amostras/s para o T padrão (10 ms; 1 ms a
16 k e 50 k/s), para o T mais curto (o período do sensor a 1 600/s; o
mínimo de 100 µs acima; sempre com o T real) e para o modo por amostra,
SPIM22 e SPIM00 (E).*

Caso 1 (data-ready), rajada de 11 bytes, toda amostra na fila, Cortex-M33
padrão. SPIM22 a 8 MHz (12,5 µs por transação), SPIM00 a 32 MHz (4,25 µs).
Corrente média do SoC em µA (E).

| Entrega | Instância (SCK) | 1 600/s | 16 k/s | 50 k/s |
|---|---|---|---|---|
| drenado, T = 10 ms (1 600/s) e 1 ms (16 k e 50 k/s) | SPIM22 (8 MHz) | **48** | **288** | **702** |
| drenado, T = 10 ms, 1 ms | SPIM00 (32 MHz) | 349 | 592 | 1 016 |
| drenado, T = período (625 µs) e 100 µs (mínimo) | SPIM22 | 176 | 675 | 952 |
| drenado, T = período, 100 µs | SPIM00 | 476 | 979 | 1 265 |
| por amostra (`APP_PER_SAMPLE_IRQ`) | SPIM22 | 59 | 245 | fora da faixa |
| por amostra | SPIM00 | 359 | 549 | fora da faixa |
| qualquer, FLPR | SPIM22 | sem número (E) | sem número (E) | sem número (E) |

Termos de cada linha (T real: 10 ms → 10 068 µs, 99 drenagens/s; 1 ms →
1 076 µs, 929/s; 625 µs → 708 µs, 1 413/s; 100 µs → 191 µs a 16 k/s,
5 222/s, e 176 µs a 50 k/s, 5 666/s):

- **Fixo** = 2,9 + PERI 20 + domínio MCU 300 só na SPIM00.
- **Barramento** = SPIM ativa × (taxa × t): a 1 600/s, 5 µA (SPIM22, 2 %)
  ou 5,4 µA (SPIM00, 0,7 %); a 16 k/s, 50 µA (20 %) ou 54 µA (6,8 %); a
  50 k/s, 156 µA (62,5 %) ou 170 µA (21 %).
- **Wraps por segundo** (anel 256): a 1 600/s e T = 10 ms, 16,1 por
  drenagem, volta de 129, 12,4 wraps/s pela IRQ de idle (período 625 µs ≥
  64), 18,5 µs cada; a 16 k/s e T = 1 ms, 17,2 por drenagem, volta de 138,
  116 wraps/s com espera acordada (período 62,5 µs < 64), 65,5 µs cada; a
  50 k/s e T = 1 ms, 53,8 por drenagem, volta de 161, 310 wraps/s, 23 µs
  cada. Com T = 625 / 100 / 100 µs: 12,4, 124 e 378 wraps/s.
- **CPU, T longo** = 2,6 mA × fração: a 1 600/s e T = 10 ms, 99
  drenagens × 21,1 µs + 12,4 × 19,1 + 1 600 × 3,5 = 7 960 µs/s = 0,80 % →
  20 µA; a 16 k/s e T = 1 ms, 19 040 + 7 600 + 56 000 = 8,3 % → 215 µA; a
  50 k/s e T = 1 ms, 19 040 + 7 120 + 175 000 = 20,1 % → 523 µA.
- **CPU, T curto** = a 1 600/s e T = 625 µs, 1 413 drenagens × (21,1 + 15
  de assentamento, 1,13 amostra por drenagem) + 12,4 × 19,1 + 5 600 =
  5,7 % → 148 µA; a 16 k/s e T = 100 µs (191 µs reais), 5 222 × (12,9 de
  acordar interpolado a esse intervalo + 5 + 15, 3,06 por drenagem) + 124 ×
  31,1 (espera de 25 µs que expira + acordar + 3) + 56 000 = 23,2 % →
  602 µA; a 50 k/s e T = 100 µs (176 µs reais), 5 666 × 20,0 (12,3 de
  acordar interpolado + 7,7, sem assentamento) + 378 × 23 + 175 000 =
  29,7 % → 772 µA. As 5 222 a 5 666 drenagens/s custam 6,7 a 6,9 % de CPU
  só em acordar.
- **CPU, por amostra** = a 1 600/s, 7,5 µs × 1 600 = 1,2 % → 31 µA; a
  16 k/s, 4,1 µs × 16 000 = 6,6 % → 172 µA. A 50 k/s o modo não vale: 20 µs
  de período contra 12,5 de transação mais 15,5 de entrada máxima da ISR
  mais a cópia (medido na TAG com 17 B: limpo até 40 µs, marginal a 30–25
  µs, `torn` em quase todas a 20 µs).
- **Por que T = 1 ms a 16 k e 50 k/s**: com T = 10 ms chegariam 160 e 500
  amostras por drenagem, e o anel precisaria de 320 e 1 000 slots (2×);
  com T = 1 ms são 17 e 54, e o anel padrão de 256 serve. É também o T da
  bancada (`bench/*.conf`).
- **Por que 100 µs e não o período**: `APP_DRAIN_PERIOD_US` vai de 100 µs
  a 1 s, e 100 µs viram 176 a 191 µs reais. A 16 k/s isso é 3 amostras de
  latência; a 50 k/s, 9. A 50 k/s a espera acordada cobre o wrap (20 µs de
  período dentro do limite de 25 µs); a 16 k/s o limite min(T/4, 8
  períodos + 8 µs) = 25 µs é menor que o período de 62,5 µs, a espera
  expira e a IRQ de wrap vem de idle, sem prejuízo (a latência de ≤ 16,3 µs
  cabe nos 62,5 µs). Se o Kconfig deixasse T = 62,5 µs a 16 k/s (124 µs
  reais, 8 052 drenagens/s), o modelo daria 816 µA na SPIM22 (E): a essa
  altura o modo por amostra (245 µA) é o caminho.
- **Modo por amostra contra T = período**: com as entradas médias medidas,
  o modo por amostra custa menos que o drenado com T = período em toda a
  faixa em que vale (59 contra 176 µA a 1 600/s; 245 contra 675 a
  16 k/s), e a 16 k/s custa até menos que T = 1 ms (245 contra 288),
  porque cada amostra paga 4,1 µs contra 3,5 µs mais a parte da drenagem e
  do wrap. Contra o T longo custa 1,2× a 1 600/s. O que limita o modo por
  amostra é o prazo (a entrada máxima da ISR), não o consumo.
- **FLPR**: sem fórmula. Roda da RAM; o custo de idle do bloco VPR vai de
  "economia" (`vpr_offloading`, R) a +0,5 mA (DevZone, R) conforme a
  configuração, e a corrente ativa do FLPR não é publicada. Só o PPK2
  decide.

Três leituras:

1. **T curto custa 3,6× (1,6 k/s), 2,3× (16 k/s) e 1,4× (50 k/s) o T
   longo** (176 contra 48 µA; 675 contra 288; 952 contra 702). A
   diferença é o número de drenagens, ~21 µs de CPU cada, 16,1 dos quais
   são a RRAM acordando, mais 15 µs de assentamento quando chega uma
   amostra só; o custo por amostra é o mesmo. O modo por amostra fica
   perto do T longo a 1 600/s (59 contra 48 µA) e abaixo dele a 16 k/s
   (245 contra 288), porque a ISR de `END` acorda em 0,4 a 4 µs de média,
   não 15,5: o core raramente chega ao power-down da RRAM entre amostras.
2. **A SPIM00 custa ~300 µA a mais em qualquer taxa** (o domínio MCU
   ligado). Só paga pela sobra de barramento: SCK acima de 8 MHz, rajada
   longa a 64 k/s, ou taxa acima de ≈ 71–80 k/s (E). Nunca por consumo.
3. **O custo fixo é o domínio PERI.** A 1 600/s são 48 µA: 20 de PERI (R),
   20 de CPU, 5 de SPIM, 2,9 de base. Uma variante 100 % LP (GPIOTE30 +
   SPIM30, não testada) tiraria os 20 µA do PERI (E). RRAM em standby
   tiraria ≈ 14 µs de cada acordar de idle (a 1 600/s e T = 10 ms, 3,7 µA;
   por amostra, ≈ 15 µA) ao custo de uma corrente de idle não publicada.

Regra que sai da tabela: data-ready + modo drenado com T = 10 ms na
SPIM22, salvo se a latência de uma amostra for requisito (então o modo por
amostra até o seu limite de prazo, ≈ 25 k/s; T = 100 µs só de 25 k/s até o
teto, onde o modo por amostra não cabe) ou o barramento não couber a 8 MHz
(então SPIM00). Próximo passo: medir com PPK2 FLPR contra M33 no caso 1, e
o custo real de uma drenagem.

## nRF5340 (Thingy:53, SPIM4, caso 1, 11 B)

Termos: base 1,5 + GPIOTE 48 + SPIM × (taxa × t) + CPU 3,3 mA × fração,
com 2,7 µs de acordar (média medida) + 8 µs por drenagem (mais 15 µs de
assentamento com ≤ 4 amostras), 2,7 + 3 µs por wrap de idle (um período +
3 na espera acordada), 3,5 µs por amostra, 4,5 µs por amostra no modo por
amostra (0,5 de entrada média, M, + 1,5 + 2,5); T real com o tick de
30,5 µs (E). Sem HFXO no caso 1.

| Taxa | Drenado, T longo | Drenado, T curto | Por amostra | Observação |
|---|---|---|---|---|
| 1 600/s, 8 MHz | ≈ 0,11 mA (T = 10 ms) | ≈ 0,22 mA (T = 625 µs) | ≈ 0,11 mA | SPIM 1,7 mA × 2 % = 34 µA; CPU 23, 141 ou 24 µA; o GPIOTE (48 µA, D) é o maior termo fixo |
| 64 k/s, 8 MHz | ≈ 2,2 mA (T = 1 ms) | ≈ 2,4 mA (T = 100 µs) | fora da faixa | SPIM 1,7 mA × 80 % = 1,36 mA; CPU 954 drenagens × 10,7 + 477 wraps × 18,6 + 64 000 × 3,5 = 24,3 % → 0,80 mA; com T = 100 µs (163 µs reais), 29,8 % → 0,98 mA |
| 64 k/s, 16 MHz | ≈ 1,7 mA (T = 1 ms) | ≈ 1,9 mA (T = 100 µs) | fora da faixa | SPIM 1,9 mA × 45 % = 0,85 mA |

Os testes da Thingy a 400 Hz rodaram a 4 MHz (default do Kconfig; transação
de 23,5 µs); os benches de 64 k e por amostra rodaram a 8 MHz
(`bench/bus-64k-thingy.conf`, `bench/per-sample-thingy.conf`). O modo por
amostra no nRF5340 foi medido limpo até 40 µs de período (25 k/s) pelo
critério da latência (M; a 40 µs com −5 % de novas sem contraparte
drenada, atribuição em aberto; limpo sem ressalva até 50 µs, 20 k/s): a
−1,7 % de amostras novas a 50 µs é o bit de
data-ready do sensor nesse espaçamento, igual ao modo drenado (366/s nos
dois). A fórmula (12,5 de transação + 26,3 de entrada máxima + 2 µs ≈ 41
µs) dá 24 k/s; a partir de 30 µs a latência mínima cai abaixo da
transação e aparecem cópias atropeladas.

Leituras: no nRF5340 a SPIM ativa custa 7× a do nRF54L15 (1,7 mA contra
~0,25 mA) e a CPU 3,3 contra 2,6 mA; a 64 k/s com T = 1 ms o nRF5340 gasta
1,9 a 2,5× o nRF54L15 (1,7–2,2 mA contra 0,88 mA), e a 1 600/s 2,2× (0,11
contra 0,048 mA), porque ali o GPIOTE de 48 µA domina.

## Caso 2 (TIMER) no nRF54L15

SPIM22, 17 B (18,5 µs por transação, BMI270), TIMER de disparo 121 µA + HFXO
34 µA em vez do GPIOTE (o TIMER em PERI já mantém o domínio ligado). CPU com
filtro: 0,5 µs por transação (checar o bit na drenagem) + 3,5 µs por amostra
nova (put, get, decode); sem filtro, 3,5 µs por transação; mais ≈ 20 µs por
drenagem e os wraps (E).

| Taxa do timer | T | Com filtro | Sem filtro | Termos |
|---|---|---|---|---|
| 408/s (sensor a 402/s) | 10 ms | ≈ 169 µA | ≈ 169 µA | 2,9 + 155 + 0,25 mA × 0,75 % + ≈ 10 de CPU (0,37 %) |
| 10 k/s (sensor a 400 Hz) | 1 ms | ≈ 274 µA | ≈ 348 µA | 2,9 + 155 + 0,25 mA × 18,5 % (46) + 70 ou 144 de CPU (2,7 ou 5,6 %; 77 wraps/s pela IRQ de idle) |
| 50 k/s (20 µs; o teto é 52,6 k/s a 19 µs) | 1 ms | ≈ 526 µA | ≈ 0,91 mA | 2,9 + 155 + 0,25 mA × 92 % (231) + 137 ou 523 de CPU (5,3 ou 20,1 %; 310 wraps/s com espera acordada de 20 µs) |

O TIMER de disparo mais o HFXO custam 155 µA fixos, contra 20 µA do
GPIOTE: é a razão de o caso 1 ser o recomendado quando o pino existe. No
FLPR o custo de idle do VPR não tem número (ver premissas).

## ADXL382 a 64 k amostras/s (caso 1, 11 B, não testado)

| SoC / instância | T = 1 ms (≈ 1,08 ms reais) | T = 100 µs (mínimo; ≈ 176 µs reais) | Termos do T = 1 ms |
|---|---|---|---|
| nRF5340 SPIM4, 8 MHz | ≈ 2,2 mA | ≈ 2,4 mA | 1,5 + 48 + 1,7 mA × 80 % + 3,3 mA × 24,3 % |
| nRF5340 SPIM4, 16 MHz | ≈ 1,7 mA | ≈ 1,9 mA | idem com 1,9 mA × 45 % |
| nRF54L15 SPIM22, 8 MHz | ≈ 0,88 mA | ≈ 1,12 mA | 2,9 + 20 + 0,25 mA × 80 % + 2,6 mA × 25,2 % (654 µA) |
| nRF54L15 SPIM00, 32 MHz | ≈ 1,20 mA | ≈ 1,44 mA | 2,9 + 20 + 300 + 0,8 mA × 27 % + 654 |

A 64 k/s com T = 1 ms (1 076 µs reais) chegam 68,9 amostras por drenagem:
929 drenagens × 21,1 µs, 464 wraps/s com espera acordada (um período,
15,6 µs, + 3) e 64 000 × 3,5 µs dão 25,3 % de CPU no nRF54L15, dominados
pelos 3,5 µs por amostra (22,4 %). Com T = 100 µs (176 µs reais, ≈ 11
amostras de latência) as 5 666 drenagens custam 11,3 % sozinhas (19,9 µs
cada, 12,3 de acordar interpolados): 34,6 % de CPU, 900 µA. O modo por amostra não serve (15,6 µs de período contra 12,5
de transação mais 15,5 ou 26 µs de entrada máxima mais a cópia). No
nRF54L15 a SPIM2x atende 64 k/s com 11 B a 80 % do barramento, sem margem;
a SPIM00 a 27 % dá margem. A errata 8 não se aplica ao ADXL382 (primeiro
byte 0x23, bit mais significativo 0). O ADXL382 em si (não incluído)
consome na casa de 1 mA em alto desempenho, conferir no datasheet do
sensor.

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

Contra: pinos dedicados do P2 com *drive* E0/E1 (as classes de corrente de
saída mais altas do GPIO do nRF54L15, exigidas para 32 MHz), errata 8
quando o primeiro byte do comando tem o bit mais significativo em 1 (CPHA =
1 ou trocar o comando; não é o caso do ADXL382; o BMI270 aceita modo 3,
não testado) e disparo em PERI atravessando o PPIB (latência não
especificada). O wrap não muda de uma instância para outra.

## O que reduziria em produto

1. Disparo por INT quando o pino existe: sem HFXO, sem TIMER de disparo, sem
   repetidas.
2. T = 10 ms sempre que a latência de 10 ms for aceitável; o modo por
   amostra (até ≈ 25 k/s, o seu limite de prazo; custa 1,2× o T longo a
   1 600/s) quando a latência de uma amostra for requisito; T ≈ período do
   sensor só acima disso (custa 2,3 a 3,6× o T longo).
3. Medir com PPK2 o que o modelo assume: custo do domínio PERI mantido pelo
   GPIOTE IN, corrente da SPIM do nRF54L15, custo real de acordar (a
   RRAM) e de uma drenagem, e FLPR contra M33 no caso 1.
4. Filtro de repetidas na drenagem sempre que o timer for mais rápido que
   o ODR.
5. Variante 100 % LP (GPIOTE30 + SPIM30 no nRF54L15 DK): tira os 20 µA do
   PERI; não testada.
6. RRAM em standby (`APP_RRAM_STANDBY`) se a corrente de idle que ela
   custa (não publicada) for menor que os ≈ 14 µs × 2,6 mA por acordar que
   ela economiza: a 1 600/s e T = 10 ms são 3,7 µA, pouco; no modo por
   amostra a 1 600/s, ≈ 15 µA.
7. O limite de T é RAM (anel de (slots + 8) rajadas mais a fila: a 64 kHz
   com T = 1 ms, anel de 256 e fila de 256, 2,9 KB + 2,8 KB) e a latência
   de entrega (o T real: T arredondado ao tick mais um tick mais a
   drenagem; em taxa alta a amostra mais nova de cada drenagem sai na
   seguinte). Além dos 8 slots de guarda o EasyDMA corrompe a RAM: a regra
   anel ≥ 2 × taxa × T real vale com o T real.
