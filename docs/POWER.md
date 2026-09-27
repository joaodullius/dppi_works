# Consumo — modelo de corrente média (nRF54L15 e nRF5340)

**TL;DR: nada aqui foi medido com PPK2. É um modelo de corrente média do SoC
montado com os números que os datasheets publicam (D) e, onde eles não
publicam, com estimativas marcadas (E) ou relatos (R). Só o SoC: o sensor
não entra. A tabela que importa é a de
[N × instância × taxa](#nrf54l15-n--instância--taxa-caso-1-11-b).**

Marcação: **M** = medido em bancada (log em `*/test-logs/`), **E** =
estimado, **D** = datasheet (`ps_nrf54l15` e `ps_nrf5340`, tabelas "Current
consumption", típicos a 3 V, 25 °C, DC/DC, consultados pelo MCP da Nordic),
**R** = relato (Nordic Academy, DevZone), não é especificação.

Este modelo cobre o uso do repositório: taxa alta (a partir de ~1 k
amostras/s, indispensável acima de ~10 k/s), toda amostra na fila em blocos
de N. Abaixo de ~1 k/s um produto usa o subsistema de sensores do Zephyr, e
o consumo é outro assunto.

## Premissas (um conjunto só, usado em todas as tabelas)

| Termo | nRF54L15 | nRF5340 | Fonte |
|---|---|---|---|
| Base, System ON idle, RAM retida | 2,9 µA (`ION_IDLE8`) | 1,5 µA (`ION_IDLE7`) | D |
| Domínio mantido ligado pelo GPIOTE IN | PERI 20 µA (Academy mediu +17 µA numa DK, arredondado) | 48 µA (`ION_IDLE4`, GPIOTE IN event, LowLatency) | 54L: R; 5340: D |
| Domínio MCU ligado por um periférico dele (SPIM00) | 300 µA (`ITIMER0` TIMER00 450 µA − `ITIMER1` TIMER20 142 µA; a Academy mostra que o TIMER00 a 1 MHz custa quase o mesmo que a 128 MHz: o custo é o domínio) | — | E, proxy D + R |
| SPIM ativa | 0,25 mA (SPIM2x; proxy: TIMER20 a 16 MHz 142 µA, TWIM ~250 µA na Academy); 0,8 mA (SPIM00, razão TIMER00/TIMER20) | 1,7 mA a 8 Mbps (`ISPIM2`, HFINT); 1,9 mA a 16 Mbps (E, entre `ISPIM2` e `ISPIM4` 2,1 mA a 32 Mbps) | 54L: E; 5340: D |
| Contador de transações (TIMER em modo contador) | 121 µA (`ITIMER2`, TIMER20 a 1 MHz: o datasheet não tem valor para modo contador; usado como custo do TIMER ligado) | 475 µA (`ITIMER0`, 1 MHz, HFINT) | E, proxy D |
| TIMER de disparo (caso 2) | 121 µA + HFXO 34 µA (`ISTBY_X32M_X2`) | 670 µA (`ITIMER1`, HFXO64M) + HFXO 135 µA | D |
| CPU ativa | 2,6 mA (`IAPPCPU0`, 128 MHz) | 3,3 mA (`IAPPCPU5`, 64 MHz, HFINT) | D |
| CPU por amostra, N = 64 | 3,8 µs (ISR de bloco ≈ 69 µs por 64 + wrap 2 µs a cada 128 + `k_msgq_get` e decode 2,5 µs + entrada de ISR e troca de contexto rateadas) | idem | E, ordem de grandeza de bancada |
| CPU por amostra, N = 16 | 4,8 µs (entrada de ISR e wrap rateados por 16) | idem | E |
| CPU por amostra, N = 1 | 21 µs no M33 padrão (8 µs de trabalho + 13 µs de wake-up da RRAM, `tIDLE2CPU`) enquanto o core dorme entre amostras; a bancada N = 1 mostra o core deixando de dormir entre 25 e 40 k/s, e acima de ~40 k/s a latência já é a de core acordado; 8 µs com RRAM em standby, no FLPR, ou com o core acordado (a 50 k/s) | 8 µs (não há RRAM) | E; `tIDLE2CPU` D; limite de sono M |
| CPU acordada | 2,6 mA × fração do tempo ocupado (o modelo não usa 2,6 mA contínuos) | 3,3 mA × fração | E |
| Corrente ativa do FLPR | não publicada | — | — |
| RRAM em standby, custo de idle | não publicado | — | — |
| Constant latency em idle | 0,55 mA (`ION_IDLE11`); medido, não corrige a latência sozinha | — | D, M |
| FLPR em N = 1 | sem número: a amostra `vpr_offloading` da Nordic mediu 146 → 125 µA no nRF54L15 DK (app core 3,0 % de CPU contra FLPR 0,1 %, uma transferência SPI por ms) porque o FLPR roda da RAM e não acorda a RRAM; um relato de DevZone dá ≈ +0,5 mA de idle do VPR noutra configuração, confirmado por um engenheiro da Nordic como "corrente de sono padrão do VPR". As duas fontes conflitam para este uso | — | R (NCS docs, DevZone) |
| Transação | t = bytes × 8 / SCK + 1,5 µs: 11 B a 8 MHz 12,5 µs; 17 B a 8 MHz 18,5 µs; 11 B a 16 MHz 7,0 µs; 11 B a 32 MHz 4,25 µs | idem | E, coerente com M (17 B → 18,5 µs medido) |

PERI 20 µA (R) é a premissa de menor confiança; ela desloca todas as
linhas do nRF54L15 por igual e não muda nenhuma comparação.

Modelo: `I = base + domínio(s) ligado(s) + SPIM ativa × (taxa × t) + contador
+ TIMER de disparo + HFXO + CPU ativa × (taxa × µs por amostra)`.

- Caso 1 (INT): sem TIMER de disparo, sem HFXO. Caso 2 (TIMER): + TIMER de
  disparo + HFXO, e o GPIOTE não é usado.
- O contador existe sempre: é ele que gera a interrupção de bloco e a de
  wrap.
- Rajada de 17 bytes em vez de 11: somar 0,25 mA × taxa × 6 µs na SPIM22 (8
  MHz) ou 0,8 mA × taxa × 1,5 µs na SPIM00 (32 MHz), só onde o barramento
  ainda cabe (17 B a 50 k/s na SPIM22 são 92 % de ocupação, fora do
  critério de sobra).

## nRF54L15: N × instância × taxa (caso 1, 11 B)

![Consumo por N, instância e taxa](consumo_modos_nrf54l15.svg)

*Corrente média do SoC contra amostras/s para N = 64 e N = 1, SPIM22 e
SPIM00 (E). No painel N = 1, a barra tracejada azul é o M33 com RRAM em
standby e a cinza é o FLPR.*

Caso 1 (data-ready), rajada de 11 bytes, toda amostra na fila, Cortex-M33
padrão (RRAM em power-down). SPIM22 a 8 MHz (12,5 µs por transação), SPIM00
a 32 MHz (4,25 µs). Corrente média do SoC em µA (E).

| N | Instância (SCK) | 1 600/s | 16 k/s | 50 k/s |
|---|---|---|---|---|
| N = 64 | SPIM22 (8 MHz) | **165** | **352** | **794** |
| N = 64 | SPIM00 (32 MHz) | 465 | 656 | 1 108 |
| N = 16 | SPIM22 | 169 | 394 | 924 |
| N = 16 | SPIM00 | 469 | 698 | 1 238 |
| N = 1, M33 padrão | SPIM22 | 236 | 1 068 | 1 340 |
| N = 1, M33 padrão | SPIM00 | 536 | 1 372 | 1 654 |
| N = 1, M33 com RRAM em standby | SPIM22 | 182 | 527 | 1 340 |
| N = 1, FLPR | SPIM22 | sem número (E) | sem número (E) | sem número (E) |

Termos de cada linha:

- **Fixo** = 2,9 + PERI 20 + domínio MCU 300 só na SPIM00 + contador 121.
- **Barramento** = SPIM ativa × (taxa × t): a 16 k/s, 0,25 mA × 20 % =
  50 µA na SPIM22 e 0,8 mA × 6,8 % = 54 µA na SPIM00.
- **N = 64 / N = 16** = fixo + barramento + 2,6 mA × taxa × 3,8 µs (N = 64)
  ou 4,8 µs (N = 16).
- **N = 1, M33 padrão** = fixo + barramento + 2,6 mA × taxa × 21 µs (8 µs de
  trabalho + 13 µs de wake-up da RRAM), válido enquanto o core dorme entre
  amostras. **O modelo de N = 1 não é monotônico**: a bancada N = 1 mostra
  o core deixando de dormir entre 25 e 40 k/s, e acima de ~40 k/s a
  latência já é a de core acordado, sobrando só os 8 µs de trabalho. A
  linha de 50 k/s já usa 8 µs; a de 16 k/s, 21 µs. Entre 25 e 40 k/s o
  modelo não vale: o valor real fica entre as duas fórmulas.
- **N = 1 com RRAM em standby** = fixo + barramento + 2,6 mA × taxa × 8 µs.
  O custo de idle da RRAM em standby não é publicado.
- **N = 1 no FLPR**: sem fórmula. Roda da RAM, então não paga os 13 µs de
  wake-up da RRAM sem precisar do standby; o custo de idle do bloco VPR vai
  de "economia" (`vpr_offloading`, R) a +0,5 mA (DevZone, R) conforme a
  configuração, e a corrente ativa do FLPR não é publicada. Só o PPK2
  decide.

Três leituras:

1. **N = 1 pesa a partir de ~5 k/s.** Contra N = 64 custa 1,4× a 1,6 k/s
   (236 contra 165 µA), 3,0× a 16 k/s (1,07 contra 0,35 mA) e 1,7× a 50 k/s
   (1,34 contra 0,79). A RRAM acordando a cada amostra responde por ≈ 23 %
   do N = 1 a 1,6 k/s, ≈ 51 % a 16 k/s e 0 % a 50 k/s. Com RRAM em standby
   N = 1 cai para 182 µA, 527 µA e 1,34 mA. **N = 1 no FLPR fica sem
   número (E)**: a amostra `vpr_offloading` da Nordic mediu economia no
   nRF54L15 (146 → 125 µA com ~1 k transações SPI/s, R) porque o FLPR roda
   da RAM e não acorda a RRAM; um relato de DevZone dá +0,5 mA de idle do
   VPR noutra configuração (R). O FLPR é candidato a N = 1 de baixo
   consumo, sem standby da RRAM, mas não foi medido aqui.
2. **A SPIM00 custa ~300 µA a mais em qualquer taxa** (o domínio MCU
   ligado). Só paga pela sobra de barramento: SCK acima de 8 MHz, rajada
   longa a 64 k/s, ou taxa acima de ≈ 71–80 k/s (E). Nunca por consumo.
3. **O contador TIMER (121 µA) faz parte do desenho** e não é alavanca
   nestas taxas: a 16 k/s são 34 % do N = 64 e 11 % do N = 1; a 50 k/s,
   15 % e 9 %.

Regra que sai da tabela: data-ready + N = 64 na SPIM22, salvo se a latência
de uma amostra for requisito (então N = 1 com RRAM em standby, ou FLPR se a
medição com PPK2 confirmar a economia) ou o barramento não couber a 8 MHz
(então SPIM00). Próximo passo: medir com PPK2 FLPR, M33 padrão e M33 + RRAM
standby na mesma bancada N = 1.

## nRF5340 (Thingy:53, SPIM4, caso 1, 11 B)

Termos: base 1,5 + GPIOTE 48 + SPIM × (taxa × t) + contador 475 + CPU
3,3 mA × taxa × µs por amostra. Sem HFXO no caso 1 (E).

| Taxa | N = 64 | N = 1 | Observação |
|---|---|---|---|
| 1 600/s, 8 MHz | ≈ 0,58 mA | ≈ 0,60 mA | o contador de 475 µA domina; sem RRAM, N = 1 custa quase o mesmo que N = 64 |
| 64 k/s, 8 MHz | ≈ 2,7 mA | ≈ 3,6 mA (fora do medido) | SPIM 1,7 mA × 80 %; N = 1: 1,5 + 48 + 1 360 + 475 + 3,3 mA × 51 % |
| 64 k/s, 16 MHz | ≈ 2,2 mA | ≈ 3,1 mA (fora do medido) | SPIM 1,9 mA × 45 % |

Os testes da Thingy a 400 Hz rodaram a 4 MHz (default do Kconfig; transação
de 23,5 µs); o bench de 64 k rodou a 8 MHz (`bench/bus-64k-thingy.conf`).

Leituras: no nRF5340 o contador (475 µA, D) pesa mais que no nRF54L15
(121 µA) e a SPIM ativa custa 7× (1,7 mA contra ~0,25 mA); a 64 k/s com
N = 64 o nRF5340 gasta 2,3 a 2,8× o nRF54L15 (2,2–2,7 mA contra 0,97 mA), e
a 1 600/s 3,5× (0,58 contra 0,165 mA), porque ali o contador domina.

## Caso 2 (TIMER) no nRF54L15

SPIM22, 17 B (18,5 µs por transação, BMI270), TIMER de disparo 121 µA + HFXO
34 µA em vez do GPIOTE (o TIMER em PERI já mantém o domínio ligado). CPU com
filtro: 0,5 µs por transação (checar o bit) + 4,3 µs por amostra nova (put,
get, decode); sem filtro, 4,8 µs por transação (E).

| Taxa do timer | N = 16 com filtro | N = 16 sem filtro | Termos |
|---|---|---|---|
| 408/s (sensor a 402/s) | ≈ 286 µA | ≈ 286 µA | 2,9 + 121 + 34 + 0,25 mA × 0,8 % + 121 + ≈ 5 de CPU |
| 10 k/s (sensor a 400 Hz) | ≈ 342 µA | ≈ 450 µA | 2,9 + 155 + 0,25 mA × 18,5 % + 121 + 17 ou 125 de CPU |
| 50 k/s (20 µs; o teto é 52,6 k/s a 19 µs) | ≈ 580 µA | ≈ 1,13 mA | 2,9 + 155 + 0,25 mA × 92 % + 121 + 70 ou 624 de CPU |

O TIMER de disparo mais o HFXO custam 155 µA fixos: é a razão de o caso 1
ser o recomendado quando o pino existe. No FLPR o custo de idle do VPR não
tem número (ver premissas).

## ADXL382 a 64 k amostras/s (caso 1, 11 B, não testado)

| SoC / instância | N = 64 | N = 1, core acordado (fora do medido: M33 varrido até 50 k/s, FLPR satura a 40 k/s) | Termos do N = 64 |
|---|---|---|---|
| nRF5340 SPIM4, 8 MHz | ≈ 2,7 mA | ≈ 3,6 mA | 1,5 + 48 + 1,7 mA × 80 % + 475 + 3,3 mA × 24 % |
| nRF5340 SPIM4, 16 MHz | ≈ 2,2 mA | ≈ 3,1 mA | idem com 1,9 mA × 45 % |
| nRF54L15 SPIM22, 8 MHz | ≈ 0,97 mA | ≈ 1,7 mA | 2,9 + 20 + 0,25 mA × 80 % + 121 + 2,6 mA × 24 % |
| nRF54L15 SPIM00, 32 MHz | ≈ 1,29 mA | ≈ 2,0 mA | 2,9 + 20 + 300 + 0,8 mA × 27 % + 121 + 632 |

N = 1 a 64 k/s: o core não dorme (8 µs por amostra = 51 % de CPU), por isso
soma ≈ 0,7 mA no nRF54L15 e ≈ 0,9 mA no nRF5340 sobre a coluna N = 64. No
nRF54L15 a SPIM2x atende 64 k/s com 11 B a 80 % do barramento, sem margem; a
SPIM00 a 27 % dá margem. A errata 8 não se aplica ao ADXL382 (primeiro byte
0x23, bit mais significativo 0). O ADXL382 em si (não incluído) consome na
casa de 1 mA em alto desempenho, conferir no datasheet do sensor.

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
comando; não é o caso do ADXL382; o BMI270 aceita modo 3, não testado),
disparo em PERI atravessando o PPIB
(latência não especificada) e a mesma exigência de RRAM em standby ou FLPR
das outras abaixo de 18 µs de período.

## O que reduziria em produto

1. Disparo por INT quando o pino existe: sem HFXO, sem TIMER de disparo, sem
   repetidas.
2. N = 64 sempre que a latência de 64 períodos for aceitável; N = 1 com
   RRAM em standby, ou no FLPR se o PPK2 confirmar a economia.
3. Medir com PPK2 o que o modelo assume: custo do domínio PERI mantido pelo
   GPIOTE IN, corrente da SPIM do nRF54L15, custo da RRAM em standby, e
   FLPR contra M33 padrão contra M33 + RRAM standby na mesma bancada N = 1.
4. Filtro de repetidas na ISR sempre que o timer for mais rápido que o ODR.
5. N grande reduz interrupções e a RRAM acordando; o limite é RAM (anel de
   3N rajadas mais a fila: a 64 kHz com N = 64 e fila de 512, 2,1 KB +
   5,6 KB) e a latência de entrega (N períodos).
