# Análise teórica de consumo — nRF54L15 e nRF5340

Estimativas de corrente média do SoC para cada combinação disparo × consumo,
a 400 amostras/s (caso do ADXL362/BMI270) e na taxa máxima medida no
barramento (50 k transações/s na Tag, 71 k no Thingy). Só o SoC: a corrente
do sensor não entra. Valores típicos a 3 V, 25 °C, DC/DC, conforme as
tabelas "Current consumption" dos datasheets (fonte: MCP da Nordic,
`ps_nrf54l15` e `ps_nrf5340`).

## Números de referência

| Símbolo | nRF54L15 | nRF5340 (app core) | Observação |
|---|---|---|---|
| System ON idle, RAM retida | `ION_IDLE8` 2,9 µA (256 KB, GRTC/LFXO) | `ION_IDLE7` 1,5 µA (RTC/LFXO) | base |
| CPU rodando (CoreMark) | `IAPPCPU0` 2,6 mA @128 MHz | `IAPPCPU5` 3,3 mA @64 MHz HFINT; 3,6 mA HFXO64M | usada para o tempo de ISR/thread |
| HFXO em regime | `ISTBY_X32M_X2` 34 µA | `ISTBY_X32M_X2` 135 µA | cristal 2,0×1,6 mm |
| TIMER a 1 MHz | `ITIMER2` 121 µA (TIMER20) | `ITIMER0` 475 µA; `ITIMER1` 670 µA com HFXO64M | disparo por timer |
| TIMER a 16 MHz | `ITIMER1` 142 µA (TIMER20) | `ITIMER2` 560 µA | contador de END (roda por evento, custo ≈ instância ligada) |
| SPIM transferindo @ 8 Mbps | **não publicado**; estimativa 0,25 mA¹ | `ISPIM2` 1,705 mA; `ISPIM3` 1,930 mA com HFXO64M | ponderado pelo duty cycle do barramento |
| GPIOTE IN event ativo em idle | domínio PERI mantido ligado; teto = Constant Latency `ION_IDLE11` 0,55 mA² | `ION_IDLE4` 48 µA (event mode, LATENCY = LowLatency) | disparo pelo pino do sensor |
| FLPR rodando | não publicado; observação de campo ≈ +0,5 mA com o VPR ligado³ | — | |

¹ A Academy da Nordic usa ~250 µA como ordem de grandeza de um periférico
serial (TWIM) do nRF54L15 em atividade; TIMER20 a 16 MHz (mesmo domínio,
mesmo clock PCLK16M) custa 142 µA. Tratado como estimativa ±50 %.
² A Academy documenta que o GPIOTE IN event "mantém o domínio de potência
associado ligado em System ON idle" sem dar o valor; o cenário de Constant
Latency (domínios mantidos ligados) é o limite superior.
³ DevZone (VPR current draw after shutting down): bloco VPR ligado em WFI
≈ +500 µA. Não é dado de datasheet.

Tempos de barramento medidos: rajada de 17 B (BMI270) a 8 MHz = 17 µs +
`tSPIM,START` 1 µs + CSNDUR; rajada de 11 B (ADXL362) = 11 µs + overhead.

## Modelo

`I_média = I_base + I_clock + I_disparo + I_SPIM × duty_SPIM + I_CPU × duty_CPU`

- `duty_SPIM = taxa × duração da transação`
- `duty_CPU`: LATEST ≈ 0 (um relatório por segundo); QUEUE = ISR de bloco
  (~35 µs por bloco de 64 com filtro na ISR, medido em ordem de grandeza) +
  ISR de wrap (~2 µs, 1 a cada 2N) + consumidor (~2,5 µs por `k_msgq_get`
  incluindo decode). Com `APP_QUEUE_FRESH_ONLY` o consumidor só vê as
  amostras novas.
- Disparo por INT: sem TIMER de disparo, sem HFXO obrigatório, mas o
  GPIOTE IN mantém o domínio acordado. Disparo por TIMER: TIMER a 1 MHz +
  HFXO (para período exato) e nenhum GPIOTE.
- O contador de END (TIMER em modo contador) existe nos dois modos; conta
  como uma instância de TIMER ligada.

## nRF54L15 (Tag, BMI270, 17 B a 8 MHz)

| Caso | 400/s | 50 k/s (teto) |
|---|---|---|
| INT + LATEST | 2,9 + 121 (contador) + 0,25 mA×0,7 % ≈ **125 µA** + domínio PERI ligado (≤ 550 µA)² | n/a (o sensor não gera) |
| INT + QUEUE (N = 16) | anterior + ISR 25 blocos/s × 35 µs × 2,6 mA (2 µA) + consumidor 400 × 2,5 µs × 2,6 mA (3 µA) ≈ **130 µA** + PERI² | n/a |
| TIMER + LATEST | 2,9 + 34 (HFXO) + 121 (disparo) + 121 (contador) + 2 ≈ **280 µA** | 2,9 + 34 + 242 + 0,25 mA×85 % (213) ≈ **490 µA** |
| TIMER + QUEUE, filtro na ISR | 280 + 5 ≈ **285 µA** | 490 + ISR 781 blocos/s × 35 µs (2,7 % → 71 µA) + wrap 391/s × 2 µs (2 µA) + consumidor 402 × 2,5 µs (3 µA) ≈ **565 µA** |
| TIMER + QUEUE, sem filtro | ≈ 290 µA | 565 + consumidor 50 k × 2,5 µs (12,5 % → 325 µA) + 50 k `k_msgq_put` na ISR (~1 µs cada, 5 % → 130 µA) ≈ **1,0 mA** |
| Idem no FLPR | + ~0,5 mA do bloco VPR³; o app core dorme (base) | + ~0,5 mA³ |

Leituras:

- A 400/s o que domina é **manter o domínio PERI ligado** para o GPIOTE IN
  (INT) ou o **HFXO + TIMER** (timer). Entre os dois, o TIMER é previsível
  (≈ 280 µA); o INT pode ficar entre ~130 µA e ~680 µA dependendo de quanto
  do domínio o GPIOTE IN realmente segura — é o ponto a medir com o PPK2.
- Na taxa máxima o barramento vira o termo maior mesmo com a estimativa
  conservadora de SPIM (85 % de ocupação), e o custo de CPU só aparece se as
  repetidas forem para a fila: o filtro na ISR vale ~0,4 mA.
- O FLPR não reduz consumo — acrescenta o bloco VPR — mas cumpre o prazo de
  wrap a 20 µs sem ZLI e libera o M33.
- **Latência × consumo no M33.** Em low-power idle (2,9 µA) a RRAM fica em
  power-down e uma ISR que acorda o core leva ~17 µs (`tIDLE2CPU` 13 µs).
  As duas saídas custam corrente: constant latency = 0,55 mA em idle
  (`ION_IDLE11`) e, medido, não corrige sozinha; RRAM em standby
  (`POWER.LOWPOWERCONFIG.MODE = Standby`) corrige (2,75 µs) e o datasheet
  não publica seu custo — medir com PPK2. Para os casos deste repo a
  latência não perde dados (o wrap tem a transação inteira de margem e o
  core não dorme acima de ~40 k/s), então a configuração padrão é a certa;
  RRAM standby só para prazos < 18 µs com o core dormindo entre eventos.
- **Domínio da SPIM.** SPIM30 (LP, P0) com GPIOTE30 é o único caminho que
  deixa PERI desligado entre transações; SPIM2x + GPIOTE20 mantém PERI
  ligado (Academy: ~+17 µA só pelo GPIOTE IN); SPIM00 (MCU, 32 MHz) exige
  o domínio MCU ativo e PPIB para o disparo. Nenhuma corrente de SPIM é
  publicada para o nRF54L15.

## nRF5340 (Thingy:53, ADXL362, 11 B a 8 MHz)

| Caso | 380–400/s | 71 k/s (teto) |
|---|---|---|
| INT + LATEST | 1,5 + 48 (GPIOTE IN LowLatency) + 475 (contador) + 1,705 mA×0,42 % (7) ≈ **530 µA** | n/a |
| INT + QUEUE (N = 16) | 530 + ISR/consumidor (~6 µA a 3,3 mA) ≈ **540 µA** | n/a |
| TIMER + LATEST (HFXO64M) | 1,5 + 135 + 670 (disparo) + 670 (contador) + 1,930 mA×0,44 % (8) ≈ **1,48 mA** | 1,5 + 135 + 1340 + 1,930 mA×78 % (1,5 mA) ≈ **3,0 mA** |
| TIMER + QUEUE, filtro na ISR | ≈ **1,49 mA** | 3,0 + ISR 1109 blocos/s × 35 µs (3,9 % → 140 µA) + wrap (4 µA) + consumidor (4 µA) ≈ **3,15 mA** |
| TIMER + QUEUE, sem filtro | ≈ 1,5 mA | 3,15 + consumidor 71 k × 2,5 µs (18 % → 640 µA) + puts (7 % → 250 µA) ≈ **4,0 mA** |

Leituras:

- No nRF5340 o TIMER é caro (475–670 µA por instância a 1 MHz contra 121 µA
  no nRF54L15): o contador de END, que aqui é conveniência de teste, custa
  mais do que todo o resto a 400/s. Em produto, contar END por software (uma
  ISR por bloco já existe) elimina 475–670 µA.
- INT + LATEST é o caso mais barato (~55 µA sem o contador), com o GPIOTE em
  LowLatency; em LowPower o GPIOTE IN cai para 1,3 µA de idle, ao custo de
  latência variável do PPI.
- Na taxa máxima o nRF5340 gasta ~5× o nRF54L15 pelo mesmo trabalho
  (SPIM 1,9 mA vs ~0,25 mA estimado, TIMER 670 vs 121 µA).

## ADXL382 a 64 k amostras/s por data-ready

Rajada de 11 bytes (1 comando + `STATUS0..ZDATA_L`), período 15,6 µs.
Ocupação do barramento: 12,5 µs (80 %) a 8 MHz; 7 µs (45 %) a 16 MHz.
Modo QUEUE com N = 64 → 1000 IRQ de bloco/s, 500 wraps/s, 64 k `k_msgq_put`
e `k_msgq_get` por segundo.

| Termo | nRF5340, 8 MHz | nRF5340, 16 MHz (SPIM4) | nRF54L15, SPIM2x a 8 MHz |
|---|---|---|---|
| Base + GPIOTE IN | 1,5 + 48 = 50 µA | 50 µA | 2,9 µA + domínio PERI (0 a 550 µA)² |
| HFXO (para SPIM/TIMER com clock exato) | 135 µA | 135 µA | 34 µA (opcional no caso INT) |
| TIMER contador de END (obrigatório em QUEUE) | 670 µA | 670 µA | 121 µA |
| SPIM × ocupação | 1,93 mA × 80 % = 1,54 mA | ≈ 2,05 mA¹ × 45 % = 0,92 mA | 0,25 mA¹ × 80 % = 0,20 mA |
| CPU (só QUEUE): ISR EGU 1000/s × ~69 µs + wrap 500/s × 2 µs + consumidor 64 k × 2,5 µs ≈ 23 % | 3,6 mA × 23 % = 0,83 mA | 0,83 mA | 2,6 mA × 23 % = 0,60 mA |
| **INT + LATEST** | **≈ 2,4 mA** | **≈ 1,8 mA** | **≈ 0,3–0,9 mA** |
| **INT + QUEUE (N = 64)** | **≈ 3,2 mA** | **≈ 2,6 mA** | **≈ 0,9–1,5 mA** |

¹ `ISPIM` do nRF5340 só é publicado a 8 e 32 Mbps (1,93 / 2,35 mA com
HFXO64M): 16 Mbps interpolado. No nRF54L15 vale a estimativa de 0,25 mA já
descrita; para 16 MHz seria preciso a SPIM00 (domínio MCU), sem número
publicado.

Leituras: no nRF5340 o barramento a 8 MHz e o TIMER contador dominam; subir
para 16 MHz na SPIM4 economiza ~0,6 mA e, mais importante, dobra a folga do
margem do wrap (a transação inteira: 12 → 7 µs). O modo LATEST poupa a CPU inteira (0,8 mA) mas entrega só
o último valor; para stream a 64 k/s o modo QUEUE com N = 64 é o mínimo
razoável (N menor multiplica as IRQ). O ADXL382 em si (não incluído) consome
na casa de 1 mA em alto desempenho — conferir no datasheet do sensor.

Medições de referência a ODR máximo por INT (sem consumo medido, só taxa):
BMI270 a 1600 Hz na Tag, M33 e FLPR, 1601–1616 transações/s, zero perdas.

## O que reduziria em produto

1. Contar END por software (ou não contar) em vez de um TIMER dedicado.
2. Disparo por INT quando o pino existe: sem HFXO, sem TIMER, sem repetidas.
3. No nRF54L15, verificar com PPK2 quanto o GPIOTE IN realmente mantém
   ligado (PERI) — decide entre INT e TIMER a 400/s.
4. Filtro de repetidas na ISR sempre que o timer for mais rápido que o ODR.
5. Blocos maiores (N) reduzem interrupções por amostra; o limite é RAM e
   latência de entrega (N períodos).
