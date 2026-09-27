# dppi_works — aquisição SPI sem CPU com DPPI (nRF5340, nRF54L15)

**Em uma frase:** um evento de hardware (o data-ready do sensor ou um TIMER)
dispara a SPIM por DPPI, o EasyDMA grava cada rajada num anel em RAM, e uma
thread acorda a cada período de drenagem T (10 ms por padrão) para passar as
amostras a uma fila: duas interrupções por T, qualquer que seja a taxa. Com
sensor real, medido até 1 600 Hz por data-ready com zero perdas; os tetos de
barramento, medidos com disparo por TIMER, são 71,4 k transações/s no
nRF5340 (11 B) e 52,6 k/s no nRF54L15 (17 B).

**Tem uma pergunta concreta?** Vá direto aos
[exemplos resolvidos](#exemplos-resolvidos) (BMI270 a 1 600 Hz, ADXL382 a
64 kHz nos dois SoCs, sensor sem data-ready a 16 kHz).

Este caminho é para **taxa alta**. Os resultados com sensor real vão até
1 600 Hz, o ODR máximo dos sensores disponíveis; acima disso a bancada usa o
disparo por TIMER. O repositório vale a partir de ~1 k amostras/s e é
indispensável acima de ~10 k/s, onde uma leitura por chamada satura a CPU;
abaixo de ~1 k/s um produto usa o subsistema de sensores do Zephyr ou
leituras SPI diretas.

Os exemplos usam o DPPI (nRF53/54); o mesmo desenho vale com PPI no nRF52,
não testado. nRF Connect SDK v3.4.1 (nrfx 4.0), chip select por hardware.
Validado na Thingy:53 (nRF5340 + ADXL362) e na nRF54L15 TAG da Nordic
(placa Zephyr `nrf54l15tag`, com BMI270 a bordo), nos cores Cortex-M33 e
FLPR.

Marcação de todo número deste repositório: **M** = medido (log em
`*/test-logs/`, salvo onde se diz que o log não está incluído), **E** =
estimado por modelo, **D** = datasheet, **R** = relato (Nordic Academy,
DevZone), não é especificação.

Índice: [O problema e a ideia](#o-problema-e-a-ideia) ·
[Glossário](#glossário-mínimo) · [Três decisões](#três-decisões) ·
[Caso 1](#caso-1--data-ready--gpiote--dppi--spim-recomendado) ·
[Caso 2](#caso-2--timer--dppi--spim) · [Anel, drenagem e wrap](#anel-drenagem-e-wrap) ·
[Limites](#limites) · [Como escolher](#como-escolher) · [Exemplos resolvidos](#exemplos-resolvidos) ·
[Consumo](#consumo--resumo) · [Qual SPIM](#nrf54l15-qual-spim) ·
[ADXL382](#caso-de-alta-taxa-adxl382-a-64-khz-não-testado-em-hardware) ·
[Achados](#achados)

## O problema e a ideia

**TL;DR: com a CPU no caminho, cada amostra custa uma interrupção e um
`spi_transceive`; a 64 k/s isso é a CPU inteira. Com DPPI o caminho
sensor → RAM não tem instrução nenhuma, e a CPU acorda uma vez por período
de drenagem.**

Um driver de sensor convencional faz uma transação SPI por chamada: a CPU
acorda, arma o buffer, espera o fim, copia. Isso limita a taxa, gasta
energia e introduz jitter entre a amostra e a leitura. Nos SoCs Nordic os
periféricos têm tarefas e eventos ligáveis pelo DPPI: o evento "amostra
pronta" pode acionar a tarefa `START` da SPIM sem CPU, e o EasyDMA em modo
*array list* escreve uma transação atrás da outra em RAM sem precisar de
interrupção. A CPU só entra para esvaziar o anel, num período que o
projeto escolhe. Este repositório mostra isso funcionando, mede os limites
e explica como escolher a configuração para um sensor concreto.

## Glossário mínimo

| Termo | Significado neste repositório |
|---|---|
| transação | uma rajada SPI: `START` → bytes → `END`. No caso 1 uma transação = uma amostra |
| amostra | um valor do sensor (X, Y, Z); "amostra nova" = data-ready ativo na rajada |
| rajada | os bytes de uma transação: comando + `STATUS` + dados (11 B no ADXL362, 17 B no BMI270) |
| período | intervalo entre dois `START` consecutivos (1/ODR no caso 1, período do timer no caso 2) |
| ODR | taxa de saída do sensor; "nominal" é a configurada, "real" é a medida (tolerância do oscilador do sensor) |
| T, período de drenagem | `APP_DRAIN_PERIOD_US` (10 ms por padrão, mínimo 100 µs): intervalo em que a thread de drenagem acorda. Latência de entrega = T; interrupções/s = 2/T (o acordar da thread e a IRQ de wrap), independentes da taxa de amostras. T ≈ período do sensor dá latência de uma amostra |
| drenagem | o que a thread faz a cada T: lê o head, entrega à fila os slots completos, arma o wrap |
| slot | espaço de uma rajada no anel |
| anel | buffer de `APP_RING_SLOTS` slots (256 por padrão) mais 8 slots de guarda. Precisa caber mais de uma drenagem de amostras. RAM = (slots + 8) × bytes da rajada |
| head, tail | head = índice do slot que o EasyDMA vai escrever em seguida, lido de `DMA.RX.PTR` (`RXD.PTR` no nRF5340); o slot head − 1 pode estar em curso. tail = próximo slot a entregar |
| array list | modo do EasyDMA em que o ponteiro avança um slot por transação sem CPU (`RX_POSTINC` na nrfx) |
| wrap | devolver o ponteiro ao slot 0. Feito na ISR do evento `DMA.RX.READY` (`STARTED` no nRF5340), que a drenagem habilita uma vez por T |
| prazo do wrap | tempo que a ISR tem para escrever o ponteiro: do `READY` da transação que acabou de começar até o próximo `START`, ou seja, um período. "Margem" é sempre tempo |
| espera acordada | quando chegaram ≥ 32 amostras na última drenagem, a thread não dorme depois de armar o wrap: espera por ele acordada (no máximo T/4). Evita que a IRQ de wrap pague o wake-up de idle (17 µs no M33 do nRF54L15) dentro de um período curto |
| `late_wraps` (`late` no log) | wraps escritos depois de o próximo `START` já ter ocorrido: a transação que já tinha começado usou o slot seguinte ao último (a folga) e não entra na fila (uma amostra perdida, ordem preservada, sem corrupção); o contador é a detecção |
| `overflows` (`ovf` no log) | drenagens que encontraram o head além do anel: T longo demais para o anel; os 8 slots de guarda absorvem a escrita |
| `fresh` | bit de data-ready lido no `STATUS` da própria rajada. Heurístico: com leituras espaçadas menos de ~100 µs (M, ADXL362; depende do sensor) o sensor ainda não limpou o bit |
| `queued`, `dropped`, `skipped` | amostras que passaram pela fila no período (postas pela drenagem e lidas pelo consumidor, iguais quando `dropped = 0`); que não couberam na fila; descartadas como repetidas pelo filtro (caso 2) |
| `xfers` | transações iniciadas desde a partida: laps completos × slots + head. Exato, sem contador em hardware |
| `STARTED` / `DMA.RX.READY` | evento de início de transação: `STARTED` no nRF5340, `DMA.RX.READY` no nRF54L (a nrfx o chama `RXSTARTED`). É o instante em que o hardware liberou o ponteiro para a próxima escrita; é a única IRQ da SPIM usada, e só quando armada |
| DPPI, EEP → TEP, GPPI | interconexão de periféricos: um evento (EEP) publica num canal e uma tarefa (TEP) assina o canal. GPPI é a camada da nrfx que aloca canais e, no nRF54L15, as pontes PPIB entre domínios |
| MCU / PERI / LP | domínios de potência do nRF54L15: SPIM00 e TIMER00 em MCU; SPIM2x, TIMER2x e GPIOTE20 em PERI; SPIM30 e GPIOTE30 em LP |
| SPIM2x | SPIM20, 21 e 22: mesmo domínio, clock e custo; 20 e 21 também têm pinos dedicados no P2 |
| PPIB | ponte DPPI entre domínios do nRF54L15; latência entre domínios não especificada no datasheet |
| GRTC | contador de tempo real global do nRF54L15 (LP), o relógio de sistema do Zephyr; é ele que acorda a thread de drenagem |
| `CSNDUR` | `IFTIMING.CSNDUR`: tempo entre CSN e SCK e tempo mínimo de CSN inativo, em ciclos do clock do core da SPIM (16 MHz nas SPIM2x/30, 32 MHz na SPIM4 do nRF5340) |
| RRAM standby | no nRF54L15 a RRAM (memória de código) desliga em idle e a primeira instrução de uma ISR espera 13 µs (D, `tIDLE2CPU`); `APP_RRAM_STANDBY` (bancada) a mantém em standby. Não é mais necessária para o wrap |
| FLPR / VPR | coprocessador RISC-V do nRF54L15, executa da RAM; VPR é o bloco de hardware que o contém |
| `hfxo_launcher` | imagem do app core que sobe o FLPR e pede o HFXO; substitui o `vpr_launcher` padrão no exemplo TIMER |
| HFXO / HFINT | cristal de 32 MHz e oscilador RC interno de alta frequência; um TIMER só é exato no HFXO (o HFINT desvia ~0,2 %, M) |
| constant latency | sub-modo de potência do nRF54L15 que mantém recursos ligados em idle; 0,55 mA (D) e, medido, não corrige a latência da ISR sozinho |
| CSN, CPHA | chip select (aqui gerado pela SPIM, não por GPIO) e fase do clock SPI (modos 0/1 = CPHA 0, modos 2/3 = CPHA 1) |
| errata 8 | nRF54L: com CPHA = 0, `PRESCALER > 2` e primeiro bit do comando em 1, o MOSI sai errado; o workaround da nrfx precisa de uma escrita por transação, impossível com DPPI. Saída: 8 MHz nas SPIM2x, ou comando com MSB 0, ou CPHA = 1 |
| TAG | nRF54L15 TAG, placa pública da Nordic (Zephyr `nrf54l15tag`) com BMI270 na SPIM22; sem UART, log por RTT |
| 1,5 µs | custo fixo por transação na fórmula t = bytes × 8 / SCK + 1,5 µs: `START` até o primeiro SCK mais CSN, medido como 18,5 − 17 µs no nRF54L15 (M); `CSNDUR` maior acrescenta ciclos do clock do core da SPIM |

Vocabulário fixo: "transação" para o evento SPI, "amostra" para o valor,
"período" para o intervalo entre transações, "T" para o período de
drenagem, "prazo do wrap" ou "margem" só para tempo, "ocupação" e "sobra de
barramento" para percentuais, "folga" para o slot que uma transação usa
quando o wrap atrasa, "guarda" para os 8 slots depois do anel, "bancada"
para os testes e "consumo" para corrente. Cada tabela declara a unidade de
taxa (transações/s ou amostras/s; no caso 1 são iguais). `xfers` e `STARTs`
só aparecem ao falar do log.

## Três decisões

**TL;DR: quem dispara, de quanto em quanto tempo a CPU drena o anel, e
onde roda. Toda amostra vai para a fila; T = 10 ms é o padrão de consumo,
T ≈ período do sensor o de latência; a terceira decisão só entra por
barramento.**

1. **Quem dispara a transação.** O pino de data-ready do sensor (caso 1,
   `gpiote_dppi_spim`, recomendado: uma transação por amostra, sem TIMER de
   disparo, sem HFXO, sem repetidas) ou um TIMER (caso 2, `timer_dppi_spim`:
   polling do sensor em hardware, para sensor sem pino ou taxa fixa; é
   também a bancada).
2. **T, o período de drenagem.** T = 10 ms (padrão): a thread acorda 100
   vezes por segundo, 200 interrupções/s, latência de entrega de 10 ms,
   anel de 256 slots suficiente até ~12 k/s (a 16 k e 50 k/s usa-se T =
   1 ms, ver [Consumo](#consumo--resumo)). T ≈ período do sensor: latência
   de uma amostra, uma drenagem por amostra, e 1,7× (1,6 k/s), 1,8×
   (16 k/s) ou 1,2× (50 k/s) o consumo do T longo (E). O Kconfig limita T a
   100 µs, então "uma amostra de latência" vale até 10 k/s.
3. **Onde roda**, quando importa: SoC, instância da SPIM (no nRF54L15
   SPIM2x, ou SPIM00 se o barramento não couber a 8 MHz) e core (Cortex-M33
   ou FLPR). Pesa no consumo da SPIM00 (+300 µA, E) e na arquitetura (o
   FLPR livra o M33). Não pesa mais no prazo do wrap: a espera acordada
   tira o wake-up do core do caminho.

**Consumo em três linhas (nRF54L15, caso 1, SPIM22, 11 B, E; tabela em
[Consumo](#consumo--resumo)):** T = 10 ms custa 45 µA a 1 600/s; T = 1 ms
custa 239 µA a 16 k/s e 707 µA a 50 k/s. Com latência de uma amostra
(T = 625 µs a 1 600/s; T = 100 µs, o mínimo, acima): 76 µA, 427 µA e
842 µA. O FLPR fica sem número: a amostra `vpr_offloading` da Nordic mediu
146 → 125 µA no nRF54L15 com ~1 k transações SPI/s (R), mas um relato de
DevZone dá +0,5 mA de idle do VPR noutra configuração (R). Só o PPK2
decide.

## Estrutura do repositório

| Diretório | Conteúdo |
|---|---|
| [`gpiote_dppi_spim/`](gpiote_dppi_spim/README.md) | Caso 1, recomendado: data-ready → GPIOTE IN → DPPI → SPIM. Como compilar, rodar e ler o log. |
| [`timer_dppi_spim/`](timer_dppi_spim/README.md) | Caso 2: TIMER → DPPI → SPIM. É também a bancada: varredura de período, teto do barramento, latência de ISR. |
| [`docs/`](docs) | Diagramas (`gen_diagrams.py` gera os SVG sem dependências) e [`POWER.md`](docs/POWER.md), o modelo de consumo. |
| [`tools/`](tools) | Scripts de gravação e captura de log por RTT. |

Os dois exemplos têm o mesmo engine (`src/spim_dppi.c`: anel, drenagem,
wrap), os mesmos backends de sensor e os mesmos overlays. Cada exemplo traz
só o seu disparo; o do TIMER acrescenta o filtro de repetidas, a captura de
latência e as opções de bancada.

## Caso 1 — data-ready → GPIOTE → DPPI → SPIM (recomendado)

**TL;DR: uma transação por amostra nova, sem TIMER de disparo e sem HFXO.
Medido até 1 600 Hz (ODR máximo do BMI270) com zero perdas (M).**

O pino INT do sensor vira um evento GPIOTE IN que, por DPPI, aciona
`SPIM.TASKS_START`. O CSN é do hardware e o EasyDMA grava a rajada em RAM.
É o único canal DPPI do exemplo.

![Blocos do caso 1](docs/blocos_caso1_sensor_int.svg)

*Quem liga em quem: o pino do sensor entra no GPIOTE, o DPPI leva o evento
à SPIM, o EasyDMA enche o anel; a thread de drenagem e a IRQ de wrap são a
única participação da CPU.*

![Timing do caso 1](docs/caso1_sensor_int.svg)

*Uma linha por sinal. Note que o data-ready só desce quando a rajada lê os
registradores de dados: o disparo seguinte depende da leitura anterior.*

**Partida e parada.** O data-ready é um nível, não um pulso: fica alto até
os dados serem lidos. Se o DPPI for ligado com o pino já alto, a borda de
subida nunca acontece. O exemplo dispara um `START` por software logo
depois de ligar o DPPI; a partir daí cada amostra nova gera a borda. Pelo
mesmo motivo, se uma borda se perder (o `START` chega com a SPIM ocupada,
um glitch), a aquisição para de vez com o pino alto: em produto vale um
watchdog que dispara um `START` por software quando `xfers` não avança
(não implementado nos exemplos).

Resultados no ODR máximo de cada sensor (M, transações/s = amostras/s,
T = 10 ms, anel de 256 slots; logs em `gpiote_dppi_spim/test-logs/`):

| Alvo | Sensor | ODR | SCK | Transações/s | queued = fresh | dropped | late | ovf | Log |
|---|---|---|---|---|---|---|---|---|---|
| TAG M33 | BMI270 | 1 600 Hz (D, máx.) | 8 MHz | 1 607–1 609 | sim | 0 | 0 | 0 | `u_tag_int_drain10ms_1600.log` |
| TAG FLPR | BMI270 | 1 600 Hz | 8 MHz | 1 607–1 622 (alternando: granularidade do relógio de uptime do FLPR) | sim | 0 | 0 | 0 | `u_tag_flpr_int_drain10ms_1600.log` |
| Thingy:53 M33 | ADXL362 | 400 Hz (D, máx.; real ≈ 380) | 4 MHz | 372–374 | sim | 0 | 0 | 0 | `u_thingy_int_drain10ms.log` |

Os 1 607–1 609/s são o ODR real do BMI270 desta unidade (tolerância do
oscilador do sensor); os 372–374/s da Thingy são o ODR real do ADXL362,
como nas medições anteriores (≈ 380). Imagens: TAG M33 50 168 B de flash e
27 616 B de RAM; TAG FLPR 29 292 B e 5 768 B (M).

## Caso 2 — TIMER → DPPI → SPIM

**TL;DR: polling do sensor em hardware, para sensor sem pino de data-ready
ou taxa fixa. Timer ≥ 1,05 × ODR e filtro de repetidas; abaixo do ODR real
perde amostras sem aviso (M).**

Um TIMER dispara a SPIM em período fixo. Ler os mesmos registradores em
loop basta, porque eles sempre guardam a última amostra; o bit de data-ready
no `STATUS`, lido na mesma rajada (`fresh`), separa amostras novas de
repetidas. Custo em relação ao caso 1: um TIMER de disparo, o HFXO para
período exato e as repetidas, que precisam ser filtradas
(`APP_QUEUE_FRESH_ONLY`, na drenagem). Limite: o filtro `fresh` deixa de
ser confiável com leituras espaçadas menos de ~100 µs (M, ADXL362), então
acima de ~10 k transações/s o caso 2 não garante "só amostras novas"; os
tetos medidos com ele são tetos de barramento, não contagens de amostras
novas. Um sensor sem data-ready acima de ~10 k/s não tem estratégia limpa
aqui: aceitar repetidas ou usar a FIFO do sensor. E um timer a 1,05 × ODR
para um sensor de 64 kHz (14,8 µs) não cabe com 11 B a 8 MHz.

![Blocos do caso 2](docs/blocos_caso2_timer.svg)

*O TIMER ocupa o lugar do GPIOTE; o sensor não participa do disparo. A
drenagem é a mesma do caso 1, com o filtro de repetidas.*

![Timing do caso 2](docs/caso2_timer.svg)

*Timer a 100 µs contra sensor a 400 Hz, para mostrar as repetidas: só uma
em cada 25 rajadas traz amostra nova.*

Timer × ODR, medido na TAG com o BMI270 a 402/s reais (M, varredura de
2 500 a 2 200 µs, `bench/sweep-tag.conf`; medido com o mecanismo de entrega
anterior, log não incluído): timer a 400/s perde cerca de 2 amostras/s
**sem deixar rastro**; a 408/s (1,5 % acima do ODR real, 2 % acima do
nominal) não perde nenhuma. Regra de projeto: timer acima do ODR nominal
pela tolerância máxima do oscilador que o datasheet do sensor declarar, com
5 a 10 % como valor típico.

![Timer × ODR](docs/timer_vs_odr.svg)

*Dois relógios livres: abaixo do ODR real a perda é silenciosa; acima, as
repetidas aparecem como `skipped`.*

## Anel, drenagem e wrap

**TL;DR: anel preenchido pelo EasyDMA; uma thread acorda a cada T, entrega
à fila os slots completos e habilita uma vez a IRQ de `READY`; essa ISR
devolve o ponteiro ao slot 0 dentro da janela do datasheet. Sem contador,
sem EGU, sem ISR zero-latency.**

- **Anel de `APP_RING_SLOTS` + 8 slots.** O EasyDMA em array list avança um
  slot por transação sozinho. Os 8 slots de guarda recebem a escrita se
  uma drenagem atrasar tanto que o head passe do fim do anel (contado em
  `overflows`), em vez de corromper o que vier depois. Cada item da fila é
  a rajada crua de B bytes. RAM = (slots + 8) × B mais a fila × B: com os
  padrões (256 + 8 e 256) são 8,8 KB para 17 B e 5,7 KB para 11 B (E).
- **Drenagem, a cada T.** A thread (prioridade cooperativa) acorda por
  `k_sleep`, lê o head em `DMA.RX.PTR` e entrega à fila os slots
  `[tail, head − 1)`: o slot head − 1 pode estar em curso e fica para a
  drenagem seguinte. `xfers` sai daí: laps completos mais o head, exato.
- **Wrap na IRQ de `READY`.** Depois de entregar, a drenagem habilita uma
  vez a interrupção do evento `DMA.RX.READY` (nRF54L) ou `STARTED`
  (nRF5340). Na transação k seguinte o hardware já apontou para o slot
  k + 1 e liberou o registrador; a ISR escreve `PTR = slot 0`, guarda o
  índice do último slot do lap (`wrap_last`) e desabilita a IRQ. A
  transação k + 1 escreve o slot 0. A drenagem seguinte entrega primeiro o
  resto do lap antigo até `wrap_last`, depois o lap novo desde o slot 0.
- **Prazo do wrap = um período.** O datasheet dos dois SoCs diz que o
  ponteiro é double-buffered e pode ser escrito "imediatamente após
  STARTED"; o nRF54L tem o evento explícito `DMA.RX.READY`. Escrever perto
  do `END`, como uma versão anterior fazia, colide com a atualização do
  ponteiro pelo hardware no `START` seguinte (Achado 7 do
  `gpiote_dppi_spim`). Se um segundo `READY` chegar entre a entrada da ISR e
  a escrita, a transação seguinte já consumiu o slot k + 1 (a folga):
  `late_wraps++`, uma amostra fora da fila, sem corrupção.
- **Espera acordada nas taxas altas.** Os acordares da thread são
  agendados (`k_sleep`), e o Zephyr acorda a RRAM antes deles
  (`CONFIG_NRF_SYS_EVENT_IRQ_LATENCY`), então a drenagem não paga os 13 µs
  da RRAM. A IRQ de wrap pagaria: se o core dormir entre a drenagem e o
  `READY` seguinte, a ISR entra com ~17 µs no M33 do nRF54L15 (M, 16,7 µs
  máx. a 40 µs de período), mais que um período nas taxas altas. Por isso,
  quando chegaram ≥ 32 amostras na drenagem (`SPIN_WRAP_MIN_ARRIVED`), a
  thread espera pelo wrap acordada, no máximo T/4: a latência medida cai
  para 1,5–2,4 µs (M). Em taxas baixas a IRQ vem de idle e o core dorme;
  os 17 µs não incomodam com período ≥ 40 µs.

![Anel, drenagem e wrap](docs/anel_drenagem.svg)

*8 slots para caber no desenho: a drenagem lê o head, entrega os slots
completos e arma a IRQ de `READY`; a ISR devolve o ponteiro ao slot 0 e a
transação seguinte já escreve lá.*

Contadores do log: `xfers` (transações iniciadas), `queued` (amostras que
passaram pela fila), `fresh` (com data-ready ativo), `skipped` (repetidas
descartadas pelo filtro, caso 2), `dropped` (fila cheia), `late`
(`late_wraps`) e `ovf` (`overflows`). Teste bom: `queued = fresh` no
caso 1, `dropped = 0`, `late = 0`, `ovf = 0`.

## Limites

### Teto do barramento

**TL;DR: t = bytes × 8 / SCK + 1,5 µs (E); 1/t é o teto, otimista em até
10 % (54 k previsto, 52,6 k medido; 80 k previsto, 71,4 k medido). Medido
52,6 k/s (17 B) e 71,4 k/s (11 B) a 8 MHz (M).**

| Rajada | SCK | Transação (E, fórmula) | Ocupação a 64 k/s |
|---|---|---|---|
| 11 B | 8 MHz | 12,5 µs | 80 % |
| 11 B | 16 MHz | 7,0 µs | 45 % |
| 11 B | 32 MHz | 4,25 µs | 27 % |
| 17 B | 8 MHz | 18,5 µs | acima de 100 % (teto 52,6 k/s) |

| Alvo | Rajada | SCK | Último período válido | Transações/s | Acima do teto | Fonte |
|---|---|---|---|---|---|---|
| TAG M33, SPIM22 | 17 B | 8 MHz | 19 µs | **52,6 k** | a contagem de `DMA.RX.READY` continua (55,5–62,5 k/s), com dados congelados (`fresh = 0`); contando `STARTED` (`APP_COUNT_STARTED`) a contagem fica em 0: a SPIM para | M, `u_tag_busmax.log` |
| Thingy:53 M33, SPIM4 | 11 B | 8 MHz | 14 µs | **71,4 k** | o `START` reinicia a transação em curso; a contagem continua (83–100 k/s), os dados congelam | M, `u_thingy_bus64k.log` |

![Teto do barramento](docs/teto_barramento.svg)

*A 19 µs a transação de 17 B cabe; a 18 µs o `START` chega com a SPIM
ocupada.*

Os 71,4 k/s são da SPIM4 do nRF5340. Para a SPIM2x do nRF54L15 com 11 B a
fórmula dá 80 k/s e o medido no nRF5340 sugere ≈ 71–80 k/s (E), não
medido: a TAG só tem sensor de 17 B. O 1,5 µs fixo é `START` até o
primeiro SCK mais CSN, medido como 18,5 − 17 µs no nRF54L15 (M). O teto
no FLPR não foi remedido com o mecanismo atual.

### Prazo do wrap × wake-up do core

**TL;DR: o wrap tem um período de prazo e, acima de 32 amostras por
drenagem, é esperado com o core acordado: 1,5–2,4 µs (M) até os tetos dos
dois SoCs, sem ZLI, sem RRAM em standby, sem FLPR. O limiar de 18 µs só
existiria se a IRQ de wrap chegasse com o core dormindo, o que o engine
evita por conta própria.**

No nRF54L15 a latência de uma IRQ com o core em idle é 16,8 µs (M) = 13 µs
de RRAM em power-down (D, `tIDLE2CPU`) + ~4 µs de DPPI, IRQ e entrada da
ISR (M). Com o core acordado são 1,5–2,6 µs (M). Na bancada do teto
(`bench/bus-max-tag.conf`, T = 1 ms) a 40 µs de período chegam 25 amostras
por drenagem, abaixo do limiar de 32: a IRQ de wrap vem de idle e a
latência medida é 1,5 mín. / 12,8 média / 16,7 µs máx., inofensiva porque
40 > 17. De 25 µs para baixo a thread espera acordada: 1,6–2,0 µs de média
e 2,4 µs de máximo, `late_wraps = 0` até o teto de 52,6 k/s (M). O
nRF5340 não tem RRAM, mas o wake-up de idle também passou do período:
antes da espera acordada a mesma bancada dava 58 a 352 `late_wraps` por
passo de 7 s entre 25 e 14 µs; com ela, zero em todos os passos, dados
válidos até 14 µs (M, `u_thingy_bus64k.log`). Nenhum dos dois SoCs precisa
mais de zero-latency IRQ.

O que sobra para `APP_RRAM_STANDBY` e para o FLPR: latência determinística
de outras ISRs do produto e, no FLPR, livrar o M33. O custo de CPU da
espera acordada é de no máximo um período por drenagem (E, incluído no
modelo de consumo).

### nRF54L15: Cortex-M33 × FLPR

**TL;DR: mesmo código nos dois cores. A latência de uma IRQ saindo de
idle é 16,8 µs no M33 padrão, 2,75 µs com RRAM em standby e 2,43 µs no
FLPR (M); o consumo do FLPR não foi medido aqui.**

![M33 × FLPR](docs/m33_vs_flpr_nrf54l15.svg)

*Latência do disparo até a ISR de wrap na TAG; barras cheias são a média,
claras o máximo.*

| Latência de uma IRQ (`bench/n1-tag.conf`, `APP_WRAP_LATENCY_STATS`) | M33 padrão | M33 + RRAM standby | FLPR | Fonte |
|---|---|---|---|---|
| Disparo → ISR de wrap, core em idle | 16,8 µs (máx. 17,3) | 2,75 µs (máx. 2,93) | 2,43 µs (máx. 2,50) | M, mecanismo anterior, log não incluído |
| Com o core acordado | 1,7–2,6 µs (média 2,3) | 1,7–2,3 µs | 2,43 µs, constante | M, idem |
| Constant latency (`SOC_NRF_FORCE_CONSTLAT`) sozinho | 16,8 µs, não resolve | — | — | M, idem |
| Mecanismo atual, IRQ de idle (40 µs de período) | 1,5 / 12,8 / 16,7 µs (mín. / média / máx.) | — | — | M, `u_tag_busmax.log` |
| Mecanismo atual, espera acordada (≤ 25 µs) | 1,6–2,0 µs média, 2,4 máx. | — | — | M, `u_tag_busmax.log` |

Os números de idle e de core acordado foram medidos com o mecanismo de
entrega anterior (uma IRQ por amostra), pela mesma cadeia DPPI → `START` →
`READY` → IRQ; continuam válidos como latência de uma IRQ. Naquela
bancada o M33 sustentou uma IRQ por amostra até 50 k/s (máximo varrido) e
o FLPR até 40 k/s (a 50 k/s ≈ 40 % dos wraps atrasavam); no mecanismo
atual não há IRQ por amostra.

| Caso 1 a 1 600 Hz (T = 10 ms) | M33 | FLPR | Fonte |
|---|---|---|---|
| Transações/s | 1 607–1 609 | 1 607–1 622 | M |
| TIMER exato (caso 2) | pede o HFXO no próprio app | precisa do `hfxo_launcher` no app core | M |
| Onde o código roda | RRAM (imagem de 50 KB) | RAM (29 KB de código, 5,8 KB de dados) | M |

Quando o FLPR compensa: latência determinística sem tocar a RRAM e liberar
o M33 para a pilha de rádio. Custa o `hfxo_launcher` para timers exatos.
Consumo: sem número. O FLPR evita o wake-up da RRAM por rodar da RAM (a
amostra `vpr_offloading` da Nordic mediu 146 → 125 µA no nRF54L15 com ~1 k
transações SPI/s, R), mas um relato de DevZone dá +0,5 mA de idle do VPR
noutra configuração (R); a corrente ativa do FLPR não é publicada. Próximo
passo: PPK2 no caso 1 com FLPR e M33. A 1 600 Hz os dois cores ficam
ociosos e a escolha é de arquitetura.

## Como escolher

**TL;DR: um fluxo de quatro passos com as fórmulas; quatro exemplos
resolvidos abaixo, cada um com o consumo.**

Entradas: ODR (Hz), B (bytes da rajada, com o comando), SCK máximo do
sensor, latência de entrega aceitável, SoC.

```
1. Disparo
   Sensor tem pino de data-ready?  sim -> gpiote_dppi_spim (caso 1)
                                    não -> timer_dppi_spim, período = 1/(ODR × 1,05..1,10),
                                           APP_QUEUE_FRESH_ONLY=y (caso 2)
2. Cabe no barramento?
   t_trans = B × 8 / SCK + 1,5 µs (E)      (a 8 MHz: B + 1,5 µs)
   t_per   = 1/ODR (caso 1) ou o período do timer (caso 2)
   sobra = t_per − t_trans
   sobra >= 10 % de t_per ou >= 2 µs -> ok (E, ancorado no medido: o nRF5340 deu dados
                                        válidos a 89 % de ocupação, 11 B a 14 µs)
   sobra menor                       -> subir o SCK: nRF5340 SPIM4 a 16/32 MHz;
                                        nRF54L15 só SPIM00 a 32 MHz (não testado; errata 8 se o
                                        primeiro byte do comando tiver o bit mais significativo em 1)
3. T, o período de drenagem (APP_DRAIN_PERIOD_US) e o anel
   Latência de entrega aceitável = T. Latência de uma amostra -> T ≈ t_per (mínimo 100 µs).
   Senão -> T = 10 ms (padrão); interrupções/s = 2/T, qualquer que seja a taxa.
   Amostras por drenagem = taxa × T; APP_RING_SLOTS >= 2 × isso (jitter e atraso de escalonamento),
   máximo 4 096. RAM = (APP_RING_SLOTS + 8) × B + APP_QUEUE_DEPTH × B; fila >= amostras por drenagem
   mais o atraso do consumidor.
   Referência: 1 600/s e T = 10 ms -> 16 por drenagem, anel 256, 200 IRQ/s.
               64 k/s e T = 1 ms -> 64 por drenagem, anel 256 (512 na bancada), 2 000 IRQ/s.
               50 k/s e T = 10 ms pediria anel de 1 000: use T = 1 ms.
4. Onde roda
   Prazo do wrap: resolvido pelo engine (espera acordada quando chegam >= 32 amostras por
   drenagem; medido 0 late_wraps até 52,6 k/s no nRF54L15 e 71,4 k/s no nRF5340). Nenhum core,
   RRAM standby ou ZLI é exigido por ele.
   Instância no nRF54L15: SPIM2x (M); SPIM00 só se o passo 2 mandar subir o SCK (E, não testado).
   FLPR: para livrar o M33 ou por latência de outras ISRs; consumo não medido.
```

### Exemplos resolvidos

Consumo pela tabela de [Consumo](#consumo--resumo) (E):

- **BMI270, 1 600 Hz, 17 B, nRF54L15.** Caso 1. t_trans = 18,5 µs contra
  t_per = 625 µs: 3 % de ocupação. T = 10 ms (16 amostras por drenagem,
  anel 256, 200 IRQ/s, 10 ms de latência), M33 padrão, SPIM22. Medido:
  1 607–1 609/s no M33 e 1 607–1 622/s no FLPR, zero perdas (M). Consumo:
  linha "T = 10 ms, SPIM22" a 1 600/s, 45 µA, mais 0,25 mA × 1 600 × 6 µs
  = 2 µA pelos 17 B: ≈ 47 µA (E). Com latência de uma amostra (T = 625 µs):
  ≈ 78 µA (E).
- **ADXL382, 64 kHz, 11 B, latência de 1 ms, nRF54L15.** Caso 1. 80 % na
  SPIM2x a 8 MHz (cabe, sem sobra para `CSNDUR` maior ou jitter); SPIM00 a
  32 MHz, 27 % (não testado; a errata 8 não atinge o comando `0x23`).
  T = 1 ms (64 amostras por drenagem, anel 256, 2 000 IRQ/s); a drenagem
  espera acordada pelo wrap (64 ≥ 32). Consumo: ≈ 0,87 mA na SPIM22,
  ≈ 1,18 mA na SPIM00 (E, `docs/POWER.md`, seção ADXL382). Não testado em
  hardware; o perfil de barramento foi medido no nRF5340 a 71,4 k/s.
- **ADXL382, 64 kHz, 11 B, nRF5340.** Caso 1. t_trans = 12,5 µs contra
  15,6 µs: 80 %, cabe sem sobra; SPIM4 a 16 MHz dá 45 % (a confirmar o SCK
  máximo no datasheet do ADXL382 e os pinos de alta velocidade da placa).
  T = 1 ms. Consumo: ≈ 1,7 mA a 16 MHz, ≈ 2,2 mA a 8 MHz, só o SoC (E,
  tabela do nRF5340 em `docs/POWER.md`). Não testado em hardware.
- **Sensor sem data-ready a 16 kHz, 11 B, nRF54L15.** Caso 2, timer a
  1,05 × 16 kHz = 59,5 µs: 21 % de ocupação a 8 MHz, T = 1 ms (17 por
  drenagem), M33 padrão. O filtro `fresh` a 59,5 µs de espaçamento já não é
  confiável (M, ADXL362 a 25 µs marcou 533 novas/s para ≈ 380 reais):
  aceitar repetidas ou usar a FIFO do sensor. Consumo: T = 1 ms na SPIM22
  a 16 k/s, 239 µA, mais 135 µA de TIMER e HFXO no lugar do GPIOTE
  (155 − 20): ≈ 0,37 mA (E).

## Consumo — resumo

**TL;DR: modelo, sem PPK2. O custo fixo caiu para o domínio PERI (20 µA,
R) mais a base; a CPU custa ~8 µs por drenagem e 3,5 µs por amostra (E).
T curto custa 1,2× a 1,8× o T longo; a SPIM00 soma ~300 µA constantes.
Modelo completo e premissas em [`docs/POWER.md`](docs/POWER.md).**

![Consumo por T, instância e taxa](docs/consumo_modos_nrf54l15.svg)

*Corrente média do SoC contra amostras/s para o T padrão (10 ms; 1 ms a
16 k e 50 k/s) e para o T mais curto (o período do sensor a 1 600/s; o
mínimo de 100 µs acima), SPIM22 e SPIM00 do nRF54L15 (E).*

Caso 1 (data-ready), rajada de 11 bytes, toda amostra na fila, Cortex-M33
padrão. SPIM22 a 8 MHz (12,5 µs por transação), SPIM00 a 32 MHz (4,25 µs).
Corrente média do SoC em µA (E). Premissas: base 2,9; domínio PERI mantido
pelo GPIOTE IN 20 µA (R); domínio MCU 300 (E) só na SPIM00; SPIM ativa
0,25 mA (SPIM2x) ou 0,8 mA (SPIM00) (E); CPU 2,6 mA (D) × [por drenagem:
5 µs de acordar + 3 µs de IRQ de wrap + um período de espera acordada
quando chegam ≥ 32 amostras por drenagem; por amostra: 1 µs de `k_msgq_put`
+ 2,5 µs de consumidor] (E).

| T | Instância (SCK) | 1 600/s | 16 k/s | 50 k/s |
|---|---|---|---|---|
| 10 ms (1 600/s), 1 ms (16 k e 50 k/s) | SPIM22 (8 MHz) | **45** | **239** | **707** |
| 10 ms, 1 ms | SPIM00 (32 MHz) | 345 | 544 | 1 021 |
| período (625 µs), 100 µs (mínimo) | SPIM22 | 76 | 427 | 842 |
| período, 100 µs | SPIM00 | 376 | 731 | 1 156 |
| qualquer, FLPR | SPIM22 | sem número (E) | sem número (E) | sem número (E) |

Notas da tabela: a 16 k e 50 k/s o T = 10 ms pediria anel de 320 e 1 000
slots (2 × as amostras por drenagem), por isso a linha usa T = 1 ms, que é
o T da bancada. T = período abaixo de 100 µs está fora do range do Kconfig,
e a 50 k/s com T = 100 µs chegam 5 amostras por drenagem, abaixo do limiar
da espera acordada: a IRQ de wrap viria de idle (17 µs) contra 20 µs de
período, margem não medida; para 50 k/s use T ≥ 1 ms. Rajada de 17 bytes:
somar 0,25 mA × taxa × 6 µs na SPIM22 ou 0,8 mA × taxa × 1,5 µs na SPIM00,
só onde o barramento ainda cabe (17 B a 50 k/s na SPIM22 são 92 % de
ocupação, fora do critério). PERI 20 µA (R) é a premissa de menor
confiança, mas desloca todas as linhas por igual. O modelo não conta os
13 µs de RRAM antes da IRQ de wrap em idle (o core está parado, não ativo).

Três leituras:

1. **T curto custa 1,7× (1,6 k/s), 1,8× (16 k/s) e 1,2× (50 k/s) o T
   longo.** A diferença é só o número de drenagens: ~8 µs de CPU cada
   (acordar e IRQ de wrap). O custo por amostra (3,5 µs) é o mesmo. A
   50 k/s a espera acordada (um período de 20 µs por drenagem a T = 1 ms)
   já está incluída: 20 µs × 1 000/s = 2 % de CPU.
2. **A SPIM00 custa ~300 µA a mais em qualquer taxa** (o domínio MCU
   ligado, E). Só paga pela sobra de barramento: SCK acima de 8 MHz, rajada
   longa a 64 k/s, ou taxa acima de ≈ 71–80 k/s (E). Nunca por consumo.
3. **Não há mais custo fixo de contador (121 µA) nem EGU.** Abaixo de
   ~5 k/s o SoC fica em dezenas de µA: a 1 600/s, 45 µA, dos quais 20 são o
   domínio PERI e 17 a CPU. O FLPR fica sem número (E): a amostra
   `vpr_offloading` da Nordic mediu 146 → 125 µA (R), um relato de DevZone
   dá +0,5 mA de idle do VPR (R); só o PPK2 decide.

Regra que sai da tabela: data-ready + T = 10 ms na SPIM22, salvo se a
latência de uma amostra for requisito (então T ≈ período, até 10 k/s) ou o
barramento não couber a 8 MHz (então SPIM00). Próximo passo: medir com
PPK2 FLPR contra M33 no caso 1.

nRF5340 (SPIM4, caso 1, 11 B, E, sem HFXO): a 1 600/s e T = 10 ms
≈ 0,10 mA; a 64 k/s e T = 1 ms ≈ 2,2 mA a 8 MHz ou 1,7 mA a 16 MHz; com
T = 100 µs ≈ 2,4 ou 1,9 mA. Tabela em `docs/POWER.md`.

## nRF54L15: qual SPIM

**TL;DR: SPIM2x no caso geral (testada). SPIM00 só se o barramento não
couber a 8 MHz, em teoria: a TAG não a alcança. SPIM30 (LP) é uma variante
100 % LP a um overlay de distância, não testada.**

Fatos de hardware (D):

| Instância | Domínio | Clock do core | SCK máx. | Pinos | DPPI |
|---|---|---|---|---|---|
| SPIM00 | MCU | 128 MHz | 32 MHz (`PRESCALER` 4..126) | P2, dedicados, drive E0/E1 para 32 MHz | DPPIC00, 8 canais |
| SPIM20/21/22 | PERI | 16 MHz | 8 MHz (`PRESCALER` 2..126) | P1 (20/21 também P2) | DPPIC20, 16 canais |
| SPIM30 | LP | 16 MHz | 8 MHz | P0 | DPPIC30, 4 canais |

Consequências:

- **SPIM2x (M).** Mesmo domínio do GPIOTE20: caminho inteiro sem PPIB. É o
  que os exemplos usam. As três instâncias têm o mesmo clock e o mesmo
  custo.
- **SPIM00 (E, não testado).** 4× o barramento: 17 B em 4,25 µs. O disparo
  vem de PERI pelo PPIB01/PPIB21 (latência não especificada; wake-up se o
  domínio dormir). A errata 8 vale para todo o prescaler dela (mínimo 4),
  mas só atinge comandos cujo primeiro byte tem o bit mais significativo em
  1: com CPHA = 0 e esse bit em 1 o workaround da nrfx exige uma escrita por
  transação, incompatível com disparo por DPPI. O `0x83` do BMI270 é
  afetado; o `0x0B` do ADXL362 e o `0x23` do ADXL382 não. O BMI270 aceita
  modo SPI 3 (CPHA = 1), o que contornaria a errata na SPIM00, não testado.
  Faz sentido para rajada longa a 64 k/s, taxa acima do que 8 MHz alcança
  (≈ 71–80 k/s com 11 B, E), sensor que exige SCK > 8 MHz, ou domínio MCU
  já ligado por outro motivo. Nunca por consumo.
- **SPIM30 (LP, P0): variante 100 % LP.** Com o mecanismo atual nada em
  PERI é obrigatório além do GPIOTE do pino de data-ready: não há contador
  nem EGU. Com o sensor em pinos do P0 (GPIOTE30 + SPIM30, DPPIC30) o
  caminho inteiro fica no domínio LP, junto com o GRTC que acorda a
  drenagem, e o domínio PERI pode dormir. É um overlay (`chosen app,accel`
  sob `spi30`, `pinctrl`, `cs-gpios`, `int1-gpios` no P0), não testado:
  exige o nRF54L15 DK com o sensor ligado por fio. Ganho esperado: os
  20 µA (R) do PERI; não medido.

**O que a TAG permite.** Só a SPIM22: o BMI270 está em P1.05/06/08, CSN
P1.07, INT P1.04, e os pontos de teste expostos (P0.01, P0.02, P0.04,
P1.02, P1.03, P1.13, P1.14, P2.05, P2.06, P2.07) não formam os quatro pinos
de SPIM00 (SCK P2.01/06, SDO P2.02/08, SDI P2.04/09, CSN P2.05/10) nem de
SPIM30 (quatro pinos no P0). O teste fica para o nRF54L15 DK: SPIM00 nos
pinos P2.06/08/09/10, conectados por padrão; SPIM30 em P0.00–P0.03,
desconectando a UART0 do depurador no Board Configurator, com P0.04 como
INT. No código muda só o overlay; a GPPI da nrfx 4.0 resolve o PPIB
sozinha.

Consumo por instância: tabela em [Consumo](#consumo--resumo); quando a
SPIM00 faz sentido, em [`docs/POWER.md`](docs/POWER.md#quando-a-spim00-faz-sentido).

## Caso de alta taxa: ADXL382 a 64 kHz (não testado em hardware)

**TL;DR: o caminho do caso 1 serve sem mudar a arquitetura; o que muda é o
backend do sensor, o SCK e o T. Tudo aqui é E.**

O ADXL382 gera data-ready a 16, 32 ou 64 kHz.

![ADXL382 a 64 kHz](docs/caso_adxl382_64k.svg)

*Timing esperado a 64 kHz: a 8 MHz a rajada de 11 B ocupa 80 % do período,
a 16 MHz 45 %, a 32 MHz 27 %. O wrap sai na IRQ de `READY` com o core
acordado, uma vez por drenagem.*

Ocupação e wrap:

| Instância | SCK | Transação (E) | Ocupação | Wrap | Observação |
|---|---|---|---|---|---|
| nRF5340 SPIM4 | 8 MHz | 12,5 µs | 80 % | IRQ de `STARTED`, core acordado | mesmo perfil medido a 71,4 k/s com o ADXL362, 0 `late_wraps` (M); cabe, sem sobra para jitter ou `CSNDUR` maior |
| nRF5340 SPIM4 | 16 MHz | 7,0 µs | 45 % | idem | recomendado, a confirmar o SCK máximo do ADXL382 e os pinos de alta velocidade da placa |
| nRF54L15 SPIM2x | 8 MHz | 12,5 µs | 80 % | IRQ de `DMA.RX.READY`, core acordado, 1,5–2,4 µs (M, 17 B até 19 µs) | cabe, sem sobra; não testado com 11 B |
| nRF54L15 SPIM00 | 32 MHz | 4,25 µs | 27 % | idem | sobra; não testado. Errata 8 não se aplica ao ADXL382 (primeiro byte `0x23`, MSB 0) |

O que trocar no `gpiote_dppi_spim`:

1. **Backend `src/sensor_adxl382.c`** (item novo na `choice APP_SENSOR`,
   `target_sources_ifdef` no CMake). Diferenças para o ADXL362: o primeiro
   byte é `(endereço << 1) | 1` para leitura, com auto-incremento, sem o
   comando `0x0B`; rajada `burst_tx = { (0x11 << 1) | 1 }` = `0x23`, 11 bytes
   (`STATUS0..ZDATA_L`, 0x11..0x1A); `fresh_offset = 1`, `fresh_mask = 0x01`
   (`STATUS0.DATA_READY`); dados big-endian em `rx[5..10]`; `mg_per_lsb`
   pela faixa (±15 g: 2000 LSB/g); `init()` lê `DEVID_AD` (0x00 = 0xAD) e
   põe o modo de alto desempenho com ODR 64 kHz em `OP_MODE` (0x26; código
   do ODR a confirmar no datasheet, que não está no repositório);
   `enable_drdy_int()` mapeia `DATA_READY` no INT0.
2. **Overlay**: nó `adxl382@0` sob `spi4` (nRF5340) com `int1-gpios` e
   `spi-max-frequency = <16000000>`. Não há binding `adi,adxl382` no NCS
   3.4.1: acrescentar `dts/bindings/adi,adxl382.yaml` no app.
3. **`APP_SPI_FREQ_HZ = 16000000`** na SPIM4 do nRF5340, a confirmar o SCK
   máximo no datasheet do ADXL382 e os requisitos de pino e clock da SPIM4
   a 16 MHz; no nRF54L15 a SPIM2x atende a 8 MHz (80 %, sem sobra) e só a
   SPIM00 passa de 8 MHz.
4. **T, anel e fila**: `APP_DRAIN_PERIOD_US = 1000` (64 amostras por
   drenagem, 2 000 IRQ/s, 1 ms de latência), `APP_RING_SLOTS = 256`
   (4 drenagens de folga; 512 como na bancada) e `APP_QUEUE_DEPTH ≥ 256`
   (256 itens de 11 B = 2,8 KB dão ao consumidor 4 ms de atraso tolerado;
   512 dão 8 ms). O wrap sai na IRQ de `READY` com o core acordado
   (64 ≥ 32 amostras por drenagem): sem ZLI, sem RRAM standby, nos dois
   SoCs. `late_wraps` e `overflows` no log confirmam.
5. **Verificação**: transações/s no log = 64 000 ± tolerância do oscilador
   do sensor, `fresh = queued`, `late_wraps = 0`, `ovf = 0`, Z variando
   (dados válidos).

Consumo estimado a 64 k amostras/s, só o SoC (E, [`docs/POWER.md`](docs/POWER.md)):

| SoC / instância | T = 1 ms | T = 100 µs (mínimo; 6,4 amostras de latência) |
|---|---|---|
| nRF5340 SPIM4, 8 MHz | ≈ 2,2 mA | ≈ 2,4 mA |
| nRF5340 SPIM4, 16 MHz | ≈ 1,7 mA | ≈ 1,9 mA |
| nRF54L15 SPIM22, 8 MHz | ≈ 0,87 mA | ≈ 1,01 mA |
| nRF54L15 SPIM00, 32 MHz | ≈ 1,18 mA | ≈ 1,33 mA |

No nRF5340 domina a SPIM (1,7 mA enquanto transfere, D); no nRF54L15 a
CPU (≈ 640 µA com T = 1 ms: 25 % do tempo, E) e o barramento. O consumo do
ADXL382 não está incluído.

## Achados

Os achados de silício e de bancada estão nos READMEs dos exemplos:
[`gpiote_dppi_spim`](gpiote_dppi_spim/README.md#achados) (data-ready em
nível, errata 8 da SPIM no nRF54L15, `RXDELAY` em ciclos de 16 MHz,
tempestade de IRQ da SPIM, GPIOTE compartilhado, escrita do ponteiro após
`READY` e nunca após `END`) e
[`timer_dppi_spim`](timer_dppi_spim/README.md#achados) (HFXO no FLPR,
wake-up da RRAM, wake-up do nRF5340 maior que o período, `fresh` acima do
ODR, log deferred, sysbuild).
