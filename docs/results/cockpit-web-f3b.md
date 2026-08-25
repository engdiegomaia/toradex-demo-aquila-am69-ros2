# Cockpit web — F3b, controle de simulação e identidade Toradex

Data: 24/08/2026
Máquina: workstation x86 (modo `learn`, tudo em container no host)
Cenário: `quadruped_maze11.sdf` (11,6 × 11,6 m) e `warehouse.sdf`

> **Nada aqui foi validado no Aquila AM69.** Tudo abaixo é `learn` no host.
> A execução no módulo é do M3 e continua `PENDING EXECUTION`.

---

## 1. O que foi entregue

### F3b — painéis azul e verde

| Painel | Fonte | Estado |
| --- | --- | --- |
| Azul (cena) | `/demo/cockpit/scene_{iso,top}/image_raw` | ao vivo, alternável no cabeçalho |
| Verde (navegação) | `/global_costmap/costmap`, `/plan`, `/demo/scan`, TF | ao vivo, clique manda meta |

Gate do F3b: *"os dois painéis atualizam ao vivo com o robô andando; o clique no
painel verde gera uma meta que o Nav2 aceita"* — **cumprido**. Um clique no
canvas produziu `running · 4,80 m restantes`, o Nav2 aceitou, executou, e
feedback e contagem de recuperações chegaram ao HUD.

### Controle da simulação pelo cockpit

Pedido do operador: *"permita que pela cockpit seja possível iniciar a simulação
no target, ou então resetá-la"*.

**A parte "no target" não é possível e não foi feita.** O Gazebo é OGRE 2 e
precisa de OpenGL de desktop; o AM69 expõe apenas OpenGL ES 3.2 e Vulkan 1.2
(regra 1 do CLAUDE.md). O que existe é play/pause/reset **a partir do** cockpit,
agindo sobre o Gazebo que roda na workstation. No M3, quando o cockpit for
servido pelo Aquila, o clique sai do módulo e a chamada atravessa o grafo ROS —
exatamente como a meta do Nav2 já atravessa hoje. O simulador continua no host.

### Controle das câmeras de cena

Girar, inclinar, mover e aproximar, num pad sobre a imagem, mais "recentrar".

---

## 2. A falha que definiu a arquitetura

A primeira versão chamava `/demo/sim/control`
(`ros_gz_interfaces/srv/ControlWorld`) direto do navegador. Os botões não
faziam nada, e o motivo só aparecia no log do container do cockpit:

```
call_service InvalidModuleException: Unable to import ros_gz_interfaces.srv
from package ros_gz_interfaces. Caused by: No module named 'ros_gz_interfaces'
```

O rosbridge monta o pedido importando o pacote de interfaces **dentro do próprio
container**, e o container do cockpit não tem `ros_gz_interfaces`. E não deve
ter: no M3 esse container roda no Aquila, e no modo `deploy` não existe Gazebo
nenhum — carregar para o módulo a definição de um simulador que não está lá é o
tipo de dependência que só cobra o preço depois.

**Correção:** um nó `sim_control_relay` no `demo_simulation` (host) expõe
`/demo/sim/{play,pause,reset}` como `std_srvs/Trigger` e traduz para
`ControlWorld`. A fronteira do navegador passou a falar só tipos de núcleo do
ROS. É a mesma escolha que o `scene_view_controller` já fazia para as câmeras.

Guardado por `tests/test_cockpit_web_contract.py::test_browser_never_speaks_gazebo_interfaces`.

---

## 3. O rótulo de estado não é o eco do clique

`.bar__state` mostra `rodando` / `pausado` / `sem simulador`, e o valor vem de
`/clock`, não do último botão apertado. O eco mentiria em todos os casos que
importam: container `sim` morto, chamada expirada, pausa feita pela GUI do
Gazebo, mundo resetado por outra pessoa.

A distinção entre `pausado` e `sem simulador` é a que custa: amostras chegando
com o mesmo tempo simulado é pausa; amostras parando é ausência. Coberto por
`hmi/test/sim-controls.test.js` (9 casos).

Medido:

```
estado inicial:            rodando
depois de clicar pausar:   pausado
depois de clicar retomar:  rodando
reset, 1 clique:           armado ("confirmar"), sem executar
```

O reset exige dois cliques dentro de 4 s. Ele devolve o robô à pose inicial
(`reset.all`) e, junto, apaga o costmap acumulado do Nav2 — um clique por engano
no meio da demo custa a demo.

---

## 4. Identidade Toradex

Paleta fornecida pelo time e aplicada como valor exato:

| Token | Valor | Uso |
| --- | --- | --- |
| `--brand-blue` | `#00508c` | primária, barra, plano global no mapa |
| `--brand-green` | `#96c837` | estado vivo, pegada do robô |
| `--brand-orange` | `#ff5a00` | atenção, obstáculo inscrito |
| `--bg-root` | `#ffffff` | fundo |

Duas consequências que não são cosméticas:

1. **O vermelho de falha (`#c0261b`) não é da marca, de propósito.** "Parado há
   tempo demais" e "morto" precisam parecer coisas diferentes através da sala.
   O laranja é o primeiro estado; o segundo precisa de uma cor que não seja
   nenhum dos três.
2. **A barra é a única superfície escura da tela.** As duas marcas são PNG de
   tinta branca com alfa; sobre branco elas sumiriam. Em vez de duplicar cada
   regra de botão numa variante "na barra", os tokens são redefinidos dentro de
   `.bar` e todo o CSS existente passa a ler o valor certo.

O costmap foi repaletizado junto (`buildCostLut`): num fundo claro, o gradiente
de inflação vai de azul claro ao laranja da marca, escurecendo e esquentando ao
mesmo tempo — legível também em preto e branco.

Cor de canvas deixou de existir em JavaScript: `hmi/js/panels/palette.js` lê os
tokens `--map-*` uma vez na montagem. Guardado por
`test_canvas_colours_come_from_the_tokens`.

---

## 5. Qualidade de imagem

Pedido do operador: *"melhore a qualidade da imagem da simulação, está muito
ruim, deixe a melhor qualidade possível"*.

| Parâmetro | Antes | Depois |
| --- | --- | --- |
| Sensor das câmeras de cena | 800 × 600 | 1600 × 1200 |
| Taxa | 5 Hz | 10 Hz (ver abaixo) |
| Anti-aliasing | default | 8 amostras |
| Qualidade JPEG (`web_video_server`) | 70 | 95 |

**A taxa é 10 Hz e não 15 por medição.** As duas câmeras renderizam no mesmo
processo do Gazebo do operador, e nesta resolução a workstation não entrega 15 Hz
de qualquer forma. Medido na bancada, mesmo mundo (`maze11`), 10 s de janela:

| `update_rate` | Entregue em `/demo/cockpit/scene_iso/image_raw` | Fator de tempo real |
| --- | --- | --- |
| 15 | 9,43 Hz | **0,59** |
| 10 | 9,77 Hz | **0,97** |

Pedir 15 não rendia um quadro a mais e custava 40% da velocidade da simulação —
o que estica cada meta do Nav2 na mesma proporção. Antes de subir esse número,
meça de novo.

A proporção continua **4:3**, e isso é uma restrição, não uma sobra: o
`horizontal_fov` e as poses das duas câmeras foram medidos nessa proporção. Ir
para 16:9 mantendo o hfov corta vertical e desenquadra as duas cenas de uma vez,
sem erro nenhum — só um robô fora do quadro. Guardado por
`test_scene_cameras_keep_the_measured_aspect_ratio`.

**Ressalva para o M3:** no modo `hil` o stream atravessa a Ethernet até o
Aquila, e esses números não foram medidos lá. Se apertar, o lugar de reduzir é a
qualidade JPEG em `hmi/js/config.js` (degrada suavemente), não a resolução do
sensor.

---

## 6. As caixas de detecção saíram do vídeo

Pedido do operador: *"remova o frame box que fica indo de um lado para outro na
tela da câmera"*.

O `demo_perception` de hoje é um stub determinístico: varre uma caixa sintética
pela imagem quer haja objeto ali ou não. Sobre o vídeo isso vira um retângulo
passeando de um lado para o outro — pior que nada numa demo, porque o
espectador lê aquilo como detecção de verdade.

O que **não** mudou: as detecções continuam sendo publicadas e continuam
alimentando a `perception_layer` do costmap. O contrato de tópicos do CLAUDE.md
está intacto; saiu só o desenho. `hmi/js/panels/detection-overlay.js` segue no
bundle, testado, para voltar quando o TIDL substituir o stub.

---

## 7. `/tf_static` chega uma vez só

Sintoma: o painel verde ficava em `sem TF map→base` em cerca de metade dos
carregamentos.

Medido em três inscrições consecutivas e novas: a primeira trouxe as arestas do
robô, a segunda `map→odom`, a terceira `map→odom`. O rosbridge entrega **uma**
mensagem latched por inscrição, e qual delas é sorte. `queue_length: 16` não
muda nada — a perda é acima da fila do cliente.

Correção: `nav-panel.js` reinscreve em `/tf_static` a cada 1,5 s, no máximo 8
vezes, parando em definitivo assim que `lookup('map','base')` resolve. Converge
em 2 a 4 rodadas; 3 de 3 carregamentos limpos.

---

## 8. Suítes

```
hmi:    node --test "test/**/*.test.js"   131 passando
raiz:   python3 -m pytest tests/ -q        29 passando
```

---

## 9. Em aberto

1. Verificado no Firefox. O kiosk do M3 é Chromium — a ressalva de cache do
   `<img>` em `config.js` continua vindo da literatura, não de medição.
2. Nenhum dos números de imagem foi medido sobre a Ethernet do modo `hil`.
3. O `nav2_container` segfaultou (`exit code -11`) uma vez ao configurar o
   `route_server`, sem relação com estas mudanças. Reiniciar o serviço resolveu.
   Se voltar, é candidato a issue própria.
