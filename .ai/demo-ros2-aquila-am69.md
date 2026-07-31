# Demo ROS 2 sobre Torizon OS no Aquila AM69

**Documento descritivo do projeto**
Versão 1.0, julho de 2026
Responsável: Diego Maia, FAE Toradex Brasil

---

## 1. Objetivo

Construir uma demonstração de robótica que posicione o Aquila AM69 com Torizon OS como plataforma de computação embarcada para robôs móveis, usando ROS 2 como framework de aplicação e um simulador no lugar do robô físico.

A demo precisa mostrar três coisas ao mesmo tempo. Primeiro, que uma stack ROS 2 real, com navegação autônoma, roda no módulo com folga de desempenho. Segundo, que o modelo de containers do Torizon simplifica o empacotamento e a atualização dessa stack. Terceiro, que a atualização remota via Torizon Cloud funciona em campo, o que é o argumento que diferencia a Toradex de um SoM genérico.

O foco é robótica. A capacidade de inferência de 32 TOPS do módulo não faz parte do escopo inicial, mas a arquitetura é desenhada para recebê-la depois sem retrabalho. Os pontos de extensão estão detalhados na seção 7.

Este é um projeto interno de demonstração, não desenvolvimento de produto. Isso significa que critérios de suporte de longo prazo, classificação de produção e patch de segurança não são gates do projeto, ainda que estejam registrados como limitações conhecidas.

---

## 2. Premissas e escopo

Está no escopo a navegação autônoma em ambiente simulado, com o planejamento e o controle executando no módulo, uma interface visual embarcada mostrando o estado do robô, e o ciclo de atualização de containers pela Torizon Cloud.

Está fora do escopo o robô físico, a percepção baseada em NPU, qualquer argumento de segurança funcional, e a validação de longo prazo do hardware.

A premissa central da arquitetura é que o simulador não roda no módulo. A justificativa técnica está na seção 6.

---

## 3. Plataforma

### 3.1 Hardware

O módulo é um Aquila AM69 Octa 32GB WB IT revisão V1.0A, com SoC `XAM6958ATGGHAALY`. O SoC pertence à família AM69/AM69A, com oito núcleos Arm Cortex-A72, dois Cortex-R5F, quatro aceleradores de aprendizado profundo somando 32 TOPS, GPU Imagination BXS-4-64 e ISP duplo. O prefixo X no part number indica silício não qualificado, destinado a avaliação, o que é aceitável para bancada e demonstração e não seria aceitável em produto.

A configuração traz 32 GB de RAM e 128 GB de eMMC, então armazenamento e memória não são restrição para nenhuma fase deste projeto.

A placa portadora é uma Aquila Development Board V1.2B, compatível com a revisão V1.0A do módulo. A saída de vídeo é DisplayPort, e deve ser usada com cabo direto, sem adaptador ou conversor de HDMI.

A solução térmica é obrigatória. A referência de projeto do módulo é de 28 W sustentados a 85 °C de temperatura ambiente e 35 W a 70 °C, com junção máxima de 105 °C. Em estande fechado, a temperatura ambiente sobe mais do que a intuição sugere, e falha térmica é a principal causa de demo interrompida em evento.

### 3.2 Software embarcado

Torizon OS 7.4.0, instalada por Toradex Easy Installer a partir do zero.

**Regra crítica de instalação.** Nunca chegar à 7.4.0 por atualização remota a partir de imagem anterior neste módulo. Existe uma quebra de compatibilidade entre as revisões V1.0 e V1.1 na seleção de device tree feita pelo bootloader. Um módulo V1.0 com imagem antiga que recebe atualização acaba com o bootloader antigo carregando a device tree da V1.1, e para de bootar. A recuperação exige flash por Tezi e reprovisionamento na Torizon Cloud. A combinação V1.0 com imagem nova instalada do zero é suportada e é a que este projeto usa.

A revisão V1.0 tem suporte limitado a partir da 7.4.0, com os testes automatizados descontinuados para essa revisão. Na prática, uma regressão específica dela pode não ter sido detectada. É a razão pela qual a fase 5 inclui teste de longa duração.

A 7.4.0 é também a primeira release com containers acelerados por GPU para o Aquila AM69, incluindo Weston, Qt e Chromium. Releases anteriores não os possuem, o que descartou a opção de usar a 7.3.0.

### 3.3 Estação de trabalho

Ubuntu 24.04 x86_64, com ROS 2 Jazzy e Gazebo Harmonic instalados nativamente, além de Docker com suporte a build e execução multi-arquitetura.

---

## 4. Arquitetura

### 4.1 Distribuição em três partes

O sistema se divide em três blocos, cada um onde faz sentido executar.

O **simulador** roda na estação x86. Gazebo Harmonic publica os sensores simulados e recebe os comandos de velocidade, com `ros_gz_bridge` fazendo a conversão para tópicos ROS 2.

A **stack de robótica** roda no módulo, em containers. Navegação, localização, planejamento e controle consomem os tópicos do simulador e devolvem comandos. É aqui que está a carga computacional que a demo precisa evidenciar.

A **interface visual** roda no módulo, em container acelerado por GPU, com saída em DisplayPort. Esta parte não é decorativa. Sem ela, um observador no estande vê um notebook trabalhando e um módulo escondido, o que anula a mensagem da demonstração.

As duas máquinas se comunicam por DDS sobre Ethernet, na mesma rede de camada 2.

### 4.2 Middleware

CycloneDDS como implementação RMW, definida por variável de ambiente embutida na imagem base, não configurada em tempo de execução. A implementação padrão apresenta falha conhecida de descoberta entre containers, na qual um nó enxerga o tópico com `ros2 topic list` mas não recebe mensagem alguma. A troca para CycloneDDS resolve. Configurar isso desde a primeira imagem evita perder um dia depois.

Rede em modo host nas duas pontas. Rede Docker isolada entre máquinas distintas exige configuração explícita de peers e portas no CycloneDDS e não traz benefício em bancada.

### 4.3 Estrutura do repositório

```
demo-aquila-ros2/
├── docker/
│   ├── base/            # ros:jazzy + cyclonedds, multi-arch
│   ├── navigation/      # nav2, bringup
│   ├── perception/      # stub hoje, TIDL amanha
│   ├── hmi/             # chromium kiosk + rosbridge
│   └── simulation/      # gazebo + ros_gz, apenas amd64
├── ros2_ws/src/
│   ├── demo_description/    # urdf/xacro, meshes
│   ├── demo_bringup/        # launch files, um por modo
│   ├── demo_navigation/     # parametros nav2, camadas de costmap
│   ├── demo_perception/     # no stub, interface final
│   └── demo_simulation/     # mundos sdf, spawn
├── compose/
│   ├── learn.yaml       # tudo amd64 nativo no x86
│   ├── emul.yaml        # gazebo amd64 + stack arm64 emulado
│   └── target.yaml      # gazebo no x86 + stack no AM69
└── docs/
```

O princípio de projeto é que a mesma imagem e o mesmo código funcionem nos três modos de execução. Os três arquivos de composição diferem apenas na plataforma declarada e em qual máquina cada serviço sobe. Quando o hardware entra em cena, troca-se `emul.yaml` por `target.yaml` e nada mais muda. É isso que faz o trabalho da fase de aprendizado ser aproveitado em vez de descartado.

Cada modo tem seu próprio arquivo de launch em `demo_bringup`, em vez de um arquivo único cheio de condicionais. Launch parametrizado demais é uma das principais fontes de confusão para quem está começando em ROS 2.

---

## 5. Fases

### Fase L, aprendizado (3 a 4 semanas, apenas x86)

Como não há experiência prévia com ROS 2 na equipe, esta fase antecede tudo e não depende de hardware.

**L1, fundamentos (1 semana).** Nós, tópicos, serviços, parâmetros, workspace colcon e arquivos de launch. Turtlesim como veículo de aprendizado. Critério de saída: um par publisher e subscriber escrito do zero, empacotado como `ament_python` e iniciado por launch.

**L2, o robô como modelo (1 semana).** TF2, URDF e xacro, `robot_state_publisher` e RViz2. É a semana mais importante das quatro. Árvore de transformadas mal montada é a causa mais comum de navegação que não funciona, e o problema aparece tarde, disfarçado de outra coisa.

**L3, simulação (1 semana).** Gazebo Harmonic, `ros_gz_bridge`, spawn do robô, teleoperação por teclado e então Nav2 com mapa estático. Critério de saída: robô navegando na simulação, tudo nativo.

**L4, containerização e emulação arm64 (3 a 4 dias).** Refazer L3 dentro de containers, primeiro em `amd64` nativo e depois em `arm64` emulado por QEMU. Critério de saída: stack de navegação em `arm64` emulado conversando por DDS com o Gazebo nativo na mesma máquina.

Recomendação deliberada: as fases L1 a L3 são feitas com ROS 2 instalado nativamente, sem container. Container introduz rede, volumes e encaminhamento gráfico sobre um sistema que ainda não é familiar, e quando algo falha não se sabe se a causa é o ROS ou o Docker. A containerização entra depois, na L4, quando os conceitos já estão firmes.

**Sobre o que a emulação prova.** Ela confirma que a imagem constrói para `arm64`, que os pacotes existem para a arquitetura, que os nós sobem, que o grafo de tópicos se forma e que a configuração de DDS está correta. Isso elimina a maior parte dos problemas que apareceriam no primeiro contato com o módulo. Ela não diz nada sobre desempenho, porque QEMU em modo usuário é uma ordem de grandeza mais lento, e não exercita GPU, NPU, câmeras nem device tree. Toda medição de carga só vale no hardware real.

### Fase 0, bancada (2 a 3 dias)

Instalação da 7.4.0 por Tezi, montagem e verificação da solução térmica, configuração de rede e captura da linha de base. O teste térmico deve ser feito já com carga combinada de CPU e NPU, mesmo que a NPU não seja usada no escopo atual, porque o objetivo é validar o pior caso futuro e não o atual.

Critério de saída: módulo estável sob carga por período prolongado, com temperatura registrada e saída de `tdx-info` arquivada.

### Fase 1, espinha dorsal ROS 2 (1 semana)

Imagem base `arm64` com CycloneDDS, par publisher e subscriber atravessando as duas máquinas, medição de latência e jitter.

Critério de saída: tópico estável por uma hora contínua, com números registrados.

### Fase 2, simulação e navegação (2 semanas)

Mundo SDF, modelo do robô, Gazebo com ponte no x86 e Nav2 no módulo.

Critério de saída: robô navegando de um ponto a outro com o planejamento executando no AM69, e consumo de CPU medido por nó. Esse número é o resultado técnico mais valioso do projeto inteiro, porque é a evidência que sustenta a mensagem comercial.

### Fase 3, interface embarcada (1 a 2 semanas)

Container gráfico no módulo mostrando estado do robô, mapa e trajetória em tempo real. O caminho de menor risco é Chromium em modo quiosque servindo um painel web alimentado por `rosbridge_server`, porque desacopla o ciclo de desenvolvimento do visual do ciclo de build do ROS.

Critério de saída: aceleração por GPU confirmada, sem recorrer a renderização por software.

### Fase 4, Torizon Cloud (1 semana)

Provisionamento do dispositivo, empacotamento dos containers e aplicação de uma atualização remota ao vivo.

Critério de saída: nova versão de container aplicada sem tocar no hardware.

### Fase 5, endurecimento (1 semana)

Teste de longa duração de 48 horas, reinício automático dos containers em caso de falha e elaboração do runbook de operação. A regra de nunca atualizar por OTA a partir de imagem anterior entra em destaque no runbook.

### Cronograma consolidado

A fase L ocupa três a quatro semanas e não depende de hardware, podendo começar imediatamente. As fases 0 a 5 somam aproximadamente seis semanas a partir da chegada do módulo. O total é da ordem de dez semanas.

---

## 6. Restrições técnicas conhecidas

### 6.1 O simulador não roda no módulo

Esta é a restrição que define a arquitetura e vale registrar em detalhe, porque a pergunta reaparece sempre.

O Gazebo Sim usa o motor de renderização OGRE 2 por padrão, que exige OpenGL desktop superior à versão 3.3, preferencialmente 4.3 ou mais recente. A GPU BXS-4-64 do AM69 oferece OpenGL ES até 3.2 e Vulkan até 1.2, e não expõe OpenGL desktop. OpenGL ES é uma API distinta, e portanto a versão 3.2 não satisfaz o requisito.

As alternativas foram avaliadas e descartadas. Renderização por software funciona, mas entrega poucos quadros por segundo. O motor OGRE 1 tem requisito menor de OpenGL, porém está obsoleto no Harmonic. A execução sem interface gráfica com `--headless-rendering` evita a janela, mas depende de EGL disponível apenas no OGRE 2, de modo que qualquer sensor de câmera ou lidar por GPU esbarra no mesmo requisito.

**Consequência que costuma passar despercebida:** o RViz2 usa o mesmo motor e portanto também não roda no módulo. Ferramenta de visualização de ROS fica na estação de trabalho, e a interface embarcada precisa ser construída com Qt, Slint ou tecnologia web sobre os containers acelerados existentes.

Uma alternativa que ainda merece teste é executar apenas o servidor de física, com `gz sim -s`, em um mundo sem sensores de renderização. A hipótese é que nesse caso o motor gráfico não seja instanciado e a simulação rode nos A72. Se confirmada, abriria a possibilidade de uma demo autocontida sem a estação x86 no estande. Está listada como teste da primeira semana.

### 6.2 Testes de derrubada de premissa

Cinco verificações a executar antes de escrever código de aplicação, porque cada uma pode alterar o plano.

Servidor de física isolado no módulo, conforme descrito acima. RViz2 no módulo, para confirmar a falha e encerrar a discussão. Container Weston ou Chromium do AM69 na 7.4.0, confirmando aceleração real. Tópico de imagem atravessando o DDS entre as duas máquinas, com medição de banda e perda. Comportamento térmico sob carga combinada.

---

## 7. Pontos de extensão para IA e os 32 TOPS

A integração de inferência não está no escopo atual, mas cinco costuras devem existir desde a fase 2. Reservá-las custa horas agora. Retrofitá-las custa semanas depois.

**Contrato de tópicos.** A percepção publica `vision_msgs/Detection2DArray` desde o início, implementada por um nó stub que gera detecções sintéticas. Todos os consumidores nascem contra a interface definitiva, e a substituição futura afeta apenas o publicador.

**Fonte de imagem sempre como tópico.** O nó de percepção consome `sensor_msgs/Image` e nunca conhece a origem, seja ela o Gazebo, uma câmera USB ou um arquivo de gravação. Trocar imagem sintética por câmera real vira configuração.

**Container de percepção separado desde o primeiro dia,** mesmo carregando apenas o stub. Isso estabelece a granularidade de atualização usada na fase 4 e evita refatorar um monolito mais tarde.

**Costura no Nav2.** A detecção alimenta uma camada do costmap, e não somente a tela. Quando a extensão chegar, o robô muda de comportamento diante do público, em vez de ganhar um retângulo desenhado sobre a imagem. É a diferença entre inteligência artificial acoplada e inteligência artificial integrada.

**Orçamento reservado.** Na fase 2, medir consumo de CPU por nó e estabelecer teto para a stack de robótica, algo em torno de sessenta por cento dos oito núcleos. Na fase 0, incluir a NPU no teste térmico. Na fase 3, deixar um painel vazio no layout da interface.

### 7.1 Aviso sobre o custo real dessa extensão

O material de referência de inferência multi-câmera no Aquila AM69 executa sobre imagem BSP Reference Multimedia, não sobre Torizon OS. A stack reside em `/opt/edgeai-gst-apps` e `/opt/model_zoo`, com serviços systemd no host e execução como root, sem containers.

Isso colide com o modelo do Torizon, no qual `/opt` não é modificável por TorizonCore Builder. Levar essa stack para o Torizon exige empacotar o runtime TIDL, o repositório de modelos e o acesso aos nós de device do C7x dentro de um container, ou construir uma imagem Torizon customizada via Yocto com as camadas da Texas Instruments. Nenhuma das duas alternativas é trabalho de um dia.

**Recomendação:** executar uma investigação de dois dias em paralelo à fase L, apenas para determinar se o runtime TIDL se containeriza de forma limpa na 7.4.0. Não precisa funcionar, precisa produzir uma estimativa. Se containerizar, a extensão custa cerca de duas semanas e as cinco costuras acima são suficientes. Se não containerizar, a decisão muda de natureza, e é melhor descobrir isso agora do que às vésperas de um evento.

Como material de partida, o repositório de modelos da TI já inclui `ONR-SS-7618-deeplabv3lite-mobv2-qat-robokit-768x432`, uma rede de segmentação treinada para robótica, o que combina com a narrativa da demo.

---

## 8. Ferramentas

| Camada | Ferramenta | Onde executa |
| --- | --- | --- |
| Sistema operacional embarcado | Torizon OS 7.4.0 | Aquila AM69 |
| Instalação de imagem | Toradex Easy Installer | Aquila AM69 |
| Customização de imagem | TorizonCore Builder | Estação x86 |
| Gestão de frota e atualização | Torizon Cloud, aktualizr | Ambos |
| Framework de aplicação | ROS 2 Jazzy | Ambos |
| Middleware | Eclipse CycloneDDS | Ambos |
| Simulador | Gazebo Harmonic | Estação x86 |
| Ponte de simulação | `ros_gz_bridge` | Estação x86 |
| Navegação | Nav2 | Aquila AM69 |
| Visualização de desenvolvimento | RViz2 | Estação x86, apenas |
| Interface embarcada | Chromium kiosk, `rosbridge_server` | Aquila AM69 |
| Containers | Docker, buildx, QEMU binfmt | Ambos |
| Build de workspace | colcon, ament | Ambos |
| Inferência, extensão futura | TI Edge AI, TIDL, edgeai model zoo | Aquila AM69 |

---

## 9. Riscos

O risco térmico é o mais provável de interromper a demonstração em evento e é o único da lista sem solução por software. Mitigação: teste com carga combinada já na fase 0 e teste de longa duração na fase 5.

O silício de protótipo não é qualificado e tem taxa de falha em uso final indefinida. Não bloqueia bancada. Mitigação: se surgir instabilidade não reproduzível, o silício entra na lista de hipóteses, depois de esgotadas as causas convencionais.

O suporte limitado da revisão V1.0 a partir da 7.4.0 significa cobertura de teste automatizado reduzida. Mitigação: teste de longa duração e caminho de escalação interno disponível junto ao time de Torizon OS.

A ausência de experiência prévia com ROS 2 é endereçada pela fase L, que é justamente por isso executada antes e de forma independente do hardware.

O risco de recuperação por atualização incorreta é eliminado por procedimento: instalação sempre por Tezi, nunca por atualização remota a partir de versão anterior. Consta do runbook.
