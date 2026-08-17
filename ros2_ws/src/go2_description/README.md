# `go2_description` — pacote vendorizado

Descrição do quadrúpede Unitree Go2 usada pela simulação do ML3.5 (F3 em
diante). **Este pacote não é nosso.** É código de terceiro copiado para dentro
da árvore, e este arquivo existe para registrar de onde veio, sob que licença, e
exatamente o que foi editado.

Mesmo padrão de `demo_navigation/launch/nav2_vendored/`: mudar o mínimo, provar
a procedência, deixar o diff visível.

---

## Por que este pacote existe com este nome

O `robot.xacro` referencia malhas e includes por `$(find go2_description)`:

```xml
<mesh filename="file://$(find go2_description)/meshes/trunk.dae" scale="1 1 1"/>
<xacro:include filename="$(find go2_description)/xacro/const.xacro"/>
```

Um pacote ROS chamado `go2_description` **precisa existir** para isso resolver.
Colocar o conteúdo como subdiretório de `demo_description/urdf/` foi tentado e
não funciona — `$(find)` não encontra subdiretório. A alternativa era reescrever
todos os `$(find go2_description)` para `$(find demo_description)`, o que
custaria a propriedade "idêntico ao upstream" que sustenta o argumento de
licença abaixo. Decisão: manter o nome upstream e não editar as referências.

---

## Procedência — duas camadas, com garantias diferentes

Esta é a parte que importa. As duas camadas deste pacote **não têm o mesmo grau
de prova**, e a distinção é deliberada.

### Camada 1 — malhas: procedência provada

As 7 malhas `.dae` (25 dos 25 MB do pacote) vêm de:

- **Repositório:** <https://github.com/unitreerobotics/unitree_ros>
- **Caminho:** `robots/go2_description/meshes/`
- **Licença:** BSD 3-Clause, texto completo em `LICENSE` neste diretório
- **Copyright:** (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")

Provado por hash git blob (`git hash-object`), comparando os arquivos desta
árvore contra os `sha` da API do GitHub para `unitree_ros@master`:

| Arquivo aqui | git blob sha1 | Arquivo no `unitree_ros` |
|---|---|---|
| `meshes/calf.dae`         | `83221779392a5d19af59561d0ea4eea48ef6102a` | `calf.dae` |
| `meshes/calf_mirror.dae`  | `413fd75d8b173b4597a35cc71e618a7d025069bd` | `calf_mirror.dae` |
| `meshes/foot.dae`         | `22bfcf49d7b6d07015c88f55e4ca153463bd5d0e` | `foot.dae` |
| `meshes/hip.dae`          | `1bec1a72feb8a5d941ad6767341bc1d79d7eff0d` | `hip.dae` |
| `meshes/thigh.dae`        | `a17c0482f296f2e19849f2544bf309866adfb801` | `thigh.dae` |
| `meshes/thigh_mirror.dae` | `fb424112d623ad912f75248bedb1f62c6c28d3b6` | `thigh_mirror.dae` |
| `meshes/trunk.dae`        | `4e47b5bec4a3c91ae4a1d49d73080d697a917cf1` | **`base.dae`** |

**7 de 7 idênticas byte a byte.** Única diferença: `base.dae` foi renomeado para
`trunk.dae` no reempacotamento do `legubiao`; o conteúdo é bit-idêntico.

Reproduzir:

```bash
# aqui
cd ros2_ws/src/go2_description/meshes && for f in *.dae; do
  printf "%s %s\n" "$(git hash-object "$f")" "$f"; done

# upstream
curl -sL "https://api.github.com/repos/unitreerobotics/unitree_ros/contents/robots/go2_description/meshes" \
  | grep -E '"(name|sha)"'
```

### Camada 2 — xacro e URDF: derivação interpretada, não provada

Os arquivos em `xacro/` e `urdf/robot.urdf` vieram de:

- **Repositório:** <https://github.com/legubiao/quadruped_ros2_control>
- **Commit:** `5434c5810d1a7fe223bcfd04550e9d3bfdd4b458` ("x30 repaint", 24/06/2025)
- **Caminho:** `descriptions/unitree/go2_description/`
- **Licença declarada:** `<license>BSD</license>` no `package.xml` — **sem texto
  de licença, sem header de copyright, sem autor ou maintainer identificado**

Estes arquivos **não** batem com o `unitree_ros` upstream. Foram reescritos para
ROS 2 / `ros2_control` (o upstream é ROS 1 / Gazebo Classic); os tamanhos
divergem em todos (`const.xacro` 5096 vs 7739 bytes, `leg.xacro` 7179 vs 12300,
`robot.xacro` 4523 vs 4667).

**Leitura adotada, e é interpretação:** são obra derivada da descrição Unitree —
mesma cinemática, mesmos nomes de junta e link, mesmas referências às mesmas
malhas — portanto cobertos pelo BSD-3 do upstream, com a adaptação do `legubiao`
por cima. O `<license>BSD</license>` declarado é coerente com isso.

**Risco residual, registrado e não apagado:** essa cobertura não está provada por
hash como a das malhas. É menor que o risco que fez o projeto rejeitar o
`a1_description` (que não tem licença declarada nem titular rastreável), porque
aqui há titular provado para o asset principal e declaração BSD coerente para a
camada derivada — mas não é zero.

### O que foi rejeitado e por quê

`descriptions/unitree/a1_description/` do mesmo repo declara
`<license>TODO</license>`. Foi por isso que o alvo do ML3.5 mudou de A1 para Go2
em F2. Ver `docs/ml35/estado-fases.md`.

---

## Edições feitas sobre o upstream

Duas, ambas deliberadas.

### 1. Remoção do que a demo não usa

Removido do que veio do `legubiao` (nada do que a demo usa foi tocado):

| Removido | Por quê |
|---|---|
| `config/himloco/`, `config/legged_gym/`, `config/robot_lab/` | pesos `.pt` de política RL, ~2,5 MB. Usamos `unitree_guide_controller`, que é PD clássico e não carrega política nenhuma |
| `config/ocs2/` | configs do `ocs2_quadruped_controller`, que não entra na demo |
| `launch/` | launch files upstream (`gazebo_rl_control`, `visualize`). O nosso vive em `demo_simulation`, e o upstream sobe RViz2 dentro do launch — regra 1 |
| `config/visualize_urdf.rviz` | config do RViz2 upstream, idem |
| `README.md` original | substituído por este arquivo. Conteúdo original: instruções de build e de launch do repo upstream, com links relativos que não resolvem fora dele |

De 28 MB para 25 MB. `meshes/`, `xacro/`, `urdf/` e os dois configs que a
simulação lê (`gazebo.yaml`, `robot_control.yaml`) estão **intactos**.

### 2. `package.xml` — licença e autoria

Único arquivo de conteúdo editado. Antes:

```xml
<author>TODO</author>
<maintainer email="TODO@email.com"/>
<license>BSD</license>
```

Depois: autor e copyright atribuídos à Unitree Robotics, licença precisada para
`BSD-3-Clause` (a variante real, confirmada no `LICENSE` upstream), maintainer
apontando para este projeto — quem mantém *esta cópia* somos nós, e fingir o
contrário seria pior. Ver o diff no git.

Nenhum arquivo em `meshes/`, `xacro/`, `urdf/` ou `config/` foi editado. Os
hashes da tabela acima são verificáveis a qualquer momento e devem continuar
batendo.

---

## Ao atualizar este pacote

1. Refazer a comparação de hashes das malhas contra o `unitree_ros`.
2. Se um xacro mudar, registrar o quê e por quê aqui — não editar em silêncio.
3. Não trazer de volta `launch/` nem os configs de RL sem uma razão declarada:
   `launch/` viola a regra 1 (RViz2 dentro do launch de simulação).
