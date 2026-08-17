# ML3.5 F3 — evidência de execução

Data: 17/08/2026. Host x86 (`diegom-nb`). Execução quantitativa headless;
confirmação visual posterior no Gazebo GUI pelo operador.

Portão de F3: *"Go2 em pé, estável, responde a `cmd_vel` sem cair"*, com os
pacotes e o launch do projeto — não com o spike descartável de F2.

**Medido por pose real, nunca por log.** Um controlador que carrega limpo e
deixa o robô desabar não produz erro nenhum: foi exatamente o que aconteceu na
primeira execução desta fase (ver "A execução que falhou" abaixo).

---

## Como foi executado

Build e execução dentro de container, com a árvore do projeto montada:

```bash
docker run --rm --network=host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=69 \
  -v "$PWD/ros2_ws/src:/proj/src:ro" \
  --entrypoint bash demo-sim:spike-go2 -c '
    . /opt/ros/jazzy/setup.sh
    mkdir -p /test/src && cd /test
    cp -r /proj/src/{go2_description,control_input_msgs,controller_common,\
unitree_guide_controller,gz_quadruped_hardware,demo_simulation} src/
    colcon build --symlink-install
    . install/setup.sh
    ros2 launch demo_simulation quadruped.launch.py gui:=false world:=empty.sdf
'
```

**Por que na imagem do spike e não na `sim` do projeto:** a build da imagem
`base` falhou nesta sessão com `rosdep update` retornando erro em
`raw.githubusercontent.com` — **HTTP 503**, confirmado por `curl` direto,
enquanto `api.github.com` respondia 200. Indisponibilidade externa, sem relação
com o código. A imagem do spike já tem `ros2_control` instalado e serviu para
exercitar exatamente os mesmos pacotes e o mesmo launch.

**Pendência:** reconstruir `base` e `sim` com o `sim/Dockerfile` atualizado
quando a rede permitir, e repetir esta execução via
`docker compose -f compose.host.yml`. O código está commitado; o que falta é a
imagem construída pelo caminho oficial.

---

## Resultado

Pose lida de `gz topic -e -t /world/empty/dynamic_pose/info`:

| Momento | z (altura) | x | Interpretação |
|---|---|---|---|
| Em pé, após a FSM completar | **0,352 m** | 0,041 | em pé, estável |
| Andando +3 s | 0,354 m | 0,071 | |
| Andando +6 s | 0,353 m | 0,113 | |
| Andando +9 s | 0,352 m | 0,153 | |
| Andando +12 s | 0,352 m | 0,195 | |
| Final | **0,351 m** | **0,216** | anda sem perder altura |

Comando de marcha: `/demo/cmd_vel` com `linear.x=0.03` a 10 Hz por 12 s.

Deslocamento monotônico de **0,175 m em 12 s**, altura sustentada em 0,35 m sem
oscilar, orientação final `x=-7,2e-05` (praticamente nivelado).

```
unitree_guide_controller  unitree_guide_controller/UnitreeGuideController  active
joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster    active
imu_sensor_broadcaster    imu_sensor_broadcaster/IMUSensorBroadcaster      active
```

Erros no Gazebo (`grep -c "Err\]"`): **0**.

FSM de marcha, do log do `twist_to_inputs`:

```
gait FSM: passive -> fixed down
gait FSM: fixed down -> fixed stand
gait FSM: fixed stand -> trotting. Now driven by /demo/cmd_vel.
```

### Comparação com F2

| Métrica | F2 (spike) | F3 (projeto) |
|---|---|---|
| Em pé | 0,353 m | 0,352 m |
| Andando | 0,343 m | **0,351 m** |
| Deslocamento | não medido | **0,175 m / 12 s** |
| Erros | 0 | 0 |

F3 é **mais estável**: a altura não cai durante a marcha, efeito da troca de
timers por cadeia de eventos.

---

## A execução que falhou, e por que ela importa

Primeira execução desta fase, com a `TimerAction` de 12 s copiada do plant
diff-drive:

```
RESULT stand_height_z=0.0677794438413276
RESULT walk_height_z=0.0677794438413276
unitree_guide_controller ... active
joint_state_broadcaster  ... active
imu_sensor_broadcaster   ... active
error count: 0
gait FSM: passive -> fixed down
gait FSM: fixed down -> fixed stand
gait FSM: fixed stand -> trotting. Now driven by /demo/cmd_vel.
```

**Todos os sinais verdes, robô caído.** z=0,068 é o robô desabado no chão, e o
valor é idêntico antes e depois de andar — imóvel.

Diagnóstico por amostragem da pose a cada segundo após o spawn:

```
amostra 1: z=0.49998553058432743   <- spawn correto
amostra 2: z=0.0676869866997855    <- menos de 1 s depois, no chão
amostra 3..6: z=0.0677794438411246 <- imóvel
```

Contra o log dos spawners: os controladores só ativam **~3 s depois disso**. O
robô cai de 0,5 m com as juntas soltas, sem controlador, e desaba. A FSM depois
percorre `passive → trotting` sobre um robô que já está no chão, e cada estado
reporta sucesso.

Correção em `quadruped.launch.py`: spawn imediato, encadeado por
`OnProcessExit` (`spawn → broadcasters → controlador`), sem timer — que é o que
o `gazebo.launch.py` upstream já fazia. O comentário longo no arquivo explica
por que ali não pode haver `TimerAction`.

**É a segunda vez no ML3.5 que um timer mede a coisa errada.** A primeira foi em
F1 e gerou o `wait_for_clock`.

---

## Testes

`colcon test` nos pacotes do projeto: **46 testes, 0 falhas** — sem regressão
em relação a F1.

Os 5 pacotes vendorizados tiveram o lint de estilo desativado
(`ament_lint_auto` produzia 98 falhas em código de terceiro que a política manda
não editar). Nossos pacotes mantêm os seus linters.

---

## Não validado

- **Nada em arm64. Nada no módulo.** Regras 5 e 7.
- **RViz2.** O operador confirmou em 17/08/2026 que o modelo Go2 aparece com
  corpo visível no **Gazebo GUI**, usando a imagem do spike e
  `world:=empty.sdf`. RViz2 e sua árvore TF continuam pendentes; a
  confirmação no Gazebo não prova o caminho de renderização do RViz.
- **`warehouse.sdf`.** O portão rodou em `empty.sdf`. O mundo do projeto carrega
  ~10 s com 50+ malhas; o spawn agora é imediato (seguro, `create` faz retry),
  mas não foi exercitado ali.
- **Imagem `sim` oficial.** Ver a nota sobre o HTTP 503 acima.
- **Marcha em ganho alto.** Acima de ~0,15 m/s o robô perde equilíbrio, como F2
  já media. É sintonia do mapeamento em `twist_to_inputs` — trabalho de F4.
