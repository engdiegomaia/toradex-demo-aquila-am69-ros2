# Ponto de montagem vazio para árvores de modelos externas

Este diretório existe **de propósito vazio**. Ele é o default do bind mount
`/maze/models` do serviço `sim` em `compose.host.yml`.

O Compose não tem montagem condicional: ou a linha de volume existe sempre, ou
não existe nunca. Um default apontando para um caminho inexistente faz o Compose
**criar um diretório** com aquele nome; um default apontando para `/dev/null`
falha ao montar. Este diretório é o default que sempre existe e nunca traz nada.

Quando `MAZE_MODELS` está definido no `.env`, ele substitui este caminho e a
malha do labirinto aparece. Quando não está, `/maze/models` fica vazio,
`GZ_SIM_RESOURCE_PATH` aponta para um diretório sem modelos, e nada muda.

A malha do `ros_maze_worlds` não é versionada aqui porque o `package.xml` de lá
declara `<license>TODO</license>`. Veja o cabeçalho de
`ros2_ws/src/demo_simulation/worlds/quadruped_maze.sdf` e
`docs/guides/cenarios/s6-labirinto.md`.
