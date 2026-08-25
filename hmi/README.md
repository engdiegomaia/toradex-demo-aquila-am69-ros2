# Cockpit web — bundle

Interface do operador para a demo. Um único bundle serve dois destinos: o
cockpit da workstation x86 hoje, e o kiosk Chromium no Aquila AM69 no M3.

Plano e decisões: [`docs/ml35/plano-cockpit-web.md`](../docs/ml35/plano-cockpit-web.md).
Layout aprovado: [`docs/ml35/cockpit-division-view.png`](../docs/ml35/cockpit-division-view.png).

## Sem etapa de build

Não há `npm install`, bundler, transpilador nem `node_modules`. São arquivos
HTML/CSS/ES modules servidos como estão por `nginx:alpine`
(plano-cockpit-web.md Decisão 5). O `package.json` existe por dois motivos e
nenhum deles é dependência: declarar `"type": "module"` para o Node tratar os
`.js` como ESM ao rodar os testes, e dar significado a `npm test`.

`roslibjs` **não** é vendorizado. O protocolo rosbridge v2 é JSON simples e o
cliente mínimo com reconexão está em `js/ros/rosbridge-client.js`.

## Como rodar

Pela composição (o caminho normal, a partir da raiz do repositório):

```bash
docker compose -f docker/compose.host.yml up -d cockpit hmi
xdg-open http://localhost:8081
```

Direto do diretório, sem container, durante desenvolvimento de UI — os
endpoints caem nos defaults (`ws://localhost:9090`, `http://localhost:8080`),
que é exatamente o que o serviço `cockpit` publica:

```bash
python3 -m http.server 8081 --directory hmi
```

Sobrepor host e portas sem editar arquivo:

```
http://localhost:8081/?host=aquila.local&rosbridgePort=9090&videoPort=8080
```

## Testes

```bash
cd hmi && npm test          # ou: node --test "test/**/*.test.js"
```

Cobrem o que o AGENTS.md §5.7 exige: parsing de mensagem e comportamento de
estado de conexão. Rodam no runner embutido do Node, sem instalar nada.

## Estrutura

```text
hmi/
├── index.html              as cinco regiões, HTML semântico
├── css/
│   ├── tokens.css          paleta, escala tipográfica, motion
│   ├── layout.css          o grid de cinco regiões (proporções da imagem)
│   └── panels.css          chrome dos painéis, dots de frescor, barra
├── js/
│   ├── config.js           camadas de configuração e construção de URL
│   ├── main.js             só fiação
│   ├── ros/
│   │   ├── rosbridge-client.js   protocolo v2, reconexão, replay
│   │   └── freshness.js          never / live / stale
│   └── panels/
│       ├── stream-panel.js       MJPEG + liveness por camera_info
│       ├── log-panel.js          /rosout + telemetria
│       └── control-bar.js        badge de link, build, controles
└── test/                   node --test
```

## Estado por painel

| Região | Estado | Fonte |
| --- | --- | --- |
| Cena (azul) | **pendente — F3b** | câmera de cena do mundo SDF |
| Navegação (verde) | **pendente — F3b** | canvas 2D via rosbridge |
| Câmera (rosa claro) | funcional | `/demo/camera/image_raw` via web_video_server |
| Logs (rosa) | funcional | `/rosout`, `/demo/cmd_vel`, `/demo/odom` |
| Barra (cinza) | parcial | link/build funcionais; botões **inertes até o F4** |

Os botões de controle manual estão desabilitados de propósito. Publicar em
`/demo/cmd_vel` junto com o Nav2 sem árbitro é o débito que o F4 fecha com
`twist_mux` — ver o cabeçalho de `js/panels/control-bar.js`.

## Duas linhas de transporte, e por quê

- **Pixels** vão por HTTP (`web_video_server`, MJPEG dentro de `<img>`).
- **Todo o resto** vai pelo WebSocket do rosbridge.

Imagem 640x480 em JSON base64 pelo WebSocket é o que faz um cockpit por dados
parecer lento. Em compensação, um `<img>` não é bom detector de vida: navegador
nenhum garante evento por quadro, e um stream que para deixa o último quadro
pintado sem avisar. Por isso o frescor da câmera vem de
`/demo/camera/camera_info` — poucas centenas de bytes na mesma taxa da imagem.
