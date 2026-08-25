# Assets de marca

Arquivos-fonte de identidade, guardados aqui para que os derivados usados no
cockpit sejam reproduzíveis. **Nada deste diretório é servido**: o que vai ao ar
está em `hmi/img/`, já processado.

## `US Logo_Reverse.jpg`

Marca Toradex em versão reverse (tinta branca sobre fundo azul `#00508d`),
fornecida pelo time.

Dela sai `hmi/img/toradex.png`, que precisa ser **branco sobre transparente** —
a barra do cockpit é azul `#00508c` e um retângulo opaco ali viraria uma mancha.
A conversão foi um chroma key sobre o azul de fundo, com dois cuidados que não
são opcionais:

1. **alfa despremultiplicado**, senão o ponto verde da marca (que encosta no
   fundo) some junto com o azul;
2. **recorte na bounding box** antes do redimensionamento, para a marca não
   ficar nadando dentro de margem transparente na barra.

Resultado: 720 × 244, RGBA.

## `hmi/img/ros.png`

Não tem fonte aqui — veio pronta de <https://www.ros.org/imgs/logo-white.png>,
já branca sobre transparente. 520 × 137, RGBA.

Uso da marca ROS conforme as diretrizes da Open Robotics.
