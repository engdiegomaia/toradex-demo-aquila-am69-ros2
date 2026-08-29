"""
Confirmação temporal do detector da saída, e o que ele se recusa a publicar.

`test_maze_exit_detector.py` cobre as funções puras -- a segmentação e a
geometria. O que está aqui é a parte que decide QUANDO o explorador é
interrompido: um único quadro magenta não pode cancelar uma meta de fronteira, e
uma pose sem intrínsecos não pode ser inventada.

O detector nunca sabe onde a saída está. Ele sabe que viu um painel magenta,
quantas vezes seguidas, e a que distância -- e é só isso que ele diz.
"""

from demo_perception.maze_exit_detector import MazeExitDetector
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image


PANEL = (255, 0, 255)


def image_with_panel(width: int = 64, height: int = 48,
                     box: tuple[int, int, int, int] = (16, 12, 48, 36),
                     encoding: str = 'rgb8') -> Image:
    """Um quadro com um retângulo magenta saturado, como o painel no Gazebo."""
    message = Image()
    message.header.frame_id = 'front_camera'
    message.width = width
    message.height = height
    message.encoding = encoding
    message.step = width * 3
    data = bytearray(message.step * height)
    min_x, min_y, max_x, max_y = box
    for y in range(min_y, max_y):
        for x in range(min_x, max_x):
            offset = y * message.step + x * 3
            data[offset:offset + 3] = bytes(PANEL)
    message.data = bytes(data)
    return message


def blank_image() -> Image:
    """O mesmo quadro sem painel nenhum."""
    image = image_with_panel()
    image.data = bytes(len(image.data))
    return image


def camera_info(fx: float = 40.0) -> CameraInfo:
    """
    Intrínsecos para a câmera de 64 px deste teste.

    `fx` é parâmetro porque a distância estimada é `fx * 0,8 / largura_px`: qual
    largura cai fora da banda útil depende da lente, e testar os extremos exige
    escolher a lente em que aquele extremo existe.
    """
    info = CameraInfo()
    info.k = [fx, 0.0, 32.0, 0.0, fx, 24.0, 0.0, 0.0, 1.0]
    return info


@pytest.fixture
def node():
    """
    Detector com stride 1 e as saídas capturadas em vez de publicadas.

    `detector_backend` fixado em 'magenta' porque este arquivo testa
    especificamente o portão de confirmação e a geometria do painel magenta
    (`test_maze_exit_detector.py` cobre o backend fiducial); o default do nó
    mudou para 'fiducial' quando a tag foi adicionada.
    """
    rclpy.init()
    detector = MazeExitDetector()
    detector.set_parameters([
        Parameter('sample_stride', Parameter.Type.INTEGER, 1),
        Parameter('detector_backend', Parameter.Type.STRING, 'magenta'),
    ])
    detector.detections = []
    detector.poses = []
    detector._detections_pub.publish = detector.detections.append
    detector._pose_pub.publish = detector.poses.append
    yield detector
    detector.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def confirmed(detector) -> list:
    """As mensagens de detecção que de fato carregam uma caixa."""
    return [message for message in detector.detections if message.detections]


def test_one_frame_is_not_enough_to_interrupt_the_explorer(node) -> None:
    """
    Um quadro só é ruído, e cancelar a meta de fronteira por ruído custa caro.

    O explorador cancela a meta em voo assim que a pose fica fresca. Se um
    reflexo bastasse, ele oscilaria entre `navigating` e `homing_exit` e o robô
    pararia a cada falso positivo.
    """
    node._on_image(image_with_panel())
    assert confirmed(node) == []


def test_three_frames_in_the_window_confirm(node) -> None:
    """3 de 5 é o critério declarado; o terceiro quadro é o que publica."""
    for _ in range(2):
        node._on_image(image_with_panel())
    assert confirmed(node) == []
    node._on_image(image_with_panel())
    assert len(confirmed(node)) == 1


def test_a_gap_inside_the_window_still_confirms(node) -> None:
    """O critério é 3 EM 5, não 3 seguidos: o painel pisca com a marcha."""
    node._on_image(image_with_panel())
    node._on_image(blank_image())
    node._on_image(image_with_panel())
    assert confirmed(node) == []
    node._on_image(image_with_panel())
    assert len(confirmed(node)) == 1


def test_losing_the_panel_drops_below_the_threshold_again(node) -> None:
    """Sair da janela é como o detector diz que perdeu o marcador."""
    for _ in range(3):
        node._on_image(image_with_panel())
    assert len(confirmed(node)) == 1
    for _ in range(5):
        node._on_image(blank_image())
    assert len(confirmed(node)) == 1


def test_the_current_frame_must_itself_be_valid(node) -> None:
    """
    Três confirmações antigas não autorizam publicar sobre um quadro vazio.

    A caixa publicada tem de vir do quadro que acabou de chegar; herdar a
    anterior daria ao explorador uma direção que ninguém está mais vendo.
    """
    for _ in range(3):
        node._on_image(image_with_panel())
    before = len(confirmed(node))
    node._on_image(blank_image())
    assert len(confirmed(node)) == before


def test_detections_are_published_every_frame_even_when_empty(node) -> None:
    """Silêncio e "não vejo nada" têm de ser distinguíveis do lado do consumidor."""
    for _ in range(4):
        node._on_image(blank_image())
    assert len(node.detections) == 4
    assert all(not message.detections for message in node.detections)


def test_a_panel_too_small_is_refused_even_when_repeated(node) -> None:
    """
    Abaixo da caixa mínima a distância estimada não tem precisão nenhuma.

    A distância sai da LARGURA em pixels; a poucos pixels, um pixel de erro na
    borda vira metros de erro no alvo.
    """
    tiny = image_with_panel(box=(30, 22, 36, 28))
    for _ in range(5):
        node._on_image(tiny)
    assert confirmed(node) == []


def test_no_pose_without_camera_info(node) -> None:
    """
    Sem intrínsecos não há distância, e inventar uma é pior do que não publicar.

    Isto NÃO é hipotético no HIL: a imagem chega ao módulo por um caminho
    próprio (comprimida, religada por remap de launch) e o `camera_info` chega
    por outro. Se só um dos dois atravessar, a detecção aparece e a pose nunca
    sai -- e este é o teste que nomeia esse modo de falha.
    """
    for _ in range(4):
        node._on_image(image_with_panel())
    assert confirmed(node) != []
    assert node.poses == []


def test_pose_is_published_in_the_camera_frame_once_intrinsics_arrive(node) -> None:
    """A pose sai no frame do quadro; quem a transforma para `map` é o explorador."""
    node._on_info(camera_info())
    for _ in range(3):
        node._on_image(image_with_panel())
    assert len(node.poses) == 1
    pose = node.poses[0]
    assert pose.header.frame_id == 'front_camera'
    assert pose.pose.position.x > 0.0


def test_a_panel_filling_the_frame_is_too_near_to_be_the_exit(node) -> None:
    """
    Abaixo de 0,3 m o que se vê é uma parede colada na lente, não a saída.

    fx = 20 numa imagem de 64 px é a lente em que "quadro inteiro" cai abaixo
    da banda: 20 x 0,8 / 64 = 0,25 m.
    """
    node._on_info(camera_info(fx=20.0))
    full_frame = image_with_panel(box=(0, 0, 64, 48))
    for _ in range(4):
        node._on_image(full_frame)
    assert confirmed(node) != []
    assert node.poses == []


def test_a_panel_at_the_horizon_is_too_far_to_be_trusted(node) -> None:
    """
    Acima de 8 m a largura em pixels não sustenta a estimativa.

    fx = 200 com uma caixa de 12 px -- a menor que o detector aceita -- dá
    13,3 m: a detecção é publicada, a pose não. A distinção importa: o
    explorador não deve abandonar a fronteira por um marcador que ele ainda não
    consegue medir.
    """
    node._on_info(camera_info(fx=200.0))
    distant = image_with_panel(box=(26, 18, 38, 30))
    for _ in range(4):
        node._on_image(distant)
    assert confirmed(node) != []
    assert node.poses == []


def test_the_marker_detections_never_reach_the_costmap_topic(node) -> None:
    """
    O painel é uma pista visual, não um obstáculo.

    `detections_to_cloud` assina `/demo/perception/detections`. Publicar o
    marcador ali o transformaria em obstáculo no costmap, exatamente em frente
    à abertura que o robô precisa atravessar.
    """
    topic = node._detections_pub.topic_name
    assert topic.endswith('/demo/perception/maze_exit/detections')
    assert not topic.endswith('/demo/perception/detections')
