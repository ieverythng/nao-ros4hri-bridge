from nao_scene_grounding.detector_adapters import EmorobcareDetectionAdapter
from nao_scene_grounding.detector_adapters import YoloRosDetectionAdapter
from nao_scene_grounding.detector_adapters import build_entity_id
from nao_scene_grounding.detector_adapters import load_label_class_map
from nao_scene_grounding.detector_adapters import sanitize_kb_class


class _Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _Pose:
    def __init__(self, x, y):
        self.position = _Point(x, y)


class _BBox:
    def __init__(self, x, y):
        self.center = _Pose(x, y)


class _Detection:
    def __init__(self, class_name, score, tracker_id, x, y):
        self.class_name = class_name
        self.score = score
        self.id = tracker_id
        self.bbox = _BBox(x, y)


class _DetectionArray:
    def __init__(self, detections):
        self.detections = detections


class _EmoDetection:
    def __init__(self, label, confidence, x1, y1, x2, y2):
        self.label = label
        self.confidence = confidence
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2


class _EmoDetectionArray:
    def __init__(self, detections):
        self.detections = detections


def test_sanitize_kb_class_title_cases_labels():
    assert sanitize_kb_class('cell phone') == 'CellPhone'


def test_build_entity_id_prefers_tracker_id():
    entity_id = build_entity_id(
        entity_prefix='detected',
        label='cell phone',
        tracker_id='12',
        center_x=100.0,
        center_y=200.0,
    )
    assert entity_id == 'detected_cell_phone_12'


def test_load_label_class_map_merges_json_overrides():
    label_map = load_label_class_map('{"cup":"DrinkingCup"}')
    assert label_map['cup'] == 'DrinkingCup'
    assert label_map['book'] == 'Book'


def test_yolo_adapter_filters_to_allowed_labels_and_scores():
    adapter = YoloRosDetectionAdapter(
        allowed_labels=['cup'],
        label_class_map=load_label_class_map('{}'),
    )
    msg = _DetectionArray(
        [
            _Detection('cup', 0.91, '7', 32.5, 19.0),
            _Detection('chair', 0.87, '2', 51.0, 18.0),
            _Detection('cup', 0.05, '9', 88.0, 11.0),
        ]
    )

    observations = adapter.parse_detections(msg, min_score=0.35)

    assert len(observations) == 1
    assert observations[0].label == 'cup'
    assert observations[0].kb_class == 'Cup'
    assert observations[0].entity_id == 'detected_cup_7'


def test_emorobcare_adapter_uses_bbox_center_when_no_tracker_exists():
    adapter = EmorobcareDetectionAdapter(
        allowed_labels=['tomato'],
        label_class_map=load_label_class_map('{}'),
    )
    msg = _EmoDetectionArray(
        [
            _EmoDetection('tomato', 0.88, 10.0, 20.0, 50.0, 60.0),
            _EmoDetection('pear', 0.93, 1.0, 2.0, 3.0, 4.0),
        ]
    )

    observations = adapter.parse_detections(msg, min_score=0.35)

    assert len(observations) == 1
    assert observations[0].label == 'tomato'
    assert observations[0].kb_class == 'Tomato'
    assert observations[0].entity_id == 'detected_tomato_30_40'
