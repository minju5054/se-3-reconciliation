"""Instruction-only configuration fixtures; no actual model/controller calls."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from run_join_online03 import INSTRUCTION, CONFIG, variant_config
from reconciliation.join_online02 import ORDER, INSTRUCTION as HISTORICAL_INSTRUCTION


def changes(a, b, prefix=''):
    result = []
    for key in sorted(a.keys() | b.keys()):
        path = prefix+'.'+key if prefix else key
        if key not in a or key not in b:
            result.append(path)
        elif isinstance(a[key], dict) and isinstance(b[key], dict):
            result.extend(changes(a[key], b[key], path))
        elif a[key] != b[key]:
            result.append(path)
    return result


def test_user_instruction_exact_and_four_episode_order():
    cfg = yaml.safe_load(CONFIG.read_text())
    assert cfg['instruction'] == INSTRUCTION == (
        'Go to the far end of the hallway. Pass around the supply cart without touching it, '
        'and stop only when you reach the end of the hallway.')
    assert cfg['order'] == ORDER and cfg['repeats'] == 2
    assert HISTORICAL_INSTRUCTION == 'Avoid the supply cart and continue to the end of the hallway.'


def test_only_instruction_and_experiment_label_change():
    declaration = yaml.safe_load(CONFIG.read_text())
    original = yaml.safe_load((ROOT / 'configs/robotless_online_handoffs.yaml').read_text())
    original['join_online02'] = yaml.safe_load((ROOT / 'configs/join_online_02_far_approach.yaml').read_text())
    before = deepcopy(original)
    actual = variant_config(original, declaration)
    assert changes(before, actual) == ['join_online02.experiment', 'join_online02.instruction']
    assert original == before
    assert actual['join_online02']['classification'] == before['join_online02']['classification']
    assert actual['join_online02']['guard'] == before['join_online02']['guard']
    assert actual['join_online02']['generation_environment'] == before['join_online02']['generation_environment']
    assert actual['camera'] == before['camera'] and actual['online'] == before['online']


@pytest.mark.parametrize('field,value', [('instruction', 'Go around the cart on the left.'), ('order', ORDER[::-1])])
def test_undeclared_condition_rejected(field, value):
    declaration = yaml.safe_load(CONFIG.read_text())
    declaration[field] = value
    with pytest.raises(ValueError, match='undeclared'):
        variant_config({}, declaration)


def test_historical_numerical_and_collection_implementation_reused():
    script = (ROOT / 'scripts/run_join_online03.py').read_text()
    assert 'from reconciliation.join_online02 import ORDER, guard_check, ABORT' in script
    assert 'from validate_robotless_online_handoffs import validate_episode, equal_record' in script
    assert 'scripts/analyze_join_online02.py' in script
    assert 'from report_join_online02 import presentation as legacy_presentation' in script
    assert 'monkey' not in script and 'solve_gp(' not in script
