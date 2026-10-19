"""Output relocation only; the Stage 02.4 experiment design is frozen."""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(os.environ.get('STAGE02_4_CONFIG', ROOT / 'configs/stage02_4_separation.json')).resolve()
DESIGN = json.loads(CONFIG.read_text())
OUT = ROOT / DESIGN['output']
# Input paths, model and metrics are deliberately not a general search interface.
_reference = json.loads((ROOT / 'configs/stage02_4_separation.json').read_text())
assert {k: v for k, v in DESIGN.items() if k != 'output'} == {
    k: v for k, v in _reference.items() if k != 'output'
}, 'Only output relocation is supported; do not silently alter the frozen experiment.'
