"""Conftest: mock unavailable ROS message packages before any test imports."""
import sys
from types import ModuleType


def _make_msg_class(fields_dict):
    defaults = dict(fields_dict)

    class _Msg:
        def __init__(self, **kwargs):
            for k, v in defaults.items():
                setattr(self, k, kwargs.get(k, v))

        def get_fields_and_field_types(self):
            return {k: 'boolean' if isinstance(v, bool) else type(v).__name__
                    for k, v in defaults.items()}

    return _Msg


# --- unitree_go.msg.WirelessController ---
_unitree_pkg = ModuleType('unitree_go')
_unitree_msg = ModuleType('unitree_go.msg')
_unitree_msg.WirelessController = _make_msg_class(
    {'lx': 0.0, 'ly': 0.0, 'rx': 0.0, 'ry': 0.0, 'keys': 0}
)
_unitree_pkg.msg = _unitree_msg
sys.modules.setdefault('unitree_go', _unitree_pkg)
sys.modules.setdefault('unitree_go.msg', _unitree_msg)

# --- blind_locomotion.msg.Button ---
# The real blind_locomotion package is installed, but its .msg submodule
# comes from ROS2 code generation and may not be available.  Inject a stub
# only if the real one is missing.
_bl_button_fields = {
    'up': False, 'down': False, 'start': False, 'select': False,
    'a': False, 'b': False, 'emergency_sit': False,
}
try:
    from blind_locomotion.msg import Button  # noqa: F401
except (ImportError, ModuleNotFoundError):
    import blind_locomotion as _bl_pkg
    _bl_msg = ModuleType('blind_locomotion.msg')
    _bl_msg.Button = _make_msg_class(_bl_button_fields)
    _bl_pkg.msg = _bl_msg  # type: ignore[attr-defined]
    sys.modules['blind_locomotion.msg'] = _bl_msg
