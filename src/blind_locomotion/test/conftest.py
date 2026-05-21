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
_unitree_msg.LowState = _make_msg_class(
    {
        'imu_state': _make_msg_class(
            {
                'gyroscope': [0.0, 0.0, 0.0],
                'quaternion': [1.0, 0.0, 0.0, 0.0],
            }
        )(),
        'motor_state': [],
    }
)
_unitree_msg.SportModeState = _make_msg_class(
    {
        'mode': 1,
        'body_height': 0.32,
        'velocity': [0.0, 0.0, 0.0],
        'yaw_speed': 0.0,
    }
)
_unitree_pkg.msg = _unitree_msg
sys.modules.setdefault('unitree_go', _unitree_pkg)
sys.modules.setdefault('unitree_go.msg', _unitree_msg)

# --- unitree_api.msg.Request/Response ---
try:
    from unitree_api.msg import Request, Response  # noqa: F401
except (ImportError, ModuleNotFoundError):
    _unitree_api_pkg = ModuleType('unitree_api')
    _unitree_api_msg = ModuleType('unitree_api.msg')

    class _RequestIdentity:
        def __init__(self):
            self.id = 0
            self.api_id = 0

    class _RequestLease:
        def __init__(self):
            self.id = 0

    class _RequestPolicy:
        def __init__(self):
            self.priority = 0
            self.noreply = False

    class _RequestHeader:
        def __init__(self):
            self.identity = _RequestIdentity()
            self.lease = _RequestLease()
            self.policy = _RequestPolicy()

    class _Request:
        def __init__(self):
            self.header = _RequestHeader()
            self.parameter = ''
            self.binary = []

    class _Response:
        def __init__(self):
            self.header = _RequestHeader()
            self.data = ''

    _unitree_api_msg.Request = _Request
    _unitree_api_msg.Response = _Response
    _unitree_api_pkg.msg = _unitree_api_msg
    sys.modules.setdefault('unitree_api', _unitree_api_pkg)
    sys.modules.setdefault('unitree_api.msg', _unitree_api_msg)

# --- blind_locomotion.msg.Button ---
# The real blind_locomotion package is installed, but its .msg submodule
# comes from ROS2 code generation and may not be available.  Inject a stub
# only if the real one is missing.
_bl_button_fields = {
    'up': False, 'down': False, 'start': False, 'select': False,
    'a': False, 'b': False, 'emergency_sit': False, 'f1': False,
}
try:
    from blind_locomotion.msg import Button  # noqa: F401
except (ImportError, ModuleNotFoundError):
    import blind_locomotion as _bl_pkg
    _bl_msg = ModuleType('blind_locomotion.msg')
    _bl_msg.Button = _make_msg_class(_bl_button_fields)
    _bl_pkg.msg = _bl_msg  # type: ignore[attr-defined]
    sys.modules['blind_locomotion.msg'] = _bl_msg

# --- geometry_msgs.msg.Pose/Twist ---
try:
    from geometry_msgs.msg import Pose, Twist  # noqa: F401
except (ImportError, ModuleNotFoundError):
    _geometry_pkg = ModuleType('geometry_msgs')
    _geometry_msg = ModuleType('geometry_msgs.msg')
    _geometry_msg.Pose = _make_msg_class(
        {
            'position': _make_msg_class({'x': 0.0, 'y': 0.0, 'z': 0.0})(),
            'orientation': _make_msg_class({'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})(),
        }
    )
    _geometry_msg.Twist = _make_msg_class(
        {
            'linear': _make_msg_class({'x': 0.0, 'y': 0.0, 'z': 0.0})(),
            'angular': _make_msg_class({'x': 0.0, 'y': 0.0, 'z': 0.0})(),
        }
    )
    _geometry_pkg.msg = _geometry_msg
    sys.modules.setdefault('geometry_msgs', _geometry_pkg)
    sys.modules.setdefault('geometry_msgs.msg', _geometry_msg)

# --- nav_msgs.msg.Odometry ---
try:
    from nav_msgs.msg import Odometry  # noqa: F401
except (ImportError, ModuleNotFoundError):
    _nav_pkg = ModuleType('nav_msgs')
    _nav_msg = ModuleType('nav_msgs.msg')
    _nav_msg.Odometry = _make_msg_class(
        {
            'twist': _make_msg_class(
                {
                    'twist': _make_msg_class(
                        {
                            'linear': _make_msg_class({'x': 0.0, 'y': 0.0, 'z': 0.0})(),
                        }
                    )(),
                }
            )(),
            'header': _make_msg_class({'frame_id': ''})(),
            'child_frame_id': '',
        }
    )
    _nav_pkg.msg = _nav_msg
    sys.modules.setdefault('nav_msgs', _nav_pkg)
    sys.modules.setdefault('nav_msgs.msg', _nav_msg)

# --- std_msgs.msg.Bool/Float32/Float32MultiArray ---
try:
    from std_msgs.msg import Bool, Float32, Float32MultiArray  # noqa: F401
except (ImportError, ModuleNotFoundError):
    _std_pkg = ModuleType('std_msgs')
    _std_msg = ModuleType('std_msgs.msg')
    _std_msg.Bool = _make_msg_class({'data': False})
    _std_msg.Float32 = _make_msg_class({'data': 0.0})
    _std_msg.Float32MultiArray = _make_msg_class({'data': []})
    _std_pkg.msg = _std_msg
    sys.modules.setdefault('std_msgs', _std_pkg)
    sys.modules.setdefault('std_msgs.msg', _std_msg)

# --- std_srvs.srv.Trigger ---
try:
    from std_srvs.srv import Trigger  # noqa: F401
except (ImportError, ModuleNotFoundError):
    _std_srvs_pkg = ModuleType('std_srvs')
    _std_srvs_srv = ModuleType('std_srvs.srv')
    _std_srvs_srv.Trigger = type(
        'Trigger',
        (),
        {
            'Request': type('Request', (), {}),
            'Response': _make_msg_class({'success': False, 'message': ''}),
        },
    )
    _std_srvs_pkg.srv = _std_srvs_srv
    sys.modules.setdefault('std_srvs', _std_srvs_pkg)
    sys.modules.setdefault('std_srvs.srv', _std_srvs_srv)
