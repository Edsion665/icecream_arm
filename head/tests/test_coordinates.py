from src.config import Settings
from src.coordinates import apply_role_z_offset
from src.models import Position


def test_z_offsets() -> None:
    s = Settings()
    p0 = Position(1.0, 2.0, 0.1)
    assert apply_role_z_offset(p0, "target", s).z == 0.5
    assert apply_role_z_offset(p0, "object", s).z == 0.3
    assert apply_role_z_offset(p0, "lid", s).z == 0.1
