from __future__ import annotations

from std_msgs.msg import String


def main() -> None:
    msg = String()
    msg.data = "std-ok"
    print(f"E2E_STD:{msg.data}", flush=True)
