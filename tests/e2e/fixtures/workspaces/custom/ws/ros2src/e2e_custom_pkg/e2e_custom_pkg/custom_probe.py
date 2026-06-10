from __future__ import annotations

from e2e_msgs.msg import Ping


def main() -> None:
    msg = Ping()
    msg.seq = 7
    msg.label = "custom-ok"
    print(f"E2E_CUSTOM:{msg.seq}:{msg.label}", flush=True)
