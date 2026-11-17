"""JSON + human diagnostics reports (FR-HW-65)."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

from fret.diag.core import CheckResult, CheckStatus, exit_code_for


def default_report_dir() -> Path:
    return Path("/var/log/fret/diag")


def persistent_report_dir() -> Path:
    return Path("/var/lib/fret/diag")


def write_report(
    command: str,
    results: Sequence[CheckResult],
    *,
    report_dir: Path | None = None,
    save_persistent: bool = False,
) -> Path:
    """Write JSON report; return the path written."""
    directory = report_dir or default_report_dir()
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = directory / f"{stamp}-{command}.json"
    payload: dict[str, Any] = {
        "command": command,
        "timestamp_utc": stamp,
        "exit_code": exit_code_for(list(results)),
        "results": [r.to_dict() for r in results],
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if save_persistent:
        dest_dir = persistent_report_dir()
        dest_dir.mkdir(parents=True, exist_ok=True)
        dest = dest_dir / path.name
        dest.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    return path


def format_human(results: Sequence[CheckResult]) -> str:
    lines: list[str] = []
    for result in results:
        mark = {
            CheckStatus.PASS: "PASS",
            CheckStatus.FAIL: "FAIL",
            CheckStatus.SKIP: "SKIP",
        }[result.status]
        lines.append(f"[{mark}] {result.name}: {result.message}")
    return "\n".join(lines)
