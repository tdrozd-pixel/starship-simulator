"""PostToolUse hook: flag em dashes in files Claude just wrote or edited.

CLAUDE.md asks for no em dashes in docs and comments. This reads the hook
payload from stdin, scans the edited file, and exits 2 with the offending
lines on stderr so Claude sees them and fixes them. Vendored and generated
files are skipped. Never blocks on its own errors.
"""

import json
import sys
from pathlib import Path

EM_DASH = "—"
SKIP_PARTS = {"vendor", "build", ".git", "_deps", "node_modules"}
MAX_REPORTED = 10


def main() -> int:
    try:
        payload = json.load(sys.stdin)
    except (json.JSONDecodeError, ValueError):
        return 0

    tool_input = payload.get("tool_input") or {}
    path_str = tool_input.get("file_path") or (payload.get("tool_response") or {}).get("filePath")
    if not path_str:
        return 0

    path = Path(path_str)
    if any(part in SKIP_PARTS for part in path.parts) or not path.is_file():
        return 0

    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return 0  # binary or unreadable: not our concern

    hits = [(n, line.strip()) for n, line in enumerate(text.splitlines(), 1) if EM_DASH in line]
    if not hits:
        return 0

    print(f"{path.name} contains {len(hits)} line(s) with em dashes (CLAUDE.md: avoid em dashes). "
          "Rewrite them with commas, colons, parentheses or separate sentences:", file=sys.stderr)
    for n, line in hits[:MAX_REPORTED]:
        print(f"  line {n}: {line[:160]}", file=sys.stderr)
    if len(hits) > MAX_REPORTED:
        print(f"  ... and {len(hits) - MAX_REPORTED} more", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
