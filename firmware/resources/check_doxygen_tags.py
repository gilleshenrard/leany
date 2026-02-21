# SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#!/usr/bin/env python3
"""
check_doxygen_file_tags.py

Scans all .c and .h files under a given directory and reports any that are
missing a Doxygen @file (or \\file) documentation block. Exits with code 1
if violations are found, 0 otherwise.

Usage:
    python check_doxygen_file_tags.py <directory>

Example:
    python check_doxygen_file_tags.py firmware/
"""

import argparse
import re
import sys
from pathlib import Path

# Matches /** ... @file ... */ or /*! ... \file ... */ blocks anywhere in the file.
_FILE_TAG_PATTERN = re.compile(r"/\*[*!].*?(?:@|\\)file\b.*?\*/", re.DOTALL)

_EXTENSIONS = (".c", ".h")
_MAX_VIOLATIONS = 1000  # NASA rule: fixed upper bound on iteration


def _has_file_tag(path: Path) -> bool:
    """Return True if the file contains a Doxygen @file or \\file block."""
    try:
        content = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        print(f"WARNING: could not read {path}: {exc}", file=sys.stderr)
        return True  # don't flag unreadable files as violations
    return bool(_FILE_TAG_PATTERN.search(content))


def _is_excluded(path: Path, excluded: list[Path]) -> bool:
    """Return True if path is inside any of the excluded directories."""
    return any(excluded_dir in path.parents for excluded_dir in excluded)


def _collect_sources(root: Path, excluded: list[Path]) -> list[Path]:
    """Return all .c/.h files under root, excluding the specified directories."""
    sources = []
    for ext in _EXTENSIONS:
        for path in sorted(root.rglob(f"*{ext}")):
            if not _is_excluded(path, excluded):
                sources.append(path)
    return sources


def _parse_args() -> argparse.Namespace:
    """Parse and return command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Check that all .c/.h files contain a Doxygen @file block."
    )
    parser.add_argument(
        "directory",
        type=Path,
        help="Root directory to scan.",
    )
    parser.add_argument(
        "--exclude",
        metavar="DIR",
        type=Path,
        action="append",
        default=[],
        help="Directory to exclude (relative to the root, or absolute). Repeatable.",
    )
    return parser.parse_args()


def main() -> int:
    """Entry point. Returns exit code."""
    args = _parse_args()

    root = args.directory
    if not root.is_dir():
        print(f"ERROR: '{root}' is not a directory.", file=sys.stderr)
        return 2

    # Resolve exclusions relative to cwd so parent-check works correctly
    excluded = [p.resolve() for p in args.exclude]
    for excl in excluded:
        if not excl.is_dir():
            print(f"WARNING: excluded path '{excl}' is not a directory.", file=sys.stderr)

    sources = _collect_sources(root.resolve(), excluded)
    violations = []

    for path in sources[:_MAX_VIOLATIONS]:
        if not _has_file_tag(path):
            violations.append(path)

    if violations:
        print(f"FAIL: {len(violations)} file(s) missing a Doxygen @file block:\n")
        for path in violations:
            print(f"  {path}")
        print()
        return 1

    print(f"OK: all {len(sources)} file(s) have a Doxygen @file block.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
