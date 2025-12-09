#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT
#
# Purpose:
#   Enforce documentation of enum members and struct fields in C source files.
#
# Scope:
#   - Python-only (no Clang, no preprocessing)
#   - Best-effort static analysis based on coding conventions
#
# Output:
#   - Lists all checked files
#   - Reports undocumented enum/struct members as:
#       <file>:<line>: undocumented <kind> member '<name>'
#
# Exit code:
#   0 if no issues
#   1 if at least one undocumented member is found
#

import os
import re
import sys
import argparse


# ---------------------------------------------------------------------------
# Regular expressions for member detection
# ---------------------------------------------------------------------------

# Matches a *single enum member line*
#
# Examples matched:
#   kRW,
#   kRO = 1,
#   kWO = 3  ///< Write-only
#
# Capture group 1: enum member name
#
# Notes:
#   - Assumes one enum member per line
#   - Allows optional initializer (= ...)
#   - Allows optional trailing comma
#   - Allows optional trailing ///< documentation comment
#
enumMemberRegex = re.compile(
    r'^\s*'                    # optional leading whitespace
    r'([A-Za-z_]\w*)'           # enum member name (capture group 1)
    r'\s*'
    r'(?:=\s*[^,]+)?'           # optional initializer (non-capturing)
    r'\s*,?\s*'                 # optional trailing comma
    r'(?:///<.*)?'              # optional trailing ///< comment
    r'$'
)


# Matches a *single struct field line*
#
# Examples matched:
#   int a;
#   char *name;  ///< field description
#
# Capture group 1: field name
#
# Notes:
#   - Assumes one field per line
#   - Very intentionally permissive on the type
#   - Stops at the final identifier before ';'
#
structMemberRegex = re.compile(
    r'^\s*'                    # optional leading whitespace
    r'[^;]+'                    # type and modifiers (greedy)
    r'\s+'
    r'([A-Za-z_]\w*)'           # field name (capture group 1)
    r'\s*;'
    r'\s*(?:///<.*)?'           # optional trailing ///< comment
    r'$'
)


# ---------------------------------------------------------------------------
# Helper: detect whether a line *can* start a type declaration
# ---------------------------------------------------------------------------

def isTypeDeclarationLine(line):
    """
    Return True if this line can legitimately start an enum or struct definition.

    Rationale:
      We must avoid false positives such as:
        - function parameters:  struct stat *st
        - function definitions
        - sizeof(struct foo)

    Rule used (heuristic but reliable):
      '{' must appear before any ')' or ';'

    This accepts:
        struct foo {
        typedef struct {
        enum {

    And rejects:
        int f(struct foo *x)
        struct bar *ptr;
    """
    braceIndex = line.find("{")
    if braceIndex == -1:
        return False

    semicolonIndex = line.find(";")
    parenIndex = line.find(")")

    if semicolonIndex != -1 and semicolonIndex < braceIndex:
        return False

    if parenIndex != -1 and parenIndex < braceIndex:
        return False

    return True


# ---------------------------------------------------------------------------
# Extract enum / struct blocks using a small state machine
# ---------------------------------------------------------------------------

def extractBlocks(lines):
    """
    Identify enum and struct *bodies* using brace counting.

    This function does NOT attempt to parse C.
    It only tracks balanced braces after detecting enum/struct keywords.

    Returns:
        List of tuples:
            (kind, [(lineNumber, lineText), ...])

        where kind is either "enum" or "struct"
    """

    blocks = []

    waitingForBrace = False   # saw enum/struct, waiting for '{'
    insideBlock = False       # currently inside enum/struct body
    braceDepth = 0
    blockKind = None
    blockLines = []

    for lineNumber, line in enumerate(lines, start=1):

        # Look for enum / struct keyword that starts a declaration
        if not insideBlock and not waitingForBrace:
            match = re.search(r'\b(enum|struct)\b', line)
            if match and isTypeDeclarationLine(line):
                blockKind = match.group(1)

                # Opening brace on same line
                if '{' in line:
                    insideBlock = True
                    braceDepth = line.count('{') - line.count('}')
                    blockLines = []
                else:
                    # Opening brace will be on a later line
                    waitingForBrace = True
                continue

        # Waiting for the opening brace after enum/struct
        if waitingForBrace:
            if '{' in line:
                insideBlock = True
                waitingForBrace = False
                braceDepth = line.count('{') - line.count('}')
                blockLines = []
            continue

        # Inside enum / struct body
        if insideBlock:
            blockLines.append((lineNumber, line))
            braceDepth += line.count('{') - line.count('}')
            if braceDepth == 0:
                blocks.append((blockKind, blockLines))
                insideBlock = False
                blockKind = None
                blockLines = []

    return blocks


# ---------------------------------------------------------------------------
# Documentation detection helpers
# ---------------------------------------------------------------------------

def hasLeadingDocComment(blockLines, index):
    """
    Check whether the member at blockLines[index] is immediately preceded
    by a /** ... */ documentation comment.

    Rules:
      - Blank lines are skipped
      - Only checks the closest preceding non-empty line
    """
    index -= 1
    while index >= 0:
        line = blockLines[index][1].strip()
        if not line:
            index -= 1
            continue
        return line.endswith("*/")
    return False


# ---------------------------------------------------------------------------
# File-level analysis
# ---------------------------------------------------------------------------

def checkFile(filePath):
    """
    Analyze a single .c or .h file.

    Returns:
        List of diagnostics:
            (filePath, lineNumber, message)
    """
    with open(filePath, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    diagnostics = []
    blocks = extractBlocks(lines)

    for kind, blockLines in blocks:
        for index, (lineNumber, lineText) in enumerate(blockLines):

            stripped = lineText.strip()
            if not stripped or stripped.startswith(("}", "{")):
                continue

            # Select correct member regex
            if kind == "enum":
                match = enumMemberRegex.match(stripped)
            else:
                match = structMemberRegex.match(stripped)

            if not match:
                continue

            memberName = match.group(1)

            hasTrailingDoc = "///<" in stripped
            hasLeadingDoc = hasLeadingDocComment(blockLines, index)

            if not hasTrailingDoc and not hasLeadingDoc:
                diagnostics.append(
                    (
                        filePath,
                        lineNumber,
                        f"undocumented {kind} member '{memberName}'",
                    )
                )

    return diagnostics


# ---------------------------------------------------------------------------
# Directory traversal with ignore support
# ---------------------------------------------------------------------------

def shouldIgnore(path, ignoredDirs):
    """
    Return True if any path component matches one of the ignored directories.
    """
    parts = os.path.normpath(path).split(os.sep)
    return any(part in ignoredDirs for part in parts)


def checkDirectory(rootDirectory, ignoredDirs):
    """
    Walk the directory tree and analyze all .c / .h files.

    Returns:
        (checkedFiles, diagnostics)
    """
    checkedFiles = []
    diagnostics = []

    for root, _, files in os.walk(rootDirectory):
        if shouldIgnore(root, ignoredDirs):
            continue

        for filename in files:
            if filename.endswith((".c", ".h")):
                filePath = os.path.join(root, filename)
                checkedFiles.append(filePath)
                diagnostics.extend(checkFile(filePath))

    return checkedFiles, diagnostics


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", help="Root directory to scan")
    parser.add_argument(
        "-r", "--ignore",
        help="Comma-separated list of directories to ignore",
        default=""
    )
    args = parser.parse_args()

    if not os.path.isdir(args.directory):
        print(f"Error: {args.directory} is not a directory")
        sys.exit(1)

    ignoredDirs = {
        d.strip() for d in args.ignore.split(",") if d.strip()
    }

    checkedFiles, diagnostics = checkDirectory(
        args.directory, ignoredDirs
    )

    # Always list checked files (useful for CI transparency)
    print("Checked files:")
    for filePath in sorted(checkedFiles):
        print(f"  {filePath}")

    if diagnostics:
        print("\nErrors:")
        for filePath, lineNumber, message in diagnostics:
            print(f"{filePath}:{lineNumber}: {message}")
        sys.exit(1)

    print("\nAll enum and struct members are documented.")
    sys.exit(0)


if __name__ == "__main__":
    main()
