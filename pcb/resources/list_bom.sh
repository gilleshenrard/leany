# SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#!/bin/sh

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 path/to/BOM-JLCPCB.csv"
    exit 1
fi

BOM_FILE="$1"

if [ ! -f "$BOM_FILE" ]; then
    echo "ERROR: File not found: $BOM_FILE"
    exit 1
fi

echo " "
echo "Listing BOM components from $BOM_FILE"

python3 - "$BOM_FILE" << 'PYEOF'
import csv
import re
import sys

def strip_library_prefix(value):
    """Remove any 'LibraryName:' prefix from a field value."""
    return re.sub(r'[^,:"]*:', "", value)

bom_file = sys.argv[1]

print()
print(f"{'LCSC':<14} {'Comment':<20} {'Qty':<5} Designator")
print("-" * 71)

with open(bom_file, newline="", encoding="utf-8") as f:
    for row in csv.DictReader(f):
        lcsc       = strip_library_prefix(row["LCSC"])
        comment    = strip_library_prefix(row["Comment"])
        qty        = row["Quantity"]
        designator = strip_library_prefix(row["Designator"])
        print(f"{lcsc:<14} {comment:<20} {qty:<5} {designator}")
PYEOF