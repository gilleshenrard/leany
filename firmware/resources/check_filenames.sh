#!/bin/bash

# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

invalid=0

while read -r file; do
  baseName="$(basename "$file")"
  if ! printf "%s\n" "$baseName" | grep -qE '^[a-z0-9_]+\.[ch]$'; then
    printf 'Invalid filename: %s\n' "$file"
    invalid=1
  fi
done < <(find firmware/Hardware firmware/UI -type f \( -name "*.c" -o -name "*.h" \))

exit "$invalid"
