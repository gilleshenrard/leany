#!/bin/bash

# SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#
# Find all *.c, *.h and *.inc user-defined files and check whether their path
#   follows the following pattern :
# - all-lowercase path
# - numbers are allowed
# - underscores are allwowed
#

invalid=0
while read -r file; do
    #baseName="$(basename "$file")"
    printf 'Checking %s : ' "$file"
    if ! printf "%s\n" "$file" | grep -qE '^[a-z0-9_\/]+\.([ch]|(inc))$'; then
        printf 'Invalid filename\n'
        invalid=1
    else
        printf 'OK\n'
    fi
done < <(find firmware/hardware firmware/ui firmware/dispatcher firmware/utilities -type f \( -name "*.c" -o -name "*.h" -o -name "*.inc" \))

exit "$invalid"
