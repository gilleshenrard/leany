# SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#!/bin/bash
print_violation_section() {
  local json_data="$1"
  local section_path="$2"
  local section_label="$3"
  
  if [ -n "$section_label" ]; then
    echo "$section_label:"
  fi
  
  # Handle cases where the section might be null or empty
  echo "$json_data" | jq -r "
    if (${section_path} | type) == \"array\" and (${section_path} | length) > 0 then
      ${section_path} | to_entries[] | 
        \"    Violation: \(.value.description)\",
        (if .value.items then (.value.items[] | \"        Item: \(.description)\") else empty end)
    else
      empty
    end
  "
}
