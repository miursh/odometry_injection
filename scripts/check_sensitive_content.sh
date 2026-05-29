#!/usr/bin/env bash

set -euo pipefail

echo "Running sensitive-content scan..."

# High-signal patterns only to reduce false positives.
PATTERN='(AKIA[0-9A-Z]{16}|ghp_[A-Za-z0-9]{36,}|AIza[0-9A-Za-z_-]{35}|-----BEGIN [A-Z ]*PRIVATE KEY-----|xox[baprs]-[A-Za-z0-9-]{10,}|/media/[A-Za-z0-9._/-]+|/home/[A-Za-z0-9._/-]+)'

if git grep -nI -E "$PATTERN" -- . ':(exclude).git' ':(exclude)LICENSE'; then
  echo
  echo "Sensitive-content scan FAILED."
  echo "Remove or mask sensitive values before pushing."
  exit 1
fi

echo "Sensitive-content scan passed."
