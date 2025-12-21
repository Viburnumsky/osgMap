#!/usr/bin/env bash
set -euo pipefail

MAIN_BRANCH=${1:-origin/main}
CLANG_BIN=${2:-clang-format-14}

if ! command -v "$CLANG_BIN" >/dev/null 2>&1; then
  echo "clang-format binary not found: $CLANG_BIN"
  exit 1
fi

# Upewnij się, że mamy referencję
git fetch origin main >/dev/null 2>&1 || true

OUTPUT=$(git-clang-format \
  --diff "$MAIN_BRANCH" \
  --extensions c,cpp,h,hpp \
  --binary "$CLANG_BIN")

# Brak zmian
if echo "$OUTPUT" | grep -qi "no modified files to format"; then
  exit 0
fi

# Zmiany są poprawnie sformatowane
if echo "$OUTPUT" | grep -qi "did not modify any files"; then
  exit 0
fi

echo
echo "/// code format check failed!"
echo
echo "$OUTPUT"
exit 1
