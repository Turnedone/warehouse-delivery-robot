#!/usr/bin/env bash
# Formatting check and clang-tidy over the C++ packages, the same as CI.
#   bash tools/lint.sh             check
#   bash tools/lint.sh --fix       reformat the files in place (clang-format only)
# Needs clang-format/clang-tidy 23.1.3 on PATH (pip install clang-format==23.1.3 clang-tidy==23.1.3,
# e.g. in a venv) and, for clang-tidy, a build with compile_commands.json (tools/wsl/build.sh makes one).
set -eo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
build="${WDR_BUILD_DIR:-$HOME/wdr_ws/build}"
cd "$repo"
mapfile -t files < <(find src -name '*.cpp' -o -name '*.hpp' | sort)

if [ "${1:-}" = "--fix" ]; then
    clang-format -i "${files[@]}"
    echo "formatted ${#files[@]} files"
    exit 0
fi

echo "== clang-format ($(clang-format --version))"
clang-format --dry-run --Werror "${files[@]}"

echo "== clang-tidy ($(clang-tidy --version | grep -m1 version))"
status=0
for pkg in wdr_sim wdr_mission; do
    db="$build/$pkg"
    if [ ! -f "$db/compile_commands.json" ]; then
        echo "no $db/compile_commands.json: build first (tools/wsl/build.sh)"; exit 2
    fi
    for f in $(find "src/$pkg/src" -name '*.cpp' | sort); do
        clang-tidy -p "$db" --quiet "$f" || status=1
    done
done
exit $status
