#!/bin/sh
# Run the checked-out editor from any working directory on macOS or Linux.
set -eu
case $0 in
    */*) script_dir=${0%/*} ;;
    *) script_dir=. ;;
esac
editor_dir=$(CDPATH= cd "$script_dir" && pwd)

if command -v node >/dev/null 2>&1 &&
   node -e 'process.exit(Number(process.versions.node.split(".")[0]) >= 16 ? 0 : 1)' >/dev/null 2>&1; then
    exec node "$editor_dir/tools/recipe-editor/serve.cjs" "$@"
fi
for candidate in python3 python; do
    if command -v "$candidate" >/dev/null 2>&1 &&
       "$candidate" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' >/dev/null 2>&1; then
        exec "$candidate" "$editor_dir/tools/recipe-editor/serve.py" "$@"
    fi
done

echo 'Se necesita Node.js 16+ o Python 3.10+ para iniciar el editor en macOS/Linux.' >&2
exit 1
