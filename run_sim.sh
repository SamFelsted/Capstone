#!/bin/sh
set -eu

project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$project_dir"

if [ ! -x .venv/bin/python ]; then
    printf '%s\n' 'Create .venv and install requirements.txt first; see README.md.' >&2
    exit 1
fi

export PYTHONPATH="$project_dir/build/python${PYTHONPATH:+:$PYTHONPATH}"
exec .venv/bin/python -m sim.ui.main "$@"
