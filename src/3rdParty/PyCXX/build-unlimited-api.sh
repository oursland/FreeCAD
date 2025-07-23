#!/bin/bash
set -x
set -e
set -o pipefail

PYTHON=${1? python exe}
shift 1

case "$( uname )" in
Darwin)
    OS=macosx
    ;;
Linux):
    OS=linux
    ;;
*)
    echo Unknown OS assuming Linux
    OS=linux
    ;;
esac

PYTHON_BASE=$(basename ${PYTHON})
TAG="${1}${2}${3}${4}"
${PYTHON} setup_makefile.py ${OS} tmp-${PYTHON_BASE}-unlimited-api${TAG}.mak "${@}"
make -f tmp-${PYTHON_BASE}-unlimited-api${TAG}.mak clean 2>&1 | tee tmp-${PYTHON_BASE}-unlimited-api${TAG}.log
make -f tmp-${PYTHON_BASE}-unlimited-api${TAG}.mak test 2>&1 | tee -a tmp-${PYTHON_BASE}-unlimited-api${TAG}.log
