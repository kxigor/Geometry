#!/bin/bash
source ./tests/Helpers/test_utils.sh

FILENAME=build/bin/D
INPUT=$'4 4
400 400
600 400
600 600
400 600
0 0
1000 0
1000 1000
0 1000'
EXPECTED=$'540.00000000000000000000'

run_test "./$FILENAME" "$INPUT" "$EXPECTED" "Basic test"