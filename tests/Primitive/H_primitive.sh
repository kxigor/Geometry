#!/bin/bash
source ./tests/Helpers/test_utils.sh

FILENAME=build/bin/H
INPUT=$'11
0 0
6 0
1 1
5 1
2 2
4 2
5 4
4 4
2 5
6 6
0 6
6
6 6
5 5
5 4
4 5
2 4
3 3
'
EXPECTED=$'0
0
1
0
1
2'

run_test "./$FILENAME" "$INPUT" "$EXPECTED" "Basic test"