#!/bin/bash
source ./tests/Helpers/test_utils.sh

FILENAME=F.out
INPUT=$'6
0 0 0
100 0 0
0 100 0
0 0 100
20 20 20
30 20 10
4
1 1 1
30 30 35
7 8 9
90 2 2'
EXPECTED=$'1.0000000000
2.8867513459
7.0000000000
2.0000000000'

run_test "./$FILENAME" "$INPUT" "$EXPECTED" "Basic test"