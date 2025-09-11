#!/bin/bash

# Аргументы:
#   $1 — исполняемый файл (например, "./D")
#   $2 — входные данные (строка или путь к файлу)
#   $3 — ожидаемый вывод (строка или путь к файлу)
#   $4 — название теста (опционально)
run_test() {
    local executable="$1"
    local input="$2"
    local expected_output="$3"
    local test_name="${4:-unnamed test}"

    echo "Running test: '$test_name' with $executable"

    if [[ -f "$input" ]]; then
        actual_output=$("$executable" < "$input")
    else
        actual_output=$(echo "$input" | "$executable")
    fi

    if [[ -f "$expected_output" ]]; then
        expected_content=$(cat "$expected_output")
    else
        expected_content="$expected_output"
    fi

    if diff -b -y <(echo "$expected_content") <(echo "$actual_output") > /dev/null; then
        echo -e "\e[32m✓ PASSED: '$test_name'\e[0m"
        return 0
    else
        echo -e "\e[31m✗ FAILED: '$test_name'\e[0m"
        echo "=== Expected ==="
        echo "$expected_content"
        echo "=== Actual ==="
        echo "$actual_output"
        echo "=== Diff ==="
        diff -b -y <(echo "$expected_content") <(echo "$actual_output")
        return 1
    fi
}