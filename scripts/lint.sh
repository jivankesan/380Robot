#!/bin/bash
# Lint script for 380Robot
# Runs C++ and Python linters

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$REPO_DIR/ros2_ws"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}Linting 380Robot Code${NC}"
echo "======================"

ERRORS=0

# C++ linting with clang-format
echo -e "${YELLOW}Checking C++ formatting...${NC}"
CPP_FILES=$(find "$WS_DIR/src" -name "*.cpp" -o -name "*.hpp" -o -name "*.h" 2>/dev/null || true)
if [ -n "$CPP_FILES" ]; then
    for file in $CPP_FILES; do
        if ! clang-format --dry-run --Werror "$file" 2>/dev/null; then
            echo -e "${RED}Format issue: $file${NC}"
            ERRORS=$((ERRORS + 1))
        fi
    done
    if [ $ERRORS -eq 0 ]; then
        echo -e "${GREEN}C++ formatting OK${NC}"
    fi
fi

# Python linting with ruff (if available)
echo -e "${YELLOW}Checking Python code...${NC}"
if command -v ruff &> /dev/null; then
    if ! ruff check "$WS_DIR/src"; then
        ERRORS=$((ERRORS + 1))
    else
        echo -e "${GREEN}Python linting OK${NC}"
    fi
else
    echo "ruff not found, skipping Python linting"
fi

# Summary
echo ""
if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}All checks passed!${NC}"
    exit 0
else
    echo -e "${RED}Found $ERRORS issue(s)${NC}"
    exit 1
fi
