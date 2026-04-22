#!/bin/bash

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRODUCTION_DIR="$ROOT_DIR/production"

echo "可用测试列表："

options=()
i=1

for file in "$PRODUCTION_DIR"/*_test.sh; do
    [ -f "$file" ] || continue
    name="$(basename "$file" .sh)"
    echo "$i) $name"
    options[$i]="$file"
    ((i++))
done

if [ "$i" -eq 1 ]; then
    echo "未找到可用测试脚本: $PRODUCTION_DIR/*_test.sh"
    exit 1
fi

echo ""
read -p "请选择测试编号: " choice

selected=${options[$choice]}

if [ -z "$selected" ]; then
    echo "无效选择"
    exit 1
fi

echo "启动测试: $(basename "$selected" .sh)"
bash "$selected"
