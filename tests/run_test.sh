#!/bin/bash

OUTPUT_DIR=~/Sunray/tests/output
mkdir -p $OUTPUT_DIR

echo "可用测试列表："

options=()
i=1

for file in production/*_test.sh; do
    name=$(basename "$file")
    echo "$i) $name"
    options[$i]=$file
    ((i++))
done

echo ""
read -p "请选择测试编号: " choice

selected=${options[$choice]}

if [ -z "$selected" ]; then
    echo "无效选择"
    exit 1
fi

echo "🚀 执行: $selected"
bash "$selected"
