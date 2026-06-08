#!/bin/bash
# 校验所有文档中的 [VERIFY:] 标签:抽出每个 file:line[-line] 段验证文件存在 + 行号在范围内。
# 支持复合标签(同一个 [VERIFY:] 内多 file)、跨行范围、:line 简写(沿用上一文件)。
set -u
cd "$(dirname "$0")/.."
DOCS_DIR="docs"

TOTAL=0
MISSING_FILE=0
OUT_OF_RANGE=0
OK=0
BAD_TAGS=()

# 抽出所有 tag(去 [VERIFY:  / ])
mapfile -t TAGS < <(grep -h -oE '\[VERIFY: [^]]+\]' "$DOCS_DIR"/*.md | sort -u)

for tag in "${TAGS[@]}"; do
  inner="${tag#\[VERIFY: }"
  inner="${inner%]}"
  # 跳过 "file:line" 这种文档示例
  if [[ "$inner" == "file:line" || "$inner" == "<path>:<line>" ]]; then
    continue
  fi

  # 把 inner 按逗号拆;每段可能是 "path:N[-M]" 或 ":N[-M]"(沿用上一 path)或 "path 备注"
  IFS=',' read -ra SEGMENTS <<< "$inner"
  cur_path=""
  for seg in "${SEGMENTS[@]}"; do
    # trim
    seg="$(echo -e "${seg}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
    # 抽 file 与 line range:形如 path:N 或 path:N-M 或 :N 或 :N-M
    if [[ "$seg" =~ ^([^:[:space:]][^:]*):([0-9]+)(-([0-9]+))?([[:space:]].*)?$ ]]; then
      cur_path="${BASH_REMATCH[1]}"
      first="${BASH_REMATCH[2]}"
      last="${BASH_REMATCH[4]:-$first}"
    elif [[ "$seg" =~ ^:([0-9]+)(-([0-9]+))?([[:space:]].*)?$ ]] && [[ -n "$cur_path" ]]; then
      first="${BASH_REMATCH[1]}"
      last="${BASH_REMATCH[3]:-$first}"
    elif [[ "$seg" =~ ^([0-9]+)(-([0-9]+))?$ ]] && [[ -n "$cur_path" ]]; then
      # 纯数字也按上一 path 处理
      first="${BASH_REMATCH[1]}"
      last="${BASH_REMATCH[3]:-$first}"
    else
      # 非数字段(如 "freeze_decoders/unfreeze_decoders" / "文件全文")— 跳过,不算错
      continue
    fi

    TOTAL=$((TOTAL + 1))

    if [[ ! -f "$cur_path" ]]; then
      MISSING_FILE=$((MISSING_FILE + 1))
      BAD_TAGS+=("MISSING_FILE: $cur_path  ←  $tag")
      continue
    fi
    total_lines=$(wc -l < "$cur_path")
    if (( first > total_lines || last > total_lines )); then
      OUT_OF_RANGE=$((OUT_OF_RANGE + 1))
      BAD_TAGS+=("OUT_OF_RANGE: $cur_path:$first-$last (file has $total_lines lines)  ←  $tag")
      continue
    fi
    OK=$((OK + 1))
  done
done

echo "================================="
echo "VERIFY tag audit"
echo "================================="
echo "Validated segments: $TOTAL"
echo "OK:                 $OK"
echo "MISSING_FILE:       $MISSING_FILE"
echo "OUT_OF_RANGE:       $OUT_OF_RANGE"
echo

if [[ ${#BAD_TAGS[@]} -gt 0 ]]; then
  echo "Failures:"
  for b in "${BAD_TAGS[@]}"; do
    echo "  - $b"
  done
fi
