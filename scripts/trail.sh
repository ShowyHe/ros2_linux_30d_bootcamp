#!/usr/bin/env bash
set -u
# 当脚本使用到未定义变量（例如 $foo）时，报错并退出

emit_line() {
  local level="$1"
  local status="$2"
  local msg="$3"
  echo "level=$level status=$status msg=$msg"
  # 定义 3 个函数局部变量：
  # level = 本次调用的第1个参数
  # status = 本次调用的第2个参数
  # msg = 本次调用的第3个参数
  # 然后按固定格式打印
}

check_node_exists() {
  local node_name="$1"

  # grep -q: 静默匹配，不打印内容，只通过返回码表示是否匹配成功
  # 当前写法是“匹配到一行中包含 node_name 的内容就算成功”（默认 grep 匹配）
  if ros2 node list | grep -q "$node_name"; then
    emit_line INFO PASS "node exists: $node_name"
    return 0
  else
    emit_line ERROR FAIL "node missing: $node_name"
    return 1
  fi
  # 匹配到 -> return 0（成功）
  # 没匹配到 -> return 1（失败）
}

main() {
  local fail_count=0
  local nodes=(/map_server /amcl /planner_server)
  # 在 main 函数里定义一个局部数组，包含 3 个待检查节点名

  for n in "${nodes[@]}"; do  # "${nodes[@]}" 表示按元素取出数组全部元素（保留每个元素边界）
    check_node_exists "$n"    # 检查 ros2 node list 输出中是否能匹配到当前节点名
    local rc=$?               # 保存刚刚函数调用的返回码，避免被后续命令覆盖
    if [[ $rc -ne 0 ]]; then  # -ne = 数值不等于；如果返回码不是 0（即检查失败）
      ((fail_count++))        # 失败计数 +1
    fi
  done

  emit_line INFO INFO "fail_count=$fail_count"  # 打印汇总：失败次数

  if [[ $fail_count -eq 0 ]]; then  # -eq = 数值等于；失败数为 0
    exit 0                          # 整个脚本返回成功
  else
    exit 1                          # 整个脚本返回失败
  fi
}

main "$@"
# 把“脚本收到的所有参数”原样传给 main 函数
# 例如：bash healthcheck.sh --timeout 3
# 那么 main 里就会收到同样的参数列表（$1=--timeout, $2=3）