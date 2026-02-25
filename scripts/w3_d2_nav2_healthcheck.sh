#!/usr/bin/env bash
set -u

# w3_d2_nav2_healthcheck.sh
# 目标：一键检查 Nav2 是否“基本健康”
# 检查项（MVP）：
# 1) 关键节点存在
# 2) 关键 topic 至少有 1 个 publisher
# 3) /map_server lifecycle == active [3]
# 返回码：
# 0 = PASS
# 1 = FAIL（至少一项失败）
# 2 = 脚本/环境错误（如 ros2 命令不存在）

fail_count=0

ts_now() {
  # 生成当前时间戳（用于日志）
  date +"%Y-%m-%dT%H:%M:%S%z"
}

emit_line() {
  # 统一可读输出（shell 版 status line）
  # 用法：
  #   emit_line LEVEL STATUS REASON "extra_kv..."
  # 前3个参数必填，第4个参数可选
  #
  # 说明：
  # - printf 里的 %s 是“字符串占位符”，会按顺序用后面的参数替换
  # - 单引号 '...' 用于固定格式串（不让 shell 展开变量）
  # - 双引号 "..." 用于变量值（保留空格，避免乱拆）
  local level="$1"
  local status="$2"
  local reason="$3"
  local extra="${4:-}"

  if [[ -n "$extra" ]]; then
    printf 'ts=%s tool=healthcheck level=%s status=%s reason=%s %s\n' \
      "$(ts_now)" "$level" "$status" "$reason" "$extra"
  else
    printf 'ts=%s tool=healthcheck level=%s status=%s reason=%s\n' \
      "$(ts_now)" "$level" "$status" "$reason"
  fi
}

check_cmd_exists() {
  local cmd="$1"

  # command -v "$cmd"：
  #   检查命令是否存在（能否在 PATH 中找到）
  # >/dev/null 2>&1：
  #   把 stdout(1) 和 stderr(2) 都丢到 /dev/null（黑洞）
  #   这里只关心返回码，不关心输出内容
  if command -v "$cmd" >/dev/null 2>&1; then
    emit_line OK OK cmd_found "cmd=${cmd}"
    return 0
  else
    emit_line FAIL FAIL cmd_not_found "cmd=${cmd}"
    return 1
  fi
}

check_node_exists() {
  local node="$1"
  local out

  # out 保存 ros2 node list 的输出文本（含 stderr）
  out="$(ros2 node list 2>&1)"

  # grep -Fxq "$node" <<<"$out"
  # -F : 按普通字符串匹配（不是正则）
  # -x : 整行完全匹配
  # -q : 静默模式（不打印，只看返回码）
  # <<<"$out" : 把变量 out 的内容作为 grep 的输入（here-string）
  if grep -Fxq "$node" <<<"$out"; then
    emit_line OK OK node_exists "node=${node}"
    return 0
  else
    emit_line FAIL FAIL node_missing "node=${node}"
    fail_count=$((fail_count + 1))
    return 1
  fi
}

check_topic_publishers() {
  local topic="$1"
  # 如果调用时传了第2个参数，就用它；没传则默认 1
  local min_pub="${2:-1}"
  # 声明局部变量（此时只是声明，尚未赋值）
  local out rc pub_count

  # out = 命令输出文本（stdout+stderr）
  out="$(ros2 topic info "$topic" 2>&1)"
  # rc = 上一条命令的返回码（0=成功，非0=失败）
  rc=$?

  # 注意：rc 是否为 0 与“有没有输出内容”不是一回事
  # 命令可能失败但有错误输出；也可能成功但输出很少/没有
  if [[ $rc -ne 0 ]]; then
    emit_line FAIL FAIL topic_info_error "topic=${topic} rc=${rc}"
    fail_count=$((fail_count + 1))
    return 1
  fi

  # 从 ros2 topic info 输出中提取 Publisher count 的数字
  # awk -F': '    : 用 ": " 作为字段分隔符
  # /Publisher count:/ : 匹配包含该文本的行
  # {print $2; exit}   : 打印第2列（右侧数字）并退出
  #
  # 示例：
  #   输入行：Publisher count: 11
  #   分隔后：$1="Publisher count"  $2="11"
  #   结果：pub_count="11"
  pub_count="$(awk -F': ' '/Publisher count:/ {print $2; exit}' <<<"$out")"

  if [[ -z "${pub_count}" ]]; then
    emit_line FAIL FAIL publisher_count_parse_error "topic=${topic}"
    fail_count=$((fail_count + 1))
    return 1
  fi

  # 防御性检查：pub_count 必须是“纯数字”
  # =~ 在 [[ ... ]] 中表示“正则匹配”
  # ^[0-9]+$ 表示：从头到尾都是数字，至少1位
  # 例如：1 / 11 / 001 都匹配；1a / -1 / 1.0 不匹配
  if ! [[ "$pub_count" =~ ^[0-9]+$ ]]; then
    emit_line FAIL FAIL publisher_count_not_int "topic=${topic} publisher_count=${pub_count}"
    fail_count=$((fail_count + 1))
    return 1
  fi

  # (( ... )) 是算术比较上下文，会把变量当数字处理
  # 这里比较“实际 publisher 数量”是否 >= “最低要求数量”
  if (( pub_count >= min_pub )); then
    emit_line OK OK topic_publishers_ok "topic=${topic} publisher_count=${pub_count} min_pub=${min_pub}"
    return 0
  else
    emit_line FAIL FAIL topic_no_publisher "topic=${topic} publisher_count=${pub_count} min_pub=${min_pub}"
    fail_count=$((fail_count + 1))
    return 1
  fi
}

check_lifecycle_active() {
  local node="$1"
  local out rc

  out="$(ros2 lifecycle get "$node" 2>&1)"
  rc=$?

  if [[ $rc -ne 0 ]]; then
    emit_line FAIL FAIL lifecycle_get_error "node=${node} rc=${rc}"
    fail_count=$((fail_count + 1))
    return 1
  fi

  if grep -Fq "active [3]" <<<"$out"; then
    emit_line OK OK lifecycle_active "node=${node} state=active"
    return 0
  else
    # 把多行状态输出压成一行，方便日志查看
    local state_one_line
    state_one_line="$(tr '\n' ' ' <<<"$out" | sed 's/[[:space:]]\+/ /g' | sed 's/[[:space:]]*$//')"  
    #把$out里面的内容，压缩成一行，然后空格统一成一个空格，最后去掉每行尾巴的空格
    emit_line FAIL FAIL lifecycle_not_active "node=${node} state=\"${state_one_line}\""
    fail_count=$((fail_count + 1))
    return 1
  fi
}

main() {
  emit_line OK OK start "script=w3_d2_nav2_healthcheck"

  # 如果 ros2 命令不存在，属于环境错误，直接退出码 2
  if ! check_cmd_exists ros2; then
    emit_line FAIL FAIL env_error "detail=ros2_cli_not_found"
    exit 2
  fi

  # ---------- 1) 关键节点 ----------
  check_node_exists "/bt_navigator"
  check_node_exists "/planner_server"
  check_node_exists "/controller_server"
  check_node_exists "/map_server"

  # ---------- 2) 关键 topic 发布者 ----------
  check_topic_publishers "/tf" 1
  check_topic_publishers "/tf_static" 1
  check_topic_publishers "/map" 1

  # ---------- 3) lifecycle ----------
  check_lifecycle_active "/map_server"

  # ---------- 汇总 ----------
  if [[ $fail_count -eq 0 ]]; then
    emit_line OK OK summary "result=PASS fail_count=0"
    exit 0
  else
    emit_line FAIL FAIL summary "result=FAIL fail_count=${fail_count}"
    exit 1
  fi
}

# 调用 main 函数，并把脚本收到的所有参数原样传给 main（不是“打印参数”）
main "$@"