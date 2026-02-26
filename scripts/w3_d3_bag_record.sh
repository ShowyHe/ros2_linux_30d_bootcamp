#!/usr/bin/env bash
# 解释器：用系统环境里的 bash 来执行本脚本

# 严格模式（工程化常用组合）
# -e: 任一“未被显式处理”的命令失败(退出码非0)时，脚本退出
# -u: 引用未定义变量直接报错退出（避免拼错变量名导致静默错误）
# -o pipefail: 管道 a | b | c 的退出码取“最先失败的命令”的非0值（否则默认只看最后一个命令）
set -euo pipefail

# w3_d3_bag_record.sh
# 用法：
#   ./scripts/w3_d3_bag_record.sh w3_d3_map1_run01
# 可选环境变量（用于覆盖默认行为）：
#   BAG_ROOT=~/ros2_linux_30d_bootcamp/bags   # bag 输出根目录
#   HEALTHCHECK=1                             # 录制前先跑 healthcheck（默认 1；设为 0 跳过）

SCRIPT_NAME="w3_d3_bag_record"

# 打时间戳：统一格式，便于日志检索/对齐
TS() { date +"%Y-%m-%dT%H:%M:%S%z"; }

emit() {
  # 统一 status line（w2_d3p5 契约风格）
  # 必选字段：ts/tool/level/status/reason
  #
  # 约定：emit <level> <status> <reason> [kv1] [kv2] ...
  # 例如：emit OK OK start "out_dir=..." "topic_count=9"
  local level="$1"; shift      # 取第1个参数给 level，然后 shift 把它“消费掉”
  local status="$1"; shift     # 取新的第1个参数给 status，再 shift
  local reason="$1"; shift     # 取新的第1个参数给 reason，再 shift

  # 固定头：ts/tool/level/status/reason
  printf 'ts=%s tool=bag_record level=%s status=%s reason=%s' "$(TS)" "$level" "$status" "$reason"

  # 剩余参数（"$@"）统一按“kv字段”输出：每个字段前加一个空格，保证一行结构清晰
  for kv in "$@"; do
    printf ' %s' "$kv"
  done
  printf '\n'
}

usage() {
  # 这里用 here-doc 把多行文本喂给 cat，cat 默认输出到 stdout（终端/管道/重定向）
  # 不会写文件，除非调用方做 > 重定向
  cat <<'EOF'
Usage:
  ./scripts/w3_d3_bag_record.sh <bag_name>

Example:
  ./scripts/w3_d3_bag_record.sh w3_d3_map1_run01
EOF
}

# 参数校验：脚本至少需要 1 个参数（bag_name）
# $# = 参数个数；-lt = 小于（数字比较）；这里就是“没传 bag_name 就报错”
if [[ $# -lt 1 ]]; then
  emit FAIL FAIL usage_error "need_arg=bag_name"
  usage
  exit 2   # 退出码 2：常用约定 = 用法/参数错误
fi

# 环境检查：必须能找到 ros2 命令，否则后续所有 ros2 bag/ros2 topic 都没法跑
if ! command -v ros2 >/dev/null 2>&1; then
  emit FAIL FAIL cmd_not_found "cmd=ros2"
  exit 127 # 退出码 127：常用约定 = command not found
fi

# 读取位置参数：第一个参数就是 bag 名称（目录名）
BAG_NAME="$1"

# 项目根目录（默认固定到你的 bootcamp 仓库）
BOOTCAMP_ROOT="${HOME}/ros2_linux_30d_bootcamp"

# BAG_ROOT：允许外部通过环境变量覆盖；若未设置或为空，则默认用 ${BOOTCAMP_ROOT}/bags
# :- 的含义：变量“未定义或为空”时使用默认值
BAG_ROOT="${BAG_ROOT:-${BOOTCAMP_ROOT}/bags}"

# 日志目录固定在 bootcamp/logs
LOG_ROOT="${BOOTCAMP_ROOT}/logs"

# 确保输出根目录存在（父目录要先有，ros2 bag record 才能在里面创建 OUT_DIR）
mkdir -p "${BAG_ROOT}" "${LOG_ROOT}"

# 这次录制的输出目录（ros2 bag record -o 会创建这个目录并写 metadata.yaml/db3）
OUT_DIR="${BAG_ROOT}/${BAG_NAME}"

# HEALTHCHECK：录制前是否先跑健康检查（默认 1）
# 设为 0 可以跳过：HEALTHCHECK=0 ./scripts/w3_d3_bag_record.sh xxx
HEALTHCHECK="${HEALTHCHECK:-1}"

# 白名单 topic（只录这些，不“瞎录全世界”）
TOPICS=(
  /clock
  /tf
  /tf_static
  /scan
  /map
  /odom
  /amcl_pose
  /cmd_vel
  /navigate_to_pose/_action/status
)

# 防止覆盖已有输出目录：如果 OUT_DIR 已存在，直接失败退出
# -e：文件或目录存在即为真
if [[ -e "${OUT_DIR}" ]]; then
  emit FAIL FAIL output_exists "out_dir=${OUT_DIR}"
  exit 3   # 自定义退出码 3：输出目录已存在（防覆盖）
fi

emit OK OK start "script=${SCRIPT_NAME}" "out_dir=${OUT_DIR}" "topic_count=${#TOPICS[@]}"

# 可选：录制前先健康检查（复用 w3_d2）
# HEALTHCHECK=1 => 执行；HEALTHCHECK=0 => 整段跳过
if [[ "${HEALTHCHECK}" == "1" ]]; then
  # 只有当 healthcheck 脚本“存在且可执行(-x)”才跑；否则仅告警跳过
  if [[ -x "${BOOTCAMP_ROOT}/scripts/w3_d2_nav2_healthcheck.sh" ]]; then
    emit OK OK precheck_run "script=w3_d2_nav2_healthcheck"

    # 这里临时关闭 -e：因为我们需要“即使命令失败也继续往下走”，才能抓到 PIPESTATUS
    # PIPESTATUS 是 bash 内置数组：记录“上一条管道”中每段命令的退出码
    set +e
    "${BOOTCAMP_ROOT}/scripts/w3_d2_nav2_healthcheck.sh" 2>&1 | tee -a "${LOG_ROOT}/w3_d3_precheck.log"
    pipe_rc=("${PIPESTATUS[@]}")
    hc_rc=${pipe_rc[0]}   # 管道第0段：healthcheck脚本的退出码
    tee_rc=${pipe_rc[1]}  # 管道第1段：tee 的退出码（日志是否写成功）
    set -e

    # healthcheck 失败就直接退出：避免录一堆“链路本来就没跑起来”的废 bag
    if [[ ${hc_rc} -ne 0 ]]; then
      emit FAIL FAIL precheck_failed "hc_rc=${hc_rc}" "tee_rc=${tee_rc}"
      exit 4   # 自定义退出码 4：precheck 失败
    fi
    emit OK OK precheck_pass "hc_rc=${hc_rc}" "tee_rc=${tee_rc}"
  else
    # 缺少 healthcheck 脚本：不强制失败（目前策略），仅告警并继续录制
    emit WARN WARN precheck_skip "reason=healthcheck_script_missing"
  fi
fi

# 可选：录制前扫一眼关键 topic 是否存在（缺失仅告警，不强制失败）
# 为什么不强制失败：有时你就是想“先录现场”，topic 可能稍后才出现
set +e
topic_list="$(ros2 topic list 2>/dev/null)"
topic_list_rc=$?
set -e

if [[ ${topic_list_rc} -ne 0 ]]; then
  emit WARN WARN topic_list_failed "rc=${topic_list_rc}"
else
  missing_count=0
  for t in "${TOPICS[@]}"; do
    # -q：静默；-x：整行匹配（避免 /tf 匹配到 /tf_static 这种误报）
    if ! grep -qx "${t}" <<< "${topic_list}"; then
      emit WARN WARN topic_missing_before_record "topic=${t}"
      missing_count=$((missing_count + 1))
    fi
  done
  emit OK OK topic_precheck_done "missing_count=${missing_count}"
fi

emit OK OK record_begin "cmd=ros2_bag_record" "hint=Ctrl+C_to_stop"

# 录制阶段：临时关闭 -e
# 原因：你用 Ctrl+C 停止录制时，ros2 bag record 常返回 130（SIGINT），这不是“异常失败”，要自己判断处理
set +e
ros2 bag record \
  --include-hidden-topics \
  -o "${OUT_DIR}" \
  "${TOPICS[@]}"
record_rc=$?
set -e
# 确保像 /navigate_to_pose/_action/status 这类 hidden topic 也能录到
# 输出目录（rosbag2 会创建此目录并写 metadata.yaml / *.db3）
# topic 白名单（数组展开：每个元素作为独立参数）

# 退出码判定：
# 0   => 正常结束（少见但可能）
# 130 => Ctrl+C 停止录制（SIGINT=2；128+2=130）
if [[ ${record_rc} -eq 0 || ${record_rc} -eq 130 ]]; then
  emit OK OK record_end "rc=${record_rc}" "out_dir=${OUT_DIR}"
else
  emit FAIL FAIL record_error "rc=${record_rc}" "out_dir=${OUT_DIR}"
fi

# 录制结束后做最小验证：输出目录必须存在
# 注意：OUT_DIR 的创建是 ros2 bag record 负责的（不是我们 mkdir）
if [[ -d "${OUT_DIR}" ]]; then
  emit OK OK bag_dir_exists "out_dir=${OUT_DIR}"

  # 验证 ros2 bag info 是否能读：能读说明 metadata/db3 结构基本正常
  # 同时把 bag info 输出追加写入日志，留证据
  set +e
  ros2 bag info "${OUT_DIR}" | tee -a "${LOG_ROOT}/w3_d3_bag_record_info.log"
  baginfo_pipe_rc=("${PIPESTATUS[@]}")
  bag_info_rc=${baginfo_pipe_rc[0]}  # 第0段：ros2 bag info 的退出码
  tee_info_rc=${baginfo_pipe_rc[1]}  # 第1段：tee 的退出码
  set -e

  if [[ ${bag_info_rc} -eq 0 ]]; then
    emit OK OK bag_info_ok "out_dir=${OUT_DIR}" "bag_info_rc=${bag_info_rc}" "tee_rc=${tee_info_rc}"
  else
    emit WARN WARN bag_info_failed "out_dir=${OUT_DIR}" "bag_info_rc=${bag_info_rc}" "tee_rc=${tee_info_rc}"
  fi
else
  emit FAIL FAIL bag_dir_missing "out_dir=${OUT_DIR}"
  exit 5  # 自定义退出码 5：录制结束后找不到输出目录（异常）
fi
#先检查参数和环境；然后可选做健康检查；再预扫 topic 是否出现；接着录制白名单 topic；最后验证 bag 目录和 bag info 是否正常可读