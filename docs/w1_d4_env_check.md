## 今日目标（Day4）
- 搞懂“环境变量属于谁”：属于当前 shell 进程；新开终端=新进程=环境从零开始
- 搞懂 `source` vs `bash`：为什么 ROS2 的 setup 必须用 `source`
- 能用“证据链命令”确认：当前终端里 `ros2` 从哪里来、PATH/AMENT_PREFIX_PATH 现在是什么状态
- 今日范围：不做 overlay（install/setup.bash 叠加）训练

---

## 今日操作命令一览（含用途解释）

### 1) 看 PATH（命令搜索路径）
- 用途：确认终端会去哪些目录找命令；排查 “command not found / 版本错乱”
    echo "$PATH"
- 把 PATH 拆成多行方便读（`tr ':' '\n'` 把冒号换成换行）
    echo "$PATH" | tr ':' '\n' | head -n 10

### 2) 查 `ros2` 到底解析到哪里（证据）
- 用途：不是“有没有装”，而是“现在这个终端到底在跑哪个 ros2”
    command -v ros2 || echo "ros2 NOT FOUND"

### 3) 看 AMENT_PREFIX_PATH（ROS2 包前缀路径）
- 用途：决定 ROS2/ament 体系从哪些前缀里找包与资源（后面 `ros2 run` 会用到）
    echo "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | head -n 10

### 4) export：让子进程继承变量（必须做一次实验）
- 用途：证明“变量只在当前 shell”与“export 后子进程才看得到”的差异
    FOO=123
    bash -lc 'echo "child sees FOO=$FOO"'
    export FOO=456
    bash -lc 'echo "child sees FOO=$FOO"'

### 5) .bashrc：为什么新终端会归零 & 如何持久化（今天只做查看，不乱改）
- 用途：确认 `.bashrc` 存在；理解它会影响所有新终端
    ls -la ~/.bashrc
    tail -n 20 ~/.bashrc

### 6) source vs bash：闭卷证明“环境能不能留在当前终端”
- 用途：这就是 ROS2 setup 必须 `source` 的底层原因
    cat > /tmp/demo_env.sh <<'EOF'
    export DEMO_VAR="hello"
    EOF

    bash /tmp/demo_env.sh
    echo "after bash: DEMO_VAR=$DEMO_VAR"

    source /tmp/demo_env.sh
    echo "after source: DEMO_VAR=$DEMO_VAR"

结论（必须能复述）：
- `bash script.sh`：开子进程执行，环境只在子进程里，回到当前终端就没了
- `source script.sh`：在当前 shell 执行，环境会留在当前终端

---

## 证据链（推荐写进日志/截图的“固定口径”）
### A. 加载 ROS2 base 之前（新终端）
    command -v ros2 || echo "before: ros2 NOT FOUND"
    echo "$PATH" | tr ':' '\n' | head -n 5
    echo "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | head -n 5

### B. 加载 ROS2 base 之后（如果今天有做）
    source /opt/ros/humble/setup.bash
    command -v ros2
    echo "$PATH" | tr ':' '\n' | head -n 5
    echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head -n 5

---

## Mini-Check（闭卷自检）
- 我能闭卷解释：为什么新终端环境会“归零”
- 我能闭卷解释：`source` 为什么能让变量留在当前终端，而 `bash` 不行
- 我能闭卷敲出 3 条证据命令：
  - `command -v ros2`
  - `echo "$PATH" | tr ':' '\n' | head -n 10`
  - `echo "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | head -n 10`
- 我能闭卷做一次 export 子进程实验，并解释结果
