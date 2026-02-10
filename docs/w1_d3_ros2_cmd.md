## 今日目标
- 掌握进程查看与清场的核心命令（ps/grep、top/htop、kill/pkill）
- 能定位“残留进程导致现象诡异”的根因：进程仍在运行、资源占用异常、信号未生效/匹配漏掉
- 写出并跑通一键清场脚本 `scripts/w1_d3_cleanup_ros2.sh`，生成清前/清后证据日志
- 记录 w1_d3_cleanup 脚本常见踩坑点，并形成可复用的排错清单

## 今日操作命令一览
- ps aux：列出所有进程（含无终端后台进程）
  - ps aux | grep -E "<pattern>"：按关键词筛选进程
  - grep -v grep：过滤 grep 自己那行，避免误判
- top / htop：实时查看进程资源占用
  - top 内：P（按 CPU 排序）、M（按内存排序）、q（退出）
- pkill：按“模式”批量发送信号给进程
  - pkill -2 -f "<pattern>"：发 SIGINT（更温和，等价 Ctrl+C）
  - pkill -15 -f "<pattern>"：发 SIGTERM（正常终止）
  - pkill -9 -f "<pattern>"：发 SIGKILL（强杀，最后手段）
- kill：按 PID 精准发送信号
  - kill -15 <PID>：优先正常终止
  - kill -9 <PID>：强杀（最后手段）
- sleep 2：等待 2 秒，让进程有时间处理信号并退出（避免误判/升级过快）
- pgrep：用来“验尸”
  - pgrep -af "<pattern>"：列出匹配进程（含命令行），用于 after 校验与脚本失败退出

## 进程定位：ps + grep（残留证据链）
- 一次性查 RViz / Gazebo / Nav2 常见残留：
  - ps aux | grep -E "rviz2|gzserver|gzclient|gazebo|nav2|amcl|map_server|bt_navigator|controller_server|planner_server" | grep -v grep //看 CPU/MEM 谁吃资源
  - pgrep -af "rviz2|gzserver|gzclient|gazebo|nav2|amcl|map_server|bt_navigator|controller_server|planner_server" //只给 PID + 完整命令行
- 读输出的关键字段（你需要能解释）：
  - PID：后续 kill 精准处理的依据
  - %CPU / %MEM：资源异常的证据（“卡/诡异现象”的常见原因）
  - COMMAND：确认进程类型（rviz2/gzserver/nav2_* 等）

## 资源定位：top / htop（谁在吃资源）
- top：定位 CPU/MEM 占用异常的进程（用于解释“为什么会卡/不响应”）
- 常见观察点：
  - gzserver / gzclient：仿真图形与物理常见重负载来源
  - rviz2：显示/插件异常时可能飙资源
  - nav2_* / container：导航节点异常循环时可能持续占用

## 清场策略（从轻到重）
- 批量清场（推荐先做）：
  - pkill -2 -f "rviz2|gzserver|gzclient|gazebo|nav2|amcl|map_server"
  - sleep 2
  - pkill -15 -f "rviz2|gzserver|gzclient|gazebo|nav2|amcl|map_server"
  - sleep 2
- 精准点名（顽固残留再用）：
  - 先用 ps aux | grep ... 抄到 PID
  - kill -15 <PID> → sleep 1 → ps -p <PID> -o pid,cmd
  - 仍不退出才 kill -9 <PID>

## 一键清场脚本（你已完成并跑通）
- 脚本路径：
  - scripts/w1_d3_cleanup_ros2.sh
- 关键要求（证据化）：
  - 清前（before）打印残留列表
  - 分级信号：SIGINT → SIGTERM（必要时再升级）
  - 清后（after）再次打印残留列表
  - 日志落盘：logs/ 下生成带时间戳的 log 文件

## 今日踩坑清单（w1_d3_cleanup 脚本错题本）
1) 变量赋值两边不能有空格  
   - ❌ TM= "..." / LOG= "..."  
   - ✅ TM="..." / LOG="..."

2) 命令替换要用 $(...)，不是 ${...}  
   - ❌ TM="${date +%Y%m%d_%H%M%S}"  
   - ✅ TM="$(date +%Y%m%d_%H%M%S)"

3) date 的格式字母大小写有语义，别乱改  
   - ✅ 推荐：+%Y%m%d_%H%M%S  
   - 常见坑：%M 是分钟不是月；%D 会输出 mm/dd/yy（带 /，不适合文件名）

4) 引号别用“智能引号”，用 ASCII 引号  
   - ❌ “*.txt”、‘foo’（会让命令变诡异/等待输入）  
   - ✅ "*.txt"、'foo'

5) 管道里引号必须闭合，别把 | grep ... 写进字符串  
   - ❌ grep -E "a|b|c | grep -v grep ...（引号没关/把管道吞进字符串）  
   - ✅ grep -E 'a|b|c' | grep -v grep ...

6) sleep2 不是命令，必须有空格  
   - ❌ sleep2  
   - ✅ sleep 2

7) 路径/变量一定加引号（防空格/特殊字符）  
   - ❌ tee -a $LOG  
   - ✅ tee -a "$LOG"

8) 绝对路径 vs 相对路径要统一（别建 A 用 B）  
   - 你建了：mkdir -p ~/ros2_linux_30d_bootcamp/logs  
   - 但写了：LOG="logs/..."（可能写到别的目录）  
   - ✅ 推荐：LOG="$HOME/ros2_linux_30d_bootcamp/logs/cleanup_${TM}.log"

9) pkill 匹配命令行要加 -f，否则容易杀不到（Nav2 容器经典坑）  
   - 现象：清前清后都还在 component_container_isolated ... __node:=nav2_container  
   - 原因：进程名不是 nav2，默认 pkill 只匹配进程名会漏  
   - ✅ pkill -2 -f "nav2_container|component_container_isolated|..." || true  
   - ✅ pkill -15 -f "nav2_container|component_container_isolated|..." || true

10) cleanup done 只是“脚本跑完了”，不等于“清干净了”  
   - 因为对很多命令用了 || true（允许失败继续跑）  
   - ✅ 强制清干净：after 之后用 pgrep -f pattern 检查，残留就 exit 1

## Mini-Check（闭卷自检）
- 我能闭卷敲出并解释 3 条命令：
  - ① ps aux | grep -E ... | grep -v grep（定位残留 + 拿 PID）
  - ② pkill -2/-15 -f "<pattern>" + sleep 2（分级清场）
  - ③ kill -15 <PID> / kill -9 <PID>（点名处置 + 最后手段）
- 我今天踩到的坑（必须能复述原因 + 修复）：
  - 变量赋值空格、命令替换写法、date 格式大小写、智能引号、管道引号闭合、sleep 空格、变量/路径加引号、绝对/相对路径不一致、pkill 未加 -f、清场后未做 pgrep 验尸
