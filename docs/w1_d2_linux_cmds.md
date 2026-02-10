## 今日目标
- 掌握日志查看与重定向：cat / less / tail -f + > / >> / 2> / 2>&1 / | tee
- 能把任意命令的 stdout+stderr 可靠落盘到 logs/，并能边看边写（tee）
- 理解并记录一次 `tail -f` 过程中因“覆盖写”导致的 `file truncated` 现象，以及规避方法（>> / tee -a）

## 今日操作命令一览
- cat：一次性查看文件内容（适合短文件）
  - cat logs/tmp.txt

- less：分页查看文件（适合长文件，按 q 退出）
  - less logs/tee_demo.txt

- tail：查看文件末尾若干行
  - tail -n 5 logs/tee_demo.txt

- tail -f：实时跟随文件末尾新增内容（Ctrl+C 退出）
  - tail -f logs/tee_demo.txt

- printf：构造输出，用来验证重定向/tee 行为
  - printf "hello\n"
- >（覆盖写）：把 stdout(1) 覆盖写入文件（会清空原内容）
  - printf "hello\n" > logs/tmp.txt
- >>（追加写）：把 stdout(1) 追加写入文件（不清空原内容）
  - printf "more\n" >> logs/tmp.txt
  
- 2>（只重定向 stderr）：把 stderr(2) 写入文件
  - ls no_such_file 2> logs/err_only.txt
- 2>&1（合并 stderr 到 stdout 的去处）：把 stderr(2) 跟随 stdout(1)
  - ls no_such_file > logs/err_merge.txt 2>&1
- |（管道）：把左边 stdout 送给右边 stdin
  - printf "SEE\n" | tee logs/tee_demo.txt

- tee：边显示边写文件（默认覆盖写）
  - printf "A\n" | tee logs/tee_demo.txt
- tee -a：边显示边追加写文件（日志推荐）
  - printf "B\n" | tee -a logs/tee_demo.txt

- 清空文件（truncate）：把文件长度置 0（常用于重新开始一份干净日志）
  - : > logs/tee_demo.txt

## 重定向与 tee（核心口径）
- 覆盖写 vs 追加写
  - cmd > file：覆盖写（truncate 后写）
  - cmd >> file：追加写（不清空）
- stderr 单独落盘 vs 合并落盘
  - cmd 2> file：只把错误输出写进 file
  - cmd > file 2>&1：先让 stdout 进 file，再让 stderr 跟随 stdout，所以两种输出进同一个 file
- 边看边存
  - cmd | tee file：屏幕显示 + 覆盖写入 file
  - cmd | tee -a file：屏幕显示 + 追加写入 file（推荐用于日志）

## 今日踩坑记录：file truncated
- 现象
  - 在一个终端执行 tail -f 跟随日志时，另一个终端对同一文件使用覆盖写（> 或 tee 不带 -a），tail 会提示：file truncated
- 原因
  - 覆盖写会先 truncate 文件（文件突然变短/变成 0），tail -f 检测到文件长度突变就提示 truncated
- 修复/规避
  - 需要持续记录：用 >> 或 tee -a
  - 需要重新开始一份干净日志：先 : > file，再用 >> 或 tee -a 追加写

## Mini-Check（闭卷自检）
- 我能闭卷敲出 3 条命令：
  1) 覆盖写与追加写验证
     printf "hello\n" > logs/tmp.txt
     printf "more\n" >> logs/tmp.txt
     cat logs/tmp.txt
  2) 合并 stdout+stderr 落盘
     ls no_such_file > logs/err_merge.txt 2>&1
     cat logs/err_merge.txt
  3) tee 追加写（避免 truncated）
     printf "A\n" | tee logs/tee_demo.txt
     printf "B\n" | tee -a logs/tee_demo.txt
     tail -n 5 logs/tee_demo.txt
