## 今日目标
- 掌握文件/路径相关基础命令（pwd/ls/cd/mkdir/cp/mv/rm）
- 能用 find + grep + 管道 | 在真实目录中定位“文件”和“关键词”
- 理解并记录一次 xargs 因文件名含空格导致的拆参问题，以及修复方法（-print0 / -0）

## 今日操作命令一览
- pwd：显示当前工作目录
- ls / ls -la：列出目录内容（含隐藏文件/权限等）
- cd：切换目录
  - cd ..：上一级目录
  - cd -：切换到上一次所在目录
- mkdir -p：创建目录（含中间目录）
- cp：复制文件/目录（目录用 cp -r）
- mv：移动/重命名
- rm：删除（rm -f 删除文件；rm -rf 删除目录，今天只在自己创建的 tmp/ 里练）
- printf：写入内容到文件
  - 覆盖：printf "hello\n" > a.txt
  - 追加：printf "more\n" >> a.txt
- sed：查看文件指定行范围
  - sed -n '1,20p' file.txt

## find（查找）
- 只找普通文件：
  - find <dir> -type f
- 按名字过滤（建议配合 -type f）：
  - find <dir> -type f -name "*.txt"
- 控制递归深度：
  - find <dir> -maxdepth 0   # 只显示 <dir> 本身
  - find <dir> -maxdepth 1   # 显示第一层孩子
  - find <dir> -maxdepth 2   # 再深入一层

## grep（查找关键词）
- 单文件搜索并输出行号：
  - grep -n "KEYWORD" file.txt
- 递归搜索目录并输出行号：
  - grep -Rn "KEYWORD" <dir>

## find + grep 组合拳（稳定版）
- 处理含空格文件名（避免 xargs 按空白拆参）：
  - find <dir> -type f -name "*.txt" -print0 | xargs -0 grep -n "KEYWORD"
- 替代稳法（不用 xargs）：
  - find <dir> -type f -name "*.txt" -exec grep -n "KEYWORD" {} +

## Mini-Check（闭卷自检）
- 我能闭卷敲出 3 条命令：① find+grep（递归）② find -print0 | xargs -0 grep ③ find -exec grep {} +
- 我今天踩到的坑：xargs 按空白拆参（文件名含空格）→ 修复：-print0 / -0 或 -exec
