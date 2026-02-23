from datetime import datetime

def now_iso() -> str:
    # ISO8601 + 本地时区，秒级即可
    return datetime.now().astimezone().isoformat(timespec="seconds")

def emit(tool: str, level: str, status: str, reason: str, **fields) -> None:
    """
    一行一状态，key=value，字段用空格分隔
    必选字段：ts tool level status reason
    其他字段：按需传入，但需键名稳定
    """
    parts = [
        f"ts={now_iso()}",
        f"tool={tool}",
        f"level={level}",
        f"status={status}",
        f"reason={reason}",
    ]

    for k, v in fields.items():
        if v is None:
            continue
        parts.append(f"{k}={v}")
    print(" ".join(parts), flush=True)
    #把parts的值用空格连接起来，然后立刻输出，不等缓存区