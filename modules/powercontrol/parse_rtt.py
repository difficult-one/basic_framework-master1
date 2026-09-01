"""
从 RTT 日志提取数据并填入 Fitting.py 的 data 数组.
用法: 把 RTT 日志保存为 rtt_log.txt (UTF-8), 然后 python parse_rtt.py
"""
import re
import os

# 从外部文件读取, 避免编码问题污染 py 文件
script_dir = os.path.dirname(os.path.abspath(__file__))
log_path = os.path.join(script_dir, "rtt_log.txt")

# 先试 UTF-8, 失败则试 GBK (RTT 终端常见编码)
try:
    with open(log_path, "r", encoding="utf-8") as f:
        log = f.read()
except UnicodeDecodeError:
    with open(log_path, "r", encoding="gbk") as f:
        log = f.read()

# 解析每一行: 只按数值结构匹配, 不依赖中文标签
pattern = r'\S+=([-\d.]+)\s+\S+/s\s+\S+=([-\d.]+)\s+\S+=([-\d.]+)\s+W'
matches = re.findall(pattern, log)

data_lines = []
for spd, cur, pwr in matches:
    amp = float(cur) / 819.2        # CAN raw -> A
    rpm = float(spd) / 6.0          # deg/s -> rpm, keep sign for I*W
    watt = float(pwr)
    amp_s = f"{amp:.3f}"
    rpm_s = f"{rpm:.1f}"
    watt_s = f"{watt:.3f}" if abs(watt) >= 1 else f"{watt:.4f}"
    data_lines.append(f"    ({amp_s}, {rpm_s}, {watt_s}),")

print(f"共 {len(data_lines)} 条数据\n")
print('\n'.join(data_lines))
