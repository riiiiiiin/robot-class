# 正则化时间格式替换：将 YYYY-MM-DD HH:MM:SS -> 全中文数字文本（不含阿拉伯数字）
# 说明：年份按每一位数字转为中文（如 2025 -> 二零二五），月/日/时按常用中文读法（十二、二十三、十四...），
# 分/秒保留输入的前导零信息（比如 05 -> 零五，00 -> 零）。
#
# 使用方法：调用 `convert_chinese_datetime(text)`，它会把文本中所有匹配的时间串替换为中文文本并返回新字符串。

import re

# 基本数字映射
_digit_map = {
    '0': '零', '1': '一', '2': '二', '3': '三', '4': '四',
    '5': '五', '6': '六', '7': '七', '8': '八', '9': '九'
}

def year_to_chinese(year_str: str) -> str:
    """按位转换年份，例如 '2025' -> '二零二五'"""
    return ''.join(_digit_map[d] for d in year_str)

def number_to_chinese(n: int) -> str:
    """把 0..99 的整数转换为常规中文读法
    0 -> 零; 1-> 一; 10 -> 十; 11 -> 十一; 20 -> 二十; 21-> 二十一; 30 -> 三十 etc.
    """
    if n == 0:
        return '零'
    if n < 10:
        return _digit_map[str(n)]
    if n < 20:
        # 10..19: 十, 十一, 十二...
        return '十' + (_digit_map[str(n % 10)] if n % 10 != 0 else '')
    tens = n // 10
    ones = n % 10
    tens_part = (_digit_map[str(tens)] if tens > 1 else '') + '十'
    return tens_part + (_digit_map[str(ones)] if ones != 0 else '')

def minute_second_to_chinese(mmss_str: str) -> str:
    """保留分/秒的前导零信息：'00'->'零', '05'->'零五', '12'->'十二' """
    if mmss_str == '00':
        return '零'
    if mmss_str[0] == '0':
        # 如 '05' -> '零五'
        return '零' + _digit_map[mmss_str[1]]
    # 无前导零则按数值读法
    return number_to_chinese(int(mmss_str))

# 正则匹配 YYYY-MM-DD[ 空格或T ]HH:MM:SS
_pattern = re.compile(r'(\d{4})-(\d{2})-(\d{2})[ T](\d{2}):(\d{2}):(\d{2})')

def _replace_match(m: re.Match) -> str:
    y, mo, d, h, mi, s = m.groups()
    y_cn = year_to_chinese(y)
    mo_cn = number_to_chinese(int(mo))  # 月按常规读法，'01'->'一'
    d_cn  = number_to_chinese(int(d))
    h_cn  = number_to_chinese(int(h))
    mi_cn = minute_second_to_chinese(mi)
    s_cn  = minute_second_to_chinese(s)
    # 输出格式示例：二零二五年十二月二十三日十四时零五分零九秒
    return f"{y_cn}年{mo_cn}月{d_cn}日{h_cn}时{mi_cn}分{s_cn}秒"

def convert_chinese_datetime(text: str) -> str:
    """将文本中所有 YYYY-MM-DD HH:MM:SS 格式替换为中文形式（无阿拉伯数字）"""
    return _pattern.sub(_replace_match, text)

if __name__ == '__main__':
    # --- 简单测试与示例 ---
    samples = [
        "事件发生在2025-12-23 14:05:09，随后记录到日志。",
        "备份时间：2023-01-01 00:00:00；下次运行请参考。",
        "多个时间：2020-02-02T08:07:06 与 1999-12-31 23:59:59。"
    ]

    for s in samples:
        print("原文：", s)
        print("替换后：", convert_chinese_datetime(s))
        print("---")
