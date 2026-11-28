"""Extract ordered, allowed instrument candidates from a Mandarin transcript."""
import re
import unicodedata


NUMBERS = {"三": 3, "四": 4, "十": 10, "十一": 11, "十五": 15, "二十": 20}
KNIVES = {
    (3, 10): "scalpel_handle_3_blade_10",
    (3, 11): "scalpel_handle_3_blade_11",
    (3, 15): "scalpel_handle_3_blade_15",
    (4, 20): "scalpel_handle_4_blade_20",
}
ALIASES = {
    "直头手术剪": "straight_mayo_scissor", "直头剪": "straight_mayo_scissor",
    "手术直剪": "straight_mayo_scissor", "直剪": "straight_mayo_scissor",
    "手术剪刀": "straight_mayo_scissor", "手术剪": "straight_mayo_scissor",
    "组织镊子": "tissue_forceps", "组织镊": "tissue_forceps", "镊子": "tissue_forceps",
    "止血钳": "hemostat",
}
BY_BLADE = {blade: item for (_, blade), item in KNIVES.items()}
# Only correct sound-alikes when the independent acoustic spotter agrees.
SOUND_ALIKES = {
    "straight_mayo_scissor": (re.compile(r"[直指纸质只][剪简检]"), "直剪"),
    "hemostat": (re.compile(r"[止纸只指][血雪][钱前钳]"), "止血钳"),
    "tissue_forceps": (re.compile(r"组织[你孽镍聂涅捏]"), "组织镊"),
}
NUMBER = r"[0-9]+|[零〇一二两三四五六七八九十]+"
TARGET = re.compile(
    rf"(?P<handle>{NUMBER})(?:号|型)?刀柄(?:搭配|配上|配|加上|加|和|接上|\+)?"
    rf"(?P<blade>{NUMBER})(?:号|型)?刀片|(?P<single>{NUMBER})(?:号|型)(?:手术刀|刀片)|"
    rf"(?P<name>{'|'.join(ALIASES)})"
)
HOLD = re.compile(r"不要|不拿|别拿|不用|先别|别动|别给|取消|停一下|暂停|等一下|换成|不对|不是|或者|还是|或|如果|假如|刚才.*说|他说|她说|吗|么|什么")
# Only a question addressed to the workstation with a request verb can use this
# wrapper. A question about an instrument ("这是止血钳吗") is not an order.
POLITE_REQUEST = re.compile(r"^(?:请问)?(?:你)?(?:可不可以|能不能|可以|能否|能)(?=帮我|给我|递|拿|找|取|把)")
FILLERS = re.compile(
    r"(?:请你|麻烦你|帮我|给我|需要的是|我想要的是|想要的是|我要的是|需要|我想要|想要|我想|我要|我|把|请|麻烦|帮|拿|取|递|给|找|一下|过来|"
    r"现在|(?:工作台上|台面上|桌面上|桌上|白纸上)的?|那把|这把|那件|这件|那个|这个|"
    r"首先|然后|最后|接着|之后|先|再|跟|和|与|还有|以及|也要|要|一把|一支|一件|一个|个|谢谢|好的|嗯|呃|啊|吧)+"
)


def number(raw):
    return int(raw) if raw.isascii() and raw.isdecimal() and len(raw) <= 6 else NUMBERS.get(raw)


def command_text(transcript):
    normalized = unicodedata.normalize("NFKC", transcript).lower()
    text = "".join(ch for ch in normalized if not (ch.isspace() or unicodedata.category(ch).startswith("P")))
    polite = POLITE_REQUEST.match(text)
    if polite:
        text = re.sub(r"(?:吗|么)$", "", text[polite.end():])
    return text


def parse_transcript(transcript, keyword_ids=()):
    text = command_text(transcript)
    if HOLD.search(text):
        return [], "听到否定、改口或备选项，请重说需要的器械。"
    for item in keyword_ids:
        if item in SOUND_ALIKES:
            pattern, spelling = SOUND_ALIKES[item]
            text = pattern.sub(spelling, text)
    matches = list(TARGET.finditer(text))
    if not matches or FILLERS.sub("", TARGET.sub("", text)):
        if "手术刀" in text or "刀片" in text:
            return [], "请说刀片编号，例如「十一号手术刀」。"
        return [], "有器械或型号没说清楚，请重复完整口令。"
    ids = []
    for match in matches:
        if match["name"]:
            item = ALIASES[match["name"]]
        elif match["single"]:
            item = BY_BLADE.get(number(match["single"]))
            if item is None:
                return [], "该刀片编号不在清单里，可选择 10、11、15 或 20 号。"
        else:
            item = KNIVES.get((number(match["handle"]), number(match["blade"])))
        if item is None:
            return [], "该刀柄与刀片组合不在清单里。"
        if item not in ids:
            ids.append(item)
    return ids, ""
