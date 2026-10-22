#!/usr/bin/env python3
"""文档一致性检查（基线数字、相对链接、模块实现文档、按改动提示）。

用法：
  python3 tools/check_docs.py
  python3 tools/check_docs.py --changed

退出码：0=通过；1=基线/链接/模块文档检查失败。
--changed 只打印“改动了这些源码，建议核对哪些文档”，不改变退出码。
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASELINE_SOURCE = ROOT / "docs" / "README.md"
BASELINE_RE = re.compile(r"当前基线：(\d+/\d+)")

# 当前基线必须一致的位置（docs/README.md 是唯一源，不在此列表内）。
BASELINE_TARGETS = [
    "README.md",
    "docs/模块索引.md",
    "docs/项目总览与数据流.md",
    "docs/开发进度.md",
    "docs/部署与打包.md",
    "docs/故障排查手册.md",
    "docs/archive/历史计划/主程序集成与数据链路实施计划.md",
    "include/perception/stereo_ranger.md",
    "docs/superpowers/specs/2026-10-13-阶段四控制接入路径-design.md",
]

# 检查相对链接的当前文档；archive/ 与 superpowers/ 属历史材料，不纳入日常链接门禁。
LINK_TARGETS = [
    "README.md",
    "AGENTS.md",
    "docs/README.md",
    "docs/开发进度.md",
    "docs/开发历史.md",
    "docs/项目总览与数据流.md",
    "docs/模块索引.md",
    "docs/故障排查手册.md",
    "docs/新增设备接入指南.md",
    "docs/数据接口文档.md",
    "docs/部署与打包.md",
    "docs/状态机设计.md",
    "docs/物理追踪思路.md",
    "docs/电源与失效保护设计.md",
    "docs/多机身份与功能寻址设计.md",
    "docs/地面站目标位置与时间同步协议.md",
    "docs/方案C锚点基准实施清单.md",
    "docs/S5运动目标追踪改进方案.md",
    "docs/双目深度估计思路.md",
    "docs/双目硬件事实确认记录.md",
    "docs/视觉追踪技术路线.md",
    "docs/PX4实链验证清单.md",
    "docs/archive/README.md",
]

CHANGED_RULES = [
    ("src/video_transmission/", [
        "include/video_transmission/video_sender.md",
        "docs/模块索引.md", "docs/故障排查手册.md", "docs/开发进度.md",
    ]),
    ("src/video/", [
        "include/video/*.md", "docs/模块索引.md", "docs/故障排查手册.md",
        "docs/项目总览与数据流.md", "docs/开发进度.md",
    ]),
    ("src/perception/", [
        "include/perception/*.md", "docs/模块索引.md", "docs/数据接口文档.md",
        "docs/故障排查手册.md", "docs/开发进度.md",
    ]),
    ("src/communication/", [
        "include/communication/*.md", "docs/数据接口文档.md", "docs/模块索引.md",
        "docs/故障排查手册.md", "docs/地面站目标位置与时间同步协议.md",
        "docs/开发进度.md",
    ]),
    ("src/state_machine/", [
        "include/state_machine/*.md", "docs/状态机设计.md",
        "docs/项目总览与数据流.md", "docs/开发进度.md",
    ]),
    ("src/control/", [
        "include/control/*.md", "docs/物理追踪思路.md", "docs/开发进度.md",
    ]),
    ("src/config/", [
        "include/config/配置管理.md", "docs/部署与打包.md",
        "docs/故障排查手册.md", "docs/开发进度.md",
    ]),
    ("src/application/", [
        "include/application/主程序集成.md", "docs/项目总览与数据流.md",
        "docs/开发进度.md",
    ]),
    ("tests/", ["docs/README.md（当前基线）", "docs/开发进度.md", "docs/故障排查手册.md"]),
    ("config/config.json", ["include/config/配置管理.md", "docs/部署与打包.md", "docs/开发进度.md"]),
]


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def current_baseline() -> str:
    if not BASELINE_SOURCE.exists():
        raise SystemExit(f"缺少基线源文件: {BASELINE_SOURCE}")
    match = BASELINE_RE.search(read(BASELINE_SOURCE))
    if not match:
        raise SystemExit("docs/README.md 中未找到“当前基线：N/N”")
    return match.group(1)


def check_baseline(baseline: str) -> list[str]:
    errors = []
    for rel in BASELINE_TARGETS:
        path = ROOT / rel
        if not path.exists():
            errors.append(f"缺少基线文件: {rel}")
        elif baseline not in read(path):
            errors.append(f"基线不一致: {rel}（应包含 {baseline}）")
    return errors


def strip_code_fences(text: str) -> str:
    return re.sub(r"```.*?```", "", text, flags=re.S)


def check_links() -> list[str]:
    errors = []
    link_re = re.compile(r"\[[^\]]+\]\(([^)]+)\)")
    for rel in LINK_TARGETS:
        path = ROOT / rel
        if not path.exists():
            errors.append(f"缺少链接检查文件: {rel}")
            continue
        text = strip_code_fences(read(path))
        for raw in link_re.findall(text):
            target = raw.strip().strip("<>")
            target = target.split(' "')[0].strip()
            if not target or target.startswith(("#", "http://", "https://", "mailto:")):
                continue
            target = target.split("#", 1)[0]
            if not target:
                continue
            resolved = (path.parent / target).resolve()
            if not resolved.exists():
                errors.append(f"断链: {rel} -> {raw}")
    return errors


def check_module_docs() -> list[str]:
    errors = []
    index = ROOT / "docs" / "模块索引.md"
    if not index.exists():
        return ["缺少 docs/模块索引.md"]
    text = read(index)
    for token in set(re.findall(r"`([^`]+\.md)`", text)):
        if not token.startswith("include/"):
            continue
        if any(ch in token for ch in "*<>"):
            continue
        if not (ROOT / token).exists():
            errors.append(f"模块索引中的实现文档不存在: {token}")
    return errors


def changed_files() -> list[str]:
    commands = [
        ["git", "diff", "--name-only"],
        ["git", "diff", "--cached", "--name-only"],
        ["git", "ls-files", "--others", "--exclude-standard"],
    ]
    files: set[str] = set()
    for command in commands:
        try:
            output = subprocess.run(
                command, cwd=ROOT, capture_output=True, text=True, check=False
            ).stdout
        except OSError:
            continue
        files.update(line.strip() for line in output.splitlines() if line.strip())
    return sorted(files)


def print_changed_hints() -> None:
    files = changed_files()
    if not files:
        print("未检测到未提交改动。")
        return
    hinted: dict[str, set[str]] = {}
    for rel in files:
        for prefix, docs in CHANGED_RULES:
            if rel == prefix or rel.startswith(prefix):
                hinted.setdefault(prefix, set()).update(docs)
    if not hinted:
        print("改动未命中文档矩阵；请人工确认是否需要更新文档。")
        return
    print("检测到源码/配置改动，建议核对以下文档：")
    for prefix in sorted(hinted):
        print(f"- {prefix}")
        for doc in sorted(hinted[prefix]):
            print(f"    - {doc}")
    print("提示：这只是建议；最终以 docs/README.md §4 的矩阵为准。")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--changed", action="store_true",
                        help="按 git 改动列出建议核对的文档")
    args = parser.parse_args()

    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    if args.changed:
        print_changed_hints()
        return 0

    baseline = current_baseline()
    errors = []
    errors.extend(check_baseline(baseline))
    errors.extend(check_links())
    errors.extend(check_module_docs())

    if errors:
        print(f"文档检查失败（当前基线 {baseline}，共 {len(errors)} 项）：")
        for error in errors:
            print(f"- {error}")
        return 1

    print(f"文档检查通过：基线 {baseline}，链接与模块文档一致。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
