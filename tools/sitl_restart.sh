#!/usr/bin/env bash
# 兼容入口：等价于 `tools/sitl_harness.sh stack`（2026-10-22 起由正式装置接管）。
#
# 历史：本脚本原为 SITL 任务分配验收的临时重启脚本（硬编码 $HOME/drone_test）。
# 正式入口与完整文档见 tools/sitl_harness.sh / tools/sitl_harness.md；
# 保留本文件是为了让既有证据 README（tools/sitl_evidence/assignment_20261022/）
# 记录的复现步骤仍然可用。
#
# 用法：bash tools/sitl_restart.sh [--config <config 路径>]
set -u
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sitl_harness.sh" stack "$@"
