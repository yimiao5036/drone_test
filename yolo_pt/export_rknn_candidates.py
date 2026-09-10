"""导出保持RKNN坐标归一化语义的基线/FlashAttention候选模型。

该脚本复用Ultralytics 8.4.131的RKNN专用ONNX导出路径：
- 自动关闭YOLO26 end2end/TopK分支；
- 对框坐标按640×640归一化，保护单tensor INT8分类置信度；
- 保存中间ONNX，便于Netron和后续转换对比；
- 使用RKNN Toolkit2 2.3.2分别构建默认与FlashAttention候选。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from importlib.metadata import version
from pathlib import Path

from ultralytics import YOLO
from ultralytics.utils import YAML
import ultralytics.utils.export.rknn as ultralytics_rknn


ROOT = Path(__file__).resolve().parent
PT_PATH = ROOT / "best.pt"
DATA_PATH = ROOT / "drone.v1i.yolo26" / "data.yaml"
EXPECTED_ULTRALYTICS = "8.4.131"
EXPECTED_RKNN_TOOLKIT = "2.3.2"
EXPECTED_PT_SHA256 = "e2b98214672628c2d901a18728dbe9cee60a3387d8270d10f26744d46b347ffb"


def require_version(package: str, expected: str) -> None:
    actual = version(package)
    if actual != expected:
        raise RuntimeError(
            f"{package}版本不一致：需要{expected}，当前{actual}。"
            "候选A/B必须冻结工具版本。"
        )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def check_ret(ret: int | None, operation: str) -> None:
    if ret not in {0, None}:
        raise RuntimeError(f"RKNN {operation}失败：ret={ret}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="导出YOLO26 RKNN转换候选")
    parser.add_argument(
        "--opset",
        type=int,
        choices=(11, 19),
        default=19,
        help="RKNN专用中间ONNX opset；先以19复现当前基线，再单独测试11",
    )
    parser.add_argument(
        "--verbose-rknn",
        action="store_true",
        help="启用RKNN Toolkit详细转换日志",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not PT_PATH.is_file():
        raise FileNotFoundError(f"模型不存在：{PT_PATH}")
    if not DATA_PATH.is_file():
        raise FileNotFoundError(f"数据集配置不存在：{DATA_PATH}")
    actual_pt_sha256 = sha256(PT_PATH)
    if actual_pt_sha256 != EXPECTED_PT_SHA256:
        raise RuntimeError(
            f"best.pt哈希不一致：需要{EXPECTED_PT_SHA256}，当前{actual_pt_sha256}"
        )

    require_version("ultralytics", EXPECTED_ULTRALYTICS)
    require_version("rknn-toolkit2", EXPECTED_RKNN_TOOLKIT)

    # export_rknn()在调用onnx2rknn前已经构造了RKNN专用归一化ONNX。
    # 在此替换转换函数，可保存该临时ONNX并构建受控候选；函数结束后
    # Ultralytics仍会删除原临时文件，但不会删除我们的副本。
    def build_candidates(
        onnx_file: str,
        output_dir: Path | str,
        name: str = "rk3588",
        quantize: int | str | None = None,
        dataset: Path | str | None = None,
        metadata: dict | None = None,
        prefix: str = "",
        batch: int = 1,
    ) -> str:
        if quantize != 8:
            raise ValueError("本脚本只构建INT8候选，必须quantize=8")
        if dataset is None or not Path(dataset).is_file():
            raise FileNotFoundError(f"RKNN校准列表不存在：{dataset}")

        from rknn.api import RKNN

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        stem = f"best-{name}-opset{args.opset}"
        saved_onnx = output_path / f"{stem}-normalized.onnx"
        shutil.copy2(onnx_file, saved_onnx)

        candidates = (
            ("baseline", False),
            ("flash_attention", True),
        )
        for candidate_name, enable_flash_attention in candidates:
            output_rknn = output_path / f"{stem}-{candidate_name}.rknn"
            provenance = {
                "source_pt_sha256": EXPECTED_PT_SHA256,
                "ultralytics": EXPECTED_ULTRALYTICS,
                "rknn_toolkit2": EXPECTED_RKNN_TOOLKIT,
                "opset": args.opset,
                "candidate": candidate_name,
                "enable_flash_attention": enable_flash_attention,
            }

            rknn = RKNN(verbose=args.verbose_rknn)
            try:
                check_ret(
                    rknn.config(
                        mean_values=[[0, 0, 0]],
                        std_values=[[255, 255, 255]],
                        target_platform=name,
                        quantized_dtype="w8a8",
                        quantized_algorithm="normal",
                        quantized_method="channel",
                        optimization_level=3,
                        single_core_mode=False,
                        enable_flash_attention=enable_flash_attention,
                        custom_string=json.dumps(provenance, ensure_ascii=True),
                    ),
                    "config",
                )
                check_ret(rknn.load_onnx(model=str(saved_onnx)), "load_onnx")
                check_ret(
                    rknn.build(
                        do_quantization=True,
                        dataset=str(dataset),
                        rknn_batch_size=batch,
                        auto_hybrid=False,
                    ),
                    "build",
                )
                check_ret(rknn.export_rknn(str(output_rknn)), "export_rknn")
            finally:
                rknn.release()
            print(f"{prefix}{candidate_name}导出完成：{output_rknn}")

        if metadata:
            YAML.save(output_path / f"metadata-opset{args.opset}.yaml", metadata)
        return str(output_path)

    ultralytics_rknn.onnx2rknn = build_candidates

    model = YOLO(str(PT_PATH))
    exported = model.export(
        format="rknn",
        name="rk3588",
        quantize=8,
        data=str(DATA_PATH),
        imgsz=640,
        batch=1,
        opset=args.opset,
        simplify=True,
        fraction=1.0,
    )
    print(f"候选模型目录：{exported}")


if __name__ == "__main__":
    main()
