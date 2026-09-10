"""使用训练时同版本Ultralytics导出RK3588 INT8基线模型。"""

import hashlib
from importlib.metadata import version
from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
PT_PATH = ROOT / "best.pt"
DATA_PATH = ROOT / "drone.v1i.yolo26" / "data.yaml"
EXPECTED_ULTRALYTICS = "8.4.131"
EXPECTED_RKNN_TOOLKIT = "2.3.2"
EXPECTED_PT_SHA256 = "e2b98214672628c2d901a18728dbe9cee60a3387d8270d10f26744d46b347ffb"


def require_version(package: str, expected: str) -> None:
    """冻结导出工具版本，防止升级后ONNX图和RKNN结果静默变化。"""
    actual = version(package)
    if actual != expected:
        raise RuntimeError(
            f"{package}版本不一致：需要{expected}，当前{actual}。"
            "基线复现不得自动升级依赖。"
        )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
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

    model = YOLO(str(PT_PATH))
    exported = model.export(
        format="rknn",
        name="rk3588",
        quantize=8,
        data=str(DATA_PATH),
        imgsz=640,
        batch=1,
        opset=19,
        simplify=True,
        fraction=1.0,
    )
    print(f"RKNN基线模型导出完成：{exported}")


if __name__ == "__main__":
    main()
