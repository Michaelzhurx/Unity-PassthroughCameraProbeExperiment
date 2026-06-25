from pathlib import Path
import numpy as np
from PIL import Image


def convert_rgb_raw_to_jpg(
    input_dir: str,
    image_count: int,
    width: int,
    height: int,
    output_dir: str | None = None,
    quality: int = 95,
):
    input_path = Path(input_dir)

    if output_dir is None:
        output_path = input_path / "jpg_output"
    else:
        output_path = Path(output_dir)

    output_path.mkdir(parents=True, exist_ok=True)

    expected_size = width * height * 3

    for i in range(image_count):
        raw_path = input_path / f"{i}.raw"
        jpg_path = output_path / f"{i}.jpg"

        if not raw_path.exists():
            print(f"[跳过] 文件不存在: {raw_path}")
            continue

        data = np.fromfile(raw_path, dtype=np.uint8)

        if data.size != expected_size:
            print(
                f"[失败] {raw_path.name}: 文件大小不匹配。"
                f"期望 {expected_size} bytes，实际 {data.size} bytes。"
                f"你的 width/height 或 RGB 通道数可能填错了。"
            )
            continue

        image_array = data.reshape((height, width, 3))

        image = Image.fromarray(image_array, mode="RGB")
        image.save(jpg_path, "JPEG", quality=quality)

        print(f"[完成] {raw_path.name} -> {jpg_path.name}")


def main():
    convert_rgb_raw_to_jpg(
        input_dir=r"D:\3ThirdYearWork\right",
        output_dir=r"D:\3ThirdYearWork\right\right_jpg",
        image_count=815,
        width=1280,
        height=1280,
        quality=95,
    )


if __name__ == "__main__":
    main()