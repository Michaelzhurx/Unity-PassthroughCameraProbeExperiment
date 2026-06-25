from pathlib import Path
import argparse
import numpy as np
from PIL import Image


def convert_gray_raw_to_jpg(
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

    expected_size = width * height

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
                f"你的 width/height 很可能填错了。"
            )
            continue

        image_array = data.reshape((height, width))

        image = Image.fromarray(image_array, mode="L")
        image.save(jpg_path, "JPEG", quality=quality)

        print(f"[完成] {raw_path.name} -> {jpg_path.name}")


def main():
    # parser = argparse.ArgumentParser(
    #     description="Convert Unity-generated uint8 grayscale .raw files to JPG"
    # )

    # parser.add_argument("input_dir", help="包含 0.raw, 1.raw ... 的目录")
    # parser.add_argument("image_count", type=int, help="图片数量，例如 100 表示 0.raw 到 99.raw")
    # parser.add_argument("--width", type=int, required=True, help="图像宽度")
    # parser.add_argument("--height", type=int, required=True, help="图像高度")
    # parser.add_argument("--output_dir", default=None, help="输出目录，默认 input_dir/jpg_output")
    # parser.add_argument("--quality", type=int, default=95, help="JPG 质量，默认 95")

    # args = parser.parse_args()

    # convert_gray_raw_to_jpg(
    #     input_dir=args.input_dir,
    #     image_count=args.image_count,
    #     width=args.width,
    #     height=args.height,
    #     output_dir=args.output_dir,
    #     quality=args.quality,
    # )
        convert_gray_raw_to_jpg(
        input_dir=r"C:\Users\LENOVO\OneDrive\Desktop/left",
        output_dir=r"C:\Users\LENOVO\OneDrive\Desktop/left_jpg",
        image_count=30,
        width=1280,
        height=1280,
    )


if __name__ == "__main__":
    main()