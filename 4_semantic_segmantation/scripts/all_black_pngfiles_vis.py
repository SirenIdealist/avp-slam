import cv2
import os
import csv
import argparse

def get_all_png_files(root_dir):
    png_files = []
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.lower().endswith('.png'):
                png_files.append(os.path.join(root, file))
    return png_files

def save_pixels_to_csv(image_path, output_csv_path):
    img = cv2.imread(image_path)
    if img is None:
        print(f"图片读取失败: {image_path}")
        return
    h, w, c = img.shape
    with open(output_csv_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['filename', 'x', 'y', 'B', 'G', 'R'])
        for y in range(h):
            for x in range(w):
                pixel = img[y, x]
                csv_writer.writerow([os.path.basename(image_path), x, y, pixel[0], pixel[1], pixel[2]])

def main():
    parser = argparse.ArgumentParser(description="为每个PNG图片生成一个CSV文件，保存像素值")
    parser.add_argument("data_dir", help="包含PNG图片的数据文件夹路径")
    parser.add_argument("output_dir", help="存放CSV文件的输出文件夹路径（Path A）")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    png_files = get_all_png_files(args.data_dir)
    if not png_files:
        print("未找到PNG文件。")
        return

    for img_path in png_files:
        csv_filename = os.path.splitext(os.path.basename(img_path))[0] + ".csv"
        output_csv_path = os.path.join(args.output_dir, csv_filename)
        print(f"生成CSV: {output_csv_path}")
        save_pixels_to_csv(img_path, output_csv_path)

if __name__ == "__main__":
    main()