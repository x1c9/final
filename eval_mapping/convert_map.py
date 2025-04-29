import os
from PIL import Image
import numpy as np

# Hàm chuyển đổi hình ảnh và cắt phần bên ngoài tường
def process_image(image_path, output_path):
    # Mở hình ảnh
    img = Image.open(image_path).convert('L')  # Chuyển thành ảnh xám

    # Chuyển đổi ảnh thành mảng numpy để dễ thao tác
    img_array = np.array(img)

    # Ngưỡng màu xám để chuyển thành đen
    wall_threshold = 100  # Điều chỉnh nếu cần

    # Tạo mảng mới để thay đổi màu sắc
    img_array_processed = np.where(img_array < wall_threshold, 0, 255)

    # Tìm vị trí của các tường (pixel có giá trị = 0 - màu đen)
    black_pixels = np.where(img_array_processed == 0)

    # Tìm tọa độ tường để cắt vùng
    min_row, max_row = np.min(black_pixels[0]), np.max(black_pixels[0])
    min_col, max_col = np.min(black_pixels[1]), np.max(black_pixels[1])

    # Cắt ảnh sao cho chỉ giữ lại phần bên trong tường, tạo thành hình vuông
    cropped_img_array = img_array_processed[min_row:max_row, min_col:max_col]

    # Chuyển đổi mảng numpy về kiểu uint8
    cropped_img_array = cropped_img_array.astype(np.uint8)

    # Tạo ảnh mới từ mảng đã cắt
    cropped_img = Image.fromarray(cropped_img_array)

    # Lưu hình ảnh đã cắt
    cropped_img.save(output_path)
    print(f"Đã lưu hình ảnh đã chỉnh sửa tại: {output_path}")

# Đường dẫn tới thư mục 'maps' và tệp 'my_map.pgm'
maps_dir = '/home/nguyen/catkin_ws/src/gmapping/maps'
image_path_map = os.path.join(maps_dir, 'map_test.pgm')
output_path_map = os.path.join(maps_dir, 'processed_map_test.png')

# Gọi hàm để xử lý và lưu hình ảnh
process_image(image_path_map, output_path_map)
