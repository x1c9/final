import numpy as np
from PIL import Image
import os

# Hàm xử lý và cắt phần ngoài tường
def process_image(image_path, output_path):
    # Mở hình ảnh
    img = Image.open(image_path).convert('L')  # Chuyển thành ảnh xám

    # Chuyển đổi ảnh thành mảng numpy để dễ thao tác
    img_array = np.array(img)

    # Ngưỡng màu xám để chuyển thành đen (tường) và trắng (khu vực trống)
    gray_threshold = 120  # Ngưỡng phát hiện tường xám

    # Tạo mảng mới để thay đổi màu sắc: tường xám thành đen (0), các phần còn lại thành trắng (255)
    img_array_processed = np.where(img_array > gray_threshold, 0, 255)

    # Tạo ảnh mới từ mảng đã xử lý
    processed_img = Image.fromarray(img_array_processed.astype(np.uint8))

    # Lưu hình ảnh đã xử lý
    processed_img.save(output_path)
    print(f"Đã lưu hình ảnh đã chỉnh sửa tại: {output_path}")

# Đường dẫn tới thư mục 'maps' và tệp 'map_gazebo.png'
maps_dir = '/home/nguyen/catkin_ws/src/gmapping/maps'
image_path = os.path.join(maps_dir, 'map_test_gazebo.png')
output_path = os.path.join(maps_dir, 'processed_map_test_gazebo.png')

# Gọi hàm để xử lý và lưu hình ảnh
process_image(image_path, output_path)
