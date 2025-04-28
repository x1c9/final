import os
from PIL import Image
import numpy as np
from skimage.metrics import structural_similarity as ssim
import matplotlib.pyplot as plt

# Hàm tính RMSE
def calculate_rmse(img1, img2):
    # Tính toán sai số bình phương trung bình (MSE)
    mse = np.mean((img1 - img2) ** 2)
    # RMSE là căn bậc hai của MSE
    rmse = np.sqrt(mse)
    return rmse

# Hàm tính phần trăm trùng khớp
def calculate_percentage_similarity(img1, img2):
    # Tính số pixel giống nhau
    matching_pixels = np.sum(img1 == img2)
    # Tổng số pixel
    total_pixels = img1.size
    # Tính phần trăm trùng khớp
    percentage_similarity = (matching_pixels / total_pixels) * 100
    return percentage_similarity

# Đọc hai ảnh đã xử lý
maps_dir = '/home/nguyen/catkin_ws/src/gmapping/maps'
image_path_map = os.path.join(maps_dir, 'processed_map_test.png')
image_path_gazebo = os.path.join(maps_dir, 'processed_map_test_gazebo.png')

# Mở ảnh và chuyển sang ảnh xám
img_map = Image.open(image_path_map).convert('L')
img_gazebo = Image.open(image_path_gazebo).convert('L')

# Điều chỉnh kích thước của ảnh gazebo sao cho phù hợp với ảnh my_map
img_gazebo = img_gazebo.resize(img_map.size, Image.Resampling.LANCZOS)

# Chuyển ảnh thành mảng numpy
img_map_array = np.array(img_map)
img_gazebo_array = np.array(img_gazebo)

# Tính toán SSIM (Độ tương đồng cấu trúc)
ssim_value, _ = ssim(img_map_array, img_gazebo_array, full=True)

# Tính toán RMSE
rmse_value = calculate_rmse(img_map_array, img_gazebo_array)

# Tính phần trăm trùng khớp
percentage_similarity = calculate_percentage_similarity(img_map_array, img_gazebo_array)

# Hiển thị kết quả SSIM, RMSE và phần trăm trùng khớp
print(f"SSIM (Độ tương đồng cấu trúc) giữa hai ảnh là: {ssim_value}")
print(f"RMSE (Sai số bình phương trung bình) giữa hai ảnh là: {rmse_value}")
print(f"Phần trăm trùng khớp giữa hai ảnh là: {percentage_similarity}%")

# Hiển thị ảnh để so sánh trực quan
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
ax1.imshow(img_map, cmap='gray')
ax1.set_title("Processed My Map")
ax1.axis('off')

ax2.imshow(img_gazebo, cmap='gray')
ax2.set_title("Processed Map Gazebo")
ax2.axis('off')

plt.show()
