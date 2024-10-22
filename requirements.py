import os

# Lấy địa chỉ hiện tại
current_directory = os.getcwd()

# Kết nối vào thư mục src
src_directory = os.path.join(current_directory, "src")

# Pip install requirements.txt
for dirpath, dirnames, filenames in os.walk(src_directory):
    for dirname in dirnames[:]:
        if dirname != "src":  # Bỏ qua thư mục con nếu tên là "src"
            dir_requirements = os.path.join(dirpath, dirname, "requirements.txt")
            if os.path.exists(dir_requirements):
                cmd = f"pip install -r {dir_requirements}"
                os.system(cmd)