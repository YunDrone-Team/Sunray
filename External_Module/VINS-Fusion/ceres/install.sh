set -e  # 出错立即停止执行

echo "正在准备安装 VINS 依赖 ceres-solver v1.14.0"

# 1. 安装系统依赖
sudo apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev cmake

# 定义变量
PACKAGE_PATH="./ceres-solver-1.14.0.tar.gz"
TEMP_DIR="ceres_temp_build"
PACKAGE_NAME="ceres-solver-1.14.0.tar.gz"
EXTRACTED_DIR="ceres-solver-1.14.0"

# 2. 检查并解压
if [ -f "$PACKAGE_PATH" ]; then
    echo "找到本地压缩包: $PACKAGE_PATH"
    mkdir -p "$TEMP_DIR"
    cp "$PACKAGE_PATH" "$TEMP_DIR/"
    cd "$TEMP_DIR"
else
    echo "错误: 未在本地查找到 ceres-solver 依赖，请检查路径。"
    exit 1
fi

echo "正在解压..."
tar -zxvf "$PACKAGE_NAME"
cd "$EXTRACTED_DIR"

# 3. 编译安装
mkdir -p build && cd build
echo "开始配置 CMake..."
# 建议加上 -DCMAKE_BUILD_TYPE=Release 提高 VINS 运行效率
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

echo "开始编译..."
make -j$(($(nproc)-2))

echo "执行安装..."
sudo make install
sudo ldconfig

# 4. 清理临时文件
cd ../../../  # 回到执行脚本前的目录
rm -rf "$TEMP_DIR"

echo "Ceres Solver 安装成功！"