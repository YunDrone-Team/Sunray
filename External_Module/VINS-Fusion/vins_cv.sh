#!/bin/bash

# =================================================================
# 脚本名称: fix_opencv4.sh
# 作用: 递归搜索当前目录下所有源码文件，将 OpenCV 3 的旧宏替换为 OpenCV 4 标准
# 使用方法: 
#   1. 将此脚本放入 VINS-Fusion 项目根目录
#   2. 运行: chmod +x fix_opencv4.sh
#   3. 运行: ./fix_opencv4.sh
# =================================================================

# 1. 定义要处理的文件后缀
# 查找当前目录下所有的 .cpp, .cc, .h, .hpp 文件，排除隐藏目录如 .git
SOURCE_FILES=$(find . -type f \( -name "*.cpp" -o -name "*.cc" -o -name "*.h" -o -name "*.hpp" \) -not -path '*/.*')

if [ -z "$SOURCE_FILES" ]; then
    echo "错误: 在当前目录下未找到源码文件。"
    exit 1
fi

echo "开始处理当前目录下的 OpenCV 4 兼容性替换..."

# 2. 执行批量替换逻辑
for file in $SOURCE_FILES; do
    # --- 颜色空间转换 ---
    sed -i 's/CV_GRAY2RGB/cv::COLOR_GRAY2RGB/g' "$file"
    sed -i 's/CV_RGB2GRAY/cv::COLOR_RGB2GRAY/g' "$file"
    sed -i 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g' "$file"
    sed -i 's/CV_GRAY2BGR/cv::COLOR_GRAY2BGR/g' "$file"

    # --- 图像读取与窗口 ---
    sed -i 's/CV_LOAD_IMAGE_GRAYSCALE/cv::IMREAD_GRAYSCALE/g' "$file"
    sed -i 's/CV_LOAD_IMAGE_COLOR/cv::IMREAD_COLOR/g' "$file"
    sed -i 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g' "$file"
    sed -i 's/CV_WINDOW_AUTOSIZE/cv::WINDOW_AUTOSIZE/g' "$file"

    # --- 绘图与字体 ---
    sed -i 's/CV_FONT_HERSHEY_SIMPLEX/cv::FONT_HERSHEY_SIMPLEX/g' "$file"
    sed -i 's/CV_AA/cv::LINE_AA/g' "$file"
    sed -i 's/CV_FILLED/cv::FILLED/g' "$file"

    # --- 标定/棋盘格常量 ---
    sed -i 's/CV_CALIB_CB_ADAPTIVE_THRESH/cv::CALIB_CB_ADAPTIVE_THRESH/g' "$file"
    sed -i 's/CV_CALIB_CB_NORMALIZE_IMAGE/cv::CALIB_CB_NORMALIZE_IMAGE/g' "$file"
    sed -i 's/CV_CALIB_CB_FILTER_QUADS/cv::CALIB_CB_FILTER_QUADS/g' "$file"
    sed -i 's/CV_CALIB_CB_FAST_CHECK/cv::CALIB_CB_FAST_CHECK/g' "$file"

    # --- 算法常量 (形态学/阈值/轮廓) ---
    sed -i 's/CV_ADAPTIVE_THRESH_MEAN_C/cv::ADAPTIVE_THRESH_MEAN_C/g' "$file"
    sed -i 's/CV_THRESH_BINARY/cv::THRESH_BINARY/g' "$file"
    sed -i 's/CV_THRESH_TOZERO/cv::THRESH_TOZERO/g' "$file"
    sed -i 's/CV_SHAPE_RECT/cv::MORPH_RECT/g' "$file"
    sed -i 's/CV_SHAPE_CROSS/cv::MORPH_CROSS/g' "$file"
    sed -i 's/CV_SHAPE_ELLIPSE/cv::MORPH_ELLIPSE/g' "$file"
    sed -i 's/CV_RETR_CCOMP/cv::RETR_CCOMP/g' "$file"
    sed -i 's/CV_RETR_EXTERNAL/cv::RETR_EXTERNAL/g' "$file"
    sed -i 's/CV_RETR_LIST/cv::RETR_LIST/g' "$file"
    sed -i 's/CV_RETR_TREE/cv::RETR_TREE/g' "$file"
    sed -i 's/CV_CHAIN_APPROX_SIMPLE/cv::CHAIN_APPROX_SIMPLE/g' "$file"

    # --- 核心常量与终止条件 ---
    sed -i 's/CV_TERMCRIT_EPS/cv::TermCriteria::EPS/g' "$file"
    sed -i 's/CV_TERMCRIT_ITER/cv::TermCriteria::MAX_ITER/g' "$file"
    sed -i 's/CV_INTER_LINEAR/cv::INTER_LINEAR/g' "$file"
    sed -i 's/CV_INTER_NEAREST/cv::INTER_NEAREST/g' "$file"
done

echo "替换任务执行完毕。共处理文件数: $(echo "$SOURCE_FILES" | wc -l)"
