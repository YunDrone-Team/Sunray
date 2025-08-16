#!/bin/bash

# Sunray TUI Debug Build Script
# Linus风格：简单、直接、可靠的调试构建脚本

set -e  # 发生错误时退出

echo "🔧 Sunray TUI Debug Build Script"
echo "================================="
echo ""

# Clean previous build
if [ -d "build" ]; then
    echo "🧹 Cleaning previous build directory..."
    rm -rf build
fi

# Configure with debug enabled
echo "⚙️  Configuring CMake with DEBUG mode enabled..."
cmake -B build -DSUNRAY_DEBUG_ENABLED=ON

# Build the project
echo "🔨 Building TUI with comprehensive debug logging..."
cmake --build build

# Check if build succeeded
if [ -f "../bin/sunray_tui" ]; then
    echo ""
    echo "✅ DEBUG build completed successfully!"
    echo ""
    echo "📋 Debug Features Enabled:"
    echo "   • Comprehensive event tracing"
    echo "   • Real-time debug panel (press D key to toggle)"
    echo "   • Build process debugging"
    echo "   • Focus management tracing"
    echo "   • Animation state tracking"
    echo ""
    echo "🚀 To run: ../bin/sunray_tui"
    echo "💡 Debug Panel: Press 'D' key during execution to show/hide debug info"
    echo ""
else
    echo "❌ Build failed - executable not found"
    exit 1
fi