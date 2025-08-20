#!/bin/bash
# TUI Dependencies Check and Auto-Download Script

set -e

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Script directory and third_party path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THIRD_PARTY_DIR="$SCRIPT_DIR/third_party"

# Dependency configuration with specific commit hashes
FTXUI_REPO="https://gitee.com/mirrors/ftxui.git"
FTXUI_COMMIT="994915d"
YAMLCPP_REPO="https://gitee.com/mirrors/yaml-cpp.git"
YAMLCPP_COMMIT="2f86d13"

print_status() {
    echo -e "${CYAN}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

check_git_available() {
    if ! command -v git >/dev/null 2>&1; then
        print_error "Git is not installed or not available in PATH"
        print_error "Please install Git first: brew install git (macOS) or apt-get install git (Linux)"
        return 1
    fi
    return 0
}

check_dependency_exists() {
    local dep_name="$1"
    local dep_path="$THIRD_PARTY_DIR/$dep_name"
    
    if [[ -d "$dep_path" && -n "$(ls -A "$dep_path" 2>/dev/null)" ]]; then
        return 0  # Directory exists and is not empty
    else
        return 1  # Directory doesn't exist or is empty
    fi
}

download_dependency() {
    local dep_name="$1"
    local dep_url="$2"
    local commit_hash="$3"
    local dep_path="$THIRD_PARTY_DIR/$dep_name"
    
    print_status "Downloading $dep_name from $dep_url (commit: $commit_hash)..."
    
    # Create third_party directory if it doesn't exist
    mkdir -p "$THIRD_PARTY_DIR"
    
    # Remove existing directory if it exists but is empty or corrupted
    if [[ -d "$dep_path" ]]; then
        rm -rf "$dep_path"
    fi
    
    # Clone the repository
    if git clone "$dep_url" "$dep_path"; then
        # Checkout specific commit
        if (cd "$dep_path" && git checkout "$commit_hash"); then
            print_success "Successfully downloaded $dep_name at commit $commit_hash"
            return 0
        else
            print_error "Failed to checkout commit $commit_hash for $dep_name"
            rm -rf "$dep_path"
            return 1
        fi
    else
        print_error "Failed to download $dep_name from $dep_url"
        return 1
    fi
}

confirm_download() {
    local missing_deps=("$@")
    
    echo
    print_warning "Missing third-party dependencies detected:"
    for dep in "${missing_deps[@]}"; do
        echo "  - $dep"
    done
    echo
    
    echo -e "${CYAN}These libraries are required for TUI functionality.${NC}"
    echo -e "${CYAN}Would you like to download them automatically?${NC}"
    echo
    echo "Sources:"
    if [[ " ${missing_deps[@]} " =~ " ftxui " ]]; then
        echo "  ftxui: $FTXUI_REPO (commit: $FTXUI_COMMIT)"
    fi
    if [[ " ${missing_deps[@]} " =~ " yaml-cpp " ]]; then
        echo "  yaml-cpp: $YAMLCPP_REPO (commit: $YAMLCPP_COMMIT)"
    fi
    echo
    
    while true; do
        read -p "Download missing dependencies? [y/N]: " choice
        case "$choice" in
            [Yy]|[Yy][Ee][Ss])
                return 0
                ;;
            [Nn]|[Nn][Oo]|"")
                print_warning "Dependencies not downloaded. TUI build may fail."
                return 1
                ;;
            *)
                echo "Please answer yes or no."
                ;;
        esac
    done
}

check_and_download_dependencies() {
    local missing_deps=()
    
    print_status "Checking third-party dependencies..."
    
    # Check Git availability first
    if ! check_git_available; then
        return 1
    fi
    
    # Check each dependency
    if ! check_dependency_exists "ftxui"; then
        missing_deps+=("ftxui")
    else
        print_success "ftxui is available"
    fi
    
    if ! check_dependency_exists "yaml-cpp"; then
        missing_deps+=("yaml-cpp")
    else
        print_success "yaml-cpp is available"
    fi
    
    # If no missing dependencies, we're done
    if [[ ${#missing_deps[@]} -eq 0 ]]; then
        print_success "All dependencies are available"
        return 0
    fi
    
    # Ask user for confirmation to download
    if confirm_download "${missing_deps[@]}"; then
        # Download each missing dependency
        local download_failed=false
        for dep_name in "${missing_deps[@]}"; do
            case "$dep_name" in
                "ftxui")
                    if ! download_dependency "$dep_name" "$FTXUI_REPO" "$FTXUI_COMMIT"; then
                        download_failed=true
                    fi
                    ;;
                "yaml-cpp")
                    if ! download_dependency "$dep_name" "$YAMLCPP_REPO" "$YAMLCPP_COMMIT"; then
                        download_failed=true
                    fi
                    ;;
            esac
        done
        
        if [[ "$download_failed" == true ]]; then
            print_error "Some dependencies failed to download"
            return 1
        else
            print_success "All dependencies downloaded successfully"
            return 0
        fi
    else
        return 1
    fi
}

# Main execution
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo -e "${CYAN}=== TUI Dependencies Checker ===${NC}"
    echo
    
    if check_and_download_dependencies; then
        print_success "Dependencies check completed successfully"
        exit 0
    else
        print_error "Dependencies check failed"
        exit 1
    fi
fi