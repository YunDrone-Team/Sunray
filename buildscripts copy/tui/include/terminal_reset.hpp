#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>

namespace terminal {

/**
 * Linus-style terminal reset: Simple, direct, no external dependencies
 * Embeds the essential ANSI sequences without script coupling
 */
inline void reset_terminal_state() {
    // Essential ANSI sequences to clean terminal state
    const char* reset_sequence = 
        "\033[?25h"      // Show cursor
        "\033[0m"        // Reset all attributes (colors, styles)
        "\033[?1000l"    // Disable X10 mouse reporting
        "\033[?1001l"    // Disable highlight mouse tracking
        "\033[?1002l"    // Disable button event tracking
        "\033[?1003l"    // Disable any event tracking
        "\033[?1004l"    // Disable focus events
        "\033[?1005l"    // Disable UTF-8 mouse mode
        "\033[?1006l"    // Disable SGR mouse mode
        "\033[?1015l"    // Disable Urxvt mouse mode
        "\033[?2004l"    // Disable bracketed paste mode
        "\033[?47l"      // Exit alternate screen mode
        "\033[?1049l"    // Exit alternate screen + cursor save
        "\033[c";        // Soft reset terminal

    // Write directly to TTY to avoid stdout pollution
    int tty_fd = open("/dev/tty", O_WRONLY);
    if (tty_fd >= 0) {
        write(tty_fd, reset_sequence, strlen(reset_sequence));
        fsync(tty_fd);
        close(tty_fd);
    } else {
        // Fallback to stdout if TTY unavailable
        std::cout << reset_sequence << std::flush;
    }
}

/**
 * Reset terminal on build failure with user feedback
 */
inline void reset_on_build_error() {
    reset_terminal_state();
    std::cerr << "❌ Build failed - terminal state reset\n";
}

/**
 * Reset terminal on normal exit
 */
inline void reset_on_exit() {
    reset_terminal_state();
    std::cout << "🔧 Terminal state reset\n";
}

} // namespace terminal