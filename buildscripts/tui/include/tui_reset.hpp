#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>

namespace terminal {

/**
 * Linus-style terminal reset: Simple, direct, no external dependencies
 * Only the absolutely essential sequences to avoid polluting build output
 */
inline void reset_terminal_state() {
    // ONLY essential sequences - no device queries that produce output!
    const char* reset_sequence = 
        "\033[?25h"      // Show cursor
        "\033[0m"        // Reset all attributes (colors, styles)
        "\033[?1000l"    // Disable mouse reporting
        "\033[?1002l"    // Disable button event tracking
        "\033[?1003l"    // Disable any event tracking
        "\033[?2004l"    // Disable bracketed paste mode
        "\033[?47l"      // Exit alternate screen mode
        "\033[?1049l";   // Exit alternate screen + cursor save
        // REMOVED: "\033[c" - this causes terminal to output device info!

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
 * Linus-style aggressive input disable: Cut off all input events at source
 * Used before execl() to prevent any mouse events during process replacement
 */
inline void disable_all_input_aggressive() {
    // More aggressive mouse disable sequence
    const char* disable_sequence = 
        "\033[?1000l"    // Disable X10 mouse reporting
        "\033[?1001l"    // Disable highlight mouse reporting
        "\033[?1002l"    // Disable button event tracking
        "\033[?1003l"    // Disable any event tracking
        "\033[?1004l"    // Disable focus in/out reporting
        "\033[?1005l"    // Disable UTF-8 mouse mode
        "\033[?1006l"    // Disable SGR mouse mode
        "\033[?1007l"    // Disable alternate scroll mode
        "\033[?1015l"    // Disable urxvt mouse mode
        "\033[?2004l";   // Disable bracketed paste mode

    int tty_fd = open("/dev/tty", O_WRONLY);
    if (tty_fd >= 0) {
        write(tty_fd, disable_sequence, strlen(disable_sequence));
        fsync(tty_fd);
        close(tty_fd);
    }
    
    // Force a small delay to let terminal process the disable commands
    usleep(10000); // 10ms - minimal but necessary
}

/**
 * Reset terminal on build failure - silent to avoid polluting build output
 */
inline void reset_on_build_error() {
    reset_terminal_state();
    // NO OUTPUT - let build script show its own errors
}

/**
 * Reset terminal on normal exit
 */
inline void reset_on_exit() {
    reset_terminal_state();
    std::cout << "🔧 Terminal state reset\n";
}

} // namespace terminal