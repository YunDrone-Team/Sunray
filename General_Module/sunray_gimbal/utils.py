"""
Some useful functions to manipulate bytes
Author: Mohamed Abdelkader
Contact: mohamedashraf123@gmail.com
"""

def toHex(intval, nbits):
    """
    Converts an integer to hexadecimal and reverses the order of each two characters (bytes)
    if the length of the hex string exceeds two characters.

    Params
    --
    - intval: [int] Integer number
    - nbits: [int] Number of bits (e.g., 8 for int8_t, 16 for int16_t)

    Returns
    --
    String of the hexadecimal value, padded appropriately based on the bit size, with
    reversed byte order for hex strings longer than 2 characters.
    """
    # Calculate the number of hex characters based on the number of bits
    num_hex_chars = nbits // 4  # 4 bits per hex digit

    # Format the number as hex and ensure it's properly adjusted for signed integers
    h = format((intval + (1 << nbits)) % (1 << nbits), 'x')

    # Pad the string with leading zeros to ensure it matches the number of hex characters
    h = h.zfill(num_hex_chars)

    # If the hex string is longer than 2 characters, reverse the order of each 2-character byte
    if len(h) > 2:
        # Split into chunks of 2 characters (1 byte), reverse them, and join back
        h = ''.join([h[i:i+2] for i in range(0, len(h), 2)][::-1])

    return h

def toInt(hexval):
    """
    Converts hexidecimal value to an integer number, which can be negative
    Ref: https://www.delftstack.com/howto/python/python-hex-to-int/

    Params
    --
    hexval: [string] String of the hex value
    """
    bits = 16
    val = int(hexval, bits)
    if val & (1 << (bits-1)):
        val -= 1 << bits
    return val