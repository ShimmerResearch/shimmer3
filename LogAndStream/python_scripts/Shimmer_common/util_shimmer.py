
def byte_array_to_hex_string(data):
    hex_str = ""
    first_byte = True
    for b in data:
        if not first_byte:
            hex_str += ' '
        hex_str += "0x%0.2X" % b
        first_byte = False
    return '[' + hex_str + ']'


def byte_array_to_int(data, lsb_order=True, is_signed=False):
    number = 0
    i = 0

    for b in data:
        if lsb_order:
            number += (b << i * 8)
        else:
            number = (number << 8) + b
        i += 1

    # two's compliment
    if is_signed:
        bit_length = len(data) * 8
        # sign_offset = 2 ** (bit_length - 1)
        # number -= sign_offset
        # Copied from Java implementation but I'm not sure if the maths is exactly correct
        if number >= (1 << (bit_length - 1)):
            number = -((number ^ ((2 ** bit_length) - 1)) + 1)

    # print('data={}, lsb_order={}, is_signed={}, number={}'.format(byte_array_to_hex_string(data), lsb_order, is_signed, number))

    return number
