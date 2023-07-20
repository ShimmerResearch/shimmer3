CRC_INIT = 0xB0CA


def crc_byte(crc, b):
    crc = ((crc >> 8) & 0xffff | (crc << 8) & 0xffff) & 0xffff
    crc ^= b & 0xffff
    crc ^= ((crc & 0xff) >> 4) & 0xffff
    crc ^= (crc << 12) & 0xffff
    crc ^= ((crc & 0xff) << 5) & 0xffff
    return crc


def calc_crc(length, msg):
    crc_alc = crc_byte(CRC_INIT, msg[0])
    for i in range(1, length):
        crc_alc = crc_byte(crc_alc, msg[i])

    if length % 2 == 1:
        crc_alc = crc_byte(crc_alc, 0x00)

    return crc_alc


def crc_check(length, msg):
    crc = calc_crc(length - 2, msg)
    if ((crc & 0xFF) == msg[length - 2]) and (((crc & 0xFF00) >> 8) == msg[length - 1]):
        return True
    else:
        return False
