import datetime


def ms_to_shimmer_rtc_bytes(ts_ms):
    ts_ticks = int(ts_ms * 32768)

    byte0 = (ts_ticks & 0xFF)
    byte1 = ((ts_ticks >> 8) & 0xFF)
    byte2 = ((ts_ticks >> 16) & 0xFF)
    byte3 = ((ts_ticks >> 24) & 0xFF)
    byte4 = ((ts_ticks >> 32) & 0xFF)
    byte5 = ((ts_ticks >> 40) & 0xFF)
    byte6 = ((ts_ticks >> 48) & 0xFF)
    byte7 = ((ts_ticks >> 56) & 0xFF)

    time_bytes = [byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7]
    # time_bytes = ts_ticks.to_bytes(8, 'little')
    return time_bytes


def seconds_to_time_str(seconds, show_microseconds):
    if seconds >= (2 ** 31) - 1:
        return "Not valid"
    ts = datetime.datetime.fromtimestamp(seconds)
    # ts = datetime.datetime.utcfromtimestamp(seconds)
    if show_microseconds:
        str = ts.strftime("%d %B, %Y, %H:%M:%S,%f")
    else:
        str = ts.strftime("%d %B, %Y, %H:%M:%S")

    # # Note: Microseconds can be printed using the time library
    # ts = time.gmtime(seconds)
    # str = time.strftime("%d %B, %Y, %H:%M:%S", ts)

    return str
