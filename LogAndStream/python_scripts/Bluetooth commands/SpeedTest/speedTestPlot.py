#!/usr/bin/python
import multiprocessing
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from tkinter import *

matplotlib.use('TkAgg')

global fig, ax1, axes_dict, q, canvas, window
sample_index = {}

tsChString = "Timestamp"
samples_to_plot = 1000

class ChannelDetails:
    def __init__(self, name):
        self.name = name

class ChannelIds:
    COUNTER = ChannelDetails("Counter")
    DATA_RATE_CURRENT = ChannelDetails("Data Rate Current")
    DATA_RATE_OVERALL = ChannelDetails("Data Rate Overall")
    DATA_INTEGRITY = ChannelDetails("Data Integrity")

class SensorObj:
    def __init__(self, channels, title, units):
        self.channels = channels
        self.title = title
        self.units = units

class Sensors:
    COUNTER = SensorObj({ChannelIds.COUNTER}, "Counter", "none")
    DATA_RATE = SensorObj({ChannelIds.DATA_RATE_CURRENT, ChannelIds.DATA_RATE_OVERALL}, "Data Rate", "KB/s")
    DATA_INTEGRITY = SensorObj({ChannelIds.DATA_INTEGRITY}, "Data Integrity", "boolean")


def setup_plots():
    global canvas, axes_dict, q, fig, window, sample_index

    sample_index.clear()
    sample_index[Sensors.COUNTER.title] = 1
    sample_index[Sensors.DATA_RATE.title] = 1
    sample_index[Sensors.DATA_INTEGRITY.title] = 1

    # Create a window
    window = Tk()
    window.protocol("WM_DELETE_WINDOW", on_closing)

    axes_dict = {}

    fig = plt.figure(figsize=(12, 8))
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.draw()
    canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)
    canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)

    # plt.ion()

    # Create a queue to share data between process
    q = multiprocessing.Queue()


    # fig = plt.figure(figsize=(12, 8))
    # Adjust the left margin of the Plot Window to Allow Room for Labels
    fig.subplots_adjust(left=0.18)

    # plt.title('Live Stream from Device {}'.format(asm_device.TARGET_MAC_ID))

    sensor_list = [Sensors.COUNTER, Sensors.DATA_RATE, Sensors.DATA_INTEGRITY]
    num_of_sensors = len(sensor_list)

    axes_list = fig.subplots(nrows=num_of_sensors, ncols=1, squeeze=False)
    i = 0
    for sensor in sensor_list:
        axis = axes_list[i][0]
        axes_dict[sensor.title] = [sensor, axis]
        axis.clear()

        axis.set_title('{}'.format(sensor.title))
        axis.set_ylabel(sensor.title + " (" + sensor.units + ")")
        axis.set_xlabel('Sample Number')
        axis.minorticks_on()
        axis.grid(True, which='both')
        axis.ticklabel_format(axis='both', style='plain')
        leg = axis.legend(fancybox=True, fontsize='large', loc='upper right')

        for channel in sensor.channels:
            line = axis.plot([], [], label=channel.name)

        axis.legend()
        i += 1

    # plt.show()


def start_thread():
    global window

    update_plot(q)

    window.mainloop()


class PlotObj(object):
    def __init__(self, sensor_title, timestamp, sensor_data):
        self.sensor_title = sensor_title
        self.timestamp = timestamp
        self.sensor_data = sensor_data

    def print_info(self):
        print('key={}, data={}'.format(self.sensor_title, self.sensor_data))


def plot_data(q, time_s_start, time_s_end, samples, data_rate_current, data_rate_overall, data_integrity):
    # sensor_data_cal = {tsChString: [], Sensors.COUNTER: []}
    # for sample in samples:
    #     sensor_data_cal.get(Sensors.COUNTER).append([sample])
    #     sensor_data_cal[tsChString].append([sample_index[Sensors.COUNTER.title]])
    #     sample_index[Sensors.COUNTER.title] += 1
    # sensor_data_cal[Sensors.COUNTER] = list(zip(*sensor_data_cal[Sensors.COUNTER]))
    # add_data_from_sensors(sensor_data_cal, q)

    if len(samples) > 0:
        sensor_data_cal = {tsChString: [], Sensors.COUNTER: []}

        time_diff = (time_s_end - time_s_start) / len(samples)
        new_time = time_s_start
        for sample in samples:
            sensor_data_cal.get(Sensors.COUNTER).append([sample])
            sensor_data_cal[tsChString].append([new_time])
            new_time += time_diff
        sensor_data_cal[Sensors.COUNTER] = list(zip(*sensor_data_cal[Sensors.COUNTER]))
        add_data_from_sensors(sensor_data_cal, q)

    sensor_data_cal = {tsChString: [], Sensors.DATA_RATE: []}
    sensor_data_cal.get(Sensors.DATA_RATE).append([data_rate_current, data_rate_overall])
    sensor_data_cal[tsChString].append(time_s_end)
    # sensor_data_cal[tsChString].append(sample_index[Sensors.DATA_RATE.title])
    # sample_index[Sensors.DATA_RATE.title] += 1
    sensor_data_cal[Sensors.DATA_RATE] = list(zip(*sensor_data_cal[Sensors.DATA_RATE]))
    add_data_from_sensors(sensor_data_cal, q)

    if len(data_integrity) > 0:
        sensor_data_cal = {tsChString: [], Sensors.DATA_INTEGRITY: []}
        time_diff = (time_s_end - time_s_start) / len(samples)
        new_time = time_s_start
        for sample in data_integrity:
            sensor_data_cal.get(Sensors.DATA_INTEGRITY).append([sample])
            sensor_data_cal[tsChString].append([new_time])
            new_time += time_diff
        sensor_data_cal[Sensors.DATA_INTEGRITY] = list(zip(*sensor_data_cal[Sensors.DATA_INTEGRITY]))
        add_data_from_sensors(sensor_data_cal, q)


def add_data_from_sensors(data, q):
    for key in data:
        # TODO figure out why there would be a NoneType here
        if key is None:
            continue

        # TODO figure out why instance check isn't working here
        # if isinstance(key, ASM_Device.SensorObj):
        if key != tsChString:
            sensor_data = data[key]

            if len(sensor_data) > 0:
                plot_obj = PlotObj(key.title, data[tsChString], sensor_data)
                q.put(plot_obj)


def update_plot(q_lcl):
    global canvas, axes_dict, samples_to_plot

    something_was_plotted = False

    try:       #Try to check if there is data in the queue
        plot_obj = q_lcl.get_nowait()

        while plot_obj is not None:
            if isinstance(plot_obj, str) and plot_obj == 'Q':
                # print 'done'
                pass
            else:
                # print result

                sensor_title = plot_obj.sensor_title
                obj = axes_dict.get(sensor_title)
                if obj is not None:
                    axis = obj[1]
                    if axis is not None:
                        lines = axis.get_lines()

                        line_index = 0
                        new_data_x = plot_obj.timestamp

                        min_x = 10000000000
                        max_x = 0
                        min_y = 10000000000
                        max_y = 0

                        for data_set in plot_obj.sensor_data:
                            # TODO Improve hacky approach
                            if isinstance(data_set, float):
                                data_set_to_plot = plot_obj.sensor_data
                            else:
                                data_set_to_plot = data_set

                            line = lines[line_index]

                            new_data_y = data_set_to_plot

                            data_x = np.append(line.get_xdata(), new_data_x)
                            data_y = np.append(line.get_ydata(), new_data_y)

                            # if len(data_x) > samples_to_plot:
                            #     data_x = data_x[len(data_x) - samples_to_plot:len(data_x)]
                            #     data_y = data_y[len(data_y) - samples_to_plot:len(data_y)]

                            index_to_cull_from = 0
                            endTime = data_x[len(data_x)-1]
                            for time_value in data_x:
                                if (endTime - time_value) <= 5:
                                    break
                                index_to_cull_from += 1

                            data_x = data_x[index_to_cull_from:len(data_x)]
                            data_y = data_y[index_to_cull_from:len(data_y)]

                            line.set_xdata(data_x)
                            line.set_ydata(data_y)

                            axis.draw_artist(line)
                            # fig.draw_artist(axis)

                            min_x = min(min_x, min(data_x))
                            max_x = max(max_x, max(data_x))
                            min_y = min(min_y, min(data_y))
                            max_y = max(max_y, max(data_y))

                            line_index += 1

                            # TODO Improve hacky approach
                            if isinstance(data_set, float):
                                break

                        axis.set_ylim(min_y, max_y + 1)  # +1 to avoid singular transformation warning
                        axis.set_xlim(min_x, max_x + 1)

                        something_was_plotted = True
                        # fig.canvas.draw_idle()
            plot_obj = q_lcl.get_nowait()
        # window.after(10, update_plot, q_lcl)
    except Exception as ex:
        # trace = []
        # tb = ex.__traceback__
        # while tb is not None:
        #     trace.append({
        #         "filename": tb.tb_frame.f_code.co_filename,
        #         "name": tb.tb_frame.f_code.co_name,
        #         "lineno": tb.tb_lineno
        #     })
        #     tb = tb.tb_next
        if str(ex):
            print(str({
                'type' : type(ex).__name__,
                'message': str(ex),
                # 'trace' : trace
            }))

        if something_was_plotted:
            canvas.draw()
        # print "empty"
        window.after(500, update_plot, q_lcl)


def on_closing():
    global window, q
    # if messagebox.askokcancel("Quit", "Do you want to quit?"):
    window.destroy()
    q.close()
    q.join_thread()
    window.quit()
    print("Plot closed")
