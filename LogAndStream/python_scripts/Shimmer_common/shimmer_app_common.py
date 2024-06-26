import sys

from Shimmer_common import shimmer_device


def get_selected_com_port(dock_ports=True):
    com_port = ''
    if len(sys.argv) < 2:
        if dock_ports:
            options = shimmer_device.serial_ports_shimmer_dock()
        else:
            options = shimmer_device.serial_ports_bluetooth()

        if len(options) > 0:
            user_input = ''
            input_message = "Pick an option:\n"
            for index, item in enumerate(options):
                # input_message += f'{index + 1}) {item}\n'
                input_message += str(index + 1) + ") " + item.name + ", " + item.description + ", " + item.hwid + "\n"
            input_message += "Your choice: "
            # while (not user_input.isnumeric()) or int(user_input) < 1 or int(user_input) > len(options):
            while not user_input or int(user_input) < 1 or int(user_input) > len(options):
                user_input = input(input_message)

            com_port = options[int(user_input) - 1].name
            print("You picked: " + com_port + "\n")
    else:
        com_port = sys.argv[1]

    return com_port
