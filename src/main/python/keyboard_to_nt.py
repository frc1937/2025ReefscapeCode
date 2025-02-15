import keyboard
import ntcore
import time

minimum_press_time = 0.35
keys_dict = {}

ntcoreinst = None
table = None

def connectNT():
    global ntcoreinst, table
    ntcoreinst = ntcore.NetworkTableInstance.getDefault()
    print("Setting up NetworkTables client")
    ntcoreinst.startClient4("KeyboardToNT")
    ntcoreinst.setServer("127.0.0.1")
    ntcoreinst.startDSClient()

    print("Waiting for connection to NetworkTables server...")
    while not ntcoreinst.isConnected():
        time.sleep(0.1)

    table = ntcoreinst.getTable("/SmartDashboard/keyboard")
    print("Connected!")

def turn_off_keys_with_delay():
    global ntcoreinst, table, minimum_press_time, keys_dict
    while True:
        if (ntcoreinst is not None and not ntcoreinst.isConnected()):
            restart()
            continue

        time.sleep(0.01)
        for key in keys_dict:
            if (keys_dict[key][1] and time.time() - keys_dict[key][0] > minimum_press_time):
                table.putBoolean(key, False)
                keys_dict.pop(key, None)
                print(key + " is out")
                break

def restart():
    global keys_dict
    print("Restarting")
    keys_dict = {}
    keyboard.stop_recording()
    ntcoreinst.stopClient()
    connectNT()
    keyboard.start_recording()

def on_action(event: keyboard.KeyboardEvent):
    global ntcoreinst, table, minimum_press_time, keys_dict
    if event == None or event.name == None or event.name == "/":
        return

    key = event.name.lower()

    isPressed = event.event_type == keyboard.KEY_DOWN

    if isPressed:
        table.putBoolean(key, True)
        if key not in keys_dict:
            keys_dict[key] = [time.time(), False]
        return

    if key in keys_dict:
        keys_dict[key][1] = True
    else:
        keys_dict[key] = [time.time(), False]

def main():
    connectNT()
    keyboard.start_recording()
    keyboard.hook(on_action)
    turn_off_keys_with_delay()

main()