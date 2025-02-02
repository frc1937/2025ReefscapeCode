import ntcore
import keyboard
import time
import threading

ntcoreinst = ntcore.NetworkTableInstance.getDefault()

print("Setting up NetworkTables client")
ntcoreinst.startClient4("KeyboardToNT")
ntcoreinst.setServer("127.0.0.1")
ntcoreinst.startDSClient()

# Wait for connection
print("Waiting for connection to NetworkTables server...")
while not ntcoreinst.isConnected():
    time.sleep(0.1)

table = ntcoreinst.getTable("/SmartDashboard/keyboard")

print("Connected!")

minimum_press_time = 0.3
keys_dict = {}
lock = threading.Lock()


def turn_off_keys_with_delay():
    while True:
        time.sleep(0.05)
        lock.acquire()
        for key in keys_dict:
            if time.time() - keys_dict[key] > minimum_press_time:
                table.putBoolean(key, False)
                keys_dict.pop(key, None)
                break
        lock.release()


def on_action(event: keyboard.KeyboardEvent):
    if event.name == "/":
        return

    key = event.name.lower()

    isPressed = event.event_type == keyboard.KEY_DOWN

    if isPressed:
        table.putBoolean(key, True)
        return
    lock.acquire()
    if key not in keys_dict:
        keys_dict[key] = time.time()
    lock.release()


def main():
    keyboard.hook(on_action)
    thread = threading.Thread(turn_off_keys_with_delay())
    thread.start()
    keyboard.wait()


main()
