from UnifiWebsockets import Unifi
from URSentry import URSentry
import queue
import time
import threading
import BBoxProcessor

q = queue.Queue()

x = threading.Thread(target=Unifi.run, args=(q,))
x.start()

b = BBoxProcessor.BBoxProcessor()

ur = URSentry("172.22.114.160")

joystick_queue = queue.Queue()


class CameraJoystick(threading.Thread):
    def __init__(self, joystick_queue: queue.Queue[list[int]]):
        threading.Thread.__init__(self)
        self.joystick_queue = joystick_queue
        self.keep_running = True

    def run(self):
        global current_joystick
        while self.keep_running:
            time.sleep(0.0001)
            q_val = []
            try:
                q_val = q.get_nowait()
            except queue.Empty:
                pass
            #print(bb)
            current_joystick = b.get_joystick_position_from_new_set_of_bboxes(q_val)
            #self.joystick_queue.put(bb)

    def stop(self):
        self.keep_running = False


current_joystick = [0, 0]


class JoystickScheduler(threading.Thread):
    def __init__(self, joystick_queue: queue.Queue[list[int]]):
        threading.Thread.__init__(self)
        self.joystick_queue = joystick_queue
        self.keep_running = True

    def run(self):
        while self.keep_running:
            global current_joystick
            current_joystick = self.joystick_queue.get()

    def stop(self):
        self.keep_running = False


camera_joystick = CameraJoystick(joystick_queue)
camera_joystick.start()
# joystick_scheduler = JoystickScheduler(joystick_queue)
# joystick_scheduler.start()

ur.initialize_pose()

while True:
    try:
        ur.control_robot(current_joystick)
        #print("Setting robot base velocity to: ", current_joystick[0])
        time.sleep(0.2)
    except KeyboardInterrupt:
        camera_joystick.stop()
        # joystick_scheduler.stop()
        break
