import signal
import time

from pond import Manager

is_running = True

def signal_handler(signum, frame):
    global is_running

    if signum not in (signal.SIGINT, signal.SIGTERM):
        return

    print(" INTERRUPT", flush=True)
    is_running = False


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    pm = Manager(True, False)

    pm.load_module(
        "camera", "v4l2/v4l2_camera", "thread",
        {
            "width" : 640,
            "height" : 480,
            "fps" : 30,
            "device" : "/dev/video0"
        },
        channel_mappings={"out": "color/image",},
    )

    pm.load_module(
        "streamer", "video/streamer", "thread",
        {
            "width" : 640,
            "height" : 480,
            "fps" : 30,
            "port" : 5000,
            "ip" : "127.0.0.1",
            "format" : "RGB8",
            "send_packets_times" : 1
        },
        channel_mappings={"in": "color/image",},
    )

    # pm.load_module(
    #     "streamer", "gstreamer/rtp_server", "thread",
    #     {
    #         "width": 640, "height": 480,
    #         "format": "RGB8",

    #         "ip": "127.0.0.1",
    #         "port": 5000, 
            
    #         "bitrate": 3000,
    #         "key_int_max:": 30
    #     },
    #     channel_mappings={"in": "color/image",},
    # )

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass

if __name__ == "__main__":
    main()