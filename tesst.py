import simpleaudio as sa
import threading

# Biến toàn cục để giữ play object
play_obj = None

def play_sound():
    global play_obj
    wave_obj = sa.WaveObject.from_wave_file("/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/VISUALIZATION/output.wav")
    while 1:
        play_obj = wave_obj.play()
        # play_obj.wait_done()
        print("hihi")
        time.sleep(1)

def stop_sound():
    global play_obj
    if play_obj is not None and play_obj.is_playing():
        play_obj.stop()
# Phát âm thanh
threading.Thread(target=play_sound).start()

# Dừng âm thanh sau 2 giây (ví dụ)
import time
time.sleep(3)
stop_sound()