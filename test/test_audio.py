import simpleaudio as sa

wave_obj = sa.WaveObject.from_wave_file("/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/test/KhoaDien.wav")
play_obj = wave_obj.play()
# play_obj.wait_done()  # chờ âm thanh phát xong
print("hihi")