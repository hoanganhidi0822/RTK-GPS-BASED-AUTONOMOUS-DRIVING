import simpleaudio as sa

wave_obj = sa.WaveObject.from_wave_file("mixkit-modern-technology-select-3124.wav")
play_obj = wave_obj.play()
play_obj.wait_done()  # chờ âm thanh phát xong
