# voice.py
import time
import simpleaudio as sa
import geopy.distance
import os

import config as cf

cf.latitude = None
cf.longitude = None
def area_audio_thread_func(lat, lon):
    areas = [
        {"name": "Khu C",         "center": (10.853212809462221, 106.77152053728717), "radius": 50, "audio_file": "VOICE/audio/KhoaDien.wav"},
        {"name": "Khu D",         "center": (10.852303121191104, 106.77141033390284), "radius": 50, "audio_file": "VOICE/audio/KhoaDien.wav"},
        {"name": "Khu F",         "center": (10.851896027202086, 106.77277995969585), "radius": 30, "audio_file": "VOICE/audio/F1.wav"},
        {"name": "Tòa Trung Tâm", "center": (10.851310311327158, 106.77198007567596), "radius": 100, "audio_file": "VOICE/audio/toaTrungTam.wav"},
        {"name": "Maker Space",   "center": (10.851603865021001, 106.77336561933349), "radius": 20, "audio_file": "VOICE/audio/MakerSpace.wav"},
        {"name": "Toa Viet Duc",  "center": (10.851371749677574, 106.77271857948108), "radius": 20, "audio_file": "VOICE/audio/MakerSpace.wav"},
        {"name": "Co Khi Dong Luc",  "center": (10.852306705230843, 106.77283952111178), "radius": 20, "audio_file": "VOICE/audio/cokhidongluc.wav"},
    ]

    def is_in_area(current_pos, area_center, radius_m):
        distance = geopy.distance.geodesic(current_pos, area_center).m
        return distance <= radius_m

    def play_audio_file(file_path: str):
        if not os.path.exists(file_path):
            print(f"⚠️ Không tìm thấy file âm thanh: {file_path}")
            return
        try:
            wave_obj = sa.WaveObject.from_wave_file(file_path)
            play_obj = wave_obj.play()
            play_obj.wait_done()
        except Exception as e:
            print("Lỗi khi phát âm thanh:", e)

    last_area = None
    while True:
        current_position =cf.latitude, cf.longitude 
        found_area = None

        for area in areas:
            if is_in_area(current_position, area['center'], area['radius']):
                found_area = area
                break

        if found_area and (last_area != found_area['name']):
            print(f"✅ Đã đến {found_area['name']}")
            play_audio_file(found_area['audio_file'])
            last_area = found_area['name']
        elif not found_area:
            last_area = None

        time.sleep(2)
