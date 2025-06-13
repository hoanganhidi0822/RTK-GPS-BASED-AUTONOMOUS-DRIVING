# voice.py
import time
import simpleaudio as sa
import geopy.distance
import os

import config as cf

cf.latitude = None
cf.longitude = None
cf.is_target = 0
def area_audio_thread_func(lat, lon):
    areas = [
        {"name": "Khu C",            "center": (10.853212809462221, 106.77152053728717), "radius": 50,  "audio_file": "VOICE/toanha/khu_c.wav"    },
        {"name": "Khu D",            "center": (10.852303121191104, 106.77141033390284), "radius": 50,  "audio_file": "VOICE/audio/KhoaDien.wav"    },
        {"name": "Khu F",            "center": (10.851896027202086, 106.77277995969585), "radius": 30,  "audio_file": "VOICE/toanha/toaF.wav"          },
        {"name": "Tòa Trung Tâm",    "center": (10.851304164955788, 106.77200006697142), "radius": 50, "audio_file": "VOICE/audio/toaTrungTam.wav" },
        {"name": "Maker Space",      "center": (10.851603865021001, 106.77336561933349), "radius": 20,  "audio_file": "VOICE/audio/MakerSpace.wav"  },
        {"name": "Toa Viet Duc",     "center": (10.851516990762079, 106.77274485037545), "radius": 20,  "audio_file": "VOICE/toanha/vietduc.wav"  },
        {"name": "Co Khi Dong Luc",  "center": (10.852385926815334, 106.77285134596933), "radius": 20,  "audio_file": "VOICE/audio/cokhidongluc.wav"},
        {"name": "Tien Loi",         "center": (10.851006360834074, 106.77124753517441), "radius": 30,  "audio_file": "VOICE/toanha/tien_loi.wav"}
    ]

    def is_in_area(current_pos, area_center, radius_m):
        distance = geopy.distance.geodesic(current_pos, area_center).m
        return distance <= radius_m

    last_area = None
    audio_play_obj = None  # giữ lại đối tượng phát âm để có thể dừng

    def play_audio_file(file_path: str):
        nonlocal audio_play_obj
        if not os.path.exists(file_path):
            print(f"⚠️ Không tìm thấy file âm thanh: {file_path}")
            return
        try:
            wave_obj = sa.WaveObject.from_wave_file(file_path)
            audio_play_obj = wave_obj.play()
            
        except Exception as e:
            print("Lỗi khi phát âm thanh:", e)

    while True:
        if cf.is_target == 1:
            if audio_play_obj and audio_play_obj.is_playing():
                audio_play_obj.stop()
                audio_play_obj = None
                print("🛑 Dừng âm thanh vì đang di chuyển đến mục tiêu")
            last_area = None
            time.sleep(1)
            continue
        else:
            current_position = (cf.latitude, cf.longitude)
            found_area = None

            for area in areas:
                if is_in_area(current_position, area['center'], area['radius']):
                    found_area = area
                    break
            
            
                

            if found_area and (last_area != found_area['name']):
                if audio_play_obj and audio_play_obj.is_playing():
                    audio_play_obj.stop()
                    audio_play_obj = None
                print(f"✅ Đã đến {found_area['name']}")
                play_audio_file(found_area['audio_file'])
                last_area = found_area['name']
            elif not found_area:
                last_area = None

            time.sleep(2)
