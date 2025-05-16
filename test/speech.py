import requests
import json
from pydub import AudioSegment
import simpleaudio as sa
import os
import time
import textwrap

# Cấu hình API
API_KEY = 'K5jOdAj46wLFqOHkgDwROHE7AFs7ZlCx'
TTS_URL = 'https://api.fpt.ai/hmi/tts/v5'
VOICE = 'banmai'
SPEED = '-0.2'

# Văn bản cần chuyển đổi
full_text = """Tòa nhà F1 tại Đại học Sư phạm Kỹ thuật TP.HCM . Là công trình tương đối mới, nơi đây được trang bị nhiều phòng học và xưởng thực hành hiện đại, phục vụ hiệu quả cho việc giảng dạy, học tập và rèn luyện kỹ năng thực tế cho sinh viên."""

# Tách đoạn (mỗi đoạn khoảng 100 ký tự, không ngắt từ)
segments = textwrap.wrap(full_text, width=100, break_long_words=False)

# Danh sách file WAV từng đoạn
wav_files = []

# Hàm xử lý TTS từng đoạn
def process_tts_segment(segment_text, index):
    headers = {
        'api-key': API_KEY,
        'speed': SPEED,
        'voice': VOICE
    }

    print(f"🟡 Đang xử lý đoạn {index + 1}/{len(segments)}...")
    response = requests.post(TTS_URL, data=segment_text.encode('utf-8'), headers=headers)
    
    if response.status_code != 200:
        print(f"❌ Lỗi trong quá trình gửi yêu cầu TTS: {response.text}")
        return

    response_data = json.loads(response.text)
    audio_url = response_data.get("async")

    if not audio_url:
        print("❌ Không tìm thấy URL âm thanh.")
        return

    # Chờ file mp3 sẵn sàng
    max_wait = 15
    waited = 0
    interval = 2

    while waited < max_wait:
        audio_response = requests.get(audio_url)
        if audio_response.status_code == 200 and audio_response.headers.get('Content-Type') == 'audio/mpeg':
            mp3_path = f"part{index + 1}.mp3"
            wav_path = f"part{index + 1}.wav"
            with open(mp3_path, "wb") as f:
                f.write(audio_response.content)

            # Chuyển mp3 -> wav
            sound = AudioSegment.from_mp3(mp3_path)
            sound.export(wav_path, format="wav")

            # Lưu để ghép sau
            wav_files.append(wav_path)
            print(f"✅ Đã lưu file: {wav_path}")
            return
        else:
            print(f"⏳ Chưa sẵn sàng... chờ {interval}s")
            time.sleep(interval)
            waited += interval

    print("❌ Quá thời gian chờ file mp3.")

# Gửi tất cả đoạn
for idx, seg in enumerate(segments):
    process_tts_segment(seg, idx)

# Ghép file WAV
if wav_files:
    print("🔧 Đang ghép các đoạn âm thanh...")
    combined = AudioSegment.empty()
    silence = AudioSegment.silent(duration=50)  # 50ms lặng để nối mượt

    for path in wav_files:
        segment = AudioSegment.from_wav(path)

        # Cắt bớt phần đuôi (nếu >50ms)
        if len(segment) > 50:
            segment = segment[:-50]

        combined += segment + silence

    # Lưu file cuối
    combined.export("toaF1.wav", format="wav")
    print("✅ Đã tạo file: toaF1.wav")

    # Phát thử
    wave_obj = sa.WaveObject.from_wave_file("toaF1.wav")
    wave_obj.play().wait_done()

    # Xóa file tạm
    for path in wav_files:
        os.remove(path)
        mp3_path = path.replace(".wav", ".mp3")
        if os.path.exists(mp3_path):
            os.remove(mp3_path)
else:
    print("❌ Không có đoạn nào để ghép.")
