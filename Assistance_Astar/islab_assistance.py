import os
import wave
import sounddevice as sd
import numpy as np
from gtts import gTTS
import time
import requests
import json
from playsound import playsound
from google import genai
import noisereduce as nr
import config as cf
from pydub import AudioSegment
import simpleaudio as sa
cf.cf_destination = "none"


url = 'https://api.fpt.ai/hmi/tts/v5'

headers = {
    'api-key': 'K5jOdAj46wLFqOHkgDwROHE7AFs7ZlCx',
    'speed': '0',
    'voice': 'banmai'
}




class virtual_assistance:
    def __init__(self):
        print("Bắt đầu trợ lý ảo.")
        self.count_call = 0

    def get_record(self):
        CHANNELS = 1
        RATE = 16000
        DURATION_LIMIT = 4  # Giới hạn thời gian ghi tối đa (phòng trường hợp không im lặng)
        CHUNK = 1024
        OUTPUT_FILENAME = "recorded_audio_1.wav"
        SILENCE_THRESHOLD = 3000
        SILENCE_DURATION = 1.1
        
        print("🔴 Đang ghi âm...")

        frames = []
        silence_start = None
        start_time = time.time()

        def callback(indata, frames_count, time_info, status):
            nonlocal frames, silence_start, start_time, stream
            audio_data = indata[:, 0]  # Mono channel
            frames.append(audio_data.copy())

            energy = np.abs(audio_data).mean() * 32767  # Tính năng lượng (giống như int16)
            if energy < SILENCE_THRESHOLD:
                if silence_start is None:
                    silence_start = time.time()
                elif time.time() - silence_start > SILENCE_DURATION:
                    print("🟢 Khoảng lặng dài, dừng ghi âm!")
                    raise sd.CallbackStop()
            else:
                silence_start = None

            if time.time() - start_time > DURATION_LIMIT:
                print("⏱️ Hết thời gian ghi tối đa.")
                raise sd.CallbackStop()

        stream = sd.InputStream(callback=callback, samplerate=RATE,
                                channels=CHANNELS, blocksize=CHUNK)
        with stream:
            sd.sleep(int(DURATION_LIMIT * 1000))  # Cho chạy stream trong thời gian tối đa

        print("🟢 Ghi âm hoàn tất!")

        audio_np = np.concatenate(frames)
        # reduced_noise = nr.reduce_noise(y=audio_np, sr=RATE)
        reduced_noise = nr.reduce_noise(
        y=audio_np,
        sr=RATE,
        stationary=True,
        prop_decrease=1.0,      # giảm nhiễu tối đa
        time_mask_smooth_ms=50, # làm mượt ngắn hơn
    )

        # Ghi vào file WAV
        wf = wave.open(OUTPUT_FILENAME, "wb")
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # 2 bytes cho int16
        wf.setframerate(RATE)
        wf.writeframes((reduced_noise * 32767).astype(np.int16).tobytes())
        wf.close()

        print(f"✅ File đã được lưu: {OUTPUT_FILENAME}")

    def understanding_record(self):
        client = genai.Client(
            api_key="AIzaSyAIptARWvsfvfWfubmwI0eBMrBZm2t34oc",
        )

        myfile = client.files.upload(file='recorded_audio_1.wav')

        prompt = """ Bạn là trợ lý ảo trên xe tự hành của phòng thí nghiệm hệ thống thông minh trường đại học sư phạm kỹ thuật thành phố Hồ Chí Minh,  
                    lấy nội dung đoạn hôi thoại trên, người dùng có 3 nhu cầu hãy phân loại thành 3 loại bên dưới: 
                    - bạn có thể đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ thì trả về đoạn text tương ứng khu_c, khu_d, trung_tam, viet_duc, go

                    Đây là thông tin của bạn:
                    - bạn có thể chở mọi người đến các địa điểm sau: khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ của trường đại học sư phạm kỹ thuật thành phố Hồ Chí Minh.
                    - nếu có câu hỏi nào khác về bạn hãy trả lời một cách hài hước nhưng vẫn lịch sự.

                    lưu ý khi phản hồi:
                    - không kèm các icon trong nội dung.
                    - phản hồi xúc tích, không lặp toàn bộ prompt này.
                    - nếu không nhận được yêu cầu hãy phản hồi: tôi có thể đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ.
                    """
        # prompt = """
        #         Bạn là trợ lý ảo trên xe tự hành của phòng thí nghiệm Hệ thống Thông minh, Đại học Sư phạm Kỹ thuật TP.HCM. 
        #         Dựa vào đoạn hội thoại đầu vào, người dùng có thể có một trong 3 loại nhu cầu sau:

        #         1. Nếu họ muốn đến một trong các địa điểm sau: khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ — hãy phân loại yêu cầu và trả về một trong các nhãn: khu_c, khu_d, cong_truong, trung_tam, viet_duc, go.
        #         2. Nếu họ hỏi về bạn hoặc xe tự hành, hãy trả lời một cách hài hước nhưng vẫn lịch sự.
        #         3. Nếu không nhận ra yêu cầu cụ thể, hãy trả lời: "Tôi có thể đến các khu C, khu D, cổng trường, tòa nhà trung tâm, tòa Việt Đức, xưởng gỗ."

        #         Lưu ý:
        #         - Không dùng icon trong phản hồi.
        #         - Phản hồi ngắn gọn, không lặp lại toàn bộ hướng dẫn này.
        #         """

        response = client.models.generate_content(
            model='gemini-2.0-flash',
            contents=[prompt, myfile]
        )

        return response.text

    def get_speech(self, text):
        is_run = False

        print(f"Nhận được text: {repr(text)}")

        destination = {
            'khu_c': "Tôi sẽ đưa bạn đến Khu c. ",
            'khu_d': "Tôi sẽ đưa bạn đến Khu d. ",
            'trung_tam': "Tôi sẽ đưa bạn đến tòa nhà Trung tâm.",
            'viet_duc': "Tôi sẽ đưa bạn đến tòa Việt Đức.",
            'go': "Tôi sẽ đưa bạn đến xưởng gỗ."
        }

        if text.strip().lower() in destination:
            text_respond = destination[text.strip().lower()]
            cf.cf_destination = text.strip().lower()
            is_run = True
        else:
            text_respond = text

        print("phản hồi: ", text_respond)

        # Gửi yêu cầu API
        response = requests.post(url, data=text_respond.encode('utf-8'), headers=headers)

        # Chuyển đổi phản hồi thành JSON
        response_data = json.loads(response.text)

        # Lấy URL file âm thanh
        audio_url = response_data.get("async")

        if audio_url:
            print("Tải file từ:", audio_url)
            for attempt in range(5):
                audio_response = requests.get(audio_url, allow_redirects=True)
                if audio_response.status_code == 200 and audio_response.content:
                    with open("output.mp3", "wb") as f:
                        f.write(audio_response.content)
                    print("Tải xuống thành công: output.mp3")

                    # Convert MP3 to WAV
                    sound = AudioSegment.from_mp3("output.mp3")
                    sound.export("output.wav", format="wav")

                    # Play WAV
                    try:
                        voice_obj = sa.WaveObject.from_wave_file("output.wav")
                        play_voice = voice_obj.play()
                        play_voice.wait_done()
                    except Exception as e:
                        print("Lỗi khi phát âm thanh:", e)
                    finally:
                        os.remove("output.mp3")
                        os.remove("output.wav")
                    break
                else:
                    print(f"Lần {attempt+1}: chưa có file. Thử lại sau 1 giây...")
                    time.sleep(1)
            else:
                print("Không tải được file sau nhiều lần thử.")
        else:
            print("Không tìm thấy URL âm thanh!")
        return is_run

    def run(self):
        self.get_record()
        text = self.understanding_record()
        is_run = self.get_speech(text)
        return is_run


def run_assistance():
    islab_assistance = virtual_assistance()

    while True:
        is_run = islab_assistance.run()
        print(is_run)
        if is_run:
            break

if __name__ == "__main__":
    run_assistance()
