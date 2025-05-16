import os
import pyaudio
import wave
from google import genai
import time
import requests
import json
from playsound import playsound
import numpy as np
from gtts import gTTS
import config as cf
import noisereduce as nr  # Thêm thư viện giảm nhiễu


cf.cf_destination = "none"


class virtual_assistance:
    def __init__(self):
        print("Bắt đầu trợ lý ảo.")
        self.count_call = 0

    def get_record(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK = 1024
        OUTPUT_FILENAME = "recorded_audio_1.wav"
        SILENCE_THRESHOLD = 4000  # Ngưỡng năng lượng để phát hiện khoảng lặng
        SILENCE_DURATION = 1.5  # Số giây im lặng liên tiếp để dừng ghi âm

        audio = pyaudio.PyAudio()
        stream = audio.open(format=FORMAT, channels=CHANNELS,
                            rate=RATE, input=True,
                            frames_per_buffer=CHUNK)

        print("🔴 Đang ghi âm... (Tự động dừng khi phát hiện khoảng lặng)")

        frames = []
        silence_start = None  # Thời điểm bắt đầu khoảng lặng

        while True:
            data = stream.read(CHUNK)
            # print("--------------1---------------")

            frames.append(data)
            # print("--------------2---------------")

            # Chuyển dữ liệu âm thanh thành mảng numpy
            audio_data = np.frombuffer(data, dtype=np.int16)
            energy = np.abs(audio_data).mean()  # Tính năng lượng trung bình

            # print(f"energy: {energy}")

            # Kiểm tra nếu âm thanh nhỏ hơn ngưỡng khoảng lặng
            if energy < SILENCE_THRESHOLD:
                if silence_start is None:
                    silence_start = time.time()  # Bắt đầu đếm thời gian khoảng lặng
                elif time.time() - silence_start >= SILENCE_DURATION:
                    print("🟢 Khoảng lặng dài, dừng ghi âm!")
                    break
            else:
                silence_start = None  # Reset nếu có âm thanh trở lại

        print("🟢 Ghi âm hoàn tất!")

        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Chuyển đổi frames thành mảng numpy để lọc nhiễu
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        reduced_noise = nr.reduce_noise(y=audio_data, sr=RATE)

        # Lưu file sau khi lọc nhiễu
        wf = wave.open(OUTPUT_FILENAME, "wb")
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(reduced_noise.astype(np.int16).tobytes())
        wf.close()


        # wf = wave.open(OUTPUT_FILENAME, "wb")
        # wf.setnchannels(CHANNELS)
        # wf.setsampwidth(audio.get_sample_size(FORMAT))
        # wf.setframerate(RATE)
        # wf.writeframes(b''.join(frames))
        # wf.close()

        print(f"✅ File đã được lưu: {OUTPUT_FILENAME}")


    def understanding_record(self):
        # api_key: AIzaSyCTqQM557q_5iUS-RQs661124KDC_wGryM
        client = genai.Client(
        api_key="AIzaSyAIptARWvsfvfWfubmwI0eBMrBZm2t34oc",
        )

        myfile = client.files.upload(file='recorded_audio_1.wav')

        prompt =  """ Bạn là trợ lý ảo trên xe tự hành của phòng thí nghiệm hệ thống thông minh trường đại học sư phạm kỹ thuật thành phố Hồ Chí Minh,  
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

        response = client.models.generate_content(
        model='gemini-2.0-flash',
        contents=[prompt, myfile]
        )

        return response.text
    
    def get_speech(self, text):

        # print(text)
        is_run = False

        print(f"Nhận được text: {repr(text)}")

        destination = {
            'khu_c': "Tôi sẽ đưa bạn đến Khu c nhé.",
            'khu_d': "Tôi sẽ đưa bạn đến Khu d nhé.",
            'trung_tam': "Tôi sẽ đưa bạn đến tòa nhà Trung tâm nhé.",
            'viet_duc': "Tôi sẽ đưa bạn đến tòa Việt Đức nhé.",
            'go': "Tôi sẽ đưa bạn đến tòa xưởng gỗ nhé."
        }

        if text.strip().lower() in destination:
            text_respond = destination[text.strip().lower()]
            cf.cf_destination = text.strip().lower()
            is_run = True
        else:
            text_respond = text


        print("phản hồi: ", text_respond)


        tts = gTTS(text = text_respond, lang="vi")

        tts.save("output.mp3")
        time.sleep(0.1)
        playsound("output.mp3")
        os.remove("output.mp3")

        return is_run
    
    def run(self):

        self.get_record()
        # time.sleep(0.2)
        text = self.understanding_record()
        # time.sleep(0.2)
        # print(text)
        is_run = self.get_speech(text)

        return is_run

        # print(text)

def run_assistance():
    islab_assistance = virtual_assistance()

    while True:

        is_run = islab_assistance.run()

        print(is_run)

        if is_run == True:
            break

if __name__ == "__main__":
    run_assistance()
