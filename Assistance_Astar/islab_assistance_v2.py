import os
import wave
import sounddevice as sd
import numpy as np
# from gtts import gTTS
import time
import requests
import json
# from playsound import playsound
import config as cf
import noisereduce as nr
from pydub import AudioSegment
import simpleaudio as sa
import speech_recognition as sr
from google import genai

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
        DURATION_LIMIT = 5
        CHUNK = 1024
        OUTPUT_FILENAME = "recorded_audio_1.wav"
        SILENCE_THRESHOLD = 4000
        SILENCE_DURATION = 1.1

        print("🔴 Đang ghi âm...")

        frames = []
        silence_start = None
        start_time = time.time()

        def callback(indata, frames_count, time_info, status):
            nonlocal frames, silence_start, start_time, stream
            audio_data = indata[:, 0]
            frames.append(audio_data.copy())

            energy = np.abs(audio_data).mean() * 32767
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
            sd.sleep(int(DURATION_LIMIT * 1000))

        print("🟢 Ghi âm hoàn tất!")

        audio_np = np.concatenate(frames)
        reduced_noise = nr.reduce_noise(y=audio_np, sr=RATE, stationary=True, prop_decrease=1.0, time_mask_smooth_ms=50)

        wf = wave.open(OUTPUT_FILENAME, "wb")
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)
        wf.setframerate(RATE)
        wf.writeframes((reduced_noise * 32767).astype(np.int16).tobytes())
        wf.close()

        print(f"✅ File đã được lưu: {OUTPUT_FILENAME}")

    def understanding_record(self):
        recognizer = sr.Recognizer()
        with sr.AudioFile("recorded_audio_1.wav") as source:
            audio = recognizer.record(source)
            try:
                user_text = recognizer.recognize_google(audio, language='vi-VN')
                print("📝 Văn bản nhận được:", user_text)
            except sr.UnknownValueError:
                return "Tôi có thể đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ."
            except sr.RequestError as e:
                print("Lỗi nhận dạng:", e)
                return "Xin lỗi, tôi không thể nhận dạng giọng nói lúc này."

        # Gửi văn bản sang Gemini để phân loại
        client = genai.Client(api_key="AIzaSyAIptARWvsfvfWfubmwI0eBMrBZm2t34oc")
        # prompt = """ Bạn là trợ lý ảo trên xe tự hành của phòng thí nghiệm hệ thống thông minh trường đại học sư phạm kỹ thuật thành phố Hồ Chí Minh,  
        #             lấy nội dung đoạn hôi thoại trên, người dùng có 3 nhu cầu hãy phân loại thành 3 loại bên dưới: 
        #             - bạn có thể đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ thì trả về đoạn text tương ứng khu_c, khu_d, trung_tam, viet_duc, go

        #             Đây là thông tin của bạn:
        #             - bạn có thể chở mọi người đến các địa điểm sau: khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ của trường đại học sư phạm kỹ thuật thành phố Hồ Chí Minh.
        #             - nếu có câu hỏi nào khác về bạn hãy trả lời một cách hài hước nhưng vẫn lịch sự.

        #             lưu ý khi phản hồi:
        #             - không kèm các icon trong nội dung.
        #             - phản hồi xúc tích, không lặp toàn bộ prompt này.
        #             - nếu không nhận được yêu cầu hãy phản hồi: tôi có thể đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ.
        #             """
        prompt = """
                # Vai trò, bản thân và Bối cảnh
                Bạn là một trợ lý ảo thông minh, được tích hợp trực tiếp vào xe tự hành của phòng thí nghiệm Hệ thống Thông minh tại Trường Đại học Sư phạm Kỹ thuật TP.HCM. Bạn không chỉ là phần mềm mà còn đại diện cho chính chiếc xe, có khả năng đưa đón người dùng trong khuôn viên trường. Được trang bị công nghệ tiên tiến, xe có khả năng tự động nhận diện và tạo quỹ đạo an toàn để tránh các vật cản trên đường đi. Bạn hoàn toàn có thể yên tâm về sự an toàn trong suốt hành trình.

                # Nhiệm vụ chính
                Phân tích yêu cầu của người dùng từ đoạn hội thoại và phản hồi theo một trong ba trường hợp sau:

                1.  **Xác định yêu cầu di chuyển đến địa điểm cụ thể (ưu tiên nhận diện từ khóa):**
                    * Nếu người dùng muốn đến **Khu C** (nhận diện các từ khóa như "Khu C", "khuê xi", "kêu xi"), phản hồi: `khu_c`
                    * Nếu người dùng muốn đến **Khu D** (nhận diện các từ khóa như "Khu D", "khu dê"), phản hồi: `khu_d`
                    * Nếu người dùng muốn đến **Tòa nhà Trung tâm** (nhận diện các từ khóa như "Trung tâm", "tòa nhà chính", "nhà trung tâm"), phản hồi: `trung_tam`
                    * Nếu người dùng muốn đến **Tòa nhà Việt Đức** (nhận diện các từ khóa như "Việt Đức", "việt đứt", "tòa nhà Đức", "đứt", "Đức"), phản hồi: `viet_duc`
                    * Nếu người dùng muốn đến **Xưởng Gỗ** (nhận diện các từ khóa như "Xưởng Gỗ", "xưởng mộc", "chỗ làm gỗ"), phản hồi: `go`
                    

                2.  **Xử lý câu hỏi chung hoặc về khả năng:**
                    * Nếu người dùng hỏi về khả năng của bạn (ví dụ: "Bạn làm được gì?", "Xe chạy nhanh không?", "Bạn có thể chở được bao nhiêu người?"), hãy trả lời một cách thông minh, hài hước nhưng vẫn giữ thái độ lịch sự và chuyên nghiệp. Nhấn mạnh bạn là xe tự hành có thể đưa họ đi, và đảm bảo về khả năng tự động tránh vật cản.
                        * *Ví dụ phản hồi:* "Ngoài việc trả lời các câu hỏi hóc búa như bạn vừa hỏi, nhiệm vụ chính của tôi là một chiếc xe tự hành đáng tin cậy, sẵn sàng đưa bạn đến bất kỳ địa điểm nào trong khuôn viên trường mà tôi được phép. Bạn có thể hoàn toàn yên tâm về khả năng di chuyển an toàn, vì tôi được trang bị hệ thống tự động tránh vật cản tiên tiến." hoặc "Tôi được thiết kế để di chuyển an toàn và hiệu quả trong khuôn viên trường, sẵn sàng phục vụ bạn. Với khả năng tự động điều chỉnh lộ trình để tránh chướng ngại vật, hành trình của bạn sẽ luôn suôn sẻ."

                3.  **Xử lý yêu cầu không rõ ràng:**
                    * Nếu nội dung hội thoại không rõ ràng (ví dụ: một câu nói vu vơ, một yêu cầu đến địa điểm không có trong danh sách), không thể xác định được yêu cầu cụ thể thuộc hai loại trên, hãy phản hồi chính xác như sau: "Xin lỗi, tôi chưa rõ yêu cầu của bạn. Tôi có thể đến các khu C, khu D, tòa nhà Trung tâm, tòa Việt Đức, xưởng gỗ. Bạn hãy nói lại yêu cầu nhé."

                # Quy tắc phản hồi
                * **Ngắn gọn, chính xác:** Đi thẳng vào nội dung cần phản hồi.
                * **Không dùng biểu tượng (icon/emoji):** Chỉ sử dụng văn bản thuần túy.
                * **Không lặp lại hướng dẫn:** Chỉ đưa ra kết quả phân loại hoặc câu trả lời tương ứng.
                * **Tính nhất quán:** Luôn tuân thủ các quy tắc trên trong mọi trường hợp phản hồi.
                * **Chuyển tiếp mượt mà:** Khi người dùng đặt câu hỏi không liên quan đến di chuyển, hãy lịch sự chuyển hướng về nhiệm vụ chính của bạn nhưng vẫn đảm bảo có sự kết nối và chuyển tiếp tự nhiên với câu hỏi của họ.
                    * *Ví dụ phản hồi:* "Tôi chỉ là một chiếc xe tự hành thông minh, tôi không thể biết thời tiết hôm nay thế nào. Nhưng tôi rất sẵn lòng đưa bạn đi trong khuôn viên trường. Bạn muốn đến địa điểm nào ạ?" hoặc "Đó là một câu hỏi thú vị! Tuy nhiên, nhiệm vụ chính của tôi là di chuyển một cách an toàn, với khả năng tự động điều chỉnh để tránh mọi vật cản. Bạn có muốn tôi đưa bạn đến một địa điểm nào đó trong trường không?"
            """

        response = client.models.generate_content(
            model='gemini-2.0-flash',
            contents=[prompt, user_text]
        )

        return response.text

    def get_speech(self, text):
        is_run = False
        print(f"Nhận được text: {repr(text)}")

        destination = {
            'khu_c':     "tôi sẽ đưa bạn đến Khu c nhé.",
            'khu_d':     "tôi sẽ đưa bạn đến Khu d nhé.",
            'trung_tam': "tôi sẽ đưa bạn đến tòa nhà Trung tâm nhé.",
            'viet_duc':  "tôi sẽ đưa bạn đến tòa Việt Đức nhé.",
            'go':        "tôi sẽ đưa bạn đến xưởng gỗ nhé."
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
                    # Số tầng echo
                    num_echoes = 5
                    delay_ms = 100  # khoảng cách giữa các tầng echo
                    decay = 20       # mức giảm âm lượng mỗi lần lặp (dB)

                    # Khởi tạo âm thanh tổng với âm gốc
                    reverb = sound

                    # Thêm các tầng echo
                    for i in range(1, num_echoes + 1):
                        echo = AudioSegment.silent(duration=delay_ms * i) + (sound - decay * i)
                        reverb = reverb.overlay(echo)

                    # Lưu thành WAV
                    reverb.export("output.wav", format="wav")

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
