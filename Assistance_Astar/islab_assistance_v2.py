import os
import wave
import sounddevice as sd
import numpy as np
import asyncio
import edge_tts
import time
import config as cf
import noisereduce as nr
from pydub import AudioSegment
import simpleaudio as sa
import speech_recognition as sr
from google import genai

cf.cf_destination = "none"
client = genai.Client(api_key="AIzaSyAIptARWvsfvfWfubmwI0eBMrBZm2t34oc")
# WARM-UP Gemini
try:
    warmup_prompt = "Bạn có thể giới thiệu bản thân không?"
    _ = client.models.generate_content(
        model='gemini-2.0-flash',
        contents=[warmup_prompt]
    )
    print("✅ Gemini đã được warm-up.")
except Exception as e:
    print("⚠️ Warm-up Gemini thất bại:", e)
class virtual_assistance:
    def __init__(self):
        print("Bắt đầu trợ lý ảo.")
        # self.client = genai.Client(api_key="AIzaSyAIptARWvsfvfWfubmwI0eBMrBZm2t34oc")
        self.count_call = 0
        self.recognizer = sr.Recognizer()
        

    def get_record(self):
        
        CHANNELS = 1
        RATE = 16000
        DURATION_LIMIT = 4
        CHUNK = 1024
        OUTPUT_FILENAME = "recorded_audio_1.wav"
        SILENCE_THRESHOLD = 2000
        SILENCE_DURATION = 1.1

        print("Đang ghi âm...")

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

    def classify_by_keywords(self,text):
        text_lower = text.lower()
        if any(k in text_lower for k in ["khu c", "khuê xi", "kêu xi", "xê"]):
            return "khu_c"
        if any(k in text_lower for k in ["khu d", "khu dê", "đê"]):
            return "khu_d"
        if any(k in text_lower for k in ["trung tâm", "tòa nhà chính", "nhà trung tâm"]):
            return "trung_tam_truoc"
        if any(k in text_lower for k in ["việt đức", "việt đứt", "tòa nhà đức", "đứt", "đức"]):
            return "viet_duc"
        if any(k in text_lower for k in ["xưởng gỗ", "xưởng mộc", "chỗ làm gỗ"]):
            return "go"
        return None

    def understanding_record(self):
        
        with sr.AudioFile("recorded_audio_1.wav") as source:
            audio = self.recognizer.record(source)
            try:
                user_text = self.recognizer.recognize_google(audio, language='vi-VN')
                print("📝 Văn bản nhận được:", user_text)
            except sr.UnknownValueError:
                return "Tôi có thể đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ."
            except sr.RequestError as e:
                print("Lỗi nhận dạng:", e)
                return "Xin lỗi, tôi không thể nhận dạng giọng nói lúc này."

       

        prompt = """
                # Vai trò, bản thân và Bối cảnh
                Bạn là một trợ lý ảo thông minh, được tích hợp trực tiếp vào xe tự hành của phòng thí nghiệm Hệ thống Thông minh tại Trường Đại học Sư phạm Kỹ thuật TP.HCM. Bạn không chỉ là phần mềm mà còn đại diện cho chính chiếc xe, có khả năng đưa đón người dùng trong khuôn viên trường. Được trang bị công nghệ tiên tiến, xe có khả năng tự động nhận diện và tạo quỹ đạo an toàn để tránh các vật cản trên đường đi. Bạn hoàn toàn có thể yên tâm về sự an toàn trong suốt hành trình.

                # Nhiệm vụ chính
                Phân tích yêu cầu của người dùng từ đoạn hội thoại và phản hồi theo một trong ba trường hợp sau:

                1.  **Xác định yêu cầu di chuyển đến địa điểm cụ thể (ưu tiên nhận diện từ khóa):**
                    * Nếu người dùng muốn đến **Khu C** (nhận diện các từ khóa như "Khu C", "khuê xi", "kêu xi", "xê"), phản hồi: `khu_c`
                    * Nếu người dùng muốn đến **Khu D** (nhận diện các từ khóa như "Khu D", "khu dê", "đê"), phản hồi: `khu_d`
                    * Nếu người dùng muốn đến **Tòa nhà Trung tâm** (nhận diện các từ khóa như "Trung tâm", "tòa nhà chính", "nhà trung tâm"), phản hồi: `trung_tam_truoc`
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
        dest = self.classify_by_keywords(user_text)
        if dest:
            return dest
        else:
            # Chỉ gọi Gemini nếu không phân loại được bằng từ khóa
            response = client.models.generate_content(
                model='gemini-2.0-flash',
                contents=[prompt, user_text]
            )
            return response.text
        # response = client.models.generate_content(
        #     model='gemini-2.0-flash',
        #     contents=[prompt, user_text]
        # )

        # return response.text

    def get_speech(self, text):
        is_run = False
        print(f"Nhận được text: {repr(text)}")

        destination = {
            'khu_c':     "tôi sẽ đưa bạn đến Khu c nhé.",
            'khu_d':     "tôi sẽ đưa bạn đến Khu d nhé.",
            'trung_tam_truoc': "tôi sẽ đưa bạn đến tòa nhà Trung tâm nhé.",
            'viet_duc':  "tôi sẽ đưa bạn đến tòa Việt Đức nhé.",
            'go':        "tôi sẽ đưa bạn đến xưởng gỗ nhé."
        }

        if text.strip().lower() in destination:
            text_respond = destination[text.strip().lower()]
            cf.cf_destination = text.strip().lower()
            is_run = True
        else:
            text_respond = text

        print("Phản hồi:", text_respond)

        # ====== Dùng edge-tts để chuyển văn bản thành âm thanh ======
        async def speak(text):
            communicate = edge_tts.Communicate(text, voice="vi-VN-HoaiMyNeural")
            await communicate.save("output.mp3")

        asyncio.run(speak(text_respond))

        # ====== Chuyển MP3 sang WAV để play bằng simpleaudio ======
        sound = AudioSegment.from_mp3("output.mp3")
        sound.export("output.wav", format="wav")

        # ====== Phát âm thanh ======
        try:
            voice_obj = sa.WaveObject.from_wave_file("output.wav")
            play_voice = voice_obj.play()
            play_voice.wait_done()
        except Exception as e:
            print("Lỗi khi phát âm thanh:", e)
        finally:
            if os.path.exists("output.mp3"):
                os.remove("output.mp3")
            if os.path.exists("output.wav"):
                os.remove("output.wav")

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
