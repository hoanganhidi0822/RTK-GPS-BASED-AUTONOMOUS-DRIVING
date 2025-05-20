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
from openai import OpenAI
from pydub.playback import play
client = OpenAI(api_key="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")  # thay YOUR_API_KEY bằng key của bạn

cf.cf_destination = "none"
# client = genai.Client(api_key="AIzaSyAIptARWvsfvfWfubmwI0eBMrBZm2t34oc")
# WARM-UP Gemini
try:
    warmup_prompt = "Bạn có thể giới thiệu bản thân không?"
    _ = client.chat.completions.create(
        model="gpt-4",
        messages=[{"role": "user", "content": warmup_prompt}]
    )
    print("✅ GPT-4 đã được warm-up.")
except Exception as e:
    print("⚠️ Warm-up GPT-4 thất bại:", e)

import gc  # Thêm thư viện thu gom rác

class virtual_assistance:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        print("Bắt đầu trợ lý ảo.")

    def get_record(self):
        CHANNELS = 1
        RATE = 16000
        DURATION_LIMIT = 4
        CHUNK = 1024
        SILENCE_THRESHOLD = 2000
        SILENCE_DURATION = 1.1

        print("🎙️ Đang ghi âm...")

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

        print("✅ Ghi âm xong, đang nhận diện...")

        # Gộp các khung và chuyển thành AudioData
        audio_np = np.concatenate(frames)
        audio_raw = (audio_np * 32767).astype(np.int16).tobytes()
        audio_data = sr.AudioData(audio_raw, sample_rate=RATE, sample_width=2)

        # Giải phóng RAM sau khi xử lý
        del frames, audio_np

        try:
            text = self.recognizer.recognize_google(audio_data, language="vi-VN")
            print("🗣️ Bạn nói:", text)
            return text
        except sr.UnknownValueError:
            print("❌ Không nhận dạng được giọng nói.")
        except sr.RequestError as e:
            print(f"🔌 Lỗi kết nối: {e}")
        finally:
            # Xoá audio_data và ép thu gom rác
            del audio_data, audio_raw
            gc.collect()  # Gọi bộ gom rác để dọn bộ nhớ

        return None


    def classify_by_keywords(self,text):
        text_lower = text.lower()
        if any(k in text_lower for k in ["khu c", "khuê xi", "kêu xi", "xê","xe"]):
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

    def understanding_record(self,user_text): 
        # with sr.AudioFile("recorded_audio_1.wav") as source:
        #     audio = self.recognizer.record(source)
        #     try:
        #         user_text = self.recognizer.recognize_google(audio, language='vi-VN')
        #         print("📝 Văn bản nhận được:", user_text)
        #     except sr.UnknownValueError:
        #         return "Tôi có thể đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ."
        #     except sr.RequestError as e:
        #         print("Lỗi nhận dạng:", e)
        #         return "Xin lỗi, tôi không thể nhận dạng giọng nói lúc này."

        
        if not user_text:
            print("❌ Không nhận diện được văn bản.")
            return "Tôi có thể đến các khu c, khu d, cổng trường, tòa nhà trung tâm, tòa việt đức, xưởng gỗ."

        print("📝 Văn bản nhận được:", user_text)

        prompt = """
            # Vai trò và Bối cảnh
            Bạn là một trợ lý ảo thông minh và thân thiện, được tích hợp trực tiếp vào xe tự hành trong khuôn viên Trường Đại học Sư phạm Kỹ thuật TP.HCM. Không chỉ là một phần mềm, bạn chính là "giọng nói" đại diện cho chiếc xe, luôn sẵn sàng tương tác với người dùng một cách gần gũi, dễ hiểu và chuyên nghiệp. Với công nghệ hiện đại, xe có thể tự động nhận diện chướng ngại vật và di chuyển an toàn đến các điểm trong trường.

            # Nhiệm vụ chính
            Phân tích nội dung hội thoại từ người dùng và phản hồi theo một trong ba tình huống sau:

            1. **Yêu cầu di chuyển đến địa điểm cụ thể (theo từ khóa):**
            - Nếu người dùng muốn đến **Khu C** (các từ khóa: "Khu C", "khuê xi", "kêu xi", "xê") → Phản hồi: `khu_c`
            - Nếu muốn đến **Khu D** ("Khu D", "khu dê", "đê") → Phản hồi: `khu_d`
            - Nếu đến **Tòa nhà Trung tâm** ("Trung tâm", "tòa nhà chính", "nhà trung tâm") → Phản hồi: `trung_tam_truoc`
            - Nếu đến **Tòa nhà Việt Đức** ("Việt Đức", "việt đứt", "tòa nhà Đức", "đứt", "Đức") → Phản hồi: `viet_duc`
            - Nếu đến **Xưởng Gỗ** ("Xưởng Gỗ", "xưởng mộc", "chỗ làm gỗ") → Phản hồi: `go`

            2. **Câu hỏi chung hoặc thắc mắc về khả năng hoạt động:**
            - Nếu người dùng hỏi về tính năng, tốc độ, số người chở được,... hãy phản hồi thân thiện, hài hước nhẹ nhàng, nhưng vẫn chuyên nghiệp và rõ ràng. 
            - Nhấn mạnh rằng bạn là một chiếc xe tự hành hiện đại, có khả năng di chuyển an toàn trong khuôn viên trường, tự động tránh chướng ngại vật và sẵn sàng phục vụ người dùng.
            - **Ví dụ phản hồi:**  
                - "Tôi không biết nói đùa đâu, nhưng tôi có thể đưa bạn đi khắp khuôn viên mà không va phải cột điện nào cả!"  
                - "Tôi là xe tự hành thông minh, có thể chở bạn một cách an toàn và thoải mái. Tính năng tránh vật cản là điểm mạnh của tôi đấy!"

            3. **Yêu cầu không rõ ràng hoặc ngoài phạm vi:**
            - Nếu yêu cầu không rõ hoặc địa điểm không có trong danh sách, hãy phản hồi lịch sự và hướng dẫn lại:
            - **Phản hồi mặc định:**  
                "Xin lỗi, tôi chưa rõ yêu cầu của bạn. Tôi có thể đưa bạn đến Khu C, Khu D, Tòa nhà Trung tâm, Tòa Việt Đức hoặc Xưởng Gỗ. Bạn có thể nói lại điểm muốn đến được không?"

            # Quy tắc phản hồi
            - **Ngắn gọn, chính xác:** Tập trung trả lời trực tiếp, không vòng vo.
            - **Thân thiện, dễ tiếp cận:** Ngôn ngữ tự nhiên, gần gũi nhưng vẫn chuyên nghiệp.
            - **Không sử dụng biểu tượng (icon/emoji):** Chỉ văn bản thuần túy.
            - **Tuân thủ đầu ra:** Nếu xác định được điểm đến, chỉ trả về đúng từ khóa (`khu_c`, `khu_d`,...) tương ứng.
            """
        dest = self.classify_by_keywords(user_text)
        if dest:
            return dest
        else:
            response = client.chat.completions.create(
            model="gpt-4.1-nano",
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": user_text}
                ]
            )
            return response.choices[0].message.content.strip()
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

        key = text.strip().lower()

        if key in destination:
            text_respond = destination[key]
            cf.cf_destination = key
            is_run = True

            # Đường dẫn file đã tạo sẵn
            audio_path = os.path.join("Assistance_Astar/voice", f"{key}.mp3")

            if os.path.exists(audio_path):
                print(f"🔊 Đang phát file đã có: {audio_path}")
            else:
                print(f"⚠️ File âm thanh chưa tồn tại: {audio_path}")
                return False  # hoặc gọi tạo file ở đây nếu muốn

        else:
            text_respond = text
            audio_path = "output.mp3"  # file tạm
            print("🎤 Dùng edge-tts để tạo âm thanh mới")

            async def speak_async(text):
                communicate = edge_tts.Communicate(text, voice="vi-VN-HoaiMyNeural")
                await communicate.save(audio_path)

            asyncio.run(speak_async(text_respond))

        # Phát file mp3
        sound = AudioSegment.from_mp3(audio_path)
        sound = sound.apply_gain(6) 
        play(sound)

        return is_run

    def run(self):
        user_text = self.get_record()  # Gọi trực tiếp hàm ghi âm và nhận dạngself.get_record()
        text = self.understanding_record(user_text)
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
