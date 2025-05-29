import openai
import speech_recognition as sr
from pydub import AudioSegment
from pydub.playback import play

from openai import OpenAI
client = OpenAI(api_key="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")  # thay YOUR_API_KEY bằng key của bạn

# Hàm ghi âm và nhận diện tiếng nói tiếng Việt
import sounddevice as sd
import soundfile as sf
import speech_recognition as sr

def listen_vietnamese(duration=3, filename="recorded.wav"):
    print("Đang ghi âm (nói gì đó bằng tiếng Việt)...")
    samplerate = 16000
    audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1)
    sd.wait()
    sf.write(filename, audio, samplerate)
    print("Đã ghi xong.")

    # Nhận diện giọng nói từ file vừa ghi
    recognizer = sr.Recognizer()
    with sr.AudioFile(filename) as source:
        audio_data = recognizer.record(source)

    try:
        text = recognizer.recognize_google(audio_data, language="vi-VN")
        print("Bạn nói:", text)
        return text
    except sr.UnknownValueError:
        print("Không nhận dạng được giọng nói.")
    except sr.RequestError as e:
        print(f"Lỗi kết nối: {e}")
    return None


# Hàm gọi GPT và tạo phản hồi audio tiếng Việt
def chatbot_respond(text):
    # Gửi câu hỏi đến GPT
    response = client.chat.completions.create(
        model="gpt-4.1-nano",
        messages=[{"role": "user", "content": text}]
    )
    robot_brain = response.choices[0].message.content
    print("Trợ lý:", robot_brain)

    # Tạo audio tiếng Việt từ phản hồi
    audio_response = client.audio.speech.create(
        model="tts-1",
        voice="nova",  # Giọng nói, hiện tại chưa hỗ trợ chọn tiếng Việt, nhưng bạn vẫn có thể đọc tiếng Việt
        input=robot_brain
    )

    # Ghi ra file
    with open("voice.mp3", "wb") as f:
        f.write(audio_response.content)

    # Phát âm thanh
    sound = AudioSegment.from_mp3("voice.mp3")
    play(sound)

# Vòng lặp chính
if __name__ == "__main__":
    while True:
        user_input = listen_vietnamese()
        if user_input:
            chatbot_respond(user_input)
