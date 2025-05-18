import asyncio
import edge_tts
import simpleaudio as sa
from pydub import AudioSegment

reply = "Tôi sẽ đưa bạn đến khu Đê nhé"

# TTS với edge-tts → lưu thành MP3
async def speak(text):
    communicate = edge_tts.Communicate(text, voice="vi-VN-HoaiMyNeural")
    await communicate.save("reply.mp3")

asyncio.run(speak(reply))

# Chuyển MP3 → WAV
sound = AudioSegment.from_file("reply.mp3", format="mp3")
sound.export("reply.wav", format="wav")

# Phát âm thanh WAV với simpleaudio
wave_obj = sa.WaveObject.from_wave_file("reply.wav")
play_obj = wave_obj.play()
play_obj.wait_done()
