import os
import asyncio
import edge_tts

# Từ điển phản hồi
destination = {
    'khu_c':     "tôi sẽ đưa bạn đến Khu c nhé.",
    'khu_d':     "tôi sẽ đưa bạn đến Khu d nhé.",
    'trung_tam_truoc': "tôi sẽ đưa bạn đến tòa nhà Trung tâm nhé.",
    'viet_duc':  "tôi sẽ đưa bạn đến tòa Việt Đức nhé.",
    'go':        "tôi sẽ đưa bạn đến xưởng gỗ nhé."
}

# Tạo thư mục lưu file
output_dir = "voice"
os.makedirs(output_dir, exist_ok=True)

# Hàm bất đồng bộ để tạo từng file âm thanh
async def create_audio(key, text):
    output_path = os.path.join(output_dir, f"{key}.mp3")
    communicate = edge_tts.Communicate(text, voice="vi-VN-HoaiMyNeural")
    await communicate.save(output_path)
    print(f"✅ Đã tạo: {output_path}")

# Tạo danh sách task bất đồng bộ
async def main():
    tasks = []
    for key, text in destination.items():
        print(f"🔊 Đang tạo file cho '{key}'...")
        tasks.append(create_audio(key, text))
    await asyncio.gather(*tasks)

# Chạy toàn bộ
asyncio.run(main())
