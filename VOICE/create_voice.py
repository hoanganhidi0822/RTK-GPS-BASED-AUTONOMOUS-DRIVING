import os
import asyncio
import edge_tts

# Từ điển phản hồi
destination = {
    'hieugiang':     "Chào thầy Hiếu Giang! Rất hân hạnh được phục vụ thầy hôm nay. Xe đã sẵn sàng để đưa thầy đến điểm đến mong muốn trong khuôn viên trường. Thầy cần tôi hỗ trợ gì ạ?",
    'dinhthanh':     "Chào thầy Đình Thành! Rất hân hạnh được phục vụ thầy hôm nay. Xe đã sẵn sàng để đưa thầy đến điểm đến mong muốn trong khuôn viên trường. Thầy cần tôi hỗ trợ gì ạ?",
    'thanhhai': "Chào thầy Thanh Hải! Rất hân hạnh được phục vụ thầy hôm nay. Xe đã sẵn sàng để đưa thầy đến điểm đến mong muốn trong khuôn viên trường. Thầy cần tôi hỗ trợ gì ạ?",
    'myha': "Chào thầy Mỹ Hà! Rất hân hạnh được phục vụ thầy hôm nay. Xe đã sẵn sàng để đưa thầy đến điểm đến mong muốn trong khuôn viên trường. Thầy cần tôi hỗ trợ gì ạ?",
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
