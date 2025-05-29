import os
import asyncio
import edge_tts
from pydub import AudioSegment
# Từ điển phản hồi
# destination = {
#     'vietduc': "Bên cạnh chúng ta là Trung Tâm Việt Đức, một trong những công trình kiến trúc nổi bật tại Đại học Sư phạm Kỹ thuật TP HCM. Tòa nhà là nơi tập trung các phòng học, phòng thí nghiệm hiện đại, phục vụ đắc lực cho công tác giảng dạy và nghiên cứu của nhà trường. Với trang thiết bị tiên tiến, Tòa Việt Đức là một môi trường học tập và làm việc lý tưởng.",
#     'khu_c':"Chào mừng bạn đến với Khu C của Khoa Điện - Điện tử, của trường Đại học Sư phạm Kỹ thuật TP HCM! Nơi đây không chỉ là trung tâm đào tạo và nghiên cứu mà còn là cầu nối vững chắc giữa nhà trường và doanh nghiệp. Khu C tự hào sở hữu các phòng lab hiện đại, được đầu tư và hợp tác phát triển cùng các công ty hàng đầu trong lĩnh vực Điện - Điện tử, mang đến cho sinh viên cơ hội học tập và thực hành sát với thực tế công nghiệp.",
#     'tien_loi':"Bên cạnh bạn là cửa hàng tiện lợi của trường! Trong khuôn viên Đại học Sư phạm Kỹ thuật có cửa hàng tiện lợi phục vụ nhu cầu của sinh viên và giảng viên. Bạn có thể tìm thấy đồ ăn nhanh, nước uống, văn phòng phẩm và các vật dụng cá nhân thiết yếu tại đây.",
#     'toaF':"Bên cạnh bạn là  Khoa Đào tạo Quốc tế, trường Đại học Sư phạm Kỹ thuật TP HCM. Với các chương trình liên kết đào tạo với các trường đại học uy tín trên thế giới, giảng viên giàu kinh nghiệm và cơ hội phát triển toàn diện, Khoa đào tạo quốc tế sẽ là bệ phóng vững chắc cho sự nghiệp toàn cầu của bạn."
# }

destination = {
    'hieugiang':     "Chào thầy Hiếu Giang! Rất hân hạnh được phục vụ thầy hôm nay. Xe có thể đến các địa điểm như Khu C, Khu Đê, tòa nhà trung tâm, tòa Việt Đức và xưởng gỗ. Thầy có thể chọn địa điểm bằng các ấn vào nút micro để ra lệnh bằng giọng nói hoặc chọn trực tiếp trên giao diện. Thầy cần tôi hỗ trợ gì ạ?",
    'dinhthanh':     "Chào thầy Đình Thành! Rất hân hạnh được phục vụ thầy hôm nay. Xe có thể đến các địa điểm như Khu C, Khu Đê, tòa nhà trung tâm, tòa Việt Đức và xưởng gỗ. Thầy có thể chọn địa điểm bằng các ấn vào nút micro để ra lệnh bằng giọng nói hoặc chọn trực tiếp trên giao diện. Thầy cần tôi hỗ trợ gì ạ?",
    'thanhhai': "Chào thầy Thanh Hải! Rất hân hạnh được phục vụ thầy hôm nay. Xe có thể đến các địa điểm như Khu C, Khu Đê, tòa nhà trung tâm, tòa Việt Đức và xưởng gỗ. Thầy có thể chọn địa điểm bằng các ấn vào nút micro để ra lệnh bằng giọng nói hoặc chọn trực tiếp trên giao diện. Thầy cần tôi hỗ trợ gì ạ?",
    'myha': "Chào thầy Mỹ Hà! Rất hân hạnh được phục vụ thầy hôm nay. Xe có thể đến các địa điểm như Khu C, Khu Đê, tòa nhà trung tâm, tòa Việt Đức và xưởng gỗ. Thầy có thể chọn địa điểm bằng các ấn vào nút micro để ra lệnh bằng giọng nói hoặc chọn trực tiếp trên giao diện. Thầy cần tôi hỗ trợ gì ạ?",
}

# Tạo thư mục lưu file
output_dir = "toanha"
os.makedirs(output_dir, exist_ok=True)

# Hàm bất đồng bộ để tạo từng file .mp3 rồi chuyển sang .wav
async def create_audio(key, text):
    mp3_path = os.path.join(output_dir, f"{key}.mp3")
    wav_path = os.path.join(output_dir, f"{key}.wav")

    communicate = edge_tts.Communicate(text, voice="vi-VN-HoaiMyNeural")
    await communicate.save(mp3_path)
    print(f"🎧 Đã tạo MP3: {mp3_path}")

    # Chuyển MP3 sang WAV
    sound = AudioSegment.from_mp3(mp3_path)
    sound.export(wav_path, format="wav")
    print(f"✅ Đã tạo WAV: {wav_path}")

    # Xoá file mp3 nếu không cần
    # os.remove(mp3_path)


# Tạo danh sách task bất đồng bộ
async def main():
    tasks = []
    for key, text in destination.items():
        print(f"🔊 Đang tạo file cho '{key}'...")
        tasks.append(create_audio(key, text))
    await asyncio.gather(*tasks)

# Chạy toàn bộ
asyncio.run(main())