from pydub import AudioSegment

# Đọc file gốc
sound = AudioSegment.from_mp3("/mnt/NewVolume/Documents/Researches/2024_Project/RTK_GPS/Waypoint-Tracking/Pure-pursuit/frenet-optimal-trajectory/Assistance_Astar/Tesla  Seatbelt Warning.mp3")

# Cắt đoạn đầu tiên (ví dụ: 500ms đầu tiên)
short_beep = sound[0:500]

# Lưu lại file mới
short_beep.export("start_beep_short.mp3", format="mp3")

print("✅ Đã crop file thành công.")
