#!/usr/bin/env python3
"""Export report figures from actual RViz captures and observed Nav2 results."""
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT/'reports/part4'

def font(size):
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',size)

def scene_crop(source):
    # Opening RViz's left panels shifts the map center by about 175 pixels.
    left=710 if sum(source.getpixel((100,100)))>600 else 535
    return source.crop((left,150,left+900,1050))

def main():
    data=json.loads((REPORT/'navigation_results.json').read_text())
    goals=[goal for goal in data['goals'] if goal['status']==4]
    interrupted=[goal for goal in data['goals'] if goal['status']!=4]
    if len(goals)!=3 or not data['passed']:
        raise RuntimeError('Need three verified successful goals before exporting a success report')
    selected=[]
    for goal in goals:
        midpoint=goal['started_wall_s']+.3*(goal['finished_wall_s']-goal['started_wall_s'])
        available=[frame for frame in data['frames']
                   if goal['started_wall_s']<frame['wall_s']<goal['finished_wall_s']]
        frame=min(available,key=lambda item:abs(item['wall_s']-midpoint))
        selected.append(frame)
    canvas=Image.new('RGB',(1860,850),'white');draw=ImageDraw.Draw(canvas)
    draw.text((30,18),'TurtleBot 4 — Nav2 trong phòng vuông (đã bỏ dock)',font=font(30),fill='#152334')
    for index,(goal,frame) in enumerate(zip(goals,selected)):
        source=Image.open(REPORT/'frames'/frame['file'])
        source.save(REPORT/f'navigation_goal_{index+1}_rviz.png')
        scene=scene_crop(source).resize((600,600),Image.LANCZOS)
        canvas.paste(scene,(20+620*index,125))
        x,y=goal['target_map_xy']
        draw.text((25+620*index,76),f'Goal {index+1}: ({x:.2f}, {y:.2f}) m',font=font(23),fill='#152334')
        draw.text((25+620*index,741),f'SUCCEEDED · sai số {goal["position_error_m"]:.2f} m',font=font(21),fill='#166534')
    draw.text((30,798),'Đỏ: AMCL/LiDAR   |   Tím–hồng: inflation   |   Xanh lá: kế hoạch   |   Cam: đường đã đi',font=font(22),fill='#152334')
    canvas.save(REPORT/'navigation_comparison.png')
    Image.open(REPORT/'frames'/selected[2]['file']).save(REPORT/'navigation_inflation_path.png')
    frames=[frame for frame in data['frames']
            if goals[0]['started_wall_s']-2 <=frame['wall_s']<=goals[-1]['finished_wall_s']+4]
    animation=[]
    for frame in frames:
        scene=scene_crop(Image.open(REPORT/'frames'/frame['file'])).resize((720,720),Image.LANCZOS)
        overlay=ImageDraw.Draw(scene)
        overlay.rectangle((0,0,720,32),fill='white')
        overlay.text((10,5),f'RViz thật · t={frame["wall_s"]-goals[0]["started_wall_s"]:.0f}s · tua nhanh 4×',font=font(18),fill='black')
        animation.append(scene)
    animation[0].save(REPORT/'navigation_continuous.gif',save_all=True,append_images=animation[1:],duration=500,loop=0)
    rows='\n'.join(f'| {g["number"]} | ({g["target_map_xy"][0]:.2f}, {g["target_map_xy"][1]:.2f}) | SUCCEEDED | {g["position_error_m"]:.3f} | {g["plan_messages"]} | {g["bump_messages"]} |' for g in goals)
    (REPORT/'bao_cao_navigation.md').write_text(f'''# Phần 4 — Navigation với Nav2

Sử dụng bản đồ SLAM `maps/map.yaml`, AMCL và Nav2 chính thức của TurtleBot 4.
Dock đã được xóa khỏi Gazebo sau undock; tường và ba vật cản giữ nguyên.
Ba goal đạt SUCCEEDED được đặt bằng công cụ Nav2 Goal trên RViz2.
Có {len(interrupted)} goal bị ngắt/không đạt trong cùng lượt ghi; các goal đó
không được tính vào ba kết quả thành công và vẫn giữ trong JSON gốc.
Không gửi lệnh vận tốc thủ công.

| Goal | Đích trong frame map (m) | Kết quả | Sai số cuối (m) | Bản tin plan | Bumper |
| --- | --- | --- | --- | --- | --- |
{rows}

Cảm biến hazard có {data['hazard_messages']} bản tin, không báo bumper trong
lượt kiểm tra này. Khoảng hở nhỏ nhất ước tính từ pose Gazebo đến tường/vật
cản là {data['minimum_clearance_m']:.3f} m, dùng bán kính robot xấp xỉ 0,175 m.
Đây là phép kiểm tra bổ sung, không thay thế mô hình collision đầy đủ.

Ảnh chụp trực tiếp cửa sổ RViz, chỉ cắt/phóng và thêm chú thích:
`navigation_comparison.png` thể hiện ba thời điểm đang di chuyển;
`navigation_inflation_path.png` có AMCL, inflation và đường đi;
`navigation_continuous.gif` là chuỗi ảnh mỗi 2 giây, phát nhanh 4 lần.
Đường cam được ghi từ TF map/base_link; xanh lá là Global Plan.
Hạt AMCL và điểm LiDAR cùng màu đỏ. Kết quả gốc nằm trong
`navigation_results.json`, đường đã đi trong `travelled_path.json`.

Lượt thử trước khi bỏ dock gặp va chạm và không được tính là thành công;
bằng chứng riêng được giữ trong `attempt_dock_contact/`.
''')
    print('Exported actual RViz figures, animation and report.')

if __name__=='__main__':main()
