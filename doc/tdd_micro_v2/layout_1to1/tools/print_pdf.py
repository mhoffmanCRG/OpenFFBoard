import cv2, numpy as np, fp, sys
from PIL import Image, ImageDraw, ImageFont
DPI=600; PXMM=DPI/25.4; S=fp.S; O=fp.O
A4=(int(round(210*PXMM)),int(round(297*PXMM)))
page=Image.new('RGB',A4,'white'); d=ImageDraw.Draw(page)
try: font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',int(3.2*PXMM)); small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',int(2.4*PXMM))
except Exception: font=small=ImageFont.load_default()
def board_img(L):
    im=cv2.imread(f'render_{L}.png')
    x0,y0=int(round(O)),int(round(O)); x1=int(round(fp.BOARD_W*S+O)); y1=int(round(fp.BOARD_H*S+O))
    im=im[y0:y1,x0:x1]
    if L=='B': im=im[:,::-1]          # bottom printed as seen from below
    w=int(round(fp.BOARD_W*PXMM)); h=int(round(fp.BOARD_H*PXMM))
    im=cv2.resize(im,(w,h),interpolation=cv2.INTER_AREA)
    return Image.fromarray(im[:,:,::-1])
mm=lambda v:int(round(v*PXMM))
d.text((mm(15),mm(10)),'TDD Micro AASD-SERVO MMOS FFB v2.0 - 1:1 layout (reconstructed)',fill='black',font=font)
d.text((mm(15),mm(15)),'Print at 100% / "Actual size" (no fit-to-page). Check: scale bar = 50.0 mm, header pitch = 2.54 mm.',fill='black',font=small)
for L,y,lab in [('F',22,'TOP side (component side), viewed from top'),('B',22+fp.BOARD_H+16,'BOTTOM side, viewed from below (lay the board bottom-side down... flipped)')]:
    x=(210-fp.BOARD_W)/2
    page.paste(board_img(L),(mm(x),mm(y+5)))
    d.text((mm(x),mm(y)),lab.replace('bottom-side down... flipped','upside down on it'),fill='black',font=small)
    d.rectangle([mm(x),mm(y+5),mm(x+fp.BOARD_W),mm(y+5+fp.BOARD_H)],outline='black',width=2)
    d.text((mm(x+fp.BOARD_W+2),mm(y+5)),f'{fp.BOARD_W:.2f} x {fp.BOARD_H:.2f} mm',fill='black',font=small)
# scale bar
yb=285; xb=30
d.rectangle([mm(xb),mm(yb),mm(xb+50),mm(yb+1.5)],fill='black')
for k in range(0,51,10): d.line([mm(xb+k),mm(yb-1.5),mm(xb+k),mm(yb+3)],fill='black',width=3)
d.text((mm(xb+52),mm(yb-1.2)),'50 mm',fill='black',font=small)
d.text((mm(110),mm(yb-1.2)),'2026-11-19  unverified reverse-engineering',fill='gray',font=small)
page.save(sys.argv[1],resolution=DPI)
print('saved',sys.argv[1],page.size)
