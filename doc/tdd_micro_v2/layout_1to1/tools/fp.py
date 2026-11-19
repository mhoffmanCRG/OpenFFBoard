# Footprint definitions in board millimetres (origin = top-left board corner, y down)
# measured from the renders at 29.345 px/mm
import json, numpy as np
S=29.345; O=3.5; WPX=2336
BOARD_W=79.35; BOARD_H=80.95
L_LEFT=['PE3','PE5','PC13','GND','PC1','PA0','PA2','PA3','PA5','PA7','PC5','PB1','PB9','PE7','PE9','PB2','BOOT','']
R_LEFT=['PE2','PE4','PE6','3V3','PC0','PC2','PA1','PC3','PA4','PA6','PC4','PB0','PB8','PE1','PE8','PE10','BOOT','']
L_RIGHT=['5V','3V3','PB7','PB5','PB3','PD6','PD4','PD2','PD0','PC11','PA15','PA13','PA11','PA9','PC9','5V']
R_RIGHT=['GND','GND','PE0','PB6','PB4','PD7','PD5','PD3','PD1','PC12','PC10','PA14','PA12','PA10','PA8','GND']
T_BOT=['PE12','PE14','PB10','GND','3V3','PB13','PB15','PD9','PD11','PD13','PD15','PC7']
B_BOT=['PE11','PE13','PE15','PB11','PB12','PB14','PD8','PD10','PD12','PD14','PC6','PC8']
def th(num,x,y,size,drill,shape='circle',sx=None,sy=None,name=''):
    return dict(num=str(num),x=x,y=y,type='thru_hole',shape=shape,sx=sx or size,sy=sy or size,drill=drill,layer='F',name=name)
def smd(num,x,y,sx,sy,layer='F',name='',shape='rect'):
    return dict(num=str(num),x=x,y=y,type='smd',shape=shape,sx=sx,sy=sy,drill=0,layer=layer,name=name)
def npth_mount(num,x,y,size,drill):
    return th(num,x,y,size,drill)
FPS=[]
def add(ref,value,fpname,layer,pads,cx=None,cy=None,silk_ref=False,desc=''):
    if cx is None:
        cx=round(float(np.mean([p['x'] for p in pads])),3); cy=round(float(np.mean([p['y'] for p in pads])),3)
    FPS.append(dict(ref=ref,value=value,name=fpname,layer=layer,x=cx,y=cy,pads=pads,desc=desc))
# --- J1: DIY-MORE STM32F407 "black" module sockets (3 female headers) ---
P=2.54; HD=1.8; HDR=0.8
pads=[]
for i in range(18):
    pads.append(th(f'L{2*i+1}',20.775,6.63+P*i,HD,HDR,name=L_LEFT[i] or 'NC'))
    pads.append(th(f'L{2*i+2}',23.315,6.63+P*i,HD,HDR,name=R_LEFT[i] or 'NC'))
for i in range(16):
    pads.append(th(f'R{2*i+1}',58.875,6.63+P*i,HD,HDR,name=L_RIGHT[i]))
    pads.append(th(f'R{2*i+2}',61.415,6.63+P*i,HD,HDR,name=R_RIGHT[i]))
for i in range(12):
    pads.append(th(f'B{2*i+1}',25.85+P*i,52.35,HD,HDR,name=T_BOT[i]))
    pads.append(th(f'B{2*i+2}',25.85+P*i,54.89,HD,HDR,name=B_BOT[i]))
add('J1','DIY-MORE STM32F407VGT6 black module','TDD:DIYMORE_F407_Black_Sockets','F',pads,desc='2x18 + 2x16 + 2x12 female headers for the plug-in MCU module')
# --- J6 SWD/AREF 1x4 ---
add('J6','SWD/AREF','TDD:PinHeader_1x04_P2.54','F',[th(i+1,71.575,11.71+P*i,HD,HDR,name=n) for i,n in enumerate(['GND','AREF','SWDIO','SWCLK'])])
# --- J7 STM32 buttons 2x8 ---
pads=[]
lc=['GND','B8','B9','B10','B11','B12','B13','B14']; rc=['GND','B7','B6','B5','B4','B3','B2','B1']
for i in range(8):
    pads.append(th(2*i+1,71.58,24.41+P*i,HD,HDR,name=lc[i])); pads.append(th(2*i+2,74.12,24.41+P*i,HD,HDR,name=rc[i]))
add('J7','STM32 Buttons 1-14','TDD:PinHeader_2x08_P2.54','F',pads)
# --- J4 G27 shifter DB9 male (right angle, faces left edge) ---
def db9(x5,x4,y1,dirn,mount_y):
    p=[]
    for k in range(5): p.append(th(k+1,x5,y1+dirn*2.77*k,1.75,0.8,shape='rect' if k==0 else 'circle'))
    for k in range(4): p.append(th(k+6,x4,y1+dirn*(1.385+2.77*k),1.75,0.8))
    for j,my in enumerate(mount_y): p.append(th('MH',8.17,my,3.95,3.0))
    return p
sh=db9(9.545,6.87,18.47,+1,[11.29,36.37])
for q,n in zip(sh,['SCK','MISO','CS','X','NC','GND','3V3','Y','3V3']): q['name']=n
add('J4','G27 Shifter (DB9 male, SPI1)','TDD:DB9_Male_RA','F',sh)
pd_=db9(9.57,6.885,61.55,-1,[43.30,68.38])
for q,n in zip(pd_,['3V3','GAS','BRAKE','CLUTCH','NC','GND','NC','NC','NC']): q['name']=n
add('J3','G27 Pedals (DB9 female)','TDD:DB9_Female_RA','F',pd_)
# --- J2 DB25 female to AASD CN2 ---
p=[]
for n in range(1,14): p.append(th(n,57.69-2.77*(n-1),70.12,1.65,1.05,shape='rect' if n==1 else 'circle'))
for n in range(14,26): p.append(th(n,56.305-2.77*(n-14),72.96,1.65,1.05))
p.append(th('MH',17.55,71.54,3.75,3.15)); p.append(th('MH',64.60,71.54,3.75,3.15))
add('J2','DB25 female -> AASD-30A CN2','TDD:DB25_Female_RA','F',p)
# --- J5 Z sensor 1x3 ---
add('J5','Z sensor','TDD:PinHeader_1x03_P2.54','F',[th(i+1,15.32,60.31+P*i,1.75,0.8,name=n) for i,n in enumerate(['GND','3V3','Z'])])
# --- J8 E-STOP 2-pin screw terminal 3.5 mm ---
add('J8','E-STOP terminal (SCR1)','TDD:TerminalBlock_1x02_P3.50','F',[th(1,74.08,66.18,0,1.1,shape='oval',sx=4.0,sy=2.0),th(2,74.08,69.66,0,1.1,shape='oval',sx=4.0,sy=2.0)])
# --- LEDs: combined 3 mm THT + SMD pads ---
for ref,y,col in [('D3',47.97,'Red'),('D2',53.685,'Blue'),('D1',59.40,'Green')]:
    add(ref,f'LED {col}','TDD:LED_3mm_THT_plus_1206','F',
        [th(1,73.71,y,1.5,0.85),th(2,76.25,y,1.5,0.85),smd(1,73.95,y-1.17,1.1,1.55),smd(2,76.02,y-1.17,1.1,1.55)],cx=74.98,cy=y-0.5)
# --- R1-R3, C1-C3 (0805-ish pads, 2.06 mm pitch) ---
for ref,x,y,v in [('R3',69.46,44.895,'470R'),('R2',69.46,47.30,'470R'),('R1',69.46,49.70,'470R'),('C1',63.11,57.47,'100nF'),('C3',61.18,65.93,'100nF'),('C2',65.49,65.93,'100nF')]:
    add(ref,v,'TDD:R_C_0805_HandSolder','F',[smd(1,x-1.03,y,1.1,1.4),smd(2,x+1.03,y,1.1,1.4)],cx=x,cy=y)
# --- U1 LTC2644 MSOP-12 (pin names per silkscreen) ---
left=['VCC','VOUTA','VOUTB','IDLSEL','IOVCC','GND']; right=['PD','INB','INA','REF','REFSEL','GND']
p=[]
for i in range(6): p.append(smd(i+1,60.685,59.96+0.646*i,0.95,0.41,name=left[i]))
for i in range(6): p.append(smd(i+7,65.79,59.96+0.646*(5-i),0.95,0.41,name=right[i]))
add('U1','LTC2644-L12','TDD:MSOP-12_Long_Pads','F',p,cx=63.2375,cy=61.575)
# --- solder jumpers ---
for ref,x,y,lab in [('JP1',57.265,61.675,'A (left)'),('JP2',57.265,63.595,'B (left)'),('JP3',68.32,61.27,'A (right)'),('JP4',68.32,63.19,'B (right)')]:
    add(ref,f'Jumper {lab}','TDD:SolderJumper_2_Open','F',[smd(1,x-0.525,y,0.75,1.16),smd(2,x+0.525,y,0.75,1.16)],cx=x,cy=y)
for ref,x,lab in [('JP5',24.46,'PA5'),('JP6',26.37,'PA4'),('JP7',28.255,'DAC')]:
    add(ref,f'Torque src {lab}','TDD:SolderJumper_2_Open_V','F',[smd(1,x,65.27,1.19,0.75),smd(2,x,66.305,1.19,0.75)],cx=x,cy=65.79)
# shield ground jumper on bottom (mirrored position)
def mirx(xmm_bot):
    return ((WPX-1-(xmm_bot*S+O))-O)/S
x1,x2=sorted([mirx(10.03),mirx(10.92)])
add('JP8','Shield Ground','TDD:SolderJumper_2_Open_Bottom','B',[smd(1,round(x1,3),71.66,0.72,1.16,layer='B'),smd(2,round(x2,3),71.66,0.72,1.16,layer='B')])
# --- mounting holes (plated) ---
for i,(x,y) in enumerate([(4.04,4.44),(75.17,4.44),(4.04,75.56),(75.17,75.56)]):
    add(f'H{i+1}','MountingHole 3.5mm','TDD:MountingHole_3.5mm_Pad','F',[th(1,x,y,6.0,3.5)],cx=x,cy=y)
def all_pads():
    for f in FPS:
        for p in f['pads']:
            yield f,p
if __name__=='__main__':
    # verify against detected holes
    hp=json.load(open('holes.json'))['top.jpg']['holes']
    det=np.array([((x-O)/S,(y-O)/S,d/S) for x,y,d in hp if d>20])
    used=np.zeros(len(det),bool); errs=[]
    for f,p in all_pads():
        if p['type']!='thru_hole': continue
        d=np.hypot(det[:,0]-p['x'],det[:,1]-p['y']); i=d.argmin(); used[i]=True; errs.append((d[i],f['ref'],p['num']))
    errs.sort(reverse=True); print('worst pos errors mm',[(round(e,3),r,n) for e,r,n in errs[:8]])
    print('unmatched detected holes',det[~used].round(2).tolist())
    print('footprints',len(FPS),'pads',sum(len(f['pads']) for f in FPS))
