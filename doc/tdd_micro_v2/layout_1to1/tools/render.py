import json, cv2, numpy as np, fp
S=fp.S; O=fp.O; H,W=2383,2336
D=json.load(open('layout_data.json'))
P=lambda x,y:(int(round(x*S+O)),int(round(y*S+O)))
def render(L):
    img=np.zeros((H,W,3),np.uint8); img[:]=(0,110,0)
    for a,b,wd,nm in D['tracks'][L]:
        cv2.line(img,P(*a),P(*b),(0,200,0) if nm else (0,0,255),max(1,int(wd*S)))
    for f,p in fp.all_pads():
        if p['type']=='smd' and p['layer']!=L: continue
        c=P(p['x'],p['y'])
        if p['shape']=='circle': cv2.circle(img,c,int(p['sx']*S/2),(11,164,225),-1)
        else:
            hx,hy=int(p['sx']*S/2),int(p['sy']*S/2)
            if p['shape']=='oval':
                r=min(hx,hy); cv2.rectangle(img,(c[0]-hx+r,c[1]-hy),(c[0]+hx-r,c[1]+hy),(11,164,225),-1)
                cv2.circle(img,(c[0]-hx+r,c[1]),r,(11,164,225),-1); cv2.circle(img,(c[0]+hx-r,c[1]),r,(11,164,225),-1)
            else: cv2.rectangle(img,(c[0]-hx,c[1]-hy),(c[0]+hx,c[1]+hy),(11,164,225),-1)
        if p['drill']: cv2.circle(img,c,int(p['drill']*S/2),(0,0,0),-1)
    for vx,vy,nm in D['vias']:
        cv2.circle(img,P(vx,vy),int(0.35*S),(0,200,0),-1); cv2.circle(img,P(vx,vy),int(0.15*S),(0,0,0),-1)
    for poly in D['silk'][L]:
        cv2.fillPoly(img,[np.array([P(x,y) for x,y in poly],np.int32)],(235,235,235))
    cv2.rectangle(img,P(0,0),P(fp.BOARD_W,fp.BOARD_H),(0,0,0),3)
    return img
for L in 'FB':
    r=render(L); cv2.imwrite(f'render_{L}.png',r)
    orig=cv2.imread('top.jpg' if L=='F' else 'bottom.jpg')
    if L=='B': orig=orig[:,::-1].copy()
    side=np.hstack([cv2.resize(orig,(1000,1020)),cv2.resize(r,(1000,1020))])
    cv2.imwrite(f'compare_{L}.png',side)
