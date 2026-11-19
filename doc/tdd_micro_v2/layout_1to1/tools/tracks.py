import cv2, numpy as np, json
from thin import zhang_suen
S=29.345; O=3.5
H,W=2383,2336
hp=json.load(open('holes.json'))
def holes_px():
    return hp['top.jpg']['holes']
def smd_px():
    out=[]
    for p in hp['top.jpg']['pads']:
        if not p['hole']: out.append(('F',p))
    for p in hp['bottom.jpg']['pads']:
        if not p['hole']:
            q=dict(p); q['cx']=W-1-p['cx']; q['x']=W-p['x']-p['w']; out.append(('B',q))
    return out
def pad_mask(layer):
    m=np.zeros((H,W),np.uint8)
    for x,y,d in holes_px():
        if d<20: r=11
        elif d<27: r=27
        elif d<34: r=25
        else: r=int(d*0.85)
        cv2.circle(m,(int(round(x)),int(round(y))),r,1,-1)
    for L,p in smd_px():
        if L==layer:
            cv2.rectangle(m,(int(p['x'])-1,int(p['y'])-1),(int(p['x']+p['w'])+1,int(p['y']+p['h'])+1),1,-1)
    return m
def extract(layer):
    cu=np.load(f'cu_{layer}.npy')
    n,lab,st,cen=cv2.connectedComponentsWithStats(cu,4)
    pour=np.argmax(np.where(np.arange(n)==0,0,st[:,4]))
    trk=((cu>0)&(lab!=pour)).astype(np.uint8)
    pm=pad_mask(layer)
    trk_np=trk&(1-pm)
    trk_np=cv2.morphologyEx(trk_np,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    dt=cv2.distanceTransform(trk_np,cv2.DIST_L2,5)
    return cu,lab,pour,trk,trk_np,dt,pm
if __name__=='__main__':
    for L in 'FB':
        cu,lab,pour,trk,trk_np,dt,pm=extract(L)
        n,l2,st,_=cv2.connectedComponentsWithStats(trk_np,8)
        widths=[]
        for i in range(1,n):
            if st[i,4]<30: continue
            m=l2==i
            widths.append((round(2*dt[m].max()/S,2),int(st[i,4]),int(st[i,0]),int(st[i,1])))
        widths.sort(reverse=True)
        print(L,len(widths),'fattest',widths[:12])
        sk=zhang_suen(trk_np)
        np.save(f'sk_{L}.npy',sk); np.save(f'trknp_{L}.npy',trk_np); np.save(f'dt_{L}.npy',dt)
        wd=2*dt[sk>0]/S
        print(' skeleton width hist', np.histogram(wd,bins=[0,0.15,0.25,0.3,0.35,0.45,0.6,0.8,1.2,3])[0].tolist())
