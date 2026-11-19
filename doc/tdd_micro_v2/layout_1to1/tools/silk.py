import cv2, numpy as np
import fp
S=fp.S;O=fp.O
def keyhole(outer,holes):
    # merge holes into outer polygon with zero-width cuts
    poly=[tuple(p) for p in outer]
    for h in holes:
        h=[tuple(p) for p in h]
        best=None
        for i,p in enumerate(poly):
            for j,q in enumerate(h[::max(1,len(h)//40)]):
                d=(p[0]-q[0])**2+(p[1]-q[1])**2
                if best is None or d<best[0]: best=(d,i,j*max(1,len(h)//40))
        _,i,j=best
        hh=h[j:]+h[:j]+[h[j]]
        poly=poly[:i+1]+hh+[poly[i]]+poly[i+1:]
    return poly
def silk_polys(L):
    from copper import classify_img
    img=cv2.imread('top.jpg' if L=='F' else 'bottom.jpg')
    if L=='B': img=img[:,::-1].copy()
    c=classify_img(img)
    m=(c==5).astype(np.uint8)
    # include anti-aliased pixels adjacent to silk
    unk=(c==0).astype(np.uint8)
    m=m|(unk&cv2.dilate(m,np.ones((3,3),np.uint8)))
    m[:10,:]=0;m[-10:,:]=0;m[:,:10]=0;m[:,-10:]=0
    m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))
    cnts,hier=cv2.findContours(m,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
    out=[]
    if hier is None: return out
    hier=hier[0]
    for i,cn in enumerate(cnts):
        if hier[i][3]!=-1: continue
        if cv2.contourArea(cn)<12: continue
        outer=cv2.approxPolyDP(cn,0.7,True).reshape(-1,2)
        holes=[]
        ch=hier[i][2]
        while ch!=-1:
            if cv2.contourArea(cnts[ch])>=6:
                holes.append(cv2.approxPolyDP(cnts[ch],0.7,True).reshape(-1,2))
            ch=hier[ch][0]
        if len(outer)<3: continue
        poly=keyhole(outer.tolist(),[h.tolist() for h in holes if len(h)>=3])
        out.append([((x-O)/S,(y-O)/S) for x,y in poly])
    return out
if __name__=='__main__':
    for L in 'FB':
        p=silk_polys(L); print(L,len(p),'polys',sum(len(q) for q in p),'pts')
