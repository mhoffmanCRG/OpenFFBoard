import cv2, numpy as np, json
from classify import classify
out={}
for f in ['top.jpg','bottom.jpg']:
    c=classify(f)
    H,W=c.shape
    blk=(c==4).astype(np.uint8)
    n,lab,st,cen=cv2.connectedComponentsWithStats(blk,8)
    holes=[]
    for i in range(1,n):
        x,y,w,h,a=st[i]
        if x==0 or y==0 or x+w>=W or y+h>=H: continue
        if a<15: continue
        if abs(w-h)>max(4,0.25*w): continue
        fill=a/(w*h)
        if fill<0.6: continue
        d=2*np.sqrt(a/np.pi)
        holes.append([float(cen[i][0]),float(cen[i][1]),float(d)])
    # gold components (pads); SMD = gold blob without black inside
    gold=(c==3).astype(np.uint8)
    gold=cv2.morphologyEx(gold,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    n2,lab2,st2,cen2=cv2.connectedComponentsWithStats(gold,8)
    pads=[]
    for i in range(1,n2):
        x,y,w,h,a=st2[i]
        if a<100: continue
        sub=blk[y:y+h,x:x+w]
        pads.append(dict(x=float(x),y=float(y),w=int(w),h=int(h),a=int(a),cx=float(cen2[i][0]),cy=float(cen2[i][1]),hole=bool(sub.sum()>20)))
    out[f]=dict(holes=holes,pads=pads)
    ds=sorted(round(h[2]) for h in holes)
    print(f,len(holes),'holes; diam hist',{d:ds.count(d) for d in sorted(set(ds))})
    print(' pads',len(pads),'smd',sum(1 for p in pads if not p['hole']))
json.dump(out,open('holes.json','w'))
