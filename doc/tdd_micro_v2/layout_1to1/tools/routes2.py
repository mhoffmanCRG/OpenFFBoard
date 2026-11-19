import numpy as np, cv2, json, math
from thin import zhang_suen
from skel2poly import trace, simplify
import fp
S=fp.S; O=fp.O; H,W=2383,2336
px=lambda v: v*S+O
hp=json.load(open('holes.json'))
VIAS=[((x-O)/S,(y-O)/S) for x,y,d in hp['top.jpg']['holes'] if d<20]
def pad_shapes(layer):
    out=[]
    for f,p in fp.all_pads():
        if p['type']=='smd' and p['layer']!=layer: continue
        out.append((f,p))
    return out
def pad_mask(layer,grow=2):
    m=np.zeros((H,W),np.uint8)
    for f,p in pad_shapes(layer):
        x,y=px(p['x']),px(p['y'])
        if p['shape']=='circle':
            cv2.circle(m,(int(round(x)),int(round(y))),int(p['sx']*S/2+grow),1,-1)
        else:
            hx=p['sx']*S/2+grow; hy=p['sy']*S/2+grow
            cv2.rectangle(m,(int(x-hx),int(y-hy)),(int(x+hx),int(y+hy)),1,-1)
    for vx,vy in VIAS:
        cv2.circle(m,(int(round(px(vx))),int(round(px(vy)))),int(0.35*S+grow),1,-1)
    return m
def destair(sk):
    sk=sk.copy(); ys,xs=np.nonzero(sk)
    for y,x in zip(ys,xs):
        if y<1 or x<1 or y>=H-1 or x>=W-1: continue
        n=sk[y-1,x];s=sk[y+1,x];e=sk[y,x+1];w=sk[y,x-1]
        nw=sk[y-1,x-1];ne=sk[y-1,x+1];sw=sk[y+1,x-1];se=sk[y+1,x+1]
        if (n and e and not s and not w and not sw) or (e and s and not n and not w and not nw) or \
           (s and w and not n and not e and not ne) or (w and n and not s and not e and not se):
            sk[y,x]=0
    return sk
_CLS={}
def cls_map(L):
    if L not in _CLS:
        from copper import classify_img
        img=cv2.imread('top.jpg' if L=='F' else 'bottom.jpg')
        if L=='B': img=img[:,::-1].copy()
        _CLS[L]=classify_img(img)
    return _CLS[L]
def clear_path(L,A,B,maxdark=3):
    c=cls_map(L)
    ax,ay=px(A[0]),px(A[1]); bx,by=px(B[0]),px(B[1])
    n=int(max(abs(bx-ax),abs(by-ay)))+1
    dark=0
    for k in range(n+1):
        t=k/max(n,1); x=int(round(ax+t*(bx-ax))); y=int(round(ay+t*(by-ay)))
        if c[y,x]==1: dark+=1
    return dark<=maxdark
def plen(p): return sum(math.hypot(p[i][0]-p[i-1][0],p[i][1]-p[i-1][1]) for i in range(1,len(p)))
def layer_routes(L):
    cu=np.load(f'cu_{L}.npy')
    n,lab,st,_=cv2.connectedComponentsWithStats(cu,4)
    areas=st[:,4].copy(); areas[0]=0; pour=int(np.argmax(areas))
    trk=((cu>0)&(lab!=pour)).astype(np.uint8)
    pm=pad_mask(L,2)
    t=trk&(1-pm)
    t=cv2.morphologyEx(t,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    dt=cv2.distanceTransform(t,cv2.DIST_L2,5)
    n2,cl,st2,_=cv2.connectedComponentsWithStats(t,8)
    pmd=cv2.dilate(pm,np.ones((7,7),np.uint8))
    keep=np.zeros(n2,bool)
    for i in range(1,n2):
        x,y,w,h,a=st2[i]
        if a<25: continue
        m=cl[y:y+h,x:x+w]==i
        mw=2*dt[y:y+h,x:x+w][m].max()/S
        touch=bool((pmd[y:y+h,x:x+w][m]>0).any())
        length=max(w,h)/S
        if mw>1.0 and not touch: continue
        if mw>1.35: continue
        if not touch and length<0.8: continue
        keep[i]=True
    m=(keep[cl]&(t>0)).astype(np.uint8)
    sk=destair(zhang_suen(m))
    paths=trace(sk)
    from collections import Counter
    for it in range(6):
        ends=Counter([p[0] for p in paths]+[p[-1] for p in paths])
        new=[]
        for p in paths:
            a,b=p[0],p[-1]
            fa=ends[a]==1 and not pmd[a]; fb=ends[b]==1 and not pmd[b]
            ja=ends[a]>=2; jb=ends[b]>=2
            l=plen(p)
            if l<15 and ((fa and jb) or (fb and ja)): continue
            if l<20 and fa and fb: continue
            new.append(p)
        if len(new)==len(paths): break
        paths=new
    polys=[]
    for p in paths:
        pts=[(float(x),float(y)) for x,y in simplify(p,1.8)]
        ws=[2*dt[y,x]+1.6 for y,x in p]
        w=float(np.median(ws))/S
        polys.append(dict(pts=[((x-O)/S,(y-O)/S) for x,y in pts],w=w))
    return polys, pm
# ---------- post-processing in mm ----------
def pad_contains(p,x,y,grow=0.0):
    if p['shape']=='circle': return math.hypot(x-p['x'],y-p['y'])<=p['sx']/2+grow
    return abs(x-p['x'])<=p['sx']/2+grow and abs(y-p['y'])<=p['sy']/2+grow
def postprocess(polys,L):
    pads=pad_shapes(L)
    targets=[(f['ref'],p['num'],p['x'],p['y'],p) for f,p in pads]
    # snap endpoints: to pads, then vias
    def snap(pt):
        x,y=pt
        best=None
        for ref,num,cx,cy,p in targets:
            if pad_contains(p,x,y,0.25):
                d=math.hypot(x-cx,y-cy)
                if best is None or d<best[0]: best=(d,(cx,cy),('pad',ref,num))
        if best: return best[1],best[2]
        for vx,vy in VIAS:
            if math.hypot(x-vx,y-vy)<0.6: return (vx,vy),('via',)
        return None,None
    segs=[]
    for pl in polys:
        pts=[list(p) for p in pl['pts']]
        if len(pts)<2: continue
        ends=[]
        for idx in (0,-1):
            tgt,info=snap(pts[idx])
            if tgt is not None:
                # SMD/THT pad: end the track at the pad centre
                if idx==0: pts.insert(0,list(tgt))
                else: pts.append(list(tgt))
            ends.append(info)
        pl['pts']=pts; pl['ends']=ends
    return polys
def direction(pts,end):
    if end==0: a,b=pts[1],pts[0]
    else: a,b=pts[-2],pts[-1]
    # use a longer baseline if available
    dx,dy=b[0]-a[0],b[1]-a[1]; l=math.hypot(dx,dy) or 1
    return dx/l,dy/l
def bridge(polys,L,maxgap=3.0):
    free=[]
    for i,pl in enumerate(polys):
        for k,e in enumerate((0,-1)):
            if pl['ends'][k] is None: free.append((i,e))
    # count connections by coordinate to detect junctions
    allends={}
    for i,pl in enumerate(polys):
        for e in (0,-1):
            key=(round(pl['pts'][e][0],2),round(pl['pts'][e][1],2)); allends[key]=allends.get(key,0)+1
    free=[(i,e) for i,e in free if allends[(round(polys[i]['pts'][e][0],2),round(polys[i]['pts'][e][1],2))]==1]
    used=set(); added=[]
    cands=[]
    for a in range(len(free)):
        i,e=free[a]; P=polys[i]['pts'][e]; d1=direction(polys[i]['pts'],e)
        for b in range(a+1,len(free)):
            j,f=free[b]
            if i==j: continue
            Q=polys[j]['pts'][f]; d2=direction(polys[j]['pts'],f)
            dx,dy=Q[0]-P[0],Q[1]-P[1]; dist=math.hypot(dx,dy)
            if dist>maxgap or dist<1e-6: continue
            ux,uy=dx/dist,dy/dist
            along1=d1[0]*ux+d1[1]*uy; along2=-(d2[0]*ux+d2[1]*uy)
            lat=abs(d1[0]*dy-d1[1]*dx)
            if along1>0.94 and along2>0.94 and lat<0.25 and abs(polys[i]['w']-polys[j]['w'])<0.25 and clear_path(L,P,Q):
                cands.append((dist,a,b))
    cands.sort()
    for dist,a,b in cands:
        if a in used or b in used: continue
        used.add(a); used.add(b)
        i,e=free[a]; j,f=free[b]
        added.append(dict(pts=[list(polys[i]['pts'][e]),list(polys[j]['pts'][f])],w=min(polys[i]['w'],polys[j]['w']),ends=['bridge','bridge']))
    remaining=[free[k] for k in range(len(free)) if k not in used]
    return added,remaining
def extend_to_pads(polys,remaining,L,reach=2.5):
    # free end pointing at a pad/via within reach -> extend
    pads=pad_shapes(L); ext=[]
    left=[]
    for i,e in remaining:
        pts=polys[i]['pts']; P=pts[e]; d=direction(pts,e); best=None
        cands=[(p['x'],p['y'],p['sx']/2) for f,p in pads]+[(vx,vy,0.35) for vx,vy in VIAS]
        for cx,cy,r in cands:
            dx,dy=cx-P[0],cy-P[1]; dist=math.hypot(dx,dy)
            if dist<1e-6 or dist>reach+r: continue
            along=(d[0]*dx+d[1]*dy); lat=abs(d[0]*dy-d[1]*dx)
            if along>0 and lat<r*0.9 and clear_path(L,P,(cx,cy)):
                if best is None or along<best[0]: best=(along,(cx,cy))
        if best:
            ext.append(dict(pts=[list(P),list(best[1])],w=polys[i]['w'],ends=['ext','ext']))
        else: left.append((i,e))
    return ext,left
def extend_to_tracks(polys,remaining,L,reach=1.5):
    # T-junction: free end pointing at a segment of another polyline
    ext=[]; left=[]
    for i,e in remaining:
        pts=polys[i]['pts']; P=pts[e]; d=direction(pts,e); best=None
        for j,pl in enumerate(polys):
            if j==i: continue
            q=pl['pts']
            for k in range(len(q)-1):
                A,B=q[k],q[k+1]
                # ray P + t d intersects segment AB
                ex,ey=B[0]-A[0],B[1]-A[1]
                den=d[0]*ey-d[1]*ex
                if abs(den)<1e-6: continue
                t=((A[0]-P[0])*ey-(A[1]-P[1])*ex)/den
                s=((A[0]-P[0])*d[1]-(A[1]-P[1])*d[0])/den
                if 0<t<reach and -0.02<=s<=1.02:
                    Q=(P[0]+t*d[0],P[1]+t*d[1])
                    if (best is None or t<best[0]) and clear_path(L,P,Q): best=(t,Q)
        if best: ext.append(dict(pts=[list(P),list(best[1])],w=polys[i]['w'],ends=['tee','tee']))
        else: left.append((i,e))
    return ext,left
def run():
    res={}
    for L in 'FB':
        polys,pm=layer_routes(L)
        polys=postprocess(polys,L)
        b,rem=bridge(polys,L); polys+=b
        e1,rem=extend_to_pads(polys,rem,L); polys+=e1
        e2,rem=extend_to_tracks(polys,rem,L); polys+=e2
        # drop tiny dangling fragments (both ends free, short)
        res[L]=dict(polys=polys,free=[polys[i]['pts'][e] for i,e in rem])
        print(L,'polylines',len(polys),'bridges',len(b),'pad-ext',len(e1),'tee',len(e2),'still free ends',len(rem))
    json.dump(res,open('routes_mm.json','w'))
    return res
def overlay(res):
    for L in 'FB':
        img=cv2.imread('top.jpg' if L=='F' else 'bottom.jpg')
        if L=='B': img=img[:,::-1].copy()
        img=(img*0.45).astype(np.uint8)
        for o in res[L]['polys']:
            pts=np.array([[px(x),px(y)] for x,y in o['pts']],np.int32)
            col=(0,0,255) if o['ends'][0] not in ('bridge','ext','tee') else (255,0,255)
            cv2.polylines(img,[pts],False,col,max(2,int(o['w']*S*0.5)))
        for x,y in res[L]['free']:
            cv2.circle(img,(int(px(x)),int(px(y))),7,(0,255,255),2)
        cv2.imwrite(f'ov2_{L}.png',cv2.resize(img,(1400,1428),interpolation=cv2.INTER_AREA))
if __name__=='__main__':
    overlay(run())
