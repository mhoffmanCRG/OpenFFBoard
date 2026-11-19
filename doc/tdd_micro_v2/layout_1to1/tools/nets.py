import json, math, numpy as np, cv2
import fp
from routes2 import VIAS, pad_shapes, pad_contains, px, S, O
R=json.load(open('routes_mm.json'))
GSTAT={}
class UF:
    def __init__(s): s.p={}
    def f(s,a):
        s.p.setdefault(a,a)
        while s.p[a]!=a: s.p[a]=s.p[s.p[a]]; a=s.p[a]
        return a
    def u(s,a,b): s.p[s.f(a)]=s.f(b)
def segdist(P,A,B):
    ax,ay=A;bx,by=B;ex,ey=bx-ax,by-ay;l2=ex*ex+ey*ey
    t=0 if l2==0 else max(0,min(1,((P[0]-ax)*ex+(P[1]-ay)*ey)/l2))
    return math.hypot(P[0]-ax-t*ex,P[1]-ay-t*ey)
def seg_hits_pad(A,B,p,margin=-0.05):
    # sample along segment
    n=max(2,int(math.hypot(B[0]-A[0],B[1]-A[1])/0.05))
    for k in range(1,n):
        t=k/n; x=A[0]+t*(B[0]-A[0]); y=A[1]+t*(B[1]-A[1])
        if pad_contains(p,x,y,margin): return True
    return False
def build(drop_bad_bridges=True):
    uf=UF()
    padkey=lambda f,p:('pad',f['ref'],p['num'])
    # same-number pads in a footprint are the same net (LED THT+SMD, MH)
    for f in fp.FPS:
        seen={}
        for p in f['pads']:
            if p['num']=='MH': continue
            if p['num'] in seen: uf.u(seen[p['num']],padkey(f,p))
            else: seen[p['num']]=padkey(f,p)
            uf.f(padkey(f,p))
    dropped=[]
    for L in 'FB':
        polys=R[L]['polys']; pads=pad_shapes(L)
        good=[]
        for i,pl in enumerate(polys):
            pts=pl['pts']
            if drop_bad_bridges and pl['ends'][0] in ('bridge','ext','tee'):
                # reject if crossing a pad interior that is not at an endpoint
                bad=False
                for f,p in pads:
                    if pad_contains(p,*pts[0],0.05) or pad_contains(p,*pts[-1],0.05): continue
                    if seg_hits_pad(pts[0],pts[-1],p): bad=True;break
                if bad: dropped.append((L,pts)); continue
            good.append(pl)
        R[L]['polys']=polys=good
        for i,pl in enumerate(polys):
            k=('trk',L,i); uf.f(k)
            pts=pl['pts']
            for f,p in pads:
                for e in (pts[0],pts[-1]):
                    if pad_contains(p,e[0],e[1],0.05): uf.u(k,padkey(f,p))
                # track running through a pad (e.g. pad on the path)
            for vi,(vx,vy) in enumerate(VIAS):
                for e in (pts[0],pts[-1]):
                    if math.hypot(e[0]-vx,e[1]-vy)<0.4: uf.u(k,('via',vi))
        for i,pl in enumerate(polys):
            for e in (pl['pts'][0],pl['pts'][-1]):
                for j,q in enumerate(polys):
                    if i==j: continue
                    qq=q['pts']
                    if any(segdist(e,qq[m],qq[m+1])<0.06 for m in range(len(qq)-1)):
                        uf.u(('trk',L,i),('trk',L,j))
    # pour (GND) detection: thermal spokes / direct pour contact in the moat ring
    from copper import classify_img
    gnd=set(); GSTAT.clear()
    for L in 'FB':
        img=cv2.imread('top.jpg' if L=='F' else 'bottom.jpg')
        if L=='B': img=img[:,::-1].copy()
        c=classify_img(img)
        cu=np.load(f'cu_{L}.npy'); n_,lab,st_,_=cv2.connectedComponentsWithStats(cu,4)
        a_=st_[:,4].copy(); a_[0]=0; pour=int(np.argmax(a_))
        def frac(samples):
            # samples: list of (ring_pt, outer_pt1, outer_pt2)
            b=d=0
            for (X,Y),(X2,Y2),(X3,Y3) in samples:
                v=c[Y,X]
                if v==2 and lab[Y2,X2]==pour and lab[Y3,X3]==pour and c[Y2,X2]==2: b+=1
                elif v in (1,2): d+=1
            return b/(b+d) if b+d>8 else None
        for f,p in pad_shapes(L):
            if p['num']=='MH': continue
            cx,cy=px(p['x']),px(p['y'])
            sm=[]
            P=lambda x,y:(int(round(x)),int(round(y)))
            if p['shape']=='circle':
                r=p['sx']*S/2
                for a in np.linspace(0,2*np.pi,120,endpoint=False):
                    co,si=np.cos(a),np.sin(a)
                    sm.append((P(cx+(r+4.5)*co,cy+(r+4.5)*si),P(cx+(r+11)*co,cy+(r+11)*si),P(cx+(r+16)*co,cy+(r+16)*si)))
            else:
                hx=p['sx']*S/2; hy=p['sy']*S/2
                for t in np.linspace(-0.9,0.9,30):
                    for (ox,oy,nx,ny) in [(t*hx,-hy,0,-1),(t*hx,hy,0,1),(-hx,t*hy,-1,0),(hx,t*hy,1,0)]:
                        sm.append((P(cx+ox+4.5*nx,cy+oy+4.5*ny),P(cx+ox+11*nx,cy+oy+11*ny),P(cx+ox+16*nx,cy+oy+16*ny)))
            fr=frac(sm); GSTAT[(L,f['ref'],p['num'])]=fr
            if fr is not None and fr>0.08: gnd.add(padkey(f,p))
        for vi,(vx,vy) in enumerate(VIAS):
            cx,cy=px(vx),px(vy)
            sm=[tuple((int(round(cx+r*np.cos(a))),int(round(cy+r*np.sin(a)))) for r in (12,17,21)) for a in np.linspace(0,2*np.pi,60,endpoint=False)]
            fr=frac(sm); GSTAT[('V',vi)]=fr
            if fr is not None and fr>0.08: gnd.add(('via',vi))
    for g in gnd: uf.u(g,('GNDPOUR',))
    return uf,dropped
if __name__=='__main__':
    uf,dropped=build()
    print('dropped bad bridges',len(dropped))
    groups={}
    for f in fp.FPS:
        for p in f['pads']:
            if p['num']=='MH': continue
            k=('pad',f['ref'],p['num']); r=uf.f(k)
            groups.setdefault(r,set()).add((f['ref'],p['num'],p.get('name','')))
    gr=uf.f(('GNDPOUR',))
    out=[]
    for r,g in groups.items():
        if len(g)<2 and r!=gr: continue
        out.append(('GND' if r==gr else '', sorted(g)))
    out.sort(key=lambda t:(t[0]!='GND',-len(t[1])))
    for n,g in out: print(n, len(g), ' '.join(f"{a}.{b}{('('+c+')') if c else ''}" for a,b,c in g))
    json.dump(R,open('routes_clean.json','w'))
