import numpy as np, cv2
NB=[(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
def trace(sk):
    sk=(sk>0).astype(np.uint8)
    H,W=sk.shape
    k=np.ones((3,3),np.uint8); k[1,1]=0
    deg=cv2.filter2D(sk,-1,k,borderType=cv2.BORDER_CONSTANT)*sk
    ys,xs=np.nonzero(sk)
    pix=set(zip(ys.tolist(),xs.tolist()))
    node=lambda p: deg[p]!=2
    visited_edges=set()
    paths=[]
    def nbrs(p):
        y,x=p
        return [(y+dy,x+dx) for dy,dx in NB if (y+dy,x+dx) in pix]
    starts=[p for p in pix if node(p)]
    for s in starts:
        for n in nbrs(s):
            e=(s,n) if s<n else (n,s)
            if e in visited_edges: continue
            path=[s,n]; visited_edges.add(e)
            prev,cur=s,n
            while not node(cur):
                nx=[q for q in nbrs(cur) if q!=prev and ((cur,q) if cur<q else (q,cur)) not in visited_edges]
                if not nx: break
                # prefer 4-neighbours
                nx.sort(key=lambda q: abs(q[0]-cur[0])+abs(q[1]-cur[1]))
                q=nx[0]; e=(cur,q) if cur<q else (q,cur); visited_edges.add(e)
                path.append(q); prev,cur=cur,q
            paths.append(path)
    # closed loops without nodes
    rem=pix-set(p for pa in paths for p in pa)
    while rem:
        s=rem.pop(); path=[s]; cur=s; prev=None
        while True:
            nx=[q for q in nbrs(cur) if q in rem]
            if not nx: break
            cur=nx[0]; rem.discard(cur); path.append(cur)
        if len(path)>3: paths.append(path)
    return paths
def simplify(path,eps=1.8):
    pts=np.array([[x,y] for y,x in path],np.float32).reshape(-1,1,2)
    ap=cv2.approxPolyDP(pts,eps,False).reshape(-1,2)
    return ap.tolist()
