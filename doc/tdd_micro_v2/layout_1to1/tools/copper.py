import cv2, numpy as np
from classify import classify
def layer(path, mirror):
    img=cv2.imread(path)
    if mirror: img=img[:,::-1].copy()
    c=classify_img(img)
    return img,c
def classify_img(img):
    im=img[:,:,::-1].astype(int); r,g,b=im[...,0],im[...,1],im[...,2]
    c=np.zeros(r.shape,np.uint8)
    c[(g>60)&(g<=150)&(r<90)&(b<90)]=1
    c[(g>150)&(r<110)&(b<110)]=2
    c[(r>150)&(g>100)&(b<90)&(r>g)]=3
    c[(r<60)&(g<60)&(b<60)]=4
    c[(r>170)&(g>170)&(b>170)]=5
    return c
def copper_mask(img):
    c=classify_img(img)
    silk=((c==5)|(c==0)).astype(np.uint8)
    silk=cv2.dilate(silk,np.ones((3,3),np.uint8))
    # build a label image: 0 no copper(dark), 255 copper (bright/gold/black-hole)
    base=np.zeros(c.shape,np.uint8)
    base[(c==2)|(c==3)|(c==4)]=255
    base[c==1]=0
    # holes at board edge/outline are black too; handled later by board mask
    rep=cv2.inpaint(base,silk,3,cv2.INPAINT_TELEA)
    cu=(rep>127).astype(np.uint8)
    return cu,c
if __name__=='__main__':
    for f,m,name in [('top.jpg',False,'F'),('bottom.jpg',True,'B')]:
        img=cv2.imread(f)
        if m: img=img[:,::-1].copy()
        cu,c=copper_mask(img)
        # board interior mask: exclude outer 8px
        cu[:8,:]=0; cu[-8:,:]=0; cu[:,:8]=0; cu[:,-8:]=0
        np.save(f'cu_{name}.npy',cu)
        n,lab,st,cen=cv2.connectedComponentsWithStats(cu,4)
        big=np.argsort(-st[:,4])[:5]
        print(name,'components',n,'largest areas',[(int(i),int(st[i,4])) for i in big])
        rng=np.random.default_rng(1); col=rng.integers(40,255,(n,3)).astype(np.uint8); col[0]=0
        col[big[1]]=[60,60,60]
        cv2.imwrite(f'comp_{name}.png',col[lab])
