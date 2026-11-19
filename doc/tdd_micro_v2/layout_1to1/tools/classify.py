import cv2, numpy as np, sys, json
def classify(path):
    im = cv2.imread(path)[:,:,::-1].astype(int)  # RGB
    r,g,b = im[...,0],im[...,1],im[...,2]
    cls = np.zeros(r.shape, np.uint8)  # 0 unknown
    white = (r>170)&(g>170)&(b>170)
    gold  = (r>150)&(g>100)&(b<90)&(r>g)
    black = (r<60)&(g<60)&(b<60)
    bright= (g>150)&(r<110)&(b<110)
    dark  = (g>60)&(g<=150)&(r<90)&(b<90)
    cls[dark]=1; cls[bright]=2; cls[gold]=3; cls[black]=4; cls[white]=5
    return cls
if __name__=='__main__':
    for f in ['top.jpg','bottom.jpg']:
        c=classify(f)
        print(f, [(k,int((c==k).sum())) for k in range(6)])
        pal=np.array([[255,0,255],[20,90,20],[0,191,0],[225,164,11],[0,0,0],[255,255,255]],np.uint8)
        cv2.imwrite(f.replace('.jpg','_cls.png'), pal[c][:,:,::-1])
