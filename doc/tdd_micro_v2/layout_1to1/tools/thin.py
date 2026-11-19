import numpy as np
def zhang_suen(img):
    I=(img>0).astype(np.uint8).copy()
    I=np.pad(I,1)
    changed=True
    while changed:
        changed=False
        for step in (0,1):
            P2=I[:-2,1:-1];P3=I[:-2,2:];P4=I[1:-1,2:];P5=I[2:,2:];P6=I[2:,1:-1];P7=I[2:,:-2];P8=I[1:-1,:-2];P9=I[:-2,:-2]
            C=I[1:-1,1:-1]
            B=P2+P3+P4+P5+P6+P7+P8+P9
            seq=[P2,P3,P4,P5,P6,P7,P8,P9,P2]
            A=sum(((seq[i]==0)&(seq[i+1]==1)).astype(np.uint8) for i in range(8))
            if step==0: m=(P2*P4*P6==0)&(P4*P6*P8==0)
            else: m=(P2*P4*P8==0)&(P2*P6*P8==0)
            D=(C==1)&(B>=2)&(B<=6)&(A==1)&m
            if D.any():
                C[D]=0; changed=True
    return I[1:-1,1:-1]
