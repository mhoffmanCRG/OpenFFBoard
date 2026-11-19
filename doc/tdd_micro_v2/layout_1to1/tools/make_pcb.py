import json, math, uuid, sys, os
import numpy as np
import fp, netdef, nets, silk
from routes2 import VIAS
OX,OY=100.0,60.0            # board origin on the KiCad page (mm)
OUT=sys.argv[1] if len(sys.argv)>1 else 'out'
os.makedirs(OUT,exist_ok=True)
N=netdef.assign()
# ---------------- track nets ----------------
def snapw(w):
    for lim,val in [(0.33,0.254),(0.45,0.4),(0.55,0.5),(0.7,0.6),(0.9,0.8),(9,1.0)]:
        if w<lim: return val
R0=json.load(open('routes_mm.json'))
def solve(R):
    nets.R=json.loads(json.dumps(R))
    uf,_=nets.build(drop_bad_bridges=True)
    return uf,nets.R
def group_names(uf):
    names={}
    for f,p in fp.all_pads():
        n=N[(f['ref'],p['num'])]
        if not n: continue
        names.setdefault(uf.f(('pad',f['ref'],p['num'])),set()).add(n)
    return names
R=R0
for it in range(4):
    # build without pour union
    nets_build_orig=nets.build
    uf,RR=solve(R)
    # undo pour union effect: recompute names ignoring GNDPOUR by rebuilding without it
    gn=group_names(uf)
    gp=uf.f(('GNDPOUR',))
    conflicts=[g for g,s in gn.items() if len(s)>1 and g!=gp]
    if not conflicts: break
    cs=set(conflicts); removed=0
    for L in 'FB':
        keep=[]
        for i,pl in enumerate(RR[L]['polys']):
            if pl['ends'][0] in ('bridge','ext','tee') and uf.f(('trk',L,i)) in cs: removed+=1; continue
            keep.append(pl)
        RR[L]['polys']=keep
    print('iteration',it,'conflict groups',len(conflicts),'removed inferred segs',removed)
    R=RR
uf,RR=solve(R)
gn=group_names(uf); gp=uf.f(('GNDPOUR',))
def name_of(key):
    g=uf.f(key)
    s=gn.get(g,set())
    s=s-{''}
    if len(s)==1: return next(iter(s))
    if len(s)>1:
        s2=s-{'GND'} if g==gp else s
        return sorted(s2)[0] if s2 else 'GND'
    return 'GND' if g==gp else ''
netnames=sorted({n for n in N.values() if n}|{'GND'})
NID={'':0}
for i,n in enumerate(netnames): NID[n]=i+1
def U(): return str(uuid.uuid4())
def f2(v): return f'{v:.4f}'.rstrip('0').rstrip('.')
L=[]
w=L.append
w('(kicad_pcb (version 20221018) (generator pcbnew)')
w('  (general (thickness 1.6))')
w('  (paper "A4")')
w('  (title_block (title "TDD Micro AASD-SERVO MMOS FFB v2.0 - 1:1 layout (reconstructed)") (date "2026-11-19") (rev "L1") (company "Reverse-engineered from CAM renders - unverified") (comment 1 "Not an official Thanos/tronicgr document") (comment 2 "Scale 29.345 px/mm; positions +/-0.05 mm; nets from copper + silkscreen + OpenFFBoard F407VG_DISCO"))')
w('''  (layers
    (0 "F.Cu" signal) (31 "B.Cu" signal)
    (32 "B.Adhes" user "B.Adhesive") (33 "F.Adhes" user "F.Adhesive")
    (34 "B.Paste" user) (35 "F.Paste" user)
    (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen")
    (38 "B.Mask" user) (39 "F.Mask" user)
    (40 "Dwgs.User" user "User.Drawings") (41 "Cmts.User" user "User.Comments")
    (42 "Eco1.User" user "User.Eco1") (43 "Eco2.User" user "User.Eco2")
    (44 "Edge.Cuts" user) (45 "Margin" user)
    (46 "B.CrtYd" user "B.Courtyard") (47 "F.CrtYd" user "F.Courtyard")
    (48 "B.Fab" user) (49 "F.Fab" user)
  )''')
w('  (setup (pad_to_mask_clearance 0) (aux_axis_origin %s %s) (grid_origin %s %s))'%(f2(OX),f2(OY),f2(OX),f2(OY)))
for n,i in sorted(NID.items(),key=lambda t:t[1]): w(f'  (net {i} "{n}")')
# ---------------- footprints ----------------
for F in fp.FPS:
    side='B' if F['layer']=='B' else 'F'
    fx,fy=F['x']+OX,F['y']+OY
    w(f'  (footprint "{F["name"]}" (layer "{side}.Cu") (tstamp {U()}) (at {f2(fx)} {f2(fy)})')
    w(f'    (descr "{F.get("desc","") or F["value"]}")')
    w('    (attr smd)' if all(p['type']=='smd' for p in F['pads']) else '    (attr through_hole)')
    xs=[p['x'] for p in F['pads']]; ys=[p['y'] for p in F['pads']]
    sx=[p['sx'] for p in F['pads']]; sy=[p['sy'] for p in F['pads']]
    x0=min(x-s/2 for x,s in zip(xs,sx))-0.4-F['x']; x1=max(x+s/2 for x,s in zip(xs,sx))+0.4-F['x']
    y0=min(y-s/2 for y,s in zip(ys,sy))-0.4-F['y']; y1=max(y+s/2 for y,s in zip(ys,sy))+0.4-F['y']
    mir=' (justify mirror)' if side=='B' else ''
    w(f'    (fp_text reference "{F["ref"]}" (at 0 {f2(y0-0.6)}) (layer "{side}.Fab") (effects (font (size 0.8 0.8) (thickness 0.12)){mir}) (tstamp {U()}))')
    w(f'    (fp_text value "{F["value"]}" (at 0 {f2(y1+0.6)}) (layer "{side}.Fab") hide (effects (font (size 0.8 0.8) (thickness 0.12)){mir}) (tstamp {U()}))')
    w(f'    (fp_rect (start {f2(x0)} {f2(y0)}) (end {f2(x1)} {f2(y1)}) (stroke (width 0.05) (type default)) (fill none) (layer "{side}.CrtYd") (tstamp {U()}))')
    for p in F['pads']:
        dx,dy=p['x']-F['x'],p['y']-F['y']
        net=N[(F['ref'],p['num'])]
        nt=f' (net {NID[net]} "{net}")' if net else ''
        num='' if p['num']=='MH' else p['num']
        pn=f' (pinfunction "{p["name"]}")' if p.get('name') else ''
        if p['type']=='thru_hole':
            w(f'    (pad "{num}" thru_hole {p["shape"]} (at {f2(dx)} {f2(dy)}) (size {f2(p["sx"])} {f2(p["sy"])}) (drill {f2(p["drill"])}) (layers "*.Cu" "*.Mask"){nt}{pn} (tstamp {U()}))')
        else:
            lay=f'"{side}.Cu" "{side}.Paste" "{side}.Mask"'
            w(f'    (pad "{num}" smd {p["shape"]} (at {f2(dx)} {f2(dy)}) (size {f2(p["sx"])} {f2(p["sy"])}) (layers {lay}){nt}{pn} (tstamp {U()}))')
    w('  )')
# ---------------- board outline ----------------
w(f'  (gr_rect (start {f2(OX)} {f2(OY)}) (end {f2(OX+fp.BOARD_W)} {f2(OY+fp.BOARD_H)}) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts") (tstamp {U()}))')
# ---------------- tracks ----------------
nseg=0; trk_out={'F':[],'B':[]}
for Lr in 'FB':
    for i,pl in enumerate(RR[Lr]['polys']):
        nm=name_of(('trk',Lr,i)); wd=snapw(pl['w'])
        pts=pl['pts']
        plen_=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(pts,pts[1:]))
        inU1=all(60.2<x<66.3 and 59.3<y<63.9 for x,y in pts)
        if not nm and (plen_<1.5 or inU1): continue
        for a,b in zip(pts,pts[1:]):
            if math.hypot(b[0]-a[0],b[1]-a[1])<0.01: continue
            w(f'  (segment (start {f2(a[0]+OX)} {f2(a[1]+OY)}) (end {f2(b[0]+OX)} {f2(b[1]+OY)}) (width {wd}) (layer "{Lr}.Cu") (net {NID[nm]}) (tstamp {U()}))')
            nseg+=1; trk_out[Lr].append((a,b,wd,nm))
# ---------------- vias ----------------
via_out=[]
for vi,(vx,vy) in enumerate(VIAS):
    nm=name_of(('via',vi))
    w(f'  (via (at {f2(vx+OX)} {f2(vy+OY)}) (size 0.7) (drill 0.3) (layers "F.Cu" "B.Cu") (net {NID[nm]}) (tstamp {U()}))')
    via_out.append((vx,vy,nm))
# ---------------- GND zones ----------------
m=0.25
for Lr in 'FB':
    w(f'  (zone (net {NID["GND"]}) (net_name "GND") (layer "{Lr}.Cu") (tstamp {U()}) (hatch edge 0.5)')
    w('    (connect_pads (clearance 0.3)) (min_thickness 0.25) (filled_areas_thickness no)')
    w('    (fill (thermal_gap 0.3) (thermal_bridge_width 0.4))')
    w(f'    (polygon (pts (xy {f2(OX+m)} {f2(OY+m)}) (xy {f2(OX+fp.BOARD_W-m)} {f2(OY+m)}) (xy {f2(OX+fp.BOARD_W-m)} {f2(OY+fp.BOARD_H-m)}) (xy {f2(OX+m)} {f2(OY+fp.BOARD_H-m)})))')
    w('  )')
# ---------------- silkscreen artwork (vectorised 1:1 from the renders) ----------------
silk_out={}
for Lr in 'FB':
    polys=silk.silk_polys(Lr); silk_out[Lr]=polys
    for poly in polys:
        pts=' '.join(f'(xy {f2(x+OX)} {f2(y+OY)})' for x,y in poly)
        w(f'  (gr_poly (pts {pts}) (stroke (width 0) (type solid)) (fill solid) (layer "{Lr}.SilkS") (tstamp {U()}))')
w(f'  (gr_text "Reconstructed 1:1 from CAM renders (2026-11-19). Copper traces auto-vectorised; check DRC/ratsnest. GND zones: press B to fill." (at {f2(OX)} {f2(OY-3)}) (layer "Cmts.User") (tstamp {U()}) (effects (font (size 1 1) (thickness 0.15)) (justify left)))')
w(')')
fn=os.path.join(OUT,'tdd_micro_v2_layout.kicad_pcb')
open(fn,'w').write('\n'.join(L)+'\n')
json.dump(dict(tracks=trk_out,vias=via_out,silk=silk_out,nid=NID),open('layout_data.json','w'))
print('wrote',fn,'segments',nseg,'vias',len(VIAS),'nets',len(NID)-1,'silk polys',sum(len(v) for v in silk_out.values()))
