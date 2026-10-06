#!/usr/bin/env python3
"""Offline-Validierung der v2.3-Terrain-Overrides (DFF + COL) und Simulation der Runtime-Auswahl (F8)."""
import struct, sys, json, collections
from pathlib import Path
import numpy as np
import os
D=Path(os.environ.get('SA_TERRAIN_DIR','/home/claude/sa/v23/modloader/SA_GlobalTerrain_v2.3'))
MAX_RV=10000; MAX_CV=10000; MAX_CT=15000; CAP=1.85; MAXZ=1.5
RADII=[3.25,4.5,6,8,10,12,14]

def chunks(b,s=0,e=None):
    e=len(b) if e is None else e; out=[]; p=s
    while p+12<=e:
        t,n,v=struct.unpack_from('<III',b,p)
        if t==0 and n==0: break
        if p+12+n>e: raise ValueError('chunk overrun')
        out.append((t,v,p+12,p+12+n)); p+=12+n
    return out,p

def parse_dff(b):
    root,_=chunks(b)
    if not root or root[0][0]!=0x10: raise ValueError('no clump')
    cl,_=chunks(b,root[0][2],root[0][3])
    atom=[c for c in cl if c[0]==0x14]
    gl=[c for c in cl if c[0]==0x1A]
    if len(gl)!=1: raise ValueError('geomlist')
    g,_=chunks(b,gl[0][2],gl[0][3]); geos=[c for c in g if c[0]==0x0F]
    if len(geos)!=1 or len(atom)!=1: raise ValueError('multi geo/atomic %d/%d'%(len(geos),len(atom)))
    gc,_=chunks(b,geos[0][2],geos[0][3])
    st=[c for c in gc if c[0]==1][0]; hd=b[st[2]:st[3]]
    flags,nt,nv,nm=struct.unpack_from('<4I',hd,0); p=16
    uvs=(flags>>16)&255
    if uvs==0: uvs=2 if flags&0x80 else (1 if flags&4 else 0)
    if flags&0x01000000: raise ValueError('native')
    if flags&8: p+=nv*4
    p+=nv*8*uvs
    tri=np.frombuffer(hd,'<u2',nt*4,p).reshape(nt,4); p+=nt*8   # b,a,mat,c
    p+=16; hv,hn=struct.unpack_from('<II',hd,p); p+=8
    if not hv: raise ValueError('no verts')
    v=np.frombuffer(hd,'<f4',nv*3,p).reshape(nv,3); p+=nv*12
    if hn: p+=nv*12
    if p!=len(hd): raise ValueError('struct size mismatch %d!=%d'%(p,len(hd)))
    idx=tri[:,[1,0,3]]
    if nt and idx.max()>=nv: raise ValueError('tri index OOB')
    # material count
    ml=[c for c in gc if c[0]==8][0]; mlc,_=chunks(b,ml[2],ml[3]); nmat=struct.unpack_from('<I',b,mlc[0][2])[0]
    if nt and tri[:,2].max()>=nmat: raise ValueError('material index OOB')
    ext=[c for c in gc if c[0]==3][0]; ec,_=chunks(b,ext[2],ext[3])
    bm=[c for c in ec if c[0]==0x50E]
    info={'binmesh':None,'night':None}
    if bm:
        bf,nmesh,tot=struct.unpack_from('<III',b,bm[0][2]); q=bm[0][2]+12; s=0
        for _ in range(nmesh):
            n,m=struct.unpack_from('<II',b,q); q+=8
            ii=np.frombuffer(b,'<u4',n,q); q+=4*n; s+=n
            if n and ii.max()>=nv: raise ValueError('binmesh index OOB')
            if m>=nmat: raise ValueError('binmesh material OOB')
        if q!=bm[0][3] or s!=tot: raise ValueError('binmesh size')
        if bf==0 and tot!=nt*3: raise ValueError('binmesh trilist count %d != %d'%(tot,nt*3))
        info['binmesh']=(bf,nmesh,tot)
    nv_=[c for c in ec if c[0]==0x253F2F9]
    if nv_:
        if nv_[0][3]-nv_[0][2]!=4+4*nv: raise ValueError('night colour size')
        info['night']=True
    if not np.isfinite(v).all(): raise ValueError('nan verts')
    # bounding sphere
    cx,cy,cz,r=struct.unpack_from('<4f',hd,16+(nv*4 if flags&8 else 0)+nv*8*uvs+nt*8)
    far=float(np.sqrt(((v-np.array([cx,cy,cz],dtype='f4'))**2).sum(1)).max())
    info.update(flags=flags,nt=nt,nv=nv,nmat=nmat,sphere_ok=far<=r*1.001+0.01,tristrip=bool(flags&1))
    return v,idx,info

def parse_col(b):
    recs=[];p=0
    while p+8<=len(b) and b[p:p+4] in (b'COLL',b'COL2',b'COL3',b'COL4'):
        sz=struct.unpack_from('<I',b,p+4)[0]+8
        if p+sz>len(b): raise ValueError('col record overrun')
        recs.append(b[p:p+sz]); p+=sz
    if any(b[p:]): raise ValueError('trailing col bytes')
    return recs

def parse_col_rec(r):
    ver=r[:4]; name=r[8:30].split(b'\0')[0].decode('ascii','replace').lower()
    if ver==b'COLL': return name,None
    bmin=struct.unpack_from('<3f',r,32); bmax=struct.unpack_from('<3f',r,44); sph=struct.unpack_from('<4f',r,56)
    nsph,nbox,ntri=struct.unpack_from('<3H',r,72); nline=r[78]; fl=struct.unpack_from('<I',r,80)[0]
    osph,obox,olin,ov,ot,opl=struct.unpack_from('<6I',r,84)
    out=dict(ver=ver.decode(),name=name,bmin=bmin,bmax=bmax,sphere=sph,raw_faces=None,nsph=nsph,nbox=nbox,ntri=ntri,flags=fl)
    if ntri==0: out['v']=None; return name,out
    vo=ov+4; to=ot+4
    if to+ntri*8>len(r): raise ValueError('faces overrun')
    out['raw_faces']=r[to:to+ntri*8]
    f=np.frombuffer(r,'<u2',ntri*4,to).reshape(ntri,4)[:,:3]
    nv=int(f.max())+1
    gap=to-(vo+nv*6)
    if gap<0: raise ValueError('vertex area overlaps faces')
    v=np.frombuffer(r,'<i2',nv*3,vo).reshape(nv,3).astype('f4')/128.
    out.update(v=v,f=f,nv=nv,gap=gap,faces_end=(to+ntri*8==len(r)),align=(to%4))
    return name,out

def main():
    dffs=sorted(D.glob('*.dff')); cols=sorted(D.glob('*.col'))
    res={'dff_total':len(dffs),'col_files':len(cols)}
    dff={};dff_err={}
    for f in dffs:
        try: dff[f.stem.lower()]=parse_dff(f.read_bytes())
        except Exception as e: dff_err[f.name]=repr(e)
    colm={};col_err={};col_total=0;vers=collections.Counter()
    for f in cols:
        try:
            for r in parse_col(f.read_bytes()):
                col_total+=1; vers[r[:4].decode()]+=1
                try:
                    n,o=parse_col_rec(r)
                    if o: o['file']=f.name; colm[n]=o
                except Exception as e: col_err[f.name+':'+r[8:30].split(b'\0')[0].decode('ascii','replace')]=repr(e)
        except Exception as e: col_err[f.name]=repr(e)
    res.update(dff_parsed=len(dff),dff_errors=dff_err,col_records=col_total,col_versions=dict(vers),col_errors=col_err)
    pairs=[n for n in dff if n in colm and colm[n]['v'] is not None]
    res['dff_without_col_in_pack']=sorted(n for n in dff if n not in colm)
    res['paired']=len(pairs)
    # structural flags
    res['dff_tristrip_flag']=sum(1 for n in dff if dff[n][2]['tristrip'])
    res['dff_sphere_bad']=[n for n in dff if not dff[n][2]['sphere_ok']]
    res['dff_no_binmesh']=[n for n in dff if not dff[n][2]['binmesh']]
    res['col_facegroup_flag']=[n for n in pairs if colm[n]['flags']&8]
    res['col_shadow_flag']=[n for n in pairs if colm[n]['flags']&16]
    res['col_unaligned_faces']=[n for n in pairs if colm[n]['align']]
    # COL vertices outside own bbox
    out_bb=[]
    for n in pairs:
        c=colm[n]; lo=np.array(c['bmin'])-0.02; hi=np.array(c['bmax'])+0.02
        if (c['v']<lo).any() or (c['v']>hi).any(): out_bb.append(n)
    res['col_verts_outside_bbox']=out_bb
    # runtime limits
    over=[n for n in pairs if dff[n][2]['nv']>MAX_RV or dff[n][2]['nt']>MAX_CT or colm[n]['nv']>MAX_CV or colm[n]['ntri']>MAX_CT]
    res['exceeds_runtime_caps']=over
    # bbox floor problem: share of col vertices that a CAP-deep crater would push below bbox min z
    below=[];
    for n in pairs:
        c=colm[n]; z=c['v'][:,2]; below.append(float((z-CAP<c['bmin'][2]).mean()))
    below=np.array(below)
    res['bbox_floor']={'models_with_any_vertex_at_risk':int((below>0).sum()),'models_majority_at_risk':int((below>0.5).sum()),'models_all_at_risk':int((below>=0.999).sum()),'mean_share_at_risk':round(float(below.mean()),3)}
    # simulate F8 on near-horizontal collision triangles
    rng=np.random.default_rng(1); hist=collections.Counter(); per_model_fail=[]; samples=0
    for n in pairs:
        rv=dff[n][0]; c=colm[n]; cv=c['v']; f=c['f']
        a,b_,cc=cv[f[:,0]],cv[f[:,1]],cv[f[:,2]]
        nrm=np.cross(b_-a,cc-a); L=np.linalg.norm(nrm,axis=1); ok=(L>1e-6)
        nz=np.zeros(len(f)); nz[ok]=np.abs(nrm[ok,2])/L[ok]
        cand=np.where(nz>=0.78)[0]
        if len(cand)==0: hist['no_flat_surface']+=1; per_model_fail.append(1.0); continue
        area=L[cand]/2; pick=rng.choice(cand,size=min(24,max(8,len(cand)//20)),p=area/area.sum())
        w=rng.dirichlet([1,1,1],size=len(pick))
        pts=a[pick]*w[:,[0]]+b_[pick]*w[:,[1]]+cc[pick]*w[:,[2]]
        fails=0
        for pt in pts:
            samples+=1; chosen=None
            dR=((rv[:,:2]-pt[:2])**2).sum(1); zR=np.abs(rv[:,2]-pt[2])<MAXZ
            dC=((cv[:,:2]-pt[:2])**2).sum(1); zC=np.abs(cv[:,2]-pt[2])<MAXZ
            for r in RADII:
                if ((dR<r*r)&zR).sum()<12 or ((dC<r*r)&zC).sum()<8: continue
                h=r*.5
                if ((dR<h*h)&zR).sum()<1 or ((dC<h*h)&zC).sum()<1: continue
                chosen=r;break
            hist[str(chosen)]+=1
            if chosen is None: fails+=1
        per_model_fail.append(fails/len(pts))
    res['f8_simulation']={'samples':samples,'radius_histogram':dict(hist),'models_always_fail':int((np.array(per_model_fail)>=0.999).sum()),'models_never_fail':int((np.array(per_model_fail)==0).sum())}
    # totals
    res['totals']={'render_verts':int(sum(dff[n][2]['nv'] for n in dff)),'render_tris':int(sum(dff[n][2]['nt'] for n in dff)),'col_verts':int(sum(colm[n]['nv'] for n in pairs)),'col_tris':int(sum(colm[n]['ntri'] for n in pairs))}
    json.dump(res,open(os.environ.get('SA_VALIDATION_OUT','validation_result.json'),'w'),indent=1)
    short={k:(v if not isinstance(v,(list,dict)) or len(v)<12 else (len(v),list(v)[:6] if isinstance(v,list) else list(v.items())[:4])) for k,v in res.items()}
    print(json.dumps(short,indent=1,default=str))
if __name__=='__main__': main()
