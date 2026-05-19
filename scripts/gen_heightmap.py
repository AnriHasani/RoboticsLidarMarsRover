import numpy as np, math, random
from PIL import Image
import os

def fade(t): return t*t*t*(t*(t*6-15)+10)
def lerp(a,b,t): return a+t*(b-a)

def grad(h,x,y):
    g = [(1,1),(-1,1),(1,-1),(-1,-1),(1,0),(-1,0),(0,1),(0,-1)]
    v = g[h&7]; return v[0]*x+v[1]*y

def make_perm(seed=42):
    p=list(range(256)); random.seed(seed); random.shuffle(p); return p+p

def noise(x,y,p):
    xi,yi=int(math.floor(x))&255,int(math.floor(y))&255
    xf,yf=x-math.floor(x),y-math.floor(y)
    u,v=fade(xf),fade(yf)
    aa=p[p[xi]+yi]; ab=p[p[xi]+yi+1]; ba=p[p[xi+1]+yi]; bb=p[p[xi+1]+yi+1]
    return lerp(lerp(grad(aa,xf,yf),grad(ba,xf-1,yf),u),
                lerp(grad(ab,xf,yf-1),grad(bb,xf-1,yf-1),u),v)

def fbm(x,y,p,octaves=7,scale=4.0):
    val,amp,freq,mx=0,1,1,0
    for _ in range(octaves):
        val+=noise(x*freq/scale,y*freq/scale,p)*amp; mx+=amp; amp*=0.5; freq*=2.0
    return val/mx

def generate(size=257, seed=1337, octaves=4, scale=12.0, craters=5):
    p=make_perm(seed)
    data=np.zeros((size,size),dtype=np.float32)
    for y in range(size):
        for x in range(size):
            data[y,x]=fbm(x,y,p,octaves,scale)

    rng=random.Random(seed+1)
    for _ in range(craters):
        cx=rng.randint(40,size-40); cy=rng.randint(40,size-40); r=rng.randint(12,28)
        for gy in range(max(0,cy-r-6),min(size,cy+r+6)):
            for gx in range(max(0,cx-r-6),min(size,cx+r+6)):
                d=math.hypot(gx-cx,gy-cy)
                if d<r:     data[gy,gx]-=0.35*(1-d/r)
                elif d<r+5: data[gy,gx]+=0.15*(1-(d-r)/5)

    data=(data-data.min())/(data.max()-data.min())
    out_dir=os.path.join(os.path.dirname(__file__),'..','models')
    os.makedirs(out_dir,exist_ok=True)
    out_path=os.path.join(out_dir,'mars_heightmap.png')
    Image.fromarray((data*255).astype(np.uint8),'L').save(out_path)
    print(f"Saved: {os.path.abspath(out_path)}  ({size}x{size})")

if __name__=='__main__':
    generate()