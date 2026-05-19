import os
from PIL import Image

def generate(scale_xy=50.0, scale_z=5.0, heightmap_path=None, output_path=None):
    if heightmap_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        heightmap_path = os.path.join(script_dir, '..', 'models', 'mars_heightmap.png')

    img = Image.open(heightmap_path).convert('L')
    pixels = list(img.getdata())
    w, h = img.size

    verts = []
    uvs = []
    for y in range(h):
        for x in range(w):
            wx = (x / (w - 1) - 0.5) * scale_xy
            wy = (y / (h - 1) - 0.5) * scale_xy
            wz = (pixels[y * w + x] / 255.0) * scale_z
            verts.append((wx, wy, wz))
            uvs.append((x / (w - 1), 1.0 - y / (h - 1)))

    faces = []
    for y in range(h - 1):
        for x in range(w - 1):
            i0 = y * w + x
            i1 = y * w + x + 1
            i2 = (y + 1) * w + x
            i3 = (y + 1) * w + x + 1
            faces.append((i0, i1, i2))
            faces.append((i1, i3, i2))

    norms = [(0.0, 0.0, 0.0) for _ in verts]
    for f in faces:
        v0, v1, v2 = verts[f[0]], verts[f[1]], verts[f[2]]
        ex = v1[0] - v0[0]; ey = v1[1] - v0[1]; ez = v1[2] - v0[2]
        fx = v2[0] - v0[0]; fy = v2[1] - v0[1]; fz = v2[2] - v0[2]
        nx = ey * fz - ez * fy
        ny = ez * fx - ex * fz
        nz = ex * fy - ey * fx
        nl = (nx * nx + ny * ny + nz * nz) ** 0.5
        if nl > 0:
            nx /= nl; ny /= nl; nz /= nl
        for idx in f:
            n = norms[idx]
            norms[idx] = (n[0] + nx, n[1] + ny, n[2] + nz)

    for i in range(len(norms)):
        n = norms[i]
        nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) ** 0.5
        if nl > 0:
            norms[i] = (n[0] / nl, n[1] / nl, n[2] / nl)
        else:
            norms[i] = (0.0, 0.0, 1.0)

    if output_path is None:
        models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models')
        output_path = os.path.join(models_dir, 'mars_terrain.obj')

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, 'w') as f:
        f.write("# Generated terrain mesh from mars_heightmap.png\n")
        f.write(f"# {len(verts)} vertices, {len(faces)} triangles\n\n")
        for v in verts:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        f.write("\n")
        for uv in uvs:
            f.write(f"vt {uv[0]:.6f} {uv[1]:.6f}\n")
        f.write("\n")
        for n in norms:
            f.write(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n")
        f.write("\n")
        for face in faces:
            idx = tuple(f + 1 for f in face)
            f.write(f"f {idx[0]}/{idx[0]}/{idx[0]} {idx[1]}/{idx[1]}/{idx[1]} {idx[2]}/{idx[2]}/{idx[2]}\n")

    print(f"Saved: {os.path.abspath(output_path)}  ({len(verts)} verts, {len(faces)} tris)")

if __name__ == '__main__':
    generate(scale_z=1.5)
