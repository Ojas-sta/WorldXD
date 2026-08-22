import trimesh
import os
import glob
import numpy as np

def analyze_stls():
    stls = glob.glob('*.stl')
    results = []
    for stl in stls:
        try:
            mesh = trimesh.load_mesh(stl)
            if not isinstance(mesh, trimesh.Trimesh):
                mesh = mesh.dump()[0]
            extents = mesh.extents
            results.append((stl, extents))
        except Exception as e:
            pass
    
    results.sort(key=lambda x: x[0])
    for stl, ext in results:
        print(f"{stl:35} | {ext[0]:6.2f} x {ext[1]:6.2f} x {ext[2]:6.2f} | Max: {max(ext):6.2f}")

if __name__ == '__main__':
    analyze_stls()
