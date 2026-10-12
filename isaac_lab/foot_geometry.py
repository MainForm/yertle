"""Foot-sole height from the authored rubber mesh, not the shin link origin.

Read-only USD inspection happens once per environment. No frames, physics
properties, model files, observations, actions or hardware mapping are edited.
"""
import torch

FOOT_NAMES = ('lf_shin', 'rf_shin', 'lb_shin', 'rb_shin')


def load_sole_points(usd_path):
    """Return rubber surface vertices in each shin body's local coordinates.

    Use authored Rubber material face subsets (even if a parent overrides the
    displayed material). Traverse instance proxies and include all authored
    mesh transforms, rather than assuming a shin length or foot offset.
    Fail explicitly if the asset has no identifiable sole; never fall back to
    the link origin. Units must be meters, as in the training articulation.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise ValueError(f'Cannot open foot geometry USD: {usd_path}')
    if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-8:
        raise ValueError('Foot geometry expects a meter-unit USD articulation')
    cache = UsdGeom.XformCache()
    prims = list(stage.Traverse(Usd.TraverseInstanceProxies()))
    results = []
    for name in FOOT_NAMES:
        # Joint prims share the same names; only the link Xform is a body.
        bodies = [p for p in prims if p.GetName() == name and p.IsA(UsdGeom.Xform)]
        if len(bodies) != 1:
            raise ValueError(f'Expected one shin body named {name}, found {len(bodies)}')
        body = bodies[0]
        inverse = cache.GetLocalToWorldTransform(body).GetInverse()
        points = set()
        for prim in prims:
            if not prim.GetPath().HasPrefix(body.GetPath()) or not prim.IsA(UsdGeom.Subset):
                continue
            targets = prim.GetRelationship('material:binding').GetTargets()
            if not any('rubber' in str(t).lower() for t in targets):
                continue
            subset = UsdGeom.Subset(prim)
            mesh = UsdGeom.Mesh(prim.GetParent())
            if not mesh or subset.GetElementTypeAttr().Get() != UsdGeom.Tokens.face:
                raise ValueError(f'Sole subset must contain mesh faces: {prim.GetPath()}')
            # Visual and collision origins can differ. Use only the authored
            # collision mesh, never a visual-only rubber surface.
            # The importer may author CollisionAPI on a parent Xform instead
            # of the mesh itself; inspect ancestors only inside this body.
            collider = mesh.GetPrim()
            collision_enabled = False
            while collider and collider != body:
                if collider.HasAPI(UsdPhysics.CollisionAPI):
                    collision_enabled = True
                    break
                collider = collider.GetParent()
            if not collision_enabled:
                continue
            vertices = mesh.GetPointsAttr().Get()
            counts = mesh.GetFaceVertexCountsAttr().Get()
            indices = mesh.GetFaceVertexIndicesAttr().Get()
            starts = [0]
            for count in counts:
                starts.append(starts[-1] + count)
            relative = cache.GetLocalToWorldTransform(mesh.GetPrim()) * inverse
            for face in subset.GetIndicesAttr().Get():
                for vertex in indices[starts[face]:starts[face+1]]:
                    point = relative.Transform(vertices[vertex])
                    points.add(tuple(float(x) for x in point))
        if not points:
            raise ValueError(f'No authored rubber sole surface for {name} in {usd_path}')
        results.append(sorted(points))
    return results


def minimum_surface_height(body_pos_w, body_quat_w, local_points):
    """Lowest rubber surface vertex in world Z; quaternions are (w,x,y,z).

    Positions: [env, foot, 3], rotations: [env, foot, 4], padded surface
    vertices: [foot, vertex, 3]. Duplicated padding vertices do not change min.
    The rotating surface (not a fixed vertical offset) handles tilted legs.
    """
    w, x, y, z = body_quat_w.unbind(-1)
    row_z = torch.stack((2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), dim=-1)
    # Direct products avoid TF32 matrix-multiply rounding near ground contact.
    heights = (row_z[..., 0, None]*local_points[None, ..., 0]
               + row_z[..., 1, None]*local_points[None, ..., 1]
               + row_z[..., 2, None]*local_points[None, ..., 2])
    return body_pos_w[..., 2] + heights.amin(dim=-1)


def foot_sole_heights(env):
    """Read live body pose buffers and reuse asset-local sole vertices."""
    asset = env.scene['robot']
    if not hasattr(env, '_yertle_sole_geometry'):
        points = load_sole_points(asset.cfg.spawn.usd_path)
        maximum = max(len(p) for p in points)
        padded = [p + [p[-1]]*(maximum-len(p)) for p in points]
        tensor = torch.tensor(padded, device=asset.device, dtype=asset.data.body_pos_w.dtype)
        ids = [asset.body_names.index(name) for name in FOOT_NAMES]
        env._yertle_sole_geometry = (ids, tensor)
    ids, points = env._yertle_sole_geometry
    return minimum_surface_height(asset.data.body_pos_w[:, ids], asset.data.body_quat_w[:, ids], points)
