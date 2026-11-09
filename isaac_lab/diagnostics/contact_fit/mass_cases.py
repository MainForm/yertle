"""Body mass-property candidates for the archived real-goal replay."""

import torch


BASELINES = {
    "original_friction": (1.0, 1.0, 1.0, 1.0),
    "fitted_friction": (0.1, 0.5, 0.5, 0.1),
}
VARIANTS = {
    "baseline": ((0, 0, 0), (1, 1, 1)),
    "com_x_minus20": ((-20, 0, 0), (1, 1, 1)),
    "com_x_plus20": ((20, 0, 0), (1, 1, 1)),
    "com_y_minus15": ((0, -15, 0), (1, 1, 1)),
    "com_y_plus15": ((0, 15, 0), (1, 1, 1)),
    "com_z_minus20": ((0, 0, -20), (1, 1, 1)),
    "com_z_plus20": ((0, 0, 20), (1, 1, 1)),
    "inertia_half": ((0, 0, 0), (0.5, 0.5, 0.5)),
    "inertia_double": ((0, 0, 0), (2, 2, 2)),
    "inertia_yaw_low": ((0, 0, 0), (1, 1, 0.6)),
    "inertia_yaw_high": ((0, 0, 0), (1, 1, 1.6)),
    "com_y_minus15_inertia_half": ((0, -15, 0), (0.5, 0.5, 0.5)),
    "com_y_plus15_inertia_half": ((0, 15, 0), (0.5, 0.5, 0.5)),
    "com_y_minus15_inertia_double": ((0, -15, 0), (2, 2, 2)),
    "com_y_plus15_inertia_double": ((0, 15, 0), (2, 2, 2)),
}
CASES = [
    ("goal", f"{material}__{variant}", friction)
    for material, friction in BASELINES.items()
    for variant in VARIANTS
]


def apply_mass_cases(robot, cases=CASES, variants=VARIANTS):
    view = robot.root_physx_view
    index = robot.body_names.index("base_link")
    coms = view.get_coms().clone()
    inertias = view.get_inertias().clone()
    masses = view.get_masses().clone()
    assert len(coms) == len(cases)
    default_com = coms[0, index].clone()
    default_inertia = inertias[0, index].clone().reshape(3, 3)
    torch.testing.assert_close(default_inertia, torch.diag(default_inertia.diag()))
    offsets = []
    applied = {}
    for row, (_, name, _) in enumerate(cases):
        variant = name.split("__", 1)[1]
        offset_mm, scale = variants[variant]
        offset = coms.new_tensor(offset_mm) / 1000.0
        inertia = default_inertia.diag() * inertias.new_tensor(scale)
        assert inertia.min() > 0 and 2 * inertia.max() < inertia.sum()
        coms[row, index, :3] = default_com[:3] + offset
        inertias[row, index] = torch.diag(inertia).flatten()
        offsets.append(offset)
        applied[name] = {
            "com_offset_mm": offset_mm,
            "com_xyz_m": coms[row, index, :3].tolist(),
            "inertia_scale_xyz": scale,
            "inertia_diagonal_kg_m2": inertia.tolist(),
        }
    ids = torch.arange(len(cases), dtype=torch.int32)
    view.set_coms(coms, ids)
    view.set_inertias(inertias, ids)
    torch.testing.assert_close(view.get_coms(), coms)
    torch.testing.assert_close(view.get_inertias(), inertias)
    torch.testing.assert_close(view.get_masses(), masses)
    report = {
        "base_mass_kg": masses[0, index].item(),
        "default_com_xyz_m": default_com[:3].tolist(),
        "default_inertia_diagonal_kg_m2": default_inertia.diag().tolist(),
        "applied": applied,
    }
    return torch.stack(offsets), report
