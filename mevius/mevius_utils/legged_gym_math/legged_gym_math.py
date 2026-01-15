from typing import Tuple


import torch


from .isaacgym_torch_utils import normalize, quat_apply


# @ torch.jit.script
def quat_apply_yaw(quat, vec):
    quat_yaw = quat.clone().view(-1, 4)
    quat_yaw[:, :2] = 0.0
    quat_yaw = normalize(quat_yaw)
    return quat_apply(quat_yaw, vec)


# @ torch.jit.script


def wrap_to_pi(angles):
    angles_mod = angles % (2 * torch.pi)
    angles_mod = torch.where(angles_mod > torch.pi, angles_mod - 2 * torch.pi, angles_mod)
    return angles_mod


# @ torch.jit.script


def torch_rand_sqrt_float(lower, upper, shape, device):
    # type: (float, float, Tuple[int, int], str) -> Tensor
    r = 2 * torch.rand(*shape, device=device) - 1
    r = torch.where(r < 0.0, -torch.sqrt(-r), torch.sqrt(r))
    r = (r + 1.0) / 2.0
    return (upper - lower) * r + lower
