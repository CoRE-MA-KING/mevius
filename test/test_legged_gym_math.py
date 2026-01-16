from mevius.mevius_utils.legged_gym_math.legged_gym_math import wrap_to_pi
import torch


def test_wrap_to_pi():
    # Test with a tensor
    angles = torch.tensor([0.0, torch.pi, 2 * torch.pi, 3 * torch.pi, -torch.pi, -2.5 * torch.pi])
    expected = torch.tensor([0.0, torch.pi, 0.0, torch.pi, torch.pi, -0.5 * torch.pi])
    torch.testing.assert_close(wrap_to_pi(angles), expected)

    # Test with single values
    torch.testing.assert_close(wrap_to_pi(torch.tensor(0.0)), torch.tensor(0.0))
    torch.testing.assert_close(wrap_to_pi(torch.tensor(torch.pi)), torch.tensor(torch.pi))
    torch.testing.assert_close(
        wrap_to_pi(torch.tensor(2.1 * torch.pi)),
        torch.tensor(0.1 * torch.pi),
    )
    torch.testing.assert_close(
        wrap_to_pi(torch.tensor(-2.1 * torch.pi)),
        torch.tensor(-0.1 * torch.pi),
    )
    torch.testing.assert_close(
        wrap_to_pi(torch.tensor(torch.pi + 1e-6)),
        torch.tensor(-torch.pi + 1e-6),
    )
    torch.testing.assert_close(
        wrap_to_pi(torch.tensor(-torch.pi)),
        torch.tensor(torch.pi),
    )  # Add test for -np.pi explicitly