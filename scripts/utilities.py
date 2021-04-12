import numpy as np

def spa_derivX4_4d(
        self, i, j, k, l, V, g
):  # Left -> right == Outer Most -> Inner Most
    left_deriv = 0.0
    right_deriv = 0.0
    if 3 not in g.pDim:
        if l == 0:
            left_boundary = 0.0
            left_boundary = V[i, j, k, l] + abs(
                V[i, j, k, l + 1] - V[i, j, k, l]
            ) * np.sign(V[i, j, k, l])
            left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[3]
            right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
        elif l == V.shape[3] - 1:
            right_boundary = 0.0
            right_boundary = V[i, j, k, l] + abs(
                V[i, j, k, l] - V[i, j, k, l - 1]
            ) * np.sign(V[i, j, k, l])
            left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
            right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[3]
        elif l != 0 and l != V.shape[3] - 1:
            left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
            right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
        return left_deriv, right_deriv
    else:
        if l == 0:
            left_boundary = 0.0
            left_boundary = V[i, j, k, V.shape[3] - 1]
            left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[3]
            right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
        elif l == V.shape[3] - 1:
            right_boundary = 0.0
            right_boundary = V[i, j, k, 0]
            left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
            right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[3]
        elif l != 0 and l != V.shape[3] - 1:
            left_deriv = (V[i, j, k, l] - V[i, j, k, l - 1]) / g.dx[3]
            right_deriv = (V[i, j, k, l + 1] - V[i, j, k, l]) / g.dx[3]
        return left_deriv, right_deriv


def spa_derivX3_4d(
        self, i, j, k, l, V, g
):  # Left -> right == Outer Most -> Inner Most
    left_deriv = 0.0
    right_deriv = 0.0
    if 2 not in g.pDim:
        if k == 0:
            left_boundary = 0.0
            left_boundary = V[i, j, k, l] + abs(
                V[i, j, k + 1, l] - V[i, j, k, l]
            ) * np.sign(V[i, j, k, l])
            left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[2]
            right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
        elif k == V.shape[2] - 1:
            right_boundary = 0.0
            right_boundary = V[i, j, k, l] + abs(
                V[i, j, k, l] - V[i, j, k - 1, l]
            ) * np.sign(V[i, j, k, l])
            left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
            right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[2]
        elif k != 0 and k != V.shape[2] - 1:
            left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
            right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
        return left_deriv, right_deriv
    else:
        if k == 0:
            left_boundary = 0.0
            left_boundary = V[i, j, V.shape[2] - 1, l]
            left_deriv = (V[i, j, k, l] - left_boundary) / g.dx[2]
            right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
        elif k == V.shape[2] - 1:
            right_boundary = 0.0
            right_boundary = V[i, j, 0, l]
            left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
            right_deriv = (right_boundary - V[i, j, k, l]) / g.dx[2]
        elif k != 0 and k != V.shape[2] - 1:
            left_deriv = (V[i, j, k, l] - V[i, j, k - 1, l]) / g.dx[2]
            right_deriv = (V[i, j, k + 1, l] - V[i, j, k, l]) / g.dx[2]
        return left_deriv, right_deriv
