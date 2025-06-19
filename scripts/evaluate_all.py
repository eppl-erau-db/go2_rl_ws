import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
from torch.utils.data import TensorDataset

# === Load data ===
data = np.load("/home/gabriel/workspaces/go2_rl_ws/motor_dataset.npz")
X, Y = data['X'], data['Y']

# === Normalize using z-score
X_mean, X_std = X.mean(0), X.std(0) + 1e-8
Y_mean, Y_std = Y.mean(0), Y.std(0) + 1e-8
X_norm = (X - X_mean) / X_std
Y_norm = (Y - Y_mean) / Y_std

# === Convert to PyTorch tensors
X_tensor = torch.tensor(X_norm, dtype=torch.float32)
Y_tensor = torch.tensor(Y_norm, dtype=torch.float32)
dataset = TensorDataset(X_tensor, Y_tensor)

# === Validation set (same split: 90/10)
val_size = int(0.1 * len(dataset))
val_set = torch.utils.data.Subset(dataset, range(len(dataset) - val_size, len(dataset)))

# === Model Definition
class SoftsignMLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(48, 32),
            nn.Softsign(),
            nn.Linear(32, 32),
            nn.Softsign(),
            nn.Linear(32, 32),
            nn.Softsign(),
            nn.Linear(32, 12)
        )

    def forward(self, x):
        return self.net(x)

# === Load best model
model = SoftsignMLP()
model.load_state_dict(torch.load("mlp_epoch100.pt"))
model.eval()

# === Evaluate RMSE on validation set
with torch.no_grad():
    xv, yv = val_set[:]
    pred = model(xv)

    # Denormalize
    pred_denorm = pred * torch.tensor(Y_std) + torch.tensor(Y_mean)
    yv_denorm = yv * torch.tensor(Y_std) + torch.tensor(Y_mean)

    # RMSE
    rmse = torch.sqrt(((pred_denorm - yv_denorm) ** 2).mean())
    print(f"\n📈 RMSE on Validation Set: {rmse:.3f} Nm")

# === Subplots for all 12 joints
fig, axs = plt.subplots(6, 2, figsize=(12, 12))
axs = axs.flatten()
sample_range = slice(0, 300)  # adjust to limit samples shown

for i in range(12):
    axs[i].plot(pred_denorm[sample_range, i].numpy(), label='Predicted', linewidth=1)
    axs[i].plot(yv_denorm[sample_range, i].numpy(), label='Ground Truth', linewidth=1, alpha=0.7)
    axs[i].set_title(f"Joint {i}")
    axs[i].set_ylabel("Torque (Nm)")
    axs[i].grid(True)

axs[0].legend(loc="upper right")
plt.tight_layout()
plt.suptitle("Torque Prediction vs Ground Truth (First 300 Samples)", fontsize=16, y=1.02)
plt.show()
