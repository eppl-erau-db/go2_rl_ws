import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, random_split
import matplotlib.pyplot as plt

# === Load your data ===
data = np.load("/home/gabriel/workspaces/go2_rl_ws/motor_dataset.npz")
X, Y = data['X'], data['Y']

# === Normalize (z-score)
X_mean, X_std = X.mean(0), X.std(0) + 1e-8
Y_mean, Y_std = Y.mean(0), Y.std(0) + 1e-8
X_norm = (X - X_mean) / X_std
Y_norm = (Y - Y_mean) / Y_std

# === Convert to PyTorch tensors
X_tensor = torch.tensor(X_norm, dtype=torch.float32)
Y_tensor = torch.tensor(Y_norm, dtype=torch.float32)
dataset = TensorDataset(X_tensor, Y_tensor)

# === Train/Val split
train_size = int(0.9 * len(dataset))
val_size = len(dataset) - train_size
train_set, val_set = random_split(dataset, [train_size, val_size])

train_loader = DataLoader(train_set, batch_size=1024, shuffle=True)
val_loader = DataLoader(val_set, batch_size=1024)

# === Softsign MLP Model
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

# === Initialize
model = SoftsignMLP()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()

# === Training Loop
train_losses, val_losses = [], []
best_val_loss = float('inf')

for epoch in range(500):
    model.train()
    epoch_loss = 0
    for xb, yb in train_loader:
        pred = model(xb)
        loss = loss_fn(pred, yb)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        epoch_loss += loss.item()
    train_loss = epoch_loss / len(train_loader)
    train_losses.append(train_loss)

    model.eval()
    with torch.no_grad():
        val_loss = sum(loss_fn(model(xv), yv).item() for xv, yv in val_loader) / len(val_loader)
        val_losses.append(val_loss)

    print(f"[{epoch+1:03d}] Train Loss: {train_loss:.4f}  |  Val Loss: {val_loss:.4f}")

    if (epoch + 1) % 100 == 0:
        torch.save(model.state_dict(), f"mlp_epoch{epoch+1}.pt")

    if val_loss < best_val_loss:
        best_val_loss = val_loss
        torch.save(model.state_dict(), "mlp_best.pt")
        print(f"✅ Saved new best model at epoch {epoch+1} with Val Loss: {val_loss:.4f}")

# === Plotting Loss
plt.plot(train_losses, label='Train')
plt.plot(val_losses, label='Validation')
plt.xlabel('Epoch')
plt.ylabel('Loss (MSE)')
plt.title('Torque Prediction Loss')
plt.legend()
plt.grid(True)
plt.show()

# === Evaluation: Load Best Model & Compute RMSE
model.load_state_dict(torch.load("mlp_best.pt"))
model.eval()

with torch.no_grad():
    xv, yv = val_set[:]
    pred = model(xv)
    pred_denorm = pred * torch.tensor(Y_std) + torch.tensor(Y_mean)
    yv_denorm = yv * torch.tensor(Y_std) + torch.tensor(Y_mean)
    rmse = torch.sqrt(((pred_denorm - yv_denorm) ** 2).mean())
    print(f"\n🏁 Final RMSE on Validation Set: {rmse:.3f} Nm")
