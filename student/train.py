import argparse
from pathlib import Path

import torch
from torch.utils.data import DataLoader, random_split

from dataset import DepthPairDataset
from model import StudentDepthNet


DATA_DIR = Path(__file__).resolve().parent.parent / "data"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--rgb-dir", default=str(DATA_DIR / "rgb"))
    p.add_argument("--depth-dir", default=str(DATA_DIR / "depth"))
    p.add_argument("--img-width", type=int, default=192)
    p.add_argument("--img-height", type=int, default=144)
    p.add_argument("--batch-size", type=int, default=8)
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--val-split", type=float, default=0.15)
    p.add_argument("--checkpoint-dir", default=str(Path(__file__).resolve().parent / "checkpoints"))
    return p.parse_args()


def main():
    args = parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    dataset = DepthPairDataset(args.rgb_dir, args.depth_dir, img_size=(args.img_width, args.img_height))

    val_size = max(1, int(len(dataset) * args.val_split)) if len(dataset) > 1 else 0
    train_size = len(dataset) - val_size
    train_set, val_set = random_split(dataset, [train_size, val_size])

    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_set, batch_size=args.batch_size) if val_size > 0 else None

    model = StudentDepthNet().to(device)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"student params: {n_params:,}")

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = torch.nn.L1Loss()

    ckpt_dir = Path(args.checkpoint_dir)
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    best_val = float("inf")

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0
        for imgs, depths in train_loader:
            imgs, depths = imgs.to(device), depths.to(device)

            pred = model(imgs)
            loss = loss_fn(pred, depths)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_loss += loss.item() * imgs.size(0)
        train_loss /= len(train_set)

        if val_loader is not None:
            model.eval()
            val_loss = 0.0
            with torch.no_grad():
                for imgs, depths in val_loader:
                    imgs, depths = imgs.to(device), depths.to(device)
                    pred = model(imgs)
                    val_loss += loss_fn(pred, depths).item() * imgs.size(0)
            val_loss /= len(val_set)
        else:
            val_loss = train_loss

        print(f"epoch {epoch:03d}  train_loss {train_loss:.4f}  val_loss {val_loss:.4f}")

        if val_loss < best_val:
            best_val = val_loss
            torch.save(model.state_dict(), ckpt_dir / "best.pt")

    torch.save(model.state_dict(), ckpt_dir / "last.pt")
    print(f"done. best val_loss {best_val:.4f}, checkpoints in {ckpt_dir}")


if __name__ == "__main__":
    main()
