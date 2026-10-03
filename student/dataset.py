from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


class DepthPairDataset(Dataset):
    def __init__(self, rgb_dir, depth_dir, img_size=(192, 144)):
        self.rgb_dir = Path(rgb_dir)
        self.depth_dir = Path(depth_dir)
        self.img_size = img_size  # (width, height)

        rgb_files = {p.stem for p in self.rgb_dir.glob("*.png")}
        depth_files = {p.stem for p in self.depth_dir.glob("*.npy")}
        self.ids = sorted(rgb_files & depth_files)

        if not self.ids:
            raise RuntimeError(f"No matching rgb/depth pairs found in {rgb_dir} / {depth_dir}")

    def __len__(self):
        return len(self.ids)

    def __getitem__(self, idx):
        stem = self.ids[idx]

        img = cv2.imread(str(self.rgb_dir / f"{stem}.png"))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, self.img_size, interpolation=cv2.INTER_AREA)
        img = img.astype(np.float32) / 255.0
        img = torch.from_numpy(img).permute(2, 0, 1)

        depth = np.load(self.depth_dir / f"{stem}.npy")
        depth = cv2.resize(depth, self.img_size, interpolation=cv2.INTER_LINEAR)
        depth = depth.astype(np.float32) / 255.0
        depth = torch.from_numpy(depth).unsqueeze(0)

        return img, depth
