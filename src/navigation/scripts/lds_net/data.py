"""Immutable source pairing and cached square 320 drivable labels."""
from __future__ import annotations

import concurrent.futures
import hashlib
import json
import random
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader

ROOT = Path(__file__).resolve().parents[2]
SEG_ROOT = ROOT / 'data/lds_net_bdd_drivable'
COCO = ROOT / 'data/coco_stuff_driving/data.yaml'


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Windows readers/indexers can briefly deny replacement of an open file.
    # Unique temporary names also prevent concurrent writers sharing one .tmp.
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
            prefix=path.name+'.', suffix='.tmp', delete=False) as handle:
        temp = Path(handle.name)
        json.dump(value, handle, ensure_ascii=False, indent=2)
    try:
        for attempt in range(9):
            try:
                temp.replace(path)
                break
            except PermissionError:
                if attempt == 8:
                    raise
                time.sleep(min(.025*2**attempt, .25))
    finally:
        try:
            temp.unlink(missing_ok=True)
        except PermissionError:
            pass


def letterbox(image, mask=None, size=320):
    h, w = image.shape[:2]
    ratio = min(size / h, size / w)
    nh, nw = round(h * ratio), round(w * ratio)
    top, left = (size - nh) // 2, (size - nw) // 2
    result = np.full((size, size, 3), 114, dtype=np.uint8)
    result[top:top + nh, left:left + nw] = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    output = None
    if mask is not None:
        output = np.full((size, size), 255, dtype=np.uint8)
        output[top:top + nh, left:left + nw] = cv2.resize(mask, (nw, nh), interpolation=cv2.INTER_NEAREST)
    return result, output, (ratio, top, left, nh, nw)


def _cache_pair(row):
    image_path, mask_path = Path(row['image']), Path(row['mask'])
    image = cv2.imread(str(image_path))
    mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
    if image is None or mask is None or mask.ndim != 2 or mask.shape != image.shape[:2]:
        raise ValueError(f'Image/mask mismatch: {image_path} {mask_path}')
    values = np.unique(mask).tolist()
    if not set(values) <= {0, 1, 2}:
        raise ValueError(f'Unexpected legacy drivable classes: {mask_path}: {values}')
    image320, mask320, geometry = letterbox(image, mask)
    target_img = SEG_ROOT / 'images' / row['split'] / (row['id'] + '.jpg')
    target_mask = SEG_ROOT / 'masks' / row['split'] / (row['id'] + '.png')
    target_img.parent.mkdir(parents=True, exist_ok=True)
    target_mask.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(target_img), image320, [cv2.IMWRITE_JPEG_QUALITY, 95]):
        raise OSError(target_img)
    if not cv2.imwrite(str(target_mask), mask320):
        raise OSError(target_mask)
    counts = np.bincount(mask320.reshape(-1), minlength=256)
    return {**row, 'cached_image': str(target_img), 'cached_mask': str(target_mask),
            'image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
            'mask_sha256': hashlib.sha256(mask_path.read_bytes()).hexdigest(),
            'original_shape': list(mask.shape), 'letterbox': geometry,
            'pixels': [int(counts[k]) for k in (0, 1, 2, 255)]}


def prepare_drivable():
    manifest_path = SEG_ROOT / 'manifest.json'
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        assert manifest['audit']['passed'], 'Existing segmentation audit failed'
        assert all(Path(r['cached_image']).exists() and Path(r['cached_mask']).exists()
                   for split in ('train', 'val') for r in manifest[split])
        return manifest
    write_json(SEG_ROOT / 'status.json', {'stage': 'pairing'})
    maskroot = ROOT / 'data/archive/bdd100k/labels/drivable_legacy/masks'
    image_root = ROOT / 'data/bdd9_scene_balanced/images'
    candidates, mismatched = {}, 0
    for source_split in ('train', 'val'):
        for image in sorted((image_root / source_split).glob('*.jpg')):
            mask = maskroot / source_split / f'{image.stem}_drivable_id.png'
            if not mask.exists():
                mismatched += 1
                continue
            candidates[image.stem] = {'id': image.stem, 'image': str(image), 'mask': str(mask),
                                      'source_split': source_split}
    native_val = sorted(r['id'] for r in candidates.values() if r['source_split'] == 'val')
    train_ids = sorted(r['id'] for r in candidates.values() if r['source_split'] == 'train')
    random.Random(20261016).shuffle(train_ids)
    # One BDD image per video. Hold out whole video IDs; official-val files never enter training.
    heldout = set(native_val + train_ids[:max(0, 1000 - len(native_val))])
    rows = [{**r, 'split': 'val' if r['id'] in heldout else 'train'}
            for r in sorted(candidates.values(), key=lambda v: v['id'])]
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        for i, row in enumerate(pool.map(_cache_pair, rows), 1):
            results.append(row)
            if i % 500 == 0 or i == len(rows):
                print(f'DRIVABLE_AUDIT {i}/{len(rows)}', flush=True)
                write_json(SEG_ROOT / 'status.json', {'stage': 'caching_and_auditing', 'done': i, 'total': len(rows)})
    # Keep a content only once, prioritizing validation so no copy leaks into training.
    unique, duplicates = {}, []
    for row in sorted(results, key=lambda r: (r['split'] != 'val', r['id'])):
        key = row['image_sha256']
        if key in unique:
            duplicates.append({'dropped': row['id'], 'kept': unique[key]['id']})
        else:
            unique[key] = row
    output = {split: sorted((r for r in unique.values() if r['split'] == split), key=lambda r: r['id'])
              for split in ('train', 'val')}
    assert len(output['train']) > 1000 and len(output['val']) >= 900
    assert not ({r['id'] for r in output['train']} & {r['id'] for r in output['val']})
    histogram = {s: np.array([r['pixels'] for r in output[s]], dtype=np.int64).sum(0).tolist() for s in output}
    assert all(histogram[s][1] > 0 and histogram[s][2] > 0 for s in output)
    manifest = {**output, 'seed': 20261016, 'imgsz': 320,
                'names': ['background', 'direct', 'alternative'], 'ignore_index': 255,
                'source_encoding': 'legacy: 0=background,1=direct,2=alternative; preserved',
                'split_policy': 'local fixed video-ID holdout plus available official val; not official benchmark',
                'audit': {'passed': True, 'all_images_masks_decoded': True, 'unmatched_skipped': mismatched,
                          'content_duplicates_dropped': duplicates, 'pixels_bg_direct_alt_ignore': histogram,
                          'counts': {s: len(output[s]) for s in output}, 'native_val_pairs': len(native_val)}}
    write_json(manifest_path, manifest)
    write_json(SEG_ROOT / 'status.json', {'stage': 'complete', **manifest['audit']})
    return manifest


class DrivableDataset(Dataset):
    def __init__(self, split='train', augment=False):
        self.rows = json.loads((SEG_ROOT / 'manifest.json').read_text(encoding='utf-8'))[split]
        self.augment = augment

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        row = self.rows[index]
        image = cv2.imread(row['cached_image'])
        mask = cv2.imread(row['cached_mask'], cv2.IMREAD_UNCHANGED)
        if self.augment:
            if random.random() < .5:
                image, mask = image[:, ::-1], mask[:, ::-1]
            if random.random() < .7:
                gain, offset = random.uniform(.8, 1.2), random.uniform(-12, 12)
                image = np.clip(image.astype(np.float32) * gain + offset, 0, 255).astype(np.uint8)
        rgb = np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1))
        return {'img': torch.from_numpy(rgb), 'mask': torch.from_numpy(np.ascontiguousarray(mask)).long(),
                'id': row['id']}


def seed_worker(_):
    seed = torch.initial_seed() % (2 ** 32)
    np.random.seed(seed)
    random.seed(seed)
    cv2.setNumThreads(0)


def seg_loader(split, batch=32, workers=2, augment=False, ids=None):
    dataset = DrivableDataset(split, augment)
    if ids is not None:
        wanted = set(ids)
        dataset.rows = [row for row in dataset.rows if row['id'] in wanted]
        if {row['id'] for row in dataset.rows} != wanted or not wanted:
            raise ValueError('Requested road evaluation IDs are empty or outside this split')
    return DataLoader(dataset, batch_size=batch, shuffle=augment,
                      num_workers=workers, pin_memory=torch.cuda.is_available(),
                      persistent_workers=workers > 0, worker_init_fn=seed_worker,
                      generator=torch.Generator().manual_seed(20261016))


@torch.inference_mode()
def evaluate_segmentation(model, loader, device, examples=None):
    from .model import segmentation_loss
    from .road_loss import boundary_ce_loss
    from .aux_boundary import balanced_boundary_bce
    from .discrimination import discrimination_loss
    model.eval()
    histogram = torch.zeros((3, 3), dtype=torch.int64, device=device)
    dtype = next(model.parameters()).dtype
    losses, seen = np.zeros(2), 0
    boundary_loss=0.
    aux_loss=0.
    da_loss=0.;validation_batch=0
    boundary = torch.zeros(4, device=device)
    for batch in loader:
        images = batch['img'].to(device, dtype=dtype, non_blocking=True) / 255
        masks = batch['mask'].to(device, non_blocking=True)
        if getattr(model,'seg_aux_enabled',False):
            logits,aux_logits=model.predict_semantic_with_boundary(images)
            aux_loss+=float(balanced_boundary_bce(aux_logits,masks))*len(images)
        else:
            logits = model.predict_semantic(images)
        ce, dice = segmentation_loss(logits, masks)
        if getattr(model,'seg_da_enabled',False):
            da_loss+=float(discrimination_loss(logits,masks,20261017+validation_batch))*len(images)
        validation_batch+=1
        losses += np.array([float(ce), float(dice)]) * len(images)
        if getattr(model,'seg_boundary_weight',0.):
            boundary_loss+=float(boundary_ce_loss(logits,masks,getattr(model,'seg_boundary_radius',2)))*len(images)
        seen += len(images)
        prediction = F_interpolate(logits.float(), masks.shape[-2:]).argmax(1)
        valid = masks != 255
        histogram += torch.bincount((masks[valid] * 3 + prediction[valid]), minlength=9).reshape(3, 3)
        # Inner boundaries, two-pixel tolerance, ignoring letterbox boundaries.
        interior = -torch.nn.functional.max_pool2d(-valid[:, None].float(), 3, 1, 1) > .5
        for category in (1, 2):
            target = (masks == category)[:, None].float()
            pred = (prediction == category)[:, None].float()
            true_edge = (target + torch.nn.functional.max_pool2d(-target, 3, 1, 1) > .5) & interior
            pred_edge = (pred + torch.nn.functional.max_pool2d(-pred, 3, 1, 1) > .5) & interior
            near_true = torch.nn.functional.max_pool2d(true_edge.float(), 5, 1, 2) > .5
            near_pred = torch.nn.functional.max_pool2d(pred_edge.float(), 5, 1, 2) > .5
            boundary += torch.stack(((pred_edge & near_true).sum(), pred_edge.sum(),
                                     (true_edge & near_pred).sum(), true_edge.sum())).float()
    h = histogram.cpu().numpy()
    union = h.sum(0) + h.sum(1) - np.diag(h)
    iou = np.divide(np.diag(h), union, out=np.full(3, np.nan), where=union > 0)
    bp = float(boundary[0] / boundary[1].clamp_min(1))
    br = float(boundary[2] / boundary[3].clamp_min(1))
    result={'road/iou_background': float(iou[0]), 'road/iou_direct': float(iou[1]),
            'road/iou_alternative': float(iou[2]), 'road/foreground_miou': float(np.nanmean(iou[1:])),
            'road/miou_all3': float(np.nanmean(iou)), 'road/confusion': h.tolist(), 'road/images': len(loader.dataset),
            'road/ce': float(losses[0] / seen), 'road/dice': float(losses[1] / seen),
            'road/boundary_f1_2px_320': 2 * bp * br / max(bp + br, 1e-9)}
    if hasattr(model,'seg_boundary_weight'):
        result['road/boundary_ce']=boundary_loss/max(seen,1)
    if hasattr(model,'seg_aux_enabled'):
        result['road/aux_bce']=aux_loss/max(seen,1)
    if hasattr(model,'seg_da_enabled'):
        result['road/da_loss']=da_loss/max(seen,1)
    return result


def F_interpolate(logits, size):
    return torch.nn.functional.interpolate(logits, size=size, mode='bilinear', align_corners=False)
