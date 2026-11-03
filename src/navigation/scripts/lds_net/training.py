"""Official detection training, separately supervised drivable batches and retention gates."""
from __future__ import annotations

import copy
import json
import os
import time
from pathlib import Path

import numpy as np
import torch
from ultralytics.data import build_yolo_dataset
from ultralytics.models.yolo.detect import DetectionTrainer, DetectionValidator

from .data import COCO, seg_loader, evaluate_segmentation, write_json
from .model import segmentation_loss
from .quant import set_observers


def load_model(path):
    # Only locally generated/trusted checkpoints, never remote pickle files.
    checkpoint = torch.load(path, map_location='cpu', weights_only=False)
    model = checkpoint.get('ema') or checkpoint['model']
    model = model.float()
    model.criterion = None
    return model


class SquareValidator(DetectionValidator):
    def build_dataset(self, img_path, mode='val', batch=None):
        return build_yolo_dataset(self.args, img_path, batch, self.data,
                                  mode=mode, rect=False, stride=32)


class RoadTrainer(DetectionTrainer):
    def get_model(self, cfg=None, weights=None, verbose=True):
        from .model import LDSNetModel
        if isinstance(weights, LDSNetModel):
            model = copy.deepcopy(weights).float()
            model.criterion = None
            assert model.nc == self.data['nc'] and model.names == self.data['names']
            return model
        if weights is not None:
            raise TypeError('LDS-Net resume requires its own checkpoint, not an unrelated model')
        return LDSNetModel(nc=self.data['nc'], names=self.data['names'])

    def __init__(self, *, experiment, stage_name, joint=False, reference_map=None,
                 reference_road=None, qat=False, max_map_drop=.02, **kwargs):
        self.experiment = Path(experiment)
        self.stage_name = stage_name
        self.joint = joint
        self.reference_map = reference_map
        self.reference_road = reference_road
        self.max_map_drop = max_map_drop
        self.qat_reference = qat
        self.road_train_loader = None
        self.road_val_loader = None
        self.road_iterator = None
        self.latest_road = {}
        self.epoch_batches = 0
        self.last_batch_report = 0.
        super().__init__(**kwargs)
        self.add_callback('on_fit_epoch_end', self.record_progress)
        self.add_callback('on_train_epoch_start', self.epoch_start)
        self.add_callback('on_train_batch_end', self.batch_progress)

    def epoch_start(self, trainer):
        self.epoch_batches = 0
        if self.qat_reference:
            set_observers(self.model, self.epoch < 2)

    def batch_progress(self, trainer):
        self.epoch_batches += 1
        now = time.time()
        if now - self.last_batch_report > 30 or self.epoch_batches == len(self.train_loader):
            self.last_batch_report = now
            write_json(self.experiment / 'status.json', {
                'state': 'training', 'stage': self.stage_name, 'epoch': self.epoch + 1,
                'epochs': self.epochs, 'batch': self.epoch_batches, 'batches': len(self.train_loader),
                'input': self.args.imgsz, 'pid': os.getpid(), 'time': now,
                'train_loss': {k: float(v) for k, v in (self.tloss or {}).items()},
                'head': 'one2one', 'nms': False, 'square': True,
                'last_checkpoint': str(self.last), 'validation_pending_for_this_epoch': True})

    def build_dataset(self, img_path, mode='train', batch=None):
        return build_yolo_dataset(self.args, img_path, batch, self.data,
                                  mode=mode, rect=False, stride=32)

    def get_validator(self):
        return SquareValidator(self.test_loader, save_dir=self.save_dir,
                               args=copy.copy(self.args), _callbacks=self.callbacks)

    def preprocess_batch(self, batch):
        batch = super().preprocess_batch(batch)
        if self.joint:
            if self.road_train_loader is None:
                self.road_train_loader = seg_loader('train', batch=32, workers=2, augment=True)
                self.road_iterator = iter(self.road_train_loader)
            try:
                road = next(self.road_iterator)
            except StopIteration:
                self.road_iterator = iter(self.road_train_loader)
                road = next(self.road_iterator)
            batch['seg_img'] = road['img'].to(self.device, non_blocking=True).float() / 255
            batch['seg_mask'] = road['mask'].to(self.device, non_blocking=True)
        return batch

    def validate(self):
        if self.qat_reference:
            set_observers(self.ema.ema, False)
        metrics = self.validator(self)
        det = float(metrics.pop('fitness', metrics['metrics/mAP50-95(B)']))
        fitness = det
        if self.joint:
            if self.road_val_loader is None:
                self.road_val_loader = seg_loader('val', batch=32, workers=2)
            road = evaluate_segmentation(self.ema.ema, self.road_val_loader, self.device)
            self.latest_road = road
            metrics.update({k: v for k, v in road.items() if isinstance(v, (float, int))})
            metrics['val/road_ce'], metrics['val/road_dice'] = road['road/ce'], road['road/dice']
            eligible = (self.reference_map is None or det >= self.reference_map - self.max_map_drop)
            eligible &= (self.reference_road is None or road['road/foreground_miou'] >= self.reference_road - .03)
            fitness = .5 * det + .5 * road['road/foreground_miou'] if eligible else -1.
            metrics['selection/eligible'] = float(eligible)
            print(f'ROAD {self.stage_name} epoch={self.epoch + 1} det={det:.4f} '
                  f'fg_mIoU={road["road/foreground_miou"]:.4f} eligible={eligible}', flush=True)
        if self.best_fitness is None or fitness > self.best_fitness:
            self.best_fitness = fitness
        return metrics, fitness

    def record_progress(self, trainer):
        row = {'stage': self.stage_name, 'epoch': self.epoch + 1, 'epochs': self.epochs,
               'input': self.args.imgsz, 'pid': os.getpid(), 'time': time.time(),
               'metrics': {k: float(v) for k, v in self.metrics.items()},
               'fitness': float(self.fitness), 'best_fitness': float(self.best_fitness),
               'head': 'one2one', 'nms': False, 'square': True,
               'best_checkpoint': str(self.best), 'last_checkpoint': str(self.last)}
        write_json(self.experiment / 'status.json', {'state': 'training', **row})
        with (self.experiment / 'history.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps(row) + '\n')
        if self.latest_road:
            write_json(self.save_dir / 'road_metrics_latest.json', self.latest_road)

    def final_eval(self):
        # Keep optimizer/scaler/epoch intact for explicit resume. Revalidate selected
        # checkpoint independently in the orchestration script instead of stripping it.
        write_json(self.save_dir / 'completion.json', {'best': str(self.best), 'last': str(self.last),
                   'epochs_completed': self.epoch + 1, 'best_fitness': self.best_fitness,
                   'checkpoints_keep_optimizer': True})


def train_detection(model, experiment, stage, size, epochs, lr, patience,
                    joint=False, reference_map=None, reference_road=None, qat=False, smoke=False,
                    finetune=False, max_map_drop=.02):
    model.criterion = None
    model.end2end = True
    for parameter in model.parameters():
        parameter.requires_grad_(True)
    initial_state = ({k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
                     if finetune else None)
    trainer = RoadTrainer(experiment=experiment, stage_name=stage, joint=joint,
                          reference_map=reference_map, reference_road=reference_road, qat=qat,
                          max_map_drop=max_map_drop,
                          overrides={'model': 'yolo26n.yaml', 'data': str(COCO), 'task': 'detect',
                            'pretrained': False, 'epochs': epochs, 'patience': patience,
                            'imgsz': size, 'batch': 32 if not smoke else 4, 'workers': 4 if not smoke else 0,
                            'device': 0, 'optimizer': 'AdamW', 'lr0': lr, 'lrf': .05,
                            'warmup_epochs': 3. if stage == 'detect416' else 1.,
                            'warmup_bias_lr': min(.01, lr) if finetune else .01,
                            'cos_lr': True, 'weight_decay': .0005,
                            'amp': False if qat else 'bf16', 'seed': 20261016,
                            'deterministic': True, 'rect': False, 'multi_scale': 0.,
                            'mosaic': .5 if stage == 'detect416' else .15 if not joint else 0.,
                            'close_mosaic': min(10, max(0, epochs - 1)), 'mixup': 0., 'cutmix': 0.,
                            'scale': .3, 'translate': .1, 'fliplr': .5,
                            'project': str(Path(experiment) / 'training'), 'name': stage,
                            'exist_ok': False, 'save': True, 'save_period': 20,
                            'max_det': 300, 'nms': False, 'plots': True, 'val': True,
                            'fraction': .003 if smoke else 1., 'verbose': False})
    trainer.model = model
    if finetune:
        def audit_finetune(t):
            assert all(torch.equal(v.detach().cpu(), initial_state[k])
                       for k, v in t.model.state_dict().items()), 'Initialization changed before training'
            assert (t.model.seg_head is not None) == joint and t.args.imgsz == size
            # Continue the final end-to-end head weighting of the completed
            # detector instead of restarting its initial one-to-many schedule.
            t.model.criterion = t.model.init_criterion()
            criterion = t.model.criterion
            criterion.o2m_copy = criterion.final_o2m
            criterion.o2m = criterion.final_o2m
            criterion.o2o = criterion.total - criterion.o2m
            write_json(Path(experiment) / 'initialization_audit.json', {
                'pid': os.getpid(), 'weights_equal_source': True, 'imgsz': size,
                'parameters': sum(p.numel() for p in t.model.parameters()),
                'trainable_parameters': sum(p.numel() for p in t.model.parameters() if p.requires_grad),
                'new_optimizer': True, 'optimizer': type(t.optimizer).__name__,
                'lr0': t.args.lr0, 'final_lr': t.args.lr0 * t.args.lrf,
                'warmup_bias_lr': t.args.warmup_bias_lr,
                'head_loss_ratio': [criterion.o2m, criterion.o2o],
                'next_stage_started': False})
            print('FINETUNE_INITIALIZATION_AUDIT_PASSED', flush=True)
        trainer.add_callback('on_train_start', audit_finetune)
    try:
        trainer.train()
    finally:
        # Explicitly release the extra BDD iterators before independent validation.
        for loader in (trainer.road_train_loader, trainer.road_val_loader):
            iterator = getattr(loader, '_iterator', None)
            if iterator is not None:
                iterator._shutdown_workers()
    best = load_model(trainer.best)
    checkpoint = torch.load(trainer.best, map_location='cpu', weights_only=False)
    return best, {k: float(v) for k, v in checkpoint['train_metrics'].items()}, str(trainer.best)


def warmup_segmentation(model, experiment, epochs=8, smoke=False):
    out = Path(experiment) / 'training/road_warmup320'
    out.mkdir(parents=True, exist_ok=False)
    model = model.to('cuda').float()
    model.criterion = None
    model.add_segmentation()
    before = {k: v.detach().cpu().clone() for k, v in model.state_dict().items() if not k.startswith('seg_head.')}
    for parameter in model.parameters():
        parameter.requires_grad_(False)
    for parameter in model.seg_head.parameters():
        parameter.requires_grad_(True)
    optimizer = torch.optim.AdamW(model.seg_head.parameters(), lr=.001, weight_decay=.0005)
    train = seg_loader('train', batch=32 if not smoke else 4, workers=2 if not smoke else 0, augment=True)
    val = seg_loader('val', batch=32, workers=2 if not smoke else 0)
    best = -1.
    last_report = 0.
    for epoch in range(epochs):
        model.eval()  # Freeze backbone BN statistics as well as weights.
        model.seg_head.train()
        total, seen = 0., 0
        for i, batch in enumerate(train):
            x = batch['img'].to('cuda', non_blocking=True).float() / 255
            y = batch['mask'].to('cuda', non_blocking=True)
            optimizer.zero_grad(set_to_none=True)
            ce, dice = segmentation_loss(model.predict_semantic(x), y)
            loss = ce + dice
            if not torch.isfinite(loss):
                raise FloatingPointError('Non-finite segmentation warmup loss')
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.seg_head.parameters(), 10.)
            optimizer.step()
            total += float(loss.detach()) * len(x)
            seen += len(x)
            if time.time() - last_report > 30:
                last_report = time.time()
                write_json(Path(experiment) / 'status.json', {
                    'state': 'training', 'stage': 'road_warmup320',
                    'epoch': epoch + 1, 'epochs': epochs, 'batch': i + 1,
                    'batches': len(train), 'input': 320, 'pid': os.getpid(),
                    'train_loss': total / seen, 'frozen_detector': True,
                    'time': last_report})
            if smoke and i >= 2:
                break
        metrics = evaluate_segmentation(model, val, 'cuda')
        improved = metrics['road/foreground_miou'] > best
        best = max(best, metrics['road/foreground_miou'])
        snapshot = copy.deepcopy(model).cpu().float()
        for key, value in before.items():
            assert torch.equal(value, snapshot.state_dict()[key]), f'Frozen detector changed: {key}'
        checkpoint = {'model': snapshot, 'ema': None, 'epoch': epoch, 'optimizer': optimizer.state_dict(),
                      'train_metrics': metrics, 'train_args': {'task': 'detect', 'imgsz': 320},
                      'detector_parameters_and_bn_unchanged': True}
        torch.save(checkpoint, out / 'last.pt')
        if improved:
            torch.save(checkpoint, out / 'best.pt')
        row = {'stage': 'road_warmup320', 'epoch': epoch + 1, 'epochs': epochs,
               'train_loss': total / seen, 'metrics': metrics, 'pid': os.getpid(),
               'detector_parameters_and_bn_unchanged': True, 'time': time.time()}
        write_json(Path(experiment) / 'status.json', {'state': 'training', **row})
        with (out / 'history.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps(row) + '\n')
        print(f'ROAD_WARMUP {epoch + 1}/{epochs} loss={total / seen:.4f} fg_mIoU={best:.4f}', flush=True)
    for loader in (train, val):
        iterator = getattr(loader, '_iterator', None)
        if iterator is not None:
            iterator._shutdown_workers()
    write_json(out / 'completion.json', {'epochs_completed': epochs, 'best': str(out / 'best.pt'),
               'best_foreground_miou': best, 'detector_parameters_and_bn_unchanged': True})
    model = load_model(out / 'best.pt')
    for parameter in model.parameters():
        parameter.requires_grad_(True)
    return model, best, str(out / 'best.pt')


def independent_validation(model, output, segmentation=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # AutoBackend may fuse BN and remove auxiliary detection branches in place.
    # Validation must never modify a model that will subsequently be trained.
    model = copy.deepcopy(model).float().eval()
    set_observers(model, False)
    validator = SquareValidator(save_dir=output / 'detection', args={'data': str(COCO), 'imgsz': 320, 'batch': 32, 'workers': 0,
                                     'device': '0', 'nms': False, 'rect': False, 'plots': True,
                                     'project': str(output), 'name': 'detection', 'exist_ok': True,
                                     'quantize': 32})
    metrics = {k: float(v) for k, v in validator(model=model).items()}
    box = validator.metrics.box
    report = {'detection': metrics, 'imgsz': [320, 320], 'head': 'one2one', 'nms': False,
              'classes': [{'id': int(c), 'name': model.names[int(c)], 'ap50': float(box.ap50[i]),
                           'ap50_95': float(box.ap[i]), 'recall': float(box.r[i])}
                          for i, c in enumerate(box.ap_class_index)]}
    if segmentation and model.seg_head is not None:
        model = model.to('cuda').float().eval()
        report['segmentation'] = evaluate_segmentation(model, seg_loader('val', workers=0), 'cuda')
    write_json(output / 'metrics.json', report)
    return report
