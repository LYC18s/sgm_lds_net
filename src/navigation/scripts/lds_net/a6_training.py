"""Six-bit activation experiments with independent common-source initialization."""
from pathlib import Path
import torch
from .training import RoadTrainer
from .quant_training import QuantTrainer
from .deploy_quant import TensorQuant,QConv,set_policy,calibrate,observers
from .quant_calibration import batches
from .aggressive_data import reset_detection_epoch
from .data import write_json


def scheduled_stage(phase,epoch):
    if phase not in ('W4A6','W2A6'):raise ValueError(phase)
    if epoch<2:return 'W4A8'
    if phase=='W4A6' or epoch<4:return 'W4A6'
    return 'W2A6'


class A6Trainer(QuantTrainer):
    def __init__(self,**kwargs):
        self.target_stage=kwargs['phase']
        super().__init__(**kwargs)

    def sync_ema_policy(self):
        ema=self.ema.ema
        ema.core.stage=self.model.core.stage
        for name,m in self.model.named_modules():
            if isinstance(m,TensorQuant):
                other=ema.get_submodule(name)
                other.bits=m.bits;other.unsigned=m.unsigned;other.observe=False
            elif isinstance(m,QConv):
                other=ema.get_submodule(name);other.bits=m.bits;other.update_weights=False
                if hasattr(m,'w2_scale_ratio'):
                    if hasattr(other,'w2_scale_ratio'):other.w2_scale_ratio.copy_(m.w2_scale_ratio)
                    else:other.register_buffer('w2_scale_ratio',m.w2_scale_ratio.detach().clone())

    def audit_start(self,t):
        super().audit_start(t)
        self.sync_ema_policy()
        write_json(self.save_dir/'a6_protocol.json',{
            'phase':self.phase,'target_stage':self.target_stage,'max_epochs':self.epochs,
            'schedule':[[1,2,'W4A8'],[3,24,'W4A6']] if self.phase=='W4A6' else
                       [[1,2,'W4A8'],[3,4,'W4A6'],[5,24,'W2A6']],
            'scale_freeze_epoch':5 if self.phase=='W4A6' else 7,
            'start_epoch':self.start_epoch+1,'resumed':bool(self.resume),
            'weight_2bit_codes':[-2,-1,0,1],'activation_relu_6bit_codes':[0,63],
            'activation_linear_6bit_codes':[-32,31],
            'sensitive_layers':'first and five terminal conv W8; true terminal inputs A8',
            'independent_from_other_group':True})

    def epoch_start(self,t):
        RoadTrainer.epoch_start(self,t)
        reset_detection_epoch(self.train_loader,self.epoch)
        wanted=scheduled_stage(self.phase,self.epoch)
        changed=self.model.core.stage!=wanted
        if changed:
            set_policy(self.model.core,wanted)
            calibrate(self.model.core,lambda:batches(self.manifest))
            self.sync_aux_policy()
        enabled=self.epoch<(4 if self.phase=='W4A6' else 6)
        observers(self.model.core,enabled)
        for m in self.model.model[-1].modules():
            if isinstance(m,TensorQuant):m.observe=enabled
            if isinstance(m,QConv):m.update_weights=enabled
        self.sync_ema_policy()
        if changed:
            # Copy newly initialized scales before the first EMA update.
            live=self.model.state_dict()
            with torch.no_grad():
                for name,v in self.ema.ema.state_dict().items():
                    if name.endswith(('.scale','.weight_scale','.w2_scale_ratio')):v.copy_(live[name])
            write_json(self.save_dir/'transitions'/f'epoch_{self.epoch+1:03d}.json',{
                'epoch':self.epoch+1,'stage':wanted,'calibration_images':1024,
                'observers_enabled':enabled,'weight_code_range':[-2,1] if wanted=='W2A6' else [-8,7]})
        if self.road_train_loader is not None:
            self.road_train_loader.batch_sampler.set_epoch(self.epoch)
            self.road_iterator=iter(self.road_train_loader)
