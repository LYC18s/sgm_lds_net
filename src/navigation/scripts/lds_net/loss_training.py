"""B0-B2 loss-only continuation from a common A2 checkpoint."""
import hashlib
import json

import torch

from .ablation_training import AblationTrainer
from .data import write_json
from .training import load_model


def initial_batch_proofs(path):
    by_batch={}
    for line in path.read_text().splitlines():
        row=json.loads(line);key=row['batch']
        if key in by_batch:
            assert by_batch[key]==row, 'Replayed initial batch differed after restart'
        by_batch[key]=row
    assert set(by_batch)=={1,2}, 'Initial paired batches were not recorded'
    return [by_batch[k] for k in sorted(by_batch)]


def configure_loss(model,dice_weight=1.,boundary_weight=0.):
    model.seg_weight=.1
    model.seg_dice_weight=float(dice_weight)
    model.seg_boundary_weight=float(boundary_weight)
    model.seg_boundary_radius=2
    model.record_task_losses=True
    return model


class LossTrainer(AblationTrainer):
    allowed_groups=['B0','B1','B2']

    def audit_start(self,t):
        super().audit_start(t)
        recipe=json.loads((self.experiment/'recipe.json').read_text())
        assert self.model.seg_dice_weight==recipe['dice_weight']
        assert self.model.seg_boundary_weight==recipe['boundary_weight']
        assert self.model.seg_weight==.1
        if not self.resume:
            source=load_model(self.root/'reference/source.pt')
            expected=source.state_dict();actual=self.model.state_dict()
            assert expected.keys()==actual.keys()
            assert all(torch.equal(v,actual[k].detach().cpu()) for k,v in expected.items()), 'B group did not start from common A2 weights'
        path=self.experiment/f'{self.stage_name}_initialization.json'
        record=json.loads(path.read_text())
        record.update({'loss_recipe':recipe,'common_A2_weights_verified':not bool(self.resume),
                       'optimizer_updates':len(self.optimizer.state)})
        write_json(path,record)

    def preprocess_batch(self,batch):
        batch=super().preprocess_batch(batch)
        if self.epoch==0 and self.epoch_batches<2:
            def digest(tensor):
                return hashlib.sha256(tensor.detach().cpu().contiguous().numpy().tobytes()).hexdigest()
            proof={'batch':self.epoch_batches+1,'detection_images':batch['im_file'],
                'road_image_sha256':digest(batch['seg_img']),
                'road_mask_sha256':digest(batch['seg_mask'])}
            with (self.experiment/'first_batches.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(proof)+'\n')
        return batch

    def validate(self):
        metrics,fitness=super().validate()
        raw=metrics['val/road_ce']+self.model.seg_dice_weight*metrics['val/road_dice']
        boundary=self.latest_road.get('road/boundary_ce',0.)
        raw+=self.model.seg_boundary_weight*boundary
        metrics['val/road_boundary_ce']=boundary
        metrics['val/road_raw']=raw
        metrics['val/road_weighted']=self.model.seg_weight*raw
        metrics['val/road_dice_weighted']=self.model.seg_dice_weight*metrics['val/road_dice']
        metrics['val/road_boundary_weighted']=self.model.seg_boundary_weight*boundary
        return metrics,fitness
